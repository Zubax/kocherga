/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Zubax Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <kocherga.hpp>
#include <utility>
#include <numeric>

// Oh C, never change.
#ifdef CAN
#undef CAN
#endif


namespace kocherga_ymodem
{
/**
 * Error codes specific to this module.
 */
static constexpr std::int16_t ErrOK                             = 0;
static constexpr std::int16_t ErrPortWriteTimedOut              = 2001;
static constexpr std::int16_t ErrRetriesExhausted               = 2002;
static constexpr std::int16_t ErrProtocolError                  = 2003;
static constexpr std::int16_t ErrTransferCancelledByRemote      = 2004;
static constexpr std::int16_t ErrRemoteRefusedToProvideFile     = 2005;
static constexpr std::int16_t ErrPortError                      = 2006;

/**
 * Abstracts a platform-specific serial port and related functions for the YMODEM protocol.
 * The application can use these functions to reset its watchdog also, provided that the watchdog timeout
 * is not less than one second (otherwise, false-positive timeouts are possible).
 */
class IYModemPlatform
{
public:
    virtual ~IYModemPlatform() = default;

    /**
     * Result of the IO operations.
     */
    enum class Result : std::uint8_t
    {
        Success,        ///< Operation was completed successfully.
        Timeout,        ///< Operation has timed out.
        Error           ///< Operation has failed.
    };

    /**
     * Emits one byte into the port.
     * @param byte      The byte to emit.
     * @param timeout   The operation will be aborted if the byte could not be emitted in this amount of time.
     * @return          @ref Result.
     */
    virtual Result emit(std::uint8_t byte, std::chrono::microseconds timeout) = 0;

    /**
     * Receives one byte from the port.
     * @param out_byte  A reference where to store the received byte.
     * @param timeout   The operation will be aborted if the byte could not be received in this amount of time.
     * @return          @ref Result.
     */
    virtual Result receive(std::uint8_t& out_byte, std::chrono::microseconds timeout) = 0;

    /**
     * Returns the time since boot as a monotonic (i.e. steady) clock.
     * The clock must never overflow.
     * This is like @ref kocherga::IPlatform::getMonotonicUptime().
     */
    virtual std::chrono::microseconds getMonotonicUptime() const = 0;
};

/**
 * Downloads data using YMODEM or XMODEM protocol over the specified ChibiOS channel
 * (e.g. serial port, USB CDC ACM, TCP, ...).
 *
 * This class will request Checksum mode, in order to retain compatibility both with XMODEM and YMODEM senders
 * (YMODEM-compatible senders do support checksum mode as well as CRC mode).
 * Both 1K and 128-byte blocks are supported.
 * Overall, the following protocols are supported:
 *      - YMODEM
 *      - XMODEM
 *      - XMODEM-1K
 *
 * Reference: http://pauillac.inria.fr/~doligez/zmodem/ymodem.txt
 */
class YModemProtocol final : public kocherga::IProtocol
{
    static constexpr std::uint16_t BlockSizeXModem = 128;
    static constexpr std::uint16_t BlockSize1K     = 1024;
    static constexpr std::uint16_t WorstCaseBlockSizeWithCRC = BlockSize1K + 2;

    /// The timeouts are according to the YMODEM specification
    static constexpr std::chrono::microseconds SendTimeout          {1'000'000};    // NOLINT
    static constexpr std::chrono::microseconds CharReceiveTimeout   {1'000'000};    // NOLINT
    static constexpr std::chrono::microseconds InitialTimeout      {60'000'000};    // NOLINT
    static constexpr std::chrono::microseconds NextBlockTimeout     {5'000'000};    // NOLINT
    static constexpr std::chrono::microseconds BlockPayloadTimeout  {1'000'000};    // NOLINT

    static constexpr std::uint8_t MaxRetries = 3;

    struct ControlCharacters
    {
        static constexpr std::uint8_t SOH = 0x01;
        static constexpr std::uint8_t STX = 0x02;
        static constexpr std::uint8_t EOT = 0x04;
        static constexpr std::uint8_t ACK = 0x06;
        static constexpr std::uint8_t NAK = 0x15;
        static constexpr std::uint8_t CAN = 0x18;
    };

    IYModemPlatform& platform_;
    std::uint8_t buffer_[WorstCaseBlockSizeWithCRC]{};


    static std::uint8_t computeChecksum(const void* data, std::uint16_t size)
    {
        auto p = static_cast<const std::uint8_t*>(data);
        return std::uint8_t(std::accumulate(p, p + size, 0));
    }

    std::int16_t send(std::uint8_t byte)
    {
        KOCHERGA_TRACE("YMODEM TX 0x%x\n", byte);
        switch (platform_.emit(byte, SendTimeout))
        {
        case IYModemPlatform::Result::Success:
        {
            return 0;
        }
        case IYModemPlatform::Result::Timeout:
        {
            return -ErrPortWriteTimedOut;
        }
        case IYModemPlatform::Result::Error:
        {
            return -ErrPortError;
        }
        }

        return -ErrPortError;
    }

    std::int16_t receive(void* data, const std::uint16_t size, std::chrono::microseconds timeout)
    {
        assert(size <= kocherga::MaxDataBlockSize);
        auto* ui8 = static_cast<std::uint8_t*>(data);
        for (std::uint16_t i = 0; i < size; i++)
        {
            std::uint8_t byte = 0;
            switch (platform_.receive(byte, CharReceiveTimeout))
            {
            case IYModemPlatform::Result::Success:
            {
                *ui8++ = std::uint8_t(byte);
                break;
            }
            case IYModemPlatform::Result::Timeout:
            {
                /*
                 * Note that we may greatly overstay the timeout here, but this is by design,
                 * since the spec requires that each character must be received with 1 second timeout.
                 */
                if (timeout <= CharReceiveTimeout)
                {
                    return std::int16_t(i);
                }
                else
                {
                    timeout -= CharReceiveTimeout;
                }
                break;
            }
            case IYModemPlatform::Result::Error:
            {
                return -ErrPortError;
            }
            }
        }

        return std::int16_t(size);
    }

    void abort()
    {
        constexpr std::uint8_t Times = 5;           // Multiple CAN are required!
        for (std::uint8_t i = 0; i < Times; i++)
        {
            if (send(ControlCharacters::CAN) != 1)
            {
                break;
            }
        }
    }

    enum class BlockReceptionResult : std::uint8_t
    {
        Success,
        Timeout,
        EndOfTransmission,
        TransmissionCancelled,
        ProtocolError,
        SystemError
    };

    /**
     * Reads a block from the channel. This function does not transmit anything.
     * @return First component: @ref BlockReceptionResult
     *         Second component: system error code, if applicable
     */
    std::pair<BlockReceptionResult, std::int16_t> receiveBlock(std::uint16_t& out_size,
                                                               std::uint8_t& out_sequence)
    {
        // Header byte
        std::uint8_t header_byte = 0;
        std::int16_t res = receive(&header_byte, 1, NextBlockTimeout);
        if (res < 0)
        {
            return { BlockReceptionResult::SystemError, res };
        }
        if (res != 1)
        {
            return { BlockReceptionResult::Timeout, 0 };
        }

        switch (header_byte)
        {
        case ControlCharacters::STX:
        {
            out_size = BlockSize1K;
            break;
        }
        case ControlCharacters::SOH:
        {
            out_size = BlockSizeXModem;
            break;
        }
        case ControlCharacters::EOT:
        {
            KOCHERGA_TRACE("YMODEM RX EOT\n");
            return { BlockReceptionResult::EndOfTransmission, 0 };
        }
        case ControlCharacters::CAN:
        {
            KOCHERGA_TRACE("YMODEM RX CAN\n");
            return { BlockReceptionResult::TransmissionCancelled, 0 };
        }
        default:
        {
            KOCHERGA_TRACE("YMODEM unexpected header 0x%x\n", header_byte);
            return { BlockReceptionResult::ProtocolError, 0 };
        }
        }

        // Sequence ID
        std::uint8_t sequence_id_bytes[2] = {};
        res = receive(sequence_id_bytes, 2, BlockPayloadTimeout);
        if (res < 0)
        {
            return { BlockReceptionResult::SystemError, res };
        }
        if (res != 2)
        {
            return { BlockReceptionResult::Timeout, 0 };
        }
        if (sequence_id_bytes[0] != static_cast<std::uint8_t>(~sequence_id_bytes[1]))       // Invalid sequence ID
        {
            KOCHERGA_TRACE("YMODEM non-inverted sequence ID: 0x%x 0x%x\n", sequence_id_bytes[0], sequence_id_bytes[1]);
            return { BlockReceptionResult::ProtocolError, 0 };
        }
        out_sequence = sequence_id_bytes[0];

        // Payload
        constexpr auto ChecksumSize = 1;
        const auto block_size_with_checksum = std::uint16_t(out_size + ChecksumSize);
        res = receive(buffer_, block_size_with_checksum, BlockPayloadTimeout);
        if (res < 0)
        {
            return { BlockReceptionResult::SystemError, res };
        }
        if (std::uint16_t(res) != block_size_with_checksum)
        {
            return { BlockReceptionResult::Timeout, 0 };
        }

        // Checksum validation
        if (computeChecksum(buffer_, out_size) != buffer_[out_size])
        {
            KOCHERGA_TRACE("YMODEM checksum error, not %d\n", buffer_[out_size]);
            return { BlockReceptionResult::ProtocolError, 0 };
        }

        return { BlockReceptionResult::Success, 0 };
    }

    static bool tryParseZeroBlock(const std::uint8_t* const data,
                                  const std::uint16_t size,
                                  bool& out_is_null_block,
                                  std::uint32_t& out_file_size)
    {
        assert(size == BlockSizeXModem || size == BlockSize1K);

        // Initializing defaults
        out_is_null_block = true;   // Paranoia
        out_file_size = 0;          // I.e. unknown

        std::uint16_t offset = 0;

        // Skipping the file name
        while ((offset < size) && (data[offset] != 0))
        {
            offset++;
        }
        if (offset >= size)
        {
            return false;                                   // No null termination, invalid block
        }
        KOCHERGA_TRACE("YMODEM file name: '%s'\n", reinterpret_cast<const char*>(data));

        // Setting the null block indication, aborting if it is null block because it won't contain file size
        out_is_null_block = offset == 0;                    // No filename means null block (end of session)
        if (out_is_null_block)
        {
            return true;
        }

        // Not a null block - parsing the file size
        offset++;
        KOCHERGA_TRACE("YMODEM all fields: '%s'\n", reinterpret_cast<const char*>(&data[offset]));
        while ((offset < size) && (data[offset] != 0) && (data[offset] != ' '))
        {
            if (data[offset] < '0' ||
                data[offset] > '9')
            {
                out_file_size = 0;                          // Bad character before termination
                break;
            }
            out_file_size *= 10;
            out_file_size += std::uint32_t(data[offset] - std::uint8_t('0'));
            offset++;
        }
        KOCHERGA_TRACE("YMODEM file size int: %u\n", unsigned(out_file_size));

        return true;
    }

    static std::int16_t processDownloadedBlock(kocherga::IDownloadSink& sink, void* data, std::uint16_t size)
    {
        KOCHERGA_TRACE("YMODEM received block of %d bytes\n", size);
        return sink.handleNextDataChunk(data, size);
    }

public:
    /**
     * @param serial_port                   the serial port channel that will be used for downloading
     */
    explicit YModemProtocol(IYModemPlatform& serial_port) :
        platform_(serial_port)
    { }

    std::int16_t downloadImage(kocherga::IDownloadSink& sink) override
    {
        // This thing will make sure there's no residual garbage in the RX buffer afterwards
        struct Flusher
        {
            IYModemPlatform& port;
            ~Flusher()
            {
                std::uint8_t dummy = 0;
                while (port.receive(dummy, std::chrono::microseconds(1'000)) == IYModemPlatform::Result::Success)
                {
                    KOCHERGA_TRACE("YMODEM FLUSH RX 0x%x\n", unsigned(dummy));
                }
            }
        } flusher_{platform_};

        // State variables
        std::uint32_t remaining_file_size = 0;
        bool file_size_known{};
        std::uint8_t expected_sequence_id = 123;             // Arbitrary invalid value

        enum class Mode
        {
            XModem,
            YModem
        } mode{};

        /*
         * Initiating the transfer, receiving the first block.
         * The sequence ID will be 0 in case of YMODEM, and 1 in case of XMODEM.
         */
        const auto started_at = platform_.getMonotonicUptime();
        for (;;)
        {
            KOCHERGA_TRACE("Trying to initiate X/YMODEM transfer...\n");

            // Abort if we couldn't get it going in InitialTimeout
            if ((platform_.getMonotonicUptime() - started_at) > InitialTimeout)
            {
                abort();
                return -ErrRetriesExhausted;
            }

            // Requesting transmission in checksum mode
            if (const auto res = send(ControlCharacters::NAK); res < 0)
            {
                abort();
                return res;
            }

            // Receiving the block
            std::uint16_t size = 0;
            const auto block_rx_res = receiveBlock(size, expected_sequence_id);
            if (block_rx_res.first == BlockReceptionResult::Success)
            {
                ;
            }
            else if (block_rx_res.first == BlockReceptionResult::Timeout ||
                     block_rx_res.first == BlockReceptionResult::ProtocolError ||
                     block_rx_res.first == BlockReceptionResult::EndOfTransmission)
            {
                continue;   // EOT cannot be sent in response to the first block, it's an error; trying again...
            }
            else if (block_rx_res.first == BlockReceptionResult::TransmissionCancelled)
            {
                abort();
                return -ErrTransferCancelledByRemote;
            }
            else
            {
                assert(block_rx_res.first == BlockReceptionResult::SystemError);
                abort();
                return block_rx_res.second;
            }

            // Processing the block
            if (expected_sequence_id == 0)
            {
                mode = Mode::YModem;

                bool is_null_block = true;
                const bool zero_block_valid = tryParseZeroBlock(buffer_, size, is_null_block, remaining_file_size);

                KOCHERGA_TRACE("YMODEM zero block: valid=%d null=%d size=%u\n",
                               zero_block_valid, is_null_block, unsigned(remaining_file_size));

                if (!zero_block_valid)
                {
                    // Invalid zero block, that's a fatal error, it's checksum protected after all
                    // Retrying here would make no sense, it's not a line hit, it's badly formed packet!
                    abort();
                    return -ErrProtocolError;
                }
                if (is_null_block)
                {
                    // Null block means that the sender is refusing to transmit the file
                    // No point retrying too, the sender isn't going to change their mind
                    abort();
                    return -ErrRemoteRefusedToProvideFile;
                }
                file_size_known = remaining_file_size > 0;

                // The zero block requires a dedicated ACK, sending it now
                if (const auto res = send(ControlCharacters::ACK); res < 0)
                {
                    abort();
                    return res;
                }
            }
            else if (expected_sequence_id == 1)
            {
                mode = Mode::XModem;
                KOCHERGA_TRACE("YMODEM zero block skipped (XMODEM mode)\n");

                if (const auto res = processDownloadedBlock(sink, buffer_, size); res < 0)
                {
                    abort();
                    return res;
                }
                file_size_known = false;
            }
            else                            // Invalid sequence number
            {
                abort();
                return -ErrProtocolError;
            }

            // Done!
            expected_sequence_id = std::uint8_t(expected_sequence_id + 1);
            break;
        }

        assert(file_size_known ? true : (remaining_file_size == 0));

        /*
         * Receiving the file
         */
        bool ack = mode == Mode::XModem;    // YMODEM requires another NAK after the zero block
        auto remaining_retries = MaxRetries;
        for (;;)
        {
            // Limiting retries
            if (remaining_retries <= 0)
            {
                abort();
                return -ErrRetriesExhausted;
            }
            remaining_retries--;

            // Confirming or re-requesting
            if (const auto res = send(ack ? ControlCharacters::ACK : ControlCharacters::NAK); res < 0)
            {
                abort();
                return res;
            }
            ack = false;

            // Receiving the block
            std::uint16_t size = 0;
            std::uint8_t sequence_id = 0;
            const auto block_rx_res = receiveBlock(size, sequence_id);
            if (block_rx_res.first == BlockReceptionResult::Success)
            {
                ;
            }
            else if (block_rx_res.first == BlockReceptionResult::Timeout ||
                     block_rx_res.first == BlockReceptionResult::ProtocolError)
            {
                continue;
            }
            else if (block_rx_res.first == BlockReceptionResult::EndOfTransmission)
            {
                if ((file_size_known) && (remaining_file_size != 0))
                {
                    // The sender said that we're done, liar!
                    KOCHERGA_TRACE("YMODEM ended %u bytes early\n", unsigned(remaining_file_size));
                    abort();
                    return -ErrProtocolError;
                }
                // Done, exiting and sending the final ACK
                KOCHERGA_TRACE("YMODEM end OK\n");
                break;
            }
            else if (block_rx_res.first == BlockReceptionResult::TransmissionCancelled)
            {
                KOCHERGA_TRACE("YMODEM cancelled\n");
                abort();
                return -ErrTransferCancelledByRemote;
            }
            else
            {
                assert(block_rx_res.first == BlockReceptionResult::SystemError);
                abort();
                return block_rx_res.second;
            }
            remaining_retries = MaxRetries;                         // Reset retries on successful reception

            // Processing the block
            if ((sequence_id + 1) == expected_sequence_id)          // Duplicate block, acknowledge silently
            {
                KOCHERGA_TRACE("YMODEM duplicate block skipped\n");
                ack = true;
                continue;
            }
            if (sequence_id != expected_sequence_id)                // Totally wrong sequence, abort
            {
                KOCHERGA_TRACE("YMODEM wrong sequence ID\n");
                abort();
                return -ErrProtocolError;
            }
            expected_sequence_id = std::uint8_t(expected_sequence_id + 1);

            // Making sure we're not past the end of file
            if (file_size_known)
            {
                if (remaining_file_size == 0)
                {
                    KOCHERGA_TRACE("YMODEM transmission past the end of file\n");
                    abort();
                    return -ErrProtocolError;
                }
                if (size > remaining_file_size)
                {
                    size = std::uint16_t(remaining_file_size);
                }
                remaining_file_size -= size;
            }

            // Sending the block over
            if (const auto res = processDownloadedBlock(sink, buffer_, size); res < 0)
            {
                abort();
                return res;
            }

            // Done, continue to the next block
            ack = true;
        }

        /*
         * Final response and then leaving.
         * Errors can be ignored - we got what we wanted anyway.
         */
        KOCHERGA_TRACE("YMODEM finalizing\n");

        (void)send(ControlCharacters::ACK);         // If it fails, who cares.

        if (mode == Mode::YModem)
        {
            // Letting the sender know we don't want any other files. Is this compliant?
            abort();
        }

        return ErrOK;
    }
};

}
