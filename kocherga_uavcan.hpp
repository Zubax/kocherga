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

// Third-party dependencies:
#include <canard.h>                     // Lightweight UAVCAN protocol stack implementation
#include <senoval/string.hpp>           // Utility library for embedded systems
#include <senoval/vector.hpp>           // Utility library for embedded systems

#include <cstddef>

/**
 * This macro can be defined by the application to provide log output from the UAVCAN node.
 * By default resolves to KOCHERGA_TRACE().
 * The expected signature is that of std::printf().
 */
#ifndef KOCHERGA_UAVCAN_LOG
# define KOCHERGA_UAVCAN_LOG(...)        KOCHERGA_TRACE(__VA_ARGS__)
#endif


namespace kocherga_uavcan
{
/**
 * Error codes specific to this protocol.
 */
static constexpr std::int16_t ErrTimeout        = 3001;
static constexpr std::int16_t ErrInterrupted    = 3002;
static constexpr std::int16_t ErrFileReadFailed = 3003;

/**
 * Abstractions needed to run the UAVCAN node.
 */
class IUAVCANPlatform
{
public:
    /**
     * CAN controller operating mode.
     * The driver must support silent mode, because it is required for automatic bit rate detection.
     * The automatic transmission abort on error feature is required for dynamic node ID allocation
     * (read the UAVCAN specification for more info).
     */
    enum class CANMode
    {
        Normal,
        Silent,
        AutomaticTxAbortOnError
    };

    /**
     * CAN acceptance filter configuration.
     * Acceptance filters may be not supported in the underlying driver, this feature is optional.
     * Bit flags used here are the same as in libcanard.
     * The default constructor makes a filter that accepts all frames.
     */
    struct CANAcceptanceFilterConfig
    {
        std::uint32_t id = 0;
        std::uint32_t mask = 0;
    };

    virtual ~IUAVCANPlatform() = default;

    /**
     * This method is invoked by the node's thread periodically as long as it functions properly.
     * The application can use it to reset a watchdog, but it is not mandatory.
     * The minimal watchdog timeout is 3 seconds! Lower values may trigger spurious resets.
     */
    virtual void resetWatchdog() = 0;

    /**
     * This method is invoked by the node's thread when it has nothing to do.
     * It can be used by the application to perform other tasks, or to sleep the current thread
     * if there is an OS available.
     */
    virtual void sleep(std::chrono::microseconds duration) const = 0;

    /**
     * Returns a pseudo-random unsigned integer within the specified range [lower_bound, upper_bound).
     * Possible implementation:
     *      const auto rnd = std::uint64_t(std::rand()) * std::uint64_t(std::rand());
     *      return lower_bound + rnd % (upper_bound - lower_bound);
     */
    virtual std::uint64_t getRandomUnsignedInteger(std::uint64_t lower_bound,
                                                   std::uint64_t upper_bound) const = 0;

    /**
     * Initializes the CAN hardware in the specified mode.
     * Observe that this implementation needs only one acceptance filter.
     * @retval 0                Success
     * @retval negative         Error
     */
    virtual std::int16_t configure(std::uint32_t bitrate,
                                   CANMode mode,
                                   const CANAcceptanceFilterConfig& acceptance_filter) = 0;

    /**
     * Transmits one CAN frame.
     *
     * @retval      1               Transmitted successfully
     * @retval      0               Timed out
     * @retval      negative        Error
     */
    virtual std::int16_t send(const ::CanardCANFrame& frame, std::chrono::microseconds timeout) = 0;

    /**
     * Reads one CAN frame from the RX queue.
     * Return integer values:
     * @retval      1               Read successfully; the second value contains a valid CAN frame object.
     * @retval      0               Timed out
     * @retval      negative        Error
     */
    virtual std::pair<std::int16_t, ::CanardCANFrame> receive(std::chrono::microseconds timeout) = 0;

    /**
     * This method is invoked by the node periodically to check if it should terminate.
     */
    virtual bool shouldExit() const = 0;

    /**
     * Invoked by the node when it is requested to reboot by a remote node.
     * Returns true on success, false if reboot cannot be performed.
     */
    virtual bool tryScheduleReboot() = 0;
};


using NodeName = senoval::String<80>;

struct HardwareInfo
{
    std::uint8_t major = 0;                                     ///< Required field
    std::uint8_t minor = 0;                                     ///< Required field

    typedef std::array<std::uint8_t, 16> UniqueID;
    UniqueID unique_id{};                                       ///< Required field

    typedef senoval::Vector<std::uint8_t, 255> CertificateOfAuthenticity;
    CertificateOfAuthenticity certificate_of_authenticity;      ///< Optional, empty if not defined
};

/**
 * Implementation details, please do not touch this.
 */
namespace impl_
{
/**
 * This is the default defined by the UAVCAN specification.
 */
static constexpr std::chrono::microseconds DefaultServiceRequestTimeout{1'000'000};  // NOLINT

/**
 * A sensible default value that should suit most applications.
 */
static constexpr std::chrono::microseconds DefaultProgressReportInterval{10'000'000};  // NOLINT


namespace dsdl
{

static inline constexpr std::uint16_t bitlen2bytelen(std::uint32_t x) noexcept
{
    return std::uint16_t((x + 7) / 8);
}

template <std::uint32_t DataTypeID_,
          std::uint64_t DataTypeSignature_,     // Not to be confused with DSDL signature
          std::uint32_t MaxEncodedBitLength_>
struct MessageTypeInfo
{
    static constexpr std::uint32_t DataTypeID           = DataTypeID_;
    static constexpr std::uint64_t DataTypeSignature    = DataTypeSignature_;

    static constexpr std::uint16_t MaxSizeBytes         = bitlen2bytelen(MaxEncodedBitLength_);
};

template <std::uint32_t DataTypeID_,
          std::uint64_t DataTypeSignature_,     // Not to be confused with DSDL signature
          std::uint32_t MaxEncodedBitLengthRequest_,
          std::uint32_t MaxEncodedBitLengthResponse_>
struct ServiceTypeInfo
{
    static constexpr std::uint32_t DataTypeID                   = DataTypeID_;
    static constexpr std::uint64_t DataTypeSignature            = DataTypeSignature_;

    static constexpr std::uint16_t MaxSizeBytesRequest          = bitlen2bytelen(MaxEncodedBitLengthRequest_);

    static constexpr std::uint16_t MaxSizeBytesResponse         = bitlen2bytelen(MaxEncodedBitLengthResponse_);
};

// The values have been obtained with the help of the script show_data_type_info.py from libcanard.
using NodeStatus                = MessageTypeInfo<  341U, 0x0f0868d0c1a7c6f1ULL,    56U>;
using NodeIDAllocation          = MessageTypeInfo<    1U, 0x0b2a812620a11d40ULL,   141U>;
using LogMessage                = MessageTypeInfo<16383U, 0xd654a48e0c049d75ULL,   983U>;

using GetNodeInfo               = ServiceTypeInfo<    1U, 0xee468a8121c46a9eULL,     0U,  3015U>;
using BeginFirmwareUpdate       = ServiceTypeInfo<   40U, 0xb7d725df72724126ULL,  1616U,  1031U>;
using FileRead                  = ServiceTypeInfo<   48U, 0x8dcdca939f33f678ULL,  1648U,  2073U>;
using RestartNode               = ServiceTypeInfo<    5U, 0x569e05394a3017f0ULL,    40U,     1U>;


enum class NodeHealth : std::uint8_t
{
    Ok = 0,
    Warning = 1,
    Error = 2
};

enum class NodeMode : std::uint8_t
{
    Initialization = 1,
    SoftwareUpdate = 3,
};

}

/**
 * See uavcan.protocol.debug.LogMessage
 */
enum class LogLevel : std::uint8_t
{
    Info    = 1,
    Warning = 2,
    Error   = 3,
};

}       // namespace impl_

/**
 * A UAVCAN node that is useful solely for the purpose of firmware update.
 *
 * This class looks like a bowl of spaghetti because is has been carefully optimized for ROM footprint.
 * Avoid reading this code unless you've familiarized yourself with the UAVCAN specification.
 *
 * The API is thread-safe.
 */
template <std::size_t MemoryPoolSize = 8192>
class BootloaderNode final : private ::kocherga::IProtocol
{
    ::kocherga::BootloaderController& bootloader_;
    IUAVCANPlatform& platform_;

    const NodeName node_name_;
    const HardwareInfo hw_info_;

    std::chrono::microseconds next_1hz_task_invocation_at_{};
    bool init_done_ = false;

    alignas(std::max_align_t) std::array<std::uint8_t, MemoryPoolSize> memory_pool_{};
    ::CanardInstance canard_{};

    std::uint32_t can_bus_bit_rate_ = 0;
    std::uint8_t confirmed_local_node_id_ = 0;          ///< This field is needed in order to avoid mutexes

    std::uint8_t remote_server_node_id_ = 0;
    senoval::String<200> firmware_file_path_;

    std::chrono::microseconds send_next_node_id_allocation_request_at_{};
    std::uint8_t node_id_allocation_unique_id_offset_ = 0;

    std::uint16_t vendor_specific_status_ = 0;

    std::uint8_t node_status_transfer_id_ = 0;
    std::uint8_t node_id_allocation_transfer_id_ = 0;
    std::uint8_t log_message_transfer_id_ = 0;
    std::uint8_t file_read_transfer_id_ = 0;

    std::array<std::uint8_t, 256> read_buffer_{};
    std::int16_t read_result_ = 0;


    std::uint64_t getMonotonicUptimeInMicroseconds() const
    {
        const auto ut = bootloader_.getMonotonicUptime();
        return std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(ut).count());
    }

    void delayAfterDriverError()
    {
        platform_.resetWatchdog();
        platform_.sleep(std::chrono::microseconds(1'000'000));
        platform_.resetWatchdog();
    }

    std::chrono::microseconds getRandomDuration(std::chrono::microseconds lower_bound,
                                                std::chrono::microseconds upper_bound) const
    {
        assert(lower_bound <= upper_bound);
        return std::chrono::microseconds(platform_.getRandomUnsignedInteger(std::uint64_t(lower_bound.count()),
                                                                            std::uint64_t(upper_bound.count())));
    }

    void makeNodeStatusMessage(std::uint8_t* buffer) const
    {
        std::memset(buffer, 0, impl_::dsdl::NodeStatus::MaxSizeBytes);

        const auto uptime_sec =
            std::uint32_t(std::chrono::duration_cast<std::chrono::seconds>(bootloader_.getMonotonicUptime()).count());

        /*
         * Bootloader State        Node Mode       Node Health
         * ----------------------------------------------------
         * NoAppToBoot             SoftwareUpdate  Error
         * BootDelay               Initialization  Ok
         * BootCancelled           SoftwareUpdate  Warning
         * AppUpgradeInProgress    SoftwareUpdate  Ok
         * ReadyToBoot             Initialization  Ok
         */
        std::uint8_t node_health{};
        std::uint8_t node_mode{};
        switch (bootloader_.getState())
        {
        case ::kocherga::State::NoAppToBoot:
        {
            node_health = std::uint8_t(impl_::dsdl::NodeHealth::Error);
            node_mode   = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
            break;
        }
        case ::kocherga::State::AppUpgradeInProgress:
        {
            node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
            node_mode   = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
            break;
        }
        case::kocherga:: State::BootCancelled:
        {
            node_health = std::uint8_t(impl_::dsdl::NodeHealth::Warning);
            node_mode   = std::uint8_t(impl_::dsdl::NodeMode::SoftwareUpdate);
            break;
        }
        case ::kocherga::State::BootDelay:
        case ::kocherga::State::ReadyToBoot:
        {
            node_health = std::uint8_t(impl_::dsdl::NodeHealth::Ok);
            node_mode   = std::uint8_t(impl_::dsdl::NodeMode::Initialization);
            break;
        }
        }

        ::canardEncodeScalar(buffer,  0, 32, &uptime_sec);
        ::canardEncodeScalar(buffer, 32,  2, &node_health);
        ::canardEncodeScalar(buffer, 34,  3, &node_mode);
        ::canardEncodeScalar(buffer, 40, 16, &vendor_specific_status_);
    }

    void sendNodeStatus()
    {
        using namespace impl_;
        std::uint8_t buffer[dsdl::NodeStatus::MaxSizeBytes]{};
        makeNodeStatusMessage(buffer);
        const auto res = ::canardBroadcast(&canard_,
                                           dsdl::NodeStatus::DataTypeSignature,
                                           dsdl::NodeStatus::DataTypeID,
                                           &node_status_transfer_id_,
                                           CANARD_TRANSFER_PRIORITY_LOW,
                                           buffer,
                                           dsdl::NodeStatus::MaxSizeBytes);
        if (res <= 0)
        {
            KOCHERGA_UAVCAN_LOG("NodeStatus bc err %d\n", res);
        }
    }

    void sendLog(const impl_::LogLevel level, const senoval::String<90>& txt)
    {
        static const senoval::String<31> SourceName("Bootloader");
        std::uint8_t buffer[1 + 31 + 90]{};
        buffer[0] = std::uint8_t(std::uint8_t(std::uint16_t(level) << 5U) | SourceName.length());
        std::copy(SourceName.begin(), SourceName.end(), &buffer[1]);
        std::copy(txt.begin(), txt.end(), &buffer[1 + SourceName.length()]);

        using impl_::dsdl::LogMessage;
        const auto res = ::canardBroadcast(&canard_,
                                           LogMessage::DataTypeSignature,
                                           LogMessage::DataTypeID,
                                           &log_message_transfer_id_,
                                           CANARD_TRANSFER_PRIORITY_LOWEST,
                                           buffer,
                                           std::uint16_t(1U + SourceName.length() + txt.length()));
        if (res < 0)
        {
            KOCHERGA_UAVCAN_LOG("Log err %d\n", res);
        }
    }

    auto initCAN(const std::uint32_t bitrate,
                 const IUAVCANPlatform::CANMode mode,
                 const IUAVCANPlatform::CANAcceptanceFilterConfig& acceptance_filter =
                     IUAVCANPlatform::CANAcceptanceFilterConfig())
    {
        const auto res = platform_.configure(bitrate, mode, acceptance_filter);
        if (res < 0)
        {
            KOCHERGA_UAVCAN_LOG("CAN init err @%u bps: %d\n", unsigned(bitrate), res);
        }
        return res;
    }

    auto receive(std::chrono::microseconds timeout)
    {
        const auto res = platform_.receive(timeout);
        if (res.first < 0)
        {
            KOCHERGA_UAVCAN_LOG("RX err %d\n", res.first);
        }
        return res;
    }

    auto send(const ::CanardCANFrame& frame, std::chrono::microseconds timeout)
    {
        const auto res = platform_.send(frame, timeout);
        if (res < 0)
        {
            KOCHERGA_UAVCAN_LOG("TX err %d\n", res);
        }
        return res;
    }

    void handle1HzTasks()
    {
        platform_.resetWatchdog();
        ::canardCleanupStaleTransfers(&canard_, getMonotonicUptimeInMicroseconds());

        // NodeStatus broadcasting
        if (init_done_ && (::canardGetLocalNodeID(&canard_) > 0))
        {
            sendNodeStatus();
        }

        platform_.resetWatchdog();
    }

    void poll()
    {
        constexpr std::uint8_t MaxFramesPerSpin = 10;

        // Receive
        for (std::uint8_t i = 0; i < MaxFramesPerSpin; i++)
        {
            platform_.resetWatchdog();

            const auto res = receive(std::chrono::microseconds(1'000));        // Blocking call
            if (res.first < 1)
            {
                break;                          // Error or no frames
            }

            ::canardHandleRxFrame(&canard_, &res.second, getMonotonicUptimeInMicroseconds());
        }

        // Transmit
        for (std::uint8_t i = 0; i < MaxFramesPerSpin; i++)
        {
            platform_.resetWatchdog();

            const ::CanardCANFrame* txf = ::canardPeekTxQueue(&canard_);
            if (txf == nullptr)
            {
                break;                          // Nothing to transmit
            }

            const auto res = send(*txf, std::chrono::microseconds{});      // Non-blocking call
            if (res == 0)
            {
                break;                          // Queue is full
            }

            ::canardPopTxQueue(&canard_);       // Transmitted successfully or error, either way remove the frame
        }

        // 1Hz process
        if (bootloader_.getMonotonicUptime() >= next_1hz_task_invocation_at_)
        {
            next_1hz_task_invocation_at_ += std::chrono::seconds(1);
            handle1HzTasks();
        }
    }

    void performCANBitRateDetection()
    {
        /// These are defined by the specification; 100 Kbps is added due to its popularity.
        static constexpr std::array<std::uint32_t, 5> StandardBitRates
        {{
            1000000,        ///< Standard, recommended by UAVCAN
             500000,        ///< Standard
             250000,        ///< Standard
             125000,        ///< Standard
             100000         ///< Popular bit rate that is not defined by the specification
        }};

        std::uint8_t current_bit_rate_index = 0;

        // Loop forever until the bit rate is detected
        while ((!platform_.shouldExit()) && (can_bus_bit_rate_ == 0))
        {
            platform_.resetWatchdog();

            const std::uint32_t br = StandardBitRates[current_bit_rate_index];
            current_bit_rate_index = std::uint8_t((current_bit_rate_index + 1U) % StandardBitRates.size());

            if (initCAN(br, IUAVCANPlatform::CANMode::Silent) >= 0)
            {
                const auto res = receive(std::chrono::microseconds(1'100'000)).first;
                if (res > 0)
                {
                    can_bus_bit_rate_ = br;
                }
                if (res < 0)
                {
                    delayAfterDriverError();
                }
            }
            else
            {
                delayAfterDriverError();
            }
        }

        platform_.resetWatchdog();
    }

    void performDynamicNodeIDAllocation()
    {
        // CAN bus initialization
        while (true)
        {
            // Accept only messages with DTID = 1 (Allocation)
            // Observe that we need both responses from allocators and requests from other nodes!
            IUAVCANPlatform::CANAcceptanceFilterConfig filt;
            filt.id   = 0b00000000000000000000100000000UL | CANARD_CAN_FRAME_EFF;
            filt.mask = 0b00000000000000000001110000000UL | CANARD_CAN_FRAME_EFF | CANARD_CAN_FRAME_RTR |
                        CANARD_CAN_FRAME_ERR;

            if (initCAN(can_bus_bit_rate_, IUAVCANPlatform::CANMode::AutomaticTxAbortOnError, filt) >= 0)
            {
                break;
            }

            delayAfterDriverError();
        }

        using namespace impl_;

        while ((!platform_.shouldExit()) && (::canardGetLocalNodeID(&canard_) == 0))
        {
            platform_.resetWatchdog();

            send_next_node_id_allocation_request_at_ =
                bootloader_.getMonotonicUptime() + getRandomDuration(std::chrono::microseconds(600'000),
                                                                     std::chrono::microseconds(1'000'000));

            while ((bootloader_.getMonotonicUptime() < send_next_node_id_allocation_request_at_) &&
                   (::canardGetLocalNodeID(&canard_) == 0))
            {
                poll();
            }

            if (::canardGetLocalNodeID(&canard_) != 0)
            {
                break;
            }

            // Structure of the request is documented in the DSDL definition
            // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
            std::uint8_t allocation_request[7]{};

            if (node_id_allocation_unique_id_offset_ == 0)
            {
                allocation_request[0] |= 1;     // First part of unique ID
            }

            static constexpr std::uint8_t MaxLenOfUniqueIDInRequest = 6;
            std::uint8_t uid_size = std::uint8_t(hw_info_.unique_id.size() - node_id_allocation_unique_id_offset_);
            if (uid_size > MaxLenOfUniqueIDInRequest)
            {
                uid_size = MaxLenOfUniqueIDInRequest;
            }

            // Paranoia time
            assert(node_id_allocation_unique_id_offset_ < hw_info_.unique_id.size());
            assert(uid_size <= MaxLenOfUniqueIDInRequest);
            assert(uid_size > 0);
            assert(std::uint16_t(uid_size + node_id_allocation_unique_id_offset_) <= hw_info_.unique_id.size());

            std::memmove(&allocation_request[1], &hw_info_.unique_id[node_id_allocation_unique_id_offset_], uid_size);

            // Broadcasting the request
            const auto bcast_res = ::canardBroadcast(&canard_,
                                                     dsdl::NodeIDAllocation::DataTypeSignature,
                                                     dsdl::NodeIDAllocation::DataTypeID,
                                                     &node_id_allocation_transfer_id_,
                                                     CANARD_TRANSFER_PRIORITY_LOW,
                                                     &allocation_request[0],
                                                     std::uint16_t(uid_size + 1U));
            if (bcast_res < 0)
            {
                KOCHERGA_UAVCAN_LOG("NID alloc bc err %d\n", bcast_res);
            }

            // Preparing for timeout; if response is received, this value will be updated from the callback.
            node_id_allocation_unique_id_offset_ = 0;
        }

        platform_.resetWatchdog();
    }

    void runNodeThread()
    {
        /*
         * CAN bit rate
         */
        platform_.resetWatchdog();

        if (can_bus_bit_rate_ == 0)
        {
            performCANBitRateDetection();
        }

        if (platform_.shouldExit())
        {
            return;
        }

        /*
         * Node ID
         */
        platform_.resetWatchdog();

        if (::canardGetLocalNodeID(&canard_) == 0)
        {
            performDynamicNodeIDAllocation();
        }

        if (platform_.shouldExit())
        {
            return;
        }

        confirmed_local_node_id_ = ::canardGetLocalNodeID(&canard_);

        // This is the only info message we output during initialization.
        // Fewer messages reduce the chances of breaking UART CLI data flow.
        KOCHERGA_UAVCAN_LOG("CAN %u bps, NID %u\n", unsigned(can_bus_bit_rate_), confirmed_local_node_id_);

        using namespace impl_;

        /*
         * Init CAN in proper mode now
         */
        platform_.resetWatchdog();

        while (true)
        {
            // Accept only correctly addressed service requests and responses
            // We don't need message transfers anymore
            IUAVCANPlatform::CANAcceptanceFilterConfig filt;
            filt.id   = 0b00000000000000000000010000000UL |
                        std::uint32_t(confirmed_local_node_id_ << 8U) | CANARD_CAN_FRAME_EFF;
            filt.mask = 0b00000000000000111111110000000UL |
                        CANARD_CAN_FRAME_EFF | CANARD_CAN_FRAME_RTR | CANARD_CAN_FRAME_ERR;

            if (initCAN(can_bus_bit_rate_, IUAVCANPlatform::CANMode::Normal, filt) >= 0)
            {
                break;
            }

            delayAfterDriverError();
        }

        init_done_ = true;

        /*
         * Update loop; run forever because there's nothing else to do
         */
        while (!platform_.shouldExit())
        {
            assert((confirmed_local_node_id_ > 0) && (::canardGetLocalNodeID(&canard_) > 0));

            platform_.resetWatchdog();

            /*
             * Waiting for the firmware update request
             */
            if (remote_server_node_id_ == 0)
            {
                while ((!platform_.shouldExit()) && (remote_server_node_id_ == 0))
                {
                    platform_.resetWatchdog();
                    poll();
                }
            }

            if (platform_.shouldExit())
            {
                break;
            }

            KOCHERGA_UAVCAN_LOG("FW server NID %u path %s\n",
                                unsigned(remote_server_node_id_), firmware_file_path_.c_str());

            /*
             * Rewriting the old firmware with the new file
             */
            platform_.resetWatchdog();
            const auto result = bootloader_.upgradeApp(*this);
            platform_.resetWatchdog();

            sendNodeStatus();   // Announcing the new status of the bootloader ASAP

            if (result >= 0)
            {
                vendor_specific_status_ = 0;
                if (bootloader_.getState() == kocherga::State::NoAppToBoot)
                {
                    sendLog(LogLevel::Error, "Downloaded image is invalid");
                }
                else
                {
                    sendLog(LogLevel::Info, "OK");
                }
            }
            else
            {
                vendor_specific_status_ = std::uint16_t(std::abs(result));
                sendLog(LogLevel::Error,
                        senoval::String<90>("Upgrade error ") + senoval::convertIntToString(result));
            }

            /*
             * Reset everything to zero and loop again, because there's nothing else to do.
             * The outer logic will request reboot if necessary.
             */
            remote_server_node_id_ = 0;
            firmware_file_path_.clear();
        }

        KOCHERGA_UAVCAN_LOG("Exit\n");
        platform_.resetWatchdog();
    }

    std::int16_t downloadImage(kocherga::IDownloadSink& sink) override
    {
        using namespace impl_;

        std::uint64_t offset = 0;
        auto next_progress_report_deadline = bootloader_.getMonotonicUptime();

        sendNodeStatus();       // Announcing the new state of the bootloader ASAP

        while (true)
        {
            platform_.resetWatchdog();

            if (platform_.shouldExit())
            {
                return -ErrInterrupted;
            }

            /*
             * Send request
             */
            {
                std::uint8_t buffer[dsdl::FileRead::MaxSizeBytesRequest]{};
                ::canardEncodeScalar(buffer, 0, 40, &offset);
                std::copy(firmware_file_path_.begin(), firmware_file_path_.end(), &buffer[5]);

                const auto res = ::canardRequestOrRespond(&canard_,
                                                          remote_server_node_id_,
                                                          dsdl::FileRead::DataTypeSignature,
                                                          dsdl::FileRead::DataTypeID,
                                                          &file_read_transfer_id_,
                                                          CANARD_TRANSFER_PRIORITY_LOW,
                                                          ::CanardRequest,
                                                          buffer,
                                                          std::uint16_t(firmware_file_path_.size() + 5U));
                if (res < 0)
                {
                    KOCHERGA_UAVCAN_LOG("File req err %d\n", res);
                    return std::int16_t(res);
                }
            }

            /*
             * Await response.
             * Note that the watchdog is not reset in the loop, since its timeout is large enough to wait for response.
             */
            const std::chrono::microseconds response_deadline =
                bootloader_.getMonotonicUptime() + DefaultServiceRequestTimeout;

            constexpr auto InvalidReadResult = std::numeric_limits<std::int16_t>::max();
            read_result_ = InvalidReadResult;

            while (read_result_ == InvalidReadResult)
            {
                poll();

                if (bootloader_.getMonotonicUptime() > response_deadline)
                {
                    return -ErrTimeout;
                }
            }

            platform_.resetWatchdog();

            if (read_result_ < 0)
            {
                return read_result_;
            }

            /*
             * Process the response.
             * Observe that we don't constrain the maximum image size - either the bootloader
             * or the storage backend will return error if we exceed it.
             */
            if (read_result_ > 0)
            {
                offset = offset + std::uint64_t(read_result_);

                const auto res = sink.handleNextDataChunk(read_buffer_.data(), std::uint16_t(read_result_));
                if (res < 0)
                {
                    platform_.resetWatchdog();
                    return res;
                }
            }
            else
            {
                platform_.resetWatchdog();
                return 0;       // Done
            }

            /*
             * Send a progress report if time is up
             */
            if (bootloader_.getMonotonicUptime() > next_progress_report_deadline)
            {
                next_progress_report_deadline += DefaultProgressReportInterval;
                sendLog(LogLevel::Info, senoval::convertIntToString(offset) + senoval::String<90>("B down..."));
            }

            /*
             * Wait in order to avoid bus congestion
             * The magic shift ensures that the relative bus utilization does not depend on the bit rate.
             */
            platform_.resetWatchdog();

            const std::chrono::microseconds wait_deadline =
                bootloader_.getMonotonicUptime() +
                std::chrono::microseconds(1'000'000UL / (1UL + (can_bus_bit_rate_ >> 16U)));

            while (bootloader_.getMonotonicUptime() < wait_deadline)
            {
                poll();
            }
        }

        assert(false);  // Should never get here
        return -1;
    }

    void onTransferReception(::CanardRxTransfer* const transfer)
    {
        using namespace impl_;

        /*
         * Dynamic node ID allocation protocol.
         * Taking this branch only if we don't have a node ID, ignoring otherwise.
         * This piece of dark magic has been carefully transplanted from the libcanard demo application.
         */
        if ((::canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID) &&
            (transfer->transfer_type == ::CanardTransferTypeBroadcast) &&
            (transfer->data_type_id == dsdl::NodeIDAllocation::DataTypeID))
        {
            // Rule C - updating the randomized time interval
            send_next_node_id_allocation_request_at_ =
                bootloader_.getMonotonicUptime() + getRandomDuration(std::chrono::microseconds(600'000),
                                                                     std::chrono::microseconds(1'000'000));

            if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
            {
                node_id_allocation_unique_id_offset_ = 0;
                return;
            }

            // Copying the unique ID from the message
            static constexpr std::uint8_t UniqueIDBitOffset = 8;
            std::uint8_t received_unique_id[std::tuple_size_v<HardwareInfo::UniqueID>];
            std::uint8_t received_unique_id_len = 0;
            for (;
                received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U));
                received_unique_id_len++)
            {
                assert(received_unique_id_len < hw_info_.unique_id.size());
                const auto bit_offset = std::uint8_t(UniqueIDBitOffset + received_unique_id_len * 8U);
                (void) ::canardDecodeScalar(transfer,
                                            bit_offset,
                                            8,
                                            false,
                                            &received_unique_id[received_unique_id_len]);
            }

            // Matching the received UID against the local one
            if (std::memcmp(received_unique_id, hw_info_.unique_id.data(), received_unique_id_len) != 0)
            {
                node_id_allocation_unique_id_offset_ = 0;
                return;         // No match, return
            }

            if (received_unique_id_len < hw_info_.unique_id.size())
            {
                // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
                node_id_allocation_unique_id_offset_ = received_unique_id_len;
                send_next_node_id_allocation_request_at_ =
                    bootloader_.getMonotonicUptime() + getRandomDuration(std::chrono::microseconds(0),
                                                                         std::chrono::microseconds(400'000));
            }
            else
            {
                // Allocation complete - copying the allocated node ID from the message
                std::uint8_t allocated_node_id = 0;
                (void) ::canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
                assert(allocated_node_id <= 127);

                ::canardSetLocalNodeID(&canard_, allocated_node_id);
            }
        }

        /*
         * GetNodeInfo request.
         * Someday this mess should be replaced with auto-generated message serialization code, like in libuavcan.
         */
        if ((transfer->transfer_type == ::CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::GetNodeInfo::DataTypeID))
        {
            std::uint8_t buffer[dsdl::GetNodeInfo::MaxSizeBytesResponse]{};

            // NodeStatus
            makeNodeStatusMessage(buffer);

            // SoftwareVersion (query the bootloader)
            if (auto sw_success = bootloader_.getAppInfo())
            {
                const ::kocherga::AppInfo sw = *sw_success;
                buffer[7] = sw.major_version;
                buffer[8] = sw.minor_version;
                buffer[9] = 3;                                              // Optional field flags
                ::canardEncodeScalar(buffer,  80, 32, &sw.vcs_commit);
                ::canardEncodeScalar(buffer, 112, 64, &sw.image_crc);
            }

            // HardwareVersion
            buffer[22] = hw_info_.major;
            buffer[23] = hw_info_.minor;
            std::memmove(&buffer[24], hw_info_.unique_id.data(), hw_info_.unique_id.size());
            buffer[40] = std::uint8_t(hw_info_.certificate_of_authenticity.size());
            std::memmove(&buffer[41],
                         hw_info_.certificate_of_authenticity.data(),
                         hw_info_.certificate_of_authenticity.size());

            // Name
            std::memcpy(&buffer[41 + hw_info_.certificate_of_authenticity.size()],
                        node_name_.c_str(),
                        node_name_.length());

            const std::size_t total_size = 41 + hw_info_.certificate_of_authenticity.size() + node_name_.length();
            assert(total_size <= dsdl::GetNodeInfo::MaxSizeBytesResponse);

            // No need to release the transfer payload, it's empty
            const auto resp_res = ::canardRequestOrRespond(&canard_,
                                                           transfer->source_node_id,
                                                           dsdl::GetNodeInfo::DataTypeSignature,
                                                           dsdl::GetNodeInfo::DataTypeID,
                                                           &transfer->transfer_id,
                                                           transfer->priority,
                                                           ::CanardResponse,
                                                           &buffer[0],
                                                           std::uint16_t(total_size));
            if (resp_res <= 0)
            {
                KOCHERGA_UAVCAN_LOG("GetNodeInfo resp err %d\n", resp_res);
            }
        }

        /*
         * RestartNode request.
         */
        if ((transfer->transfer_type == ::CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::RestartNode::DataTypeID))
        {
            std::uint8_t response = 0;                          // 1 - ok, 0 - rejected

            std::uint64_t magic_number = 0;
            (void) ::canardDecodeScalar(transfer, 0, 40, false, &magic_number);

            if (magic_number == 0xACCE551B1E)
            {
                if (platform_.tryScheduleReboot())
                {
                    response = 1U << 7U;
                }
            }

            // No need to release the transfer payload, it's single frame anyway
            (void) ::canardRequestOrRespond(&canard_,
                                            transfer->source_node_id,
                                            dsdl::RestartNode::DataTypeSignature,
                                            dsdl::RestartNode::DataTypeID,
                                            &transfer->transfer_id,
                                            transfer->priority,
                                            ::CanardResponse,
                                            &response,
                                            1U);
        }

        /*
         * BeginFirmwareUpdate request.
         */
        if ((transfer->transfer_type == ::CanardTransferTypeRequest) &&
            (transfer->data_type_id == dsdl::BeginFirmwareUpdate::DataTypeID))
        {
            const auto bl_state = bootloader_.getState();
            std::uint8_t error = 0;

            if ((bl_state == kocherga::State::AppUpgradeInProgress) || (remote_server_node_id_ != 0))
            {
                error = 2;      // Already in progress
            }
            else if ((bl_state == kocherga::State::ReadyToBoot))
            {
                error = 1;      // Invalid mode
            }
            else
            {
                // Determine the node ID of the firmware server
                (void) ::canardDecodeScalar(transfer, 0, 8, false, &remote_server_node_id_);
                if ((remote_server_node_id_ == 0) ||
                    (remote_server_node_id_ >= CANARD_MAX_NODE_ID))
                {
                    remote_server_node_id_ = transfer->source_node_id;
                }

                // Copy the path
                firmware_file_path_.clear();
                for (std::uint16_t i = 0;
                     i < std::uint16_t(std::min(transfer->payload_len - 1U,
                                                firmware_file_path_.capacity()));
                     i++)
                {
                    char val = '\0';
                    (void) ::canardDecodeScalar(transfer, i * 8U + 8U, 8, false, &val);
                    firmware_file_path_.push_back(val);
                }

                error = 0;
            }

            ::canardReleaseRxTransferPayload(&canard_, transfer);
            const auto resp_res = ::canardRequestOrRespond(&canard_,
                                                           transfer->source_node_id,
                                                           dsdl::BeginFirmwareUpdate::DataTypeSignature,
                                                           dsdl::BeginFirmwareUpdate::DataTypeID,
                                                           &transfer->transfer_id,
                                                           transfer->priority,
                                                           ::CanardResponse,
                                                           &error,
                                                           1);
            if (resp_res <= 0)
            {
                KOCHERGA_UAVCAN_LOG("BeginFWUpdate resp err %d\n", resp_res);
            }
        }

        /*
         * File read response.
         */
        if ((transfer->transfer_type == ::CanardTransferTypeResponse) &&
            (transfer->data_type_id == dsdl::FileRead::DataTypeID) &&
            (((transfer->transfer_id + 1U) & 31U) == file_read_transfer_id_))
        {
            std::int16_t error = 0;
            (void) ::canardDecodeScalar(transfer, 0, 16, false, &error);
            if (error != 0)
            {
                read_result_ = -ErrFileReadFailed;
            }
            else
            {
                read_result_ = std::min<std::int16_t>(256, std::int16_t(transfer->payload_len - 2));
                for (std::int32_t i = 0; i < read_result_; i++)
                {
                    (void) ::canardDecodeScalar(transfer,
                                                std::uint32_t(16 + i * 8),
                                                8U,
                                                false,
                                                &read_buffer_[std::uint32_t(i)]);
                }
            }
        }
    }

    bool shouldAcceptTransfer(std::uint64_t* out_data_type_signature,
                              std::uint16_t data_type_id,
                              ::CanardTransferType transfer_type,
                              std::uint8_t source_node_id)
    {
        using namespace impl_::dsdl;

        (void)source_node_id;

        if (::canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID)
        {
            // Dynamic node ID allocation broadcast
            if ((transfer_type == ::CanardTransferTypeBroadcast) &&
                (data_type_id == NodeIDAllocation::DataTypeID))
            {
                *out_data_type_signature = NodeIDAllocation::DataTypeSignature;
                return true;
            }
        }
        else
        {
            // GetNodeInfo REQUEST
            if ((transfer_type == ::CanardTransferTypeRequest) &&
                (data_type_id == GetNodeInfo::DataTypeID))
            {
                *out_data_type_signature = GetNodeInfo::DataTypeSignature;
                return true;
            }

            // BeginFirmwareUpdate REQUEST
            if ((transfer_type == ::CanardTransferTypeRequest) &&
                (data_type_id == BeginFirmwareUpdate::DataTypeID))
            {
                *out_data_type_signature = BeginFirmwareUpdate::DataTypeSignature;
                return true;
            }

            // FileRead RESPONSE (we don't serve requests of this type)
            if ((transfer_type == ::CanardTransferTypeResponse) &&
                (data_type_id == FileRead::DataTypeID))
            {
                *out_data_type_signature = FileRead::DataTypeSignature;
                return true;
            }

            // RestartNode REQUEST
            if ((transfer_type == ::CanardTransferTypeRequest) &&
                (data_type_id == RestartNode::DataTypeID))
            {
                *out_data_type_signature = RestartNode::DataTypeSignature;
                return true;
            }
        }

        return false;
    }


    static void onTransferReceptionTrampoline(::CanardInstance* ins,
                                              ::CanardRxTransfer* transfer)
    {
        assert((ins != nullptr) && (ins->user_reference != nullptr));
        auto self = reinterpret_cast<BootloaderNode*>(ins->user_reference);
        self->onTransferReception(transfer);
    }

    static bool shouldAcceptTransferTrampoline(const ::CanardInstance* ins,
                                               std::uint64_t* out_data_type_signature,
                                               std::uint16_t data_type_id,
                                               ::CanardTransferType transfer_type,
                                               std::uint8_t source_node_id)
    {
        assert((ins != nullptr) && (ins->user_reference != nullptr));
        auto self = reinterpret_cast<BootloaderNode*>(ins->user_reference);
        return self->shouldAcceptTransfer(out_data_type_signature,
                                          data_type_id,
                                          transfer_type,
                                          source_node_id);
    }

public:
    /**
     * @param blc                       mutable reference to the bootloader instance
     * @param platform                  node platform interface
     * @param name                      product ID, UAVCAN node name; e.g. com.zubax.telega
     * @param hw                        hardware version information per UAVCAN specification
     */
    BootloaderNode(::kocherga::BootloaderController& blc,
                   IUAVCANPlatform& platform,
                   const NodeName& name,
                   const HardwareInfo& hw) :
        bootloader_(blc),
        platform_(platform),
        node_name_(name),
        hw_info_(hw)
    {
        next_1hz_task_invocation_at_ = bootloader_.getMonotonicUptime();
    }

    /**
     * Runs the node thread.
     * This function never returns unless IUAVCANPlatform::shouldExit() returns true.
     * If an RTOS is available, it is advisable to run this method from a separate thread.
     * Otherwise, it is possible to perform other tasks by hijacking certain platform API functions,
     * such as sleep(), receive(), send(), and resetWatchdog().
     *
     * @param can_bus_bit_rate          set if known; defaults to zero, which initiates CAN bit rate autodetect
     * @param node_id                   set if known; defaults to zero, which initiates dynamic node ID allocation
     * @param remote_server_node_id     set if known; defaults to zero, which makes the node wait for an update request
     * @param remote_file_path          set if known; defaults to an empty string, which can be a valid path too
     */
    void run(const std::uint32_t can_bus_bit_rate = 0,
             const std::uint8_t node_id = 0,
             const std::uint8_t remote_server_node_id = 0,
             const char* const remote_file_path = "")
    {
        this->can_bus_bit_rate_ = can_bus_bit_rate;

        if ((remote_server_node_id >= CANARD_MIN_NODE_ID) &&
            (remote_server_node_id <= CANARD_MAX_NODE_ID))
        {
            this->remote_server_node_id_ = remote_server_node_id;
            this->firmware_file_path_ = remote_file_path;
        }

        ::canardInit(&canard_,
                     memory_pool_.data(),
                     memory_pool_.size(),
                     &BootloaderNode::onTransferReceptionTrampoline,
                     &BootloaderNode::shouldAcceptTransferTrampoline,
                     this);

        if ((node_id >= CANARD_MIN_NODE_ID) &&
            (node_id <= CANARD_MAX_NODE_ID))
        {
            ::canardSetLocalNodeID(&canard_, node_id);
        }

        runNodeThread();
    }

    /**
     * Returns the CAN bus bit rate, if known, otherwise zero.
     */
    std::uint32_t getCANBusBitRate() const
    {
        return can_bus_bit_rate_;               // No thread sync is needed, read is atomic
    }

    /**
     * Returns the local UAVCAN node ID, if set, otherwise zero.
     */
    std::uint8_t getLocalNodeID() const
    {
        return confirmed_local_node_id_;        // No thread sync is needed, read is atomic
    }
};

}
