// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"

namespace kocherga::serial
{
namespace detail
{
using kocherga::detail::BitsPerByte;

/// Size-optimized implementation of CRC32-C (Castagnoli).
class CRC32C
{
public:
    void update(const std::uint8_t b)
    {
        value_ ^= static_cast<std::uint32_t>(b);
        for (auto i = 0U; i < BitsPerByte; i++)
        {
            value_ = ((value_ & 1U) != 0) ? ((value_ >> 1U) ^ ReflectedPoly) : (value_ >> 1U);  // NOLINT
        }
    }

    [[nodiscard]] auto get() const { return value_ ^ Xor; }

    [[nodiscard]] auto getBytes() const -> std::array<std::uint8_t, 4>
    {
        const auto x = get();
        return {
            static_cast<std::uint8_t>(x >> (BitsPerByte * 0U)),
            static_cast<std::uint8_t>(x >> (BitsPerByte * 1U)),
            static_cast<std::uint8_t>(x >> (BitsPerByte * 2U)),
            static_cast<std::uint8_t>(x >> (BitsPerByte * 3U)),
        };
    }

    [[nodiscard]] auto isResidueCorrect() const { return value_ == Residue; }

private:
    static constexpr std::uint32_t Xor           = 0xFFFF'FFFFUL;
    static constexpr std::uint32_t ReflectedPoly = 0x82F6'3B78UL;
    static constexpr std::uint32_t Residue       = 0xB798'B438UL;

    std::uint32_t value_ = Xor;
};

}  // namespace detail

/// Bridges Kocherga/serial with the platform-specific serial port implementation.
/// Implement this and pass a reference to SerialNode.
class ISerialPort
{
public:
    /// Receive a single byte from the RX queue without blocking, if available. Otherwise, return an empty option.
    [[nodiscard]] virtual auto receive() -> std::optional<std::uint8_t> = 0;

    /// Send a single byte into the TX queue without blocking if there is free space available.
    /// Return true if enqueued or sent successfully; return false if no space available.
    [[nodiscard]] virtual auto send(const std::uint8_t b) -> bool = 0;

    virtual ~ISerialPort()           = default;
    ISerialPort()                    = default;
    ISerialPort(const ISerialPort&)  = delete;
    ISerialPort(const ISerialPort&&) = delete;
    auto operator=(const ISerialPort&) -> ISerialPort& = delete;
    auto operator=(const ISerialPort &&) -> ISerialPort& = delete;
};

/// Kocherga node implementing the UAVCAN/serial transport.
class SerialNode : public kocherga::INode
{
public:
    explicit SerialNode(ISerialPort& port) : port_(port) {}

    /// Resets the state of the frame parser. Use this method when your communication channel is reset.
    void reset() { (void) buffer_; }

private:
    void poll(IReactor& reactor, const std::chrono::microseconds uptime) override
    {
        (void) reactor;
        (void) uptime;
    }

    [[nodiscard]] auto requestFileRead(const NodeID           server_node_id,
                                       const TransferID       transfer_id,
                                       const std::size_t      payload_length,
                                       const std::byte* const payload) -> bool override
    {
        (void) server_node_id;
        (void) transfer_id;
        (void) payload_length;
        (void) payload;
        return false;
    }

    void publishHeartbeat(const TransferID transfer_id, const std::byte* const payload) override
    {
        (void) transfer_id;
        (void) payload;
    }

    void publishLogRecord(const TransferID       transfer_id,
                          const std::size_t      payload_length,
                          const std::byte* const payload) override
    {
        (void) transfer_id;
        (void) payload_length;
        (void) payload;
    }

    [[nodiscard]] auto sendEscaped(const std::uint8_t b) -> bool
    {
        if ((b == FrameDelimiter) || (b == EscapePrefix))
        {
            if (!port_.send(EscapePrefix))
            {
                return false;
            }
            return port_.send(static_cast<std::uint8_t>(~b));
        }
        return port_.send(b);
    }

    static constexpr std::size_t BufferSize = 1024;

    static constexpr std::uint8_t FrameDelimiter = 0x9E;
    static constexpr std::uint8_t EscapePrefix   = 0x8E;

    ISerialPort&                         port_;
    std::array<std::uint8_t, BufferSize> buffer_{};
};

}  // namespace kocherga::serial
