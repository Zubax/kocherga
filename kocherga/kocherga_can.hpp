// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"
#include <cassert>
#include <cstdlib>
#include <variant>

namespace kocherga::can
{
namespace detail
{
using kocherga::detail::BitsPerByte;

class CRC16CCITT
{
public:
    static constexpr std::size_t Size = 2;

    void update(const std::uint8_t b) noexcept
    {
        value_ ^= static_cast<std::uint16_t>(static_cast<std::uint16_t>(b) << 8U);
        // Manually unrolled because the difference in performance is drastic. Can't use table because size limitations.
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
        value_ = static_cast<uint16_t>(static_cast<uint16_t>(value_ << 1U) ^ (((value_ & Top) != 0U) ? Poly : 0U));
    }

    void update(const std::size_t size, const std::uint8_t* const ptr) noexcept
    {
        const auto* p = ptr;
        for (std::size_t s = 0; s < size; s++)
        {
            update(*p++);
        }
    }

    [[nodiscard]] auto get() const noexcept { return value_; }

    [[nodiscard]] auto getBytes() const noexcept -> std::array<std::uint8_t, Size>
    {
        const auto x = get();
        return {static_cast<std::uint8_t>(x >> BitsPerByte), static_cast<std::uint8_t>(x)};
    }

    [[nodiscard]] auto isResidueCorrect() const noexcept { return value_ == 0; }

private:
    static constexpr std::uint16_t Top  = 0x8000U;
    static constexpr std::uint16_t Poly = 0x1021U;

    std::uint16_t value_ = std::numeric_limits<std::uint16_t>::max();
};
}  // namespace detail

/// Frames that are not extended data frames shall always be rejected.
struct CANAcceptanceFilterConfig
{
    std::uint32_t extended_can_id = AllSet;
    std::uint32_t mask            = AllSet;

    static constexpr std::uint32_t AllSet = (1ULL << 29U) - 1U;
};

/// Bridges Kocherga/CAN with the platform-specific CAN driver implementation.
/// Implement this and pass a reference to CANNode.
class ICANDriver
{
public:
    enum class Mode : std::uint8_t
    {
        Classic,
        FD
    };

    /// The hardware should have this many acceptance filters.
    static constexpr auto NumAcceptanceFilters = 2U;

    /// Set up the specified configuration of the CAN hardware.
    /// If the hardware does not support CAN FD, then the data bitrate setting should be ignored.
    /// The "silent" flag selects the silent mode, also known as listen-only mode.
    /// The return value indicates whether the controller supports CAN FD or only Classic CAN.
    /// The return value is an empty option if the controller is unable to accept the specified configuration.
    [[nodiscard]] virtual auto configure(const std::array<std::uint32_t, 2> bitrate_arbitration_data,
                                         const bool                         silent,
                                         const std::array<CANAcceptanceFilterConfig, NumAcceptanceFilters>& filters)
        -> std::optional<Mode> = 0;

    /// Non-blocking addition to the transmission queue of a single CAN frame.
    /// The transmission queue shall be at least 100 Classic CAN frames deep, or at least 10 CAN FD frames deep.
    /// Returns true on success, false if: 1. no space available; 2. a transient error occurred; 3. payload_size > MTU.
    /// The payload pointer is invalidated immediately upon return.
    /// If a frame could not be delivered to the bus in about 1 second since being enqueued, it shall be aborted.
    [[nodiscard]] virtual auto push(const bool          force_classic_can,
                                    const std::uint32_t extended_can_id,
                                    const std::uint8_t  payload_size,
                                    const void* const   payload) -> bool = 0;

    /// Non-blocking read of a pending CAN frame from the RX queue.
    /// The frame can be either classic or FD, the bootloader doesn't care.
    /// Returns (CAN ID, payload size) if a frame has been read successfully, empty option otherwise.
    [[nodiscard]] virtual auto pop(void* const payload_buffer)
        -> std::optional<std::tuple<std::uint32_t, std::uint8_t>> = 0;

    virtual ~ICANDriver()         = default;
    ICANDriver()                  = default;
    ICANDriver(const ICANDriver&) = delete;
    ICANDriver(ICANDriver&&)      = delete;
    auto operator=(const ICANDriver&) -> ICANDriver& = delete;
    auto operator=(ICANDriver&&) -> ICANDriver& = delete;
};

class CANNode
{
    //
};

}  // namespace kocherga::can
