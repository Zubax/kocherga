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
    struct Bitrate
    {
        std::uint32_t arbitration{};
        std::uint32_t data{};  ///< This is ignored if only Classic CAN is supported.
    };

    /// Bitrates predefined by the UCANPHY specification.
    /// Only these values are passed to the configure() method if bitrate auto-detection is used.
    static constexpr std::array<Bitrate, 4> StandardBitrates{{
        {1'000'000, 4'000'000},
        {500'000, 2'000'000},
        {250'000, 1'000'000},
        {125'000, 500'000},
    }};

    /// Version of the CAN standard supported by the underlying hardware.
    enum class Mode : std::uint8_t
    {
        Classic,
        FD
    };

    /// Set up the specified configuration of the CAN hardware.
    /// If the hardware does not support CAN FD, then the data bitrate setting should be ignored.
    /// The "silent" flag selects the silent mode, also known as listen-only mode.
    /// Bitrate autodetection is performed by the bootloader by trying different configurations sequentially.
    /// The return value indicates whether the controller supports CAN FD or only Classic CAN.
    /// The return value is an empty option if the controller is unable to accept the specified configuration.
    [[nodiscard]] virtual auto configure(const Bitrate&                   bitrate,
                                         const bool                       silent,
                                         const CANAcceptanceFilterConfig& filter) -> std::optional<Mode> = 0;

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

inline auto makePseudoUniqueID(const SystemInfo::UniqueID& uid) -> std::uint64_t
{
    ::kocherga::detail::CRC64 crc;
    crc.update(uid.data(), uid.size());
    return crc.get();
}

/// If the local-ID is provided, the filter will match only on service transfers addressed to the local node.
/// If the local-ID is not provided, the filter will match on PnP allocation response messages only.
template <std::uint8_t UAVCANVersion>
[[nodiscard]] auto makeAcceptanceFilter(const std::optional<std::uint8_t> local_node_id) -> CANAcceptanceFilterConfig;
template <>
[[nodiscard]] inline auto makeAcceptanceFilter<0>(const std::optional<std::uint8_t> local_node_id)
    -> CANAcceptanceFilterConfig
{
    if (local_node_id)  // Match on v0 service transfers addressed to local_node_id.
    {
        return {
            0b00000'00000000'0'0000000'1'0000000U | (static_cast<std::uint32_t>(*local_node_id) << 8U),
            0b00000'00000000'0'1111111'1'0000000U,
        };
    }
    return {
        0b00000'0000000000000001'0'0000000U,
        0b00000'1111111111111111'1'0000000U,
    };
}
template <>
[[nodiscard]] inline auto makeAcceptanceFilter<1>(const std::optional<std::uint8_t> local_node_id)
    -> CANAcceptanceFilterConfig
{
    if (local_node_id)  // Match on v1 service transfers addressed to local_node_id.
    {
        return {
            0b000'10'0000000000'0000000'0000000U | (static_cast<std::uint32_t>(*local_node_id) << 7U),
            0b000'10'1000000000'1111111'0000000U,
        };
    }
    return {
        0b000'00'0001111111100101'00000000U,
        0b000'11'1001111111111100'10000000U,
    };
}

}  // namespace detail

/// Kocherga node implementing the UAVCAN/CAN v1 transport along with UAVCAN v0 with automatic version detection.
class CANNode : public kocherga::INode
{
public:
    /// The local UID shall be the same that is passed to the bootloader. It is used for PnP node-ID allocation.
    CANNode(ICANDriver& driver, const SystemInfo::UniqueID& local_unique_id) :
        unique_id_(local_unique_id), pseudo_unique_id_(detail::makePseudoUniqueID(local_unique_id)), driver_(driver)
    {}

    /// By default, this implementation will auto-detect the parameters of the network and do a PnP node-ID allocation.
    /// The application can opt-out of autoconfiguration by providing the required data using this method.
    /// This method shall not be invoked more than once.
    /// Unknown parameters shall be set to empty options.
    /// If this method is used, it shall be invoked before the first poll() is called on the bootloader instance.
    /// The return value is true on success, false if the configuration cannot be applied.
    auto preconfigure(const std::optional<ICANDriver::Bitrate>& can_bitrate,
                      const std::optional<std::uint8_t>         uavcan_version,
                      const std::optional<std::uint8_t>         local_node_id) -> bool
    {
        if (local_node_id && (127 < *local_node_id))
        {
            return false;
        }
        CANAcceptanceFilterConfig filter{0, 0};
        if (uavcan_version)
        {
            if (0 == *uavcan_version)
            {
                filter = detail::makeAcceptanceFilter<0>(local_node_id);
            }
            else if (1 == *uavcan_version)
            {
                filter = detail::makeAcceptanceFilter<1>(local_node_id);
            }
            else
            {
                return false;
            }
        }
        if (can_bitrate)
        {
            bus_mode_ = driver_.configure(*can_bitrate, false, filter);
            if (!bus_mode_)
            {
                return false;
            }
        }
        local_node_id_  = local_node_id;
        uavcan_version_ = uavcan_version;
        assert(!local_node_id_ || (128 > *local_node_id_));
        assert(!uavcan_version_ || (0 == *uavcan_version_) || (1 == *uavcan_version_));
        return true;
    }

private:
    void poll(IReactor& reactor, const std::chrono::microseconds uptime) override
    {
        //
    }

    [[nodiscard]] auto sendRequest(const ServiceID           service_id,
                                   const NodeID              server_node_id,
                                   const TransferID          transfer_id,
                                   const std::size_t         payload_length,
                                   const std::uint8_t* const payload) -> bool override
    {
        (void) service_id;
        (void) server_node_id;
        (void) transfer_id;
        (void) payload_length;
        (void) payload;
        return false;
    }

    void cancelRequest() override { pending_request_meta_.reset(); }

    [[nodiscard]] auto publishMessage(const SubjectID           subject_id,
                                      const TransferID          transfer_id,
                                      const std::size_t         payload_length,
                                      const std::uint8_t* const payload) -> bool override
    {
        (void) subject_id;
        (void) transfer_id;
        (void) payload_length;
        (void) payload;
        return false;
    }

    struct PendingRequestMetadata
    {
        std::uint8_t server_node_id{};
        PortID       service_id{};
        std::uint8_t transfer_id{};
    };

    const SystemInfo::UniqueID unique_id_;
    const std::uint64_t        pseudo_unique_id_;

    ICANDriver&                           driver_;
    std::optional<ICANDriver::Mode>       bus_mode_;
    std::optional<std::uint8_t>           uavcan_version_;
    std::optional<std::uint8_t>           local_node_id_;
    std::optional<PendingRequestMetadata> pending_request_meta_;

    std::chrono::microseconds pnp_next_request_at_{0};
    std::uint64_t             pnp_transfer_id_ = 0;

    std::uint8_t can_autoconfig_step_index_ = 0;
};

}  // namespace kocherga::can
