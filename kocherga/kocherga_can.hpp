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

    /// Constructs a filter that accepts all extended data frames.
    [[nodiscard]] static auto makePromiscuous() -> CANAcceptanceFilterConfig { return {0, 0}; }
};

/// Bridges Kocherga/CAN with the platform-specific CAN driver implementation.
/// Implement this and pass a reference to CANNode.
class ICANDriver
{
public:
    using PayloadBuffer = std::array<std::uint8_t, 64>;

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
    [[nodiscard]] virtual auto pop(PayloadBuffer& payload_buffer)
        -> std::optional<std::pair<std::uint32_t, std::uint8_t>> = 0;

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

static constexpr std::uint8_t TailByteStartOfTransfer = 0b1000'0000;
static constexpr std::uint8_t TailByteToggleBit       = 0b0010'0000;

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

class IAllocator
{
public:
    [[nodiscard]] virtual auto allocate(const std::size_t size) -> void* = 0;
    virtual void               deallocate(const void* const ptr)         = 0;

    template <typename T, typename... Args>
    [[nodoscard]] auto construct(Args&&... ag) -> T*
    {
        if (void* const p = allocate(sizeof(T)))
        {
            return new (p) T(std::forward<Args>(ag)...);  // NOLINT
        }
        return nullptr;
    }

    template <typename T>
    void destroy(T* const obj)
    {
        if (obj != nullptr)
        {
            obj->~T();
            deallocate(obj);
        }
    }

    virtual ~IAllocator()         = default;
    IAllocator()                  = default;
    IAllocator(const IAllocator&) = delete;
    IAllocator(IAllocator&&)      = delete;
    auto operator=(const IAllocator&) -> IAllocator& = delete;
    auto operator=(IAllocator&&) -> IAllocator& = delete;
};

template <std::size_t BlockSize, std::uint8_t BlockCount>
class BlockAllocator : public IAllocator
{
public:
    [[nodiscard]] auto allocate(const std::size_t size) -> void* override
    {
        if (size <= BlockSize)
        {
            for (auto& blk : pool_)
            {
                auto& [used, mem] = blk;
                if (!used)
                {
                    used = true;
                    return &mem;
                }
            }
        }
        return nullptr;
    }

    void deallocate(const void* const ptr) override
    {
        for (auto& blk : pool_)
        {
            auto& [used, mem] = blk;
            if (used && (&mem == ptr))
            {
                used = false;
                return;
            }
        }
        assert(false);
    }

private:
    using Block = std::pair<bool, std::aligned_storage_t<BlockSize, alignof(std::max_align_t)>>;
    std::array<Block, BlockCount> pool_{};

    static_assert(sizeof(pool_) > (BlockSize * BlockCount));
};

/// Use the standard State behavioral pattern to segregate business logic by activity.
class IActivity
{
public:
    /// If this method returns a non-null, the new activity shall replace the current one.
    [[nodiscard]] virtual auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* = 0;

    // See kocherga::INode
    [[nodiscard]] virtual auto sendRequest(const ServiceID           service_id,
                                           const NodeID              server_node_id,
                                           const TransferID          transfer_id,
                                           const std::size_t         payload_length,
                                           const std::uint8_t* const payload) -> bool
    {
        (void) service_id;
        (void) server_node_id;
        (void) transfer_id;
        (void) payload_length;
        (void) payload;
        return false;
    }

    // See kocherga::INode
    virtual void cancelRequest() {}

    // See kocherga::INode
    [[nodiscard]] virtual auto publishMessage(const SubjectID           subject_id,
                                              const TransferID          transfer_id,
                                              const std::size_t         payload_length,
                                              const std::uint8_t* const payload) -> bool
    {
        (void) subject_id;
        (void) transfer_id;
        (void) payload_length;
        (void) payload;
        return false;
    }

    virtual ~IActivity()        = default;
    IActivity()                 = default;
    IActivity(const IActivity&) = delete;
    IActivity(IActivity&&)      = delete;
    auto operator=(const IActivity&) -> IActivity& = delete;
    auto operator=(IActivity&&) -> IActivity& = delete;
};

class V0NodeIDAllocationActivity : public IActivity
{
public:
    V0NodeIDAllocationActivity(IAllocator&                 allocator,
                               ICANDriver&                 driver,
                               const SystemInfo::UniqueID& local_uid,
                               const ICANDriver::Bitrate&  bitrate) :
        allocator_(allocator), driver_(driver), local_uid_(local_uid), bitrate_(bitrate)
    {}

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        (void) uptime;
        (void) allocator_;
        (void) driver_;
        (void) local_uid_;
        (void) bitrate_;
        (void) bus_mode_;
        return nullptr;
    }

private:
    IAllocator&                     allocator_;
    ICANDriver&                     driver_;
    const SystemInfo::UniqueID      local_uid_;
    ICANDriver::Bitrate             bitrate_;
    std::optional<ICANDriver::Mode> bus_mode_;
};

class V1NodeIDAllocationActivity : public IActivity
{
public:
    V1NodeIDAllocationActivity(IAllocator&                 allocator,
                               ICANDriver&                 driver,
                               const SystemInfo::UniqueID& local_uid,
                               const ICANDriver::Bitrate&  bitrate) :
        allocator_(allocator), driver_(driver), local_uid_(local_uid), bitrate_(bitrate)
    {}

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        (void) uptime;
        (void) allocator_;
        (void) driver_;
        (void) local_uid_;
        (void) bitrate_;
        (void) bus_mode_;
        return nullptr;
    }

private:
    IAllocator&                     allocator_;
    ICANDriver&                     driver_;
    const SystemInfo::UniqueID      local_uid_;
    ICANDriver::Bitrate             bitrate_;
    std::optional<ICANDriver::Mode> bus_mode_;
};

class ProtocolVersionDetectionActivity : public IActivity
{
public:
    ProtocolVersionDetectionActivity(IAllocator&                 allocator,
                                     ICANDriver&                 driver,
                                     const SystemInfo::UniqueID& local_uid,
                                     const ICANDriver::Bitrate&  bitrate) :
        allocator_(allocator), driver_(driver), local_uid_(local_uid), bitrate_(bitrate)
    {}

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        if (!bus_mode_)
        {
            bus_mode_ = driver_.configure(bitrate_, true, CANAcceptanceFilterConfig::makePromiscuous());
        }
        else if (highest_version_seen_ && (uptime > deadline_))
        {
            return constructSuccessor(*highest_version_seen_);
        }
        else
        {
            ICANDriver::PayloadBuffer buf{};
            if (const auto frame = driver_.pop(buf))
            {
                const auto [can_id, payload_size] = *frame;
                if (payload_size > 0)  // UAVCAN frames are guaranteed to contain the tail byte always.
                {
                    if (const auto uavcan_version = tryDetectProtocolVersionFromFrame(can_id, buf.at(payload_size - 1)))
                    {
                        if (!highest_version_seen_)
                        {
                            deadline_ = uptime + ListeningPeriod;
                        }
                        if (!highest_version_seen_ || (*highest_version_seen_ < *uavcan_version))
                        {
                            highest_version_seen_ = uavcan_version;
                            assert(highest_version_seen_);
                        }
                    }
                }
            }
        }
        return nullptr;
    }

private:
    [[nodiscard]] static auto tryDetectProtocolVersionFromFrame(const std::uint32_t can_id,
                                                                const std::uint8_t  tail_byte)
        -> std::optional<std::uint8_t>
    {
        // CAN ID is not validated at the moment. This may be improved in the future to avoid misdetection if there are
        // other protocols besides UAVCAN on the same network.
        (void) can_id;
        if ((tail_byte & TailByteStartOfTransfer) != 0)
        {
            if ((tail_byte & TailByteToggleBit) != 0)
            {
                return 1;
            }
            return 0;
        }
        return {};
    }

    [[nodiscard]] auto constructSuccessor(const std::uint8_t detected_protocol_version) -> IActivity*
    {
        if (0 == detected_protocol_version)
        {
            return allocator_.construct<V0NodeIDAllocationActivity>(allocator_, driver_, local_uid_, bitrate_);
        }
        if (1 == detected_protocol_version)
        {
            return allocator_.construct<V1NodeIDAllocationActivity>(allocator_, driver_, local_uid_, bitrate_);
        }
        assert(false);
        return nullptr;
    }

    /// Heartbeats are exchanged at 1 Hz, so about one second should be enough to determine which versions are used.
    static constexpr std::chrono::microseconds ListeningPeriod{1'100'000};

    IAllocator&                     allocator_;
    ICANDriver&                     driver_;
    const SystemInfo::UniqueID      local_uid_;
    ICANDriver::Bitrate             bitrate_;
    std::optional<ICANDriver::Mode> bus_mode_;
    std::optional<std::uint8_t>     highest_version_seen_;
    std::chrono::microseconds       deadline_{};
};

class BitrateDetectionActivity : public IActivity
{
public:
    BitrateDetectionActivity(IAllocator& allocator, ICANDriver& driver, const SystemInfo::UniqueID& local_uid) :
        allocator_(allocator), driver_(driver), local_uid_(local_uid)
    {}

private:
    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        ICANDriver::PayloadBuffer buf{};
        if (bus_mode_ && driver_.pop(buf))
        {
            const auto bitrate = ICANDriver::StandardBitrates.at(setting_index_);
            return allocator_.construct<ProtocolVersionDetectionActivity>(allocator_, driver_, local_uid_, bitrate);
        }
        if (!bus_mode_ || (uptime > next_try_at_))
        {
            const auto br = ICANDriver::StandardBitrates.at((++setting_index_) % ICANDriver::StandardBitrates.size());
            bus_mode_     = driver_.configure(br, true, CANAcceptanceFilterConfig::makePromiscuous());
            next_try_at_  = uptime + ListeningPeriod;
        }
        return nullptr;
    }

    /// Heartbeats are exchanged at 1 Hz, so about one second should be enough to determine if the bit rate is correct.
    static constexpr std::chrono::microseconds ListeningPeriod{1'100'000};

    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;

    std::optional<ICANDriver::Mode> bus_mode_;
    std::uint8_t                    setting_index_ = ICANDriver::StandardBitrates.size() - 1U;
    std::chrono::microseconds       next_try_at_{};
};

}  // namespace detail

/// Kocherga node implementing the UAVCAN/CAN v1 transport along with UAVCAN v0 with automatic version detection.
class CANNode : public kocherga::INode
{
public:
    /// The local UID shall be the same that is passed to the bootloader. It is used for PnP node-ID allocation.
    /// By default, this implementation will auto-detect the parameters of the network and do a PnP node-ID allocation.
    /// The application can opt-out of autoconfiguration by providing the required data to the constructor.
    /// Unknown parameters shall be set to empty options.
    CANNode(ICANDriver&                               driver,
            const SystemInfo::UniqueID&               local_unique_id,
            const std::optional<ICANDriver::Bitrate>& can_bitrate    = {},
            const std::optional<std::uint8_t>         uavcan_version = {},
            const std::optional<std::uint8_t>         local_node_id  = {})
    {
        if (can_bitrate &&                               //
            uavcan_version && (*uavcan_version == 0) &&  //
            local_node_id && (*local_node_id > 0) && (*local_node_id < 128))
        {
            activity_ = nullptr;
            activity_ = nullptr;
        }
        else if (can_bitrate &&                               //
                 uavcan_version && (*uavcan_version == 1) &&  //
                 local_node_id && (*local_node_id < 128))
        {
            activity_ = nullptr;
        }
        else if (can_bitrate && uavcan_version && (*uavcan_version == 0))
        {
            activity_ = activity_allocator_.construct<detail::V0NodeIDAllocationActivity>(activity_allocator_,
                                                                                          driver,
                                                                                          local_unique_id,
                                                                                          *can_bitrate);
        }
        else if (can_bitrate && uavcan_version && (*uavcan_version == 1))
        {
            activity_ = activity_allocator_.construct<detail::V1NodeIDAllocationActivity>(activity_allocator_,
                                                                                          driver,
                                                                                          local_unique_id,
                                                                                          *can_bitrate);
        }
        else if (can_bitrate)
        {
            activity_ = activity_allocator_.construct<detail::ProtocolVersionDetectionActivity>(activity_allocator_,
                                                                                                driver,
                                                                                                local_unique_id,
                                                                                                *can_bitrate);
        }
        else
        {
            activity_ = activity_allocator_.construct<detail::BitrateDetectionActivity>(activity_allocator_,
                                                                                        driver,
                                                                                        local_unique_id);
        }
    }

private:
    void poll(IReactor& reactor, const std::chrono::microseconds uptime) override
    {
        if (detail::IActivity* const new_activity = activity_->poll(reactor, uptime))
        {
            activity_allocator_.destroy(activity_);
            activity_ = new_activity;
        }
    }

    [[nodiscard]] auto sendRequest(const ServiceID           service_id,
                                   const NodeID              server_node_id,
                                   const TransferID          transfer_id,
                                   const std::size_t         payload_length,
                                   const std::uint8_t* const payload) -> bool override
    {
        return activity_->sendRequest(service_id, server_node_id, transfer_id, payload_length, payload);
    }

    void cancelRequest() override { activity_->cancelRequest(); }

    [[nodiscard]] auto publishMessage(const SubjectID           subject_id,
                                      const TransferID          transfer_id,
                                      const std::size_t         payload_length,
                                      const std::uint8_t* const payload) -> bool override
    {
        return activity_->publishMessage(subject_id, transfer_id, payload_length, payload);
    }

    detail::BlockAllocator<1024, 2> activity_allocator_;
    detail::IActivity*              activity_ = nullptr;
};

}  // namespace kocherga::can
