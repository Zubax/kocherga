// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"
#include <algorithm>
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

    [[nodoscard]] auto operator==(const CANAcceptanceFilterConfig& cfg) const -> bool
    {
        return (extended_can_id == cfg.extended_can_id) && (mask == cfg.mask);
    }
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

        [[nodoscard]] auto operator==(const Bitrate& rhs) const -> bool
        {
            return (data == rhs.data) && (arbitration == rhs.arbitration);
        }
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
    /// The bootloader is always able to accept CAN FD frames regardless of the returned value.
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
        0b00000'0000000000000001'0'0000000U,  // Match on PnP node-ID allocation messages (both request/response).
        0b00000'0000000000000011'1'0000000U,  // Ignore the discriminator.
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
        0b000'00'0001111111100101'00000000U,  // Match on PnP node-ID allocation response messages of both versions.
        0b000'11'1001111111111100'10000000U,
    };
}

/// This is valid for v1 only.
struct FrameModel
{
    std::uint8_t priority    = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t transfer_id = std::numeric_limits<std::uint8_t>::max();

    bool start_of_transfer = false;
    bool end_of_transfer   = false;
    bool toggle            = false;

    std::size_t         payload_size = 0;
    const std::uint8_t* payload      = nullptr;
};

struct MessageFrameModel : public FrameModel
{
    std::uint16_t               subject_id = std::numeric_limits<std::uint16_t>::max();
    std::optional<std::uint8_t> source_node_id;
};

struct ServiceFrameModel : public FrameModel
{
    std::uint16_t service_id           = std::numeric_limits<std::uint16_t>::max();
    std::uint8_t  source_node_id       = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t  destination_node_id  = std::numeric_limits<std::uint8_t>::max();
    bool          request_not_response = false;
};

/// The payload of the returned frame will be pointing into the supplied payload buffer, so their lifetimes are linked.
/// The CAN ID shall be less than 2**29.
[[nodiscard]] inline auto parseFrame(const std::uint32_t extended_can_id,
                                     const std::size_t   payload_size,
                                     const void* const   payload)
    -> std::optional<std::variant<MessageFrameModel, ServiceFrameModel>>
{
    assert(payload != nullptr);
    if (0 != (extended_can_id & (1ULL << 23U)))  // Reserved bit.
    {
        return {};
    }
    if (payload_size < 1)
    {
        return {};
    }
    // Parse the tail byte.
    const auto         out_payload_size  = static_cast<std::uint8_t>(payload_size - 1U);
    const std::uint8_t tail              = *((static_cast<const uint8_t*>(payload)) + out_payload_size);  // NOSONAR
    const bool         start_of_transfer = (tail & 0b1000'0000U) != 0;
    const bool         end_of_transfer   = (tail & 0b0100'0000U) != 0;
    const bool         toggle            = (tail & 0b0010'0000U) != 0;
    const std::uint8_t transfer_id       = (tail & 0b0001'1111U);
    if (start_of_transfer && (!toggle))
    {
        return {};  // UAVCAN v0
    }
    if ((!start_of_transfer || !end_of_transfer) && (out_payload_size == 0))
    {
        return {};  // Multi-frame transfer frames require payload.
    }
    if (!end_of_transfer && (out_payload_size < 7))
    {
        return {};  // Non-last frame of a multi-frame transfer cannot have less than 7 bytes of payload.
    }
    // Parse the CAN ID.
    const auto priority                        = static_cast<std::uint8_t>((extended_can_id >> 26U) & 7U);
    const auto source_node_id_or_discriminator = static_cast<std::uint8_t>(extended_can_id & 0x7FU);
    if (const auto is_message = (0 == (extended_can_id & (1ULL << 25U))); is_message)
    {
        if (0 != (extended_can_id & (1ULL << 7U)))
        {
            return {};
        }
        MessageFrameModel out{};
        out.priority          = priority;
        out.transfer_id       = transfer_id;
        out.start_of_transfer = start_of_transfer;
        out.end_of_transfer   = end_of_transfer;
        out.toggle            = toggle;
        out.payload_size      = out_payload_size;
        out.payload           = static_cast<const uint8_t*>(payload);
        out.subject_id        = static_cast<std::uint16_t>((extended_can_id >> 8U) & 0x1FFFU);
        if (const bool is_anonymous = (0 != (extended_can_id & (1ULL << 24U))); !is_anonymous)
        {
            out.source_node_id = source_node_id_or_discriminator;
        }
        else
        {
            if (!(start_of_transfer && end_of_transfer))
            {
                return {};  // Anonymous transfers can be only single-frame transfers.
            }
        }
        return out;
    }
    ServiceFrameModel out{};
    out.priority             = priority;
    out.transfer_id          = transfer_id;
    out.start_of_transfer    = start_of_transfer;
    out.end_of_transfer      = end_of_transfer;
    out.toggle               = toggle;
    out.payload_size         = out_payload_size;
    out.payload              = static_cast<const uint8_t*>(payload);
    out.service_id           = static_cast<std::uint16_t>((extended_can_id >> 14U) & 0x1FFU);
    out.source_node_id       = source_node_id_or_discriminator;
    out.destination_node_id  = static_cast<std::uint8_t>((extended_can_id >> 7U) & 0x7FU);
    out.request_not_response = 0 != (extended_can_id & (1ULL << 24U));
    if (out.source_node_id == out.destination_node_id)
    {
        return {};
    }
    return out;
}

/// This transfer reassembler is only suitable for very basic applications such as this bootloader, where the edge
/// cases that are not correctly handled by this implementation can be tolerated. Other applications should not rely
/// on it; instead, a proper implementation such as that provided in libcanard should be used.
/// The main limitation is that this reassembler does not support interleaved transfers, which is OK for the bootloader.
/// The user of this class is responsible for checking the port-ID.
template <std::size_t Extent>
class SimplifiedTransferReassembler
{
public:
    using Result = std::pair<std::size_t, const std::uint8_t*>;

protected:
    SimplifiedTransferReassembler() = default;

    /// The payload pointer in the result remains valid until the next update.
    [[nodiscard]] auto updateImpl(const FrameModel& frame, const std::uint8_t source) -> std::optional<Result>
    {
        if (frame.start_of_transfer)
        {
            reset(source, frame.transfer_id);
        }
        else
        {
            const bool match = (source == source_node_id_) &&          //
                               (frame.transfer_id == transfer_id_) &&  //
                               (frame.toggle == !toggle_);
            if (!match)
            {
                return {};
            }
            toggle_ = !toggle_;
        }
        crc_.update(frame.payload_size, static_cast<const std::uint8_t*>(frame.payload));
        const auto sz = std::min(frame.payload_size, payload_.size() - stored_payload_size_);
        std::copy_n(frame.payload, sz, payload_.begin() + stored_payload_size_);
        stored_payload_size_ += sz;
        received_payload_size_ += frame.payload_size;
        if (frame.end_of_transfer)
        {
            if (frame.start_of_transfer)  // This is a single-frame transfer.
            {
                const Result out{stored_payload_size_, payload_.data()};
                reset();
                return out;
            }
            if ((received_payload_size_ >= CRC16CCITT::Size) && crc_.isResidueCorrect())
            {
                const std::size_t true_size = std::min(stored_payload_size_, received_payload_size_ - CRC16CCITT::Size);
                reset();
                return Result{true_size, payload_.data()};
            }
        }
        return {};
    }

    std::array<std::uint8_t, Extent> payload_{};

private:
    void reset() { reset(std::numeric_limits<std::uint8_t>::max(), std::numeric_limits<std::uint8_t>::max()); }

    void reset(const std::uint8_t source_node_id, const std::uint8_t transfer_id)
    {
        source_node_id_        = source_node_id;
        transfer_id_           = transfer_id;
        toggle_                = true;
        crc_                   = {};
        stored_payload_size_   = 0;
        received_payload_size_ = 0;
    }

    std::uint8_t source_node_id_ = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t transfer_id_    = std::numeric_limits<std::uint8_t>::max();
    bool         toggle_         = true;
    CRC16CCITT   crc_;
    std::size_t  stored_payload_size_   = 0;
    std::size_t  received_payload_size_ = 0;
};

/// The user of this class is responsible for checking the subject-ID on the received frames.
template <std::size_t Extent>
class SimplifiedMessageTransferReassembler : public SimplifiedTransferReassembler<Extent>
{
public:
    using typename SimplifiedTransferReassembler<Extent>::Result;

    /// The payload pointer in the result remains valid until the next update.
    [[nodiscard]] auto update(const MessageFrameModel& frame) -> std::optional<Result>
    {
        if (!frame.source_node_id)  // Anonymous frames accepted unconditionally.
        {
            const auto sz = std::min(frame.payload_size, payload_.size());
            std::copy_n(frame.payload, sz, payload_.data());
            return Result{sz, payload_.data()};
        }
        return SimplifiedTransferReassembler<Extent>::updateImpl(frame, *frame.source_node_id);
    }

private:
    using SimplifiedTransferReassembler<Extent>::payload_;
};

/// The user of this class is responsible for checking the service-ID and request/response flag on the received frames.
template <std::size_t Extent>
class SimplifiedServiceTransferReassembler : public SimplifiedTransferReassembler<Extent>
{
public:
    using typename SimplifiedTransferReassembler<Extent>::Result;

    explicit SimplifiedServiceTransferReassembler(const std::uint8_t local_node_id) : local_node_id_(local_node_id) {}

    /// The payload pointer in the result remains valid until the next update.
    [[nodiscard]] auto update(const ServiceFrameModel& frame) -> std::optional<Result>
    {
        if (local_node_id_ == frame.destination_node_id)
        {
            return SimplifiedTransferReassembler<Extent>::updateImpl(frame, frame.source_node_id);
        }
        return {};
    }

private:
    const std::uint8_t local_node_id_;
};

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
    /// It is not allowed to call this method again after it returned a non-zero pointer.
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

class V0MainActivity : public IActivity
{
public:
    V0MainActivity(IAllocator&                 allocator,
                   ICANDriver&                 driver,
                   const SystemInfo::UniqueID& local_uid,
                   const std::uint8_t          local_node_id) :
        allocator_(allocator), driver_(driver), local_uid_(local_uid), local_node_id_(local_node_id)
    {
        assert((local_node_id_ > 0) && (local_node_id_ < 128));
    }

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        (void) uptime;
        (void) allocator_;
        (void) driver_;
        (void) local_uid_;
        (void) local_node_id_;
        return nullptr;
    }

    [[nodiscard]] auto getLocalNodeID() const -> std::uint8_t { return local_node_id_; }

private:
    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;
    const std::uint8_t         local_node_id_;
};

/// The following example shows the CAN exchange dump collected from a real network using the old UAVCAN v0 GUI Tool.
///
/// Unique-ID of the allocatee: 35 FF D5 05 50 59 31 34 61 41 23 43 00 00 00 00
/// Preferred node-ID:          0       (any, no preference)
/// Allocated node-ID:          125     (125 << 1 == 0xFA)
///
///     Dir. Time        CAN ID      CAN Payload                Source
///     -------------------------------------------------------------------------------------------------------
///     ->   0.525710    1E285100    01 35 FF D5 05 50 59 D8    Anon       1st stage request, bytes [0, 6)
///     <-   0.527186    1400017F    00 35 FF D5 05 50 59 C0    127        1st stage response, 6 bytes of UID
///     ->   0.923752    1E3C1D00    00 31 34 61 41 23 43 D9    Anon       2nd stage request, bytes [6, 12)
///     <-   0.927630    1400017F    9E 3D 00 35 FF D5 05 81    127        2nd stage response, 12 bytes of UID
///     <-   0.927676    1400017F    50 59 31 34 61 41 23 21    127        ...
///     <-   0.927710    1400017F    43 41                      127        ...
///     ->   1.083750    1E909D00    00 00 00 00 00 DA          Anon       3rd stage request, bytes [12, 16)
///     <-   1.088248    1400017F    8C 7A FA 35 FF D5 05 82    127        3rd stage response, 16 bytes of UID
///     <-   1.088300    1400017F    50 59 31 34 61 41 23 22    127        ...
///     <-   1.088752    1400017F    43 00 00 00 00 42          127        ...
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
        {
            ICANDriver::PayloadBuffer buf{};
            while (const auto frame = driver_.pop(buf))
            {
                const auto [can_id, payload_size] = *frame;
                if (IActivity* const out = processReceivedFrame(uptime, can_id, payload_size, buf.data()))
                {
                    return out;
                }
            }
        }
        if (deadline_)
        {
            if (uptime >= *deadline_)
            {
                handleDeadline(uptime);
            }
        }
        else  // First call -- initialize the scheduler
        {
            reset(uptime);
            assert(deadline_);
        }
        return nullptr;
    }

    [[nodiscard]] auto getDeadline() const -> std::chrono::microseconds
    {
        return deadline_ ? *deadline_ : std::chrono::microseconds{};
    }

    [[nodiscard]] auto getStage() const { return stage_; }

private:
    using DelayRange = std::pair<std::chrono::microseconds, std::chrono::microseconds>;

    [[nodiscard]] auto processReceivedFrame(const std::chrono::microseconds now,
                                            const std::uint32_t             can_id,
                                            const std::size_t               payload_size,
                                            const std::uint8_t* const       payload) -> IActivity*
    {
        if ((can_id & 0x80U) != 0)  // Ignore if this is a service transfer frame.
        {
            return nullptr;
        }
        const std::uint8_t source_node_id = can_id & 0x7FU;
        const bool         anonymous      = source_node_id == 0;
        if (const auto data_type_id = (can_id >> 8U) & (anonymous ? 0b11U : 0xFFFFU); data_type_id != DataTypeID)
        {
            return nullptr;
        }
        if (anonymous)  // This is a request from somebody else, set up new schedule to avoid conflicts.
        {
            reset(now);
            return nullptr;
        }
        if ((payload_size < 1) || (payload_size > 8))  // Ignore bad frames
        {
            return nullptr;
        }
        const std::uint8_t tail_byte = payload[payload_size - 1U];
        const bool         sot       = (tail_byte & 0b1000'0000U) != 0;
        const bool         eot       = (tail_byte & 0b0100'0000U) != 0;
        const bool         tog       = (tail_byte & 0b0010'0000U) != 0;
        const std::uint8_t tid       = (tail_byte & 0b0001'1111U);
        return acceptAllocationResponseFrame(now, source_node_id, payload_size - 1U, payload, sot, eot, tog, tid);
    }

    [[nodiscard]] auto acceptAllocationResponseFrame(const std::chrono::microseconds now,
                                                     const std::uint8_t              source_node_id,
                                                     const std::size_t               payload_size,
                                                     const std::uint8_t* const       payload,
                                                     const bool                      start_of_transfer,
                                                     const bool                      end_of_transfer,
                                                     const bool                      toggle_bit,
                                                     const std::uint8_t              transfer_id) -> IActivity*
    {
        if (start_of_transfer && toggle_bit)  // Not UAVCAN v0
        {
            return nullptr;
        }
        if (start_of_transfer)
        {
            rx_state_ = RxState(source_node_id, transfer_id);
        }
        else
        {
            const bool match = (source_node_id == rx_state_.source_node_id) &&  //
                               (transfer_id == rx_state_.transfer_id) &&        //
                               (toggle_bit == !rx_state_.toggle);
            if (!match)
            {
                return nullptr;
            }
            rx_state_.toggle = !rx_state_.toggle;
        }
        const auto sz = std::min(payload_size, rx_state_.payload.size() - rx_state_.payload_size);
        std::copy_n(payload, sz, &rx_state_.payload.at(rx_state_.payload_size));
        rx_state_.payload_size += sz;
        if (start_of_transfer && end_of_transfer)
        {
            auto* const out = acceptAllocationResponseMessage(now, rx_state_.payload_size, &rx_state_.payload.at(0));
            rx_state_       = {};
            return out;
        }
        if (!start_of_transfer && end_of_transfer && (rx_state_.payload_size > 2))
        {
            CRC16CCITT crc;
            for (auto i = 0U; i < sizeof(DataTypeSignature); i++)
            {
                crc.update(static_cast<std::uint8_t>((DataTypeSignature >> (8U * i)) & 0xFFU));
            }
            crc.update(rx_state_.payload_size - 2U, &rx_state_.payload.at(2));
            if (crc.get() == (static_cast<std::uint32_t>(rx_state_.payload.at(1) << 8U) | rx_state_.payload.at(0)))
            {
                auto* const out =
                    acceptAllocationResponseMessage(now, rx_state_.payload_size - 2U, &rx_state_.payload.at(2));
                rx_state_ = {};
                return out;
            }
            // Transfer reassembly error -- CRC mismatch.
        }
        return nullptr;
    }

    [[nodiscard]] auto acceptAllocationResponseMessage(const std::chrono::microseconds now,
                                                       const std::size_t               message_data_size,
                                                       const std::uint8_t* const       message_data) -> IActivity*
    {
        if ((message_data_size < 2) || (message_data_size > 17))
        {
            return nullptr;
        }
        const auto received_uid_size = message_data_size - 1;
        if (0 != std::memcmp(local_uid_.data(), message_data + 1, received_uid_size))
        {
            reset(now);  // This response is meant for somebody else, set up new schedule to avoid conflicts.
            return nullptr;
        }
        if (received_uid_size < local_uid_.size())
        {
            stage_ = (received_uid_size >= 12) ? 2 : 1;
            schedule(now, DelayRangeFollowup);
            return nullptr;
        }
        const std::uint8_t node_id = message_data[0] >> 1U;
        if (node_id < 1)  // Bad allocation
        {
            return nullptr;
        }
        assert(node_id < 128);
        // Allocation done, full match.
        if (const auto bus_mode = driver_.configure(bitrate_, false, makeAcceptanceFilter<0>(node_id)))
        {
            (void) bus_mode;
            return allocator_.construct<V0MainActivity>(allocator_, driver_, local_uid_, node_id);
        }
        return nullptr;
    }

    void handleDeadline(const std::chrono::microseconds now)
    {
        if (0 == stage_)
        {
            const std::array<std::uint8_t, 8> buf{{
                1,  // first_part_of_unique_id=true, any node-ID (no preference)
                local_uid_.at(0),
                local_uid_.at(1),
                local_uid_.at(2),
                local_uid_.at(3),
                local_uid_.at(4),
                local_uid_.at(5),
                static_cast<std::uint8_t>(0b1100'0000U | tx_transfer_id_),
            }};
            (void) send(buf);
        }
        else if (1 == stage_)
        {
            const std::array<std::uint8_t, 8> buf{{
                0,  // first_part_of_unique_id=false, any node-ID (no preference)
                local_uid_.at(6),
                local_uid_.at(7),
                local_uid_.at(8),
                local_uid_.at(9),
                local_uid_.at(10),
                local_uid_.at(11),
                static_cast<std::uint8_t>(0b1100'0000U | tx_transfer_id_),
            }};
            (void) send(buf);
        }
        else
        {
            const std::array<std::uint8_t, 6> buf{{
                0,  // first_part_of_unique_id=false, any node-ID (no preference)
                local_uid_.at(12),
                local_uid_.at(13),
                local_uid_.at(14),
                local_uid_.at(15),
                static_cast<std::uint8_t>(0b1100'0000U | tx_transfer_id_),
            }};
            (void) send(buf);
        }
        tx_transfer_id_++;
        reset(now);
    }

    void reset(const std::chrono::microseconds now)
    {
        stage_    = 0;
        rx_state_ = {};
        schedule(now, DelayRangeRequest);
    }

    void schedule(const std::chrono::microseconds now, const DelayRange range)
    {
        const auto delta      = std::abs(range.first.count() - range.second.count());
        const auto randomized = (std::rand() * delta) / RAND_MAX;  // NOSONAR rand() ok
        assert((0 <= randomized) && (randomized <= delta));
        const auto delay = std::max(std::chrono::microseconds(1), range.first) + std::chrono::microseconds(randomized);
        assert(range.first <= delay);
        assert(range.second >= delay);
        deadline_ = now + delay;
    }

    template <std::size_t PayloadSize>
    [[nodoscard]] auto send(const std::array<std::uint8_t, PayloadSize>& payload) -> bool
    {
        static_assert(PayloadSize <= 8);
        CRC16CCITT discriminator_crc;
        discriminator_crc.update(PayloadSize, payload.data());
        const std::uint32_t discriminator = discriminator_crc.get() & ((1UL << 14U) - 1U);
        const std::uint32_t can_id        = CANIDMaskWithoutDiscriminator | (discriminator << 10U);
        return driver_.push(true, can_id, PayloadSize, payload.data());
    }

    static constexpr DelayRange    DelayRangeRequest{600'000, 1'000'000};
    static constexpr DelayRange    DelayRangeFollowup{0, 400'000};
    static constexpr std::uint32_t CANIDMaskWithoutDiscriminator = 0x1E'0001'00UL;

    static constexpr std::uint16_t DataTypeID        = 1;  // uavcan.protocol.dynamic_node_id.Allocation
    static constexpr std::uint64_t DataTypeSignature = 0x0B2A812620A11D40ULL;

    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;
    const ICANDriver::Bitrate  bitrate_;

    std::optional<std::chrono::microseconds> deadline_;
    std::uint8_t                             stage_ = 0;

    std::uint8_t tx_transfer_id_ = 0;

    struct RxState
    {
        std::uint8_t                       source_node_id = 0;
        std::uint8_t                       transfer_id    = std::numeric_limits<std::uint8_t>::max();
        bool                               toggle         = false;
        std::size_t                        payload_size   = 0;
        std::array<std::uint8_t, 7UL * 3U> payload{};

        RxState() = default;
        RxState(const std::uint8_t source_node_id, const std::uint8_t transfer_id) :
            source_node_id(source_node_id), transfer_id(transfer_id)
        {}
    } rx_state_{};
};

class V1MainActivity : public IActivity
{
public:
    V1MainActivity(IAllocator&                 allocator,
                   ICANDriver&                 driver,
                   const SystemInfo::UniqueID& local_uid,
                   const ICANDriver::Mode      bus_mode,
                   const std::uint8_t          local_node_id) :
        allocator_(allocator),
        driver_(driver),
        local_uid_(local_uid),
        bus_mode_(bus_mode),
        local_node_id_(local_node_id)
    {}

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        (void) uptime;
        (void) allocator_;
        (void) driver_;
        (void) local_uid_;
        (void) bus_mode_;
        (void) local_node_id_;
        return nullptr;
    }

    [[nodiscard]] auto getLocalNodeID() const -> std::uint8_t { return local_node_id_; }

private:
    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;
    const ICANDriver::Mode     bus_mode_;
    const std::uint8_t         local_node_id_;
};

class V1NodeIDAllocationActivity : public IActivity
{
    static constexpr std::uint64_t PseudoUIDMask = (1ULL << 48U) - 1U;

public:
    V1NodeIDAllocationActivity(IAllocator&                 allocator,
                               ICANDriver&                 driver,
                               const SystemInfo::UniqueID& local_uid,
                               const ICANDriver::Bitrate&  bitrate,
                               const ICANDriver::Mode      bus_mode) :
        allocator_(allocator),
        driver_(driver),
        local_uid_(local_uid),
        pseudo_uid_(makePseudoUniqueID(local_uid) & PseudoUIDMask),
        bitrate_(bitrate),
        bus_mode_(bus_mode)
    {}

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) reactor;
        {
            ICANDriver::PayloadBuffer buf{};
            while (const auto frame = driver_.pop(buf))
            {
                const auto [can_id, payload_size] = *frame;
                if (IActivity* const out = processReceivedFrame(can_id, payload_size, buf.data()))
                {
                    return out;
                }
            }
        }
        if (uptime >= deadline_)
        {
            handleDeadline();
            deadline_ = uptime + getNextPeriod();
        }
        return nullptr;
    }

private:
    // The upper bound of the randomization interval cannot be less than 1 second, the maximum is not limited by Spec.
    static constexpr std::chrono::microseconds MaxPeriod{3'000'000};

    [[nodiscard]] auto processReceivedFrame(const std::uint32_t       can_id,
                                            const std::uint8_t        payload_size,
                                            const std::uint8_t* const payload) -> IActivity*
    {
        if (const auto frame = parseFrame(can_id, payload_size, payload))
        {
            if (const auto* const m = std::get_if<MessageFrameModel>(&*frame); (m != nullptr) && m->source_node_id)
            {
                switch (m->subject_id)
                {
                case static_cast<std::uint16_t>(SubjectID::PnPNodeIDAllocationData_v1):
                {
                    if (const auto res = rx_v1_.update(*m))
                    {
                        return acceptResponseV1(res->first, res->second);
                    }
                    break;
                }
                case static_cast<std::uint16_t>(SubjectID::PnPNodeIDAllocationData_v2):
                {
                    if (const auto res = rx_v2_.update(*m))
                    {
                        return acceptResponseV2(res->first, res->second);
                    }
                    break;
                }
                default:
                {
                    break;
                }
                }
            }
        }
        return nullptr;
    }

    [[nodiscard]] auto acceptResponseV1(const std::size_t body_size, const std::uint8_t* const body) -> IActivity*
    {
        if (body_size < 9)
        {
            return nullptr;
        }
        std::uint64_t received_pseudo_uid = 0;
        for (auto i = 0U; i < 6; i++)
        {
            received_pseudo_uid |= std::uint64_t(body[i]) << (8U * i);
        }
        if (pseudo_uid_ != received_pseudo_uid)
        {
            return nullptr;  // UID mismatch
        }
        if (const auto array_length = body[6]; array_length != 1)
        {
            return nullptr;  // Unexpected array length
        }
        const auto allocated_node_id = static_cast<std::uint16_t>(static_cast<std::uint16_t>(body[8] << 8U) | body[7]);
        if (allocated_node_id > 127)
        {
            return nullptr;  // Invalid for CAN
        }
        return constructSuccessor(static_cast<uint8_t>(allocated_node_id));
    }

    [[nodiscard]] auto acceptResponseV2(const std::size_t body_size, const std::uint8_t* const body) -> IActivity*
    {
        if (body_size < 18)
        {
            return nullptr;
        }
        const auto allocated_node_id = static_cast<std::uint16_t>(static_cast<std::uint16_t>(body[1] << 8U) | body[0]);
        if ((allocated_node_id < 128) && (0 == std::memcmp(&body[2], local_uid_.data(), local_uid_.size())))
        {
            return constructSuccessor(static_cast<uint8_t>(allocated_node_id));
        }
        return nullptr;
    }

    void handleDeadline()
    {
        if (bus_mode_ != ICANDriver::Mode::Classic)
        {
            if (version_toggle_)
            {
                publishRequestV1();
                tx_transfer_id_++;
            }
            else
            {
                publishRequestV2();
            }
            version_toggle_ = !version_toggle_;
        }
        else
        {
            publishRequestV1();
            tx_transfer_id_++;
        }
    }

    void publishRequestV1()
    {
        const std::array<std::uint8_t, 8> buf{{
            static_cast<std::uint8_t>(pseudo_uid_ >> 0U),
            static_cast<std::uint8_t>(pseudo_uid_ >> 8U),
            static_cast<std::uint8_t>(pseudo_uid_ >> 16U),
            static_cast<std::uint8_t>(pseudo_uid_ >> 24U),
            static_cast<std::uint8_t>(pseudo_uid_ >> 32U),
            static_cast<std::uint8_t>(pseudo_uid_ >> 40U),
            0,
            static_cast<std::uint8_t>(0b1110'0000U | tx_transfer_id_),
        }};
        (void) send(true, SubjectID::PnPNodeIDAllocationData_v1, buf);
    }

    void publishRequestV2()
    {
        assert(bus_mode_ != ICANDriver::Mode::Classic);
        std::array<std::uint8_t, 20> buf{};
        buf.at(0) = std::numeric_limits<std::uint8_t>::max();
        buf.at(1) = std::numeric_limits<std::uint8_t>::max();
        std::copy(local_uid_.begin(), local_uid_.end(), &buf.at(2));
        buf.back() = static_cast<std::uint8_t>(0b1110'0000U | tx_transfer_id_);
        (void) send(false, SubjectID::PnPNodeIDAllocationData_v2, buf);
    }

    template <std::size_t PayloadSize>
    [[nodoscard]] auto send(const bool                                   force_classic_can,
                            const SubjectID                              sid,
                            const std::array<std::uint8_t, PayloadSize>& payload) -> bool
    {
        static constexpr std::uint32_t CANIDMask = 0b110'01'0110000000000000'0'0000000UL;
        static_assert(PayloadSize <= 64);
        CRC16CCITT discriminator_crc;
        discriminator_crc.update(PayloadSize, payload.data());
        const std::uint32_t can_id =
            CANIDMask | (static_cast<std::uint32_t>(sid) << 8U) | (discriminator_crc.get() & 0x7FU);
        return driver_.push(force_classic_can, can_id, PayloadSize, payload.data());
    }

    [[nodiscard]] auto constructSuccessor(const std::uint8_t allocated_node_id) -> IActivity*
    {
        if (const auto bus_mode = driver_.configure(bitrate_, false, makeAcceptanceFilter<1>(allocated_node_id)))
        {
            return allocator_.construct<V1MainActivity>(allocator_, driver_, local_uid_, *bus_mode, allocated_node_id);
        }
        return nullptr;
    }

    static auto getNextPeriod() -> std::chrono::microseconds
    {
        const auto randomized = (std::rand() * MaxPeriod.count()) / RAND_MAX;  // NOSONAR rand() ok
        assert((0 <= randomized) && (randomized <= MaxPeriod.count()));
        return std::max(std::chrono::microseconds(1), std::chrono::microseconds(randomized));
    }

    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;
    const std::uint64_t        pseudo_uid_;
    const ICANDriver::Bitrate  bitrate_;
    const ICANDriver::Mode     bus_mode_;

    SimplifiedMessageTransferReassembler<9>  rx_v1_;
    SimplifiedMessageTransferReassembler<18> rx_v2_;

    std::chrono::microseconds deadline_{getNextPeriod()};
    std::uint8_t              tx_transfer_id_ = 0;
    bool                      version_toggle_ = false;
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
        if (highest_version_seen_ && (uptime > deadline_))
        {
            return constructSuccessor(*highest_version_seen_);
        }
        ICANDriver::PayloadBuffer buf{};
        while (const auto frame = driver_.pop(buf))
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
        return nullptr;
    }

private:
    [[nodiscard]] static auto tryDetectProtocolVersionFromFrame(const std::uint32_t can_id,
                                                                const std::uint8_t  tail_byte)
        -> std::optional<std::uint8_t>
    {
        // CAN ID is not validated at the moment. This may be improved in the future to avoid misdetection if there
        // are other protocols besides UAVCAN on the same network.
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
            if (driver_.configure(bitrate_, false, makeAcceptanceFilter<0>({})))
            {
                return allocator_.construct<V0NodeIDAllocationActivity>(allocator_, driver_, local_uid_, bitrate_);
            }
        }
        if (1 == detected_protocol_version)
        {
            if (const auto bus_mode = driver_.configure(bitrate_, false, makeAcceptanceFilter<1>({})))
            {
                return allocator_.construct<V1NodeIDAllocationActivity>(allocator_,
                                                                        driver_,
                                                                        local_uid_,
                                                                        bitrate_,
                                                                        *bus_mode);
            }
        }
        return nullptr;
    }

    /// Heartbeats are exchanged at 1 Hz, so about one second should be enough to determine which versions are used.
    static constexpr std::chrono::microseconds ListeningPeriod{1'100'000};

    IAllocator&                 allocator_;
    ICANDriver&                 driver_;
    const SystemInfo::UniqueID  local_uid_;
    const ICANDriver::Bitrate   bitrate_;
    std::optional<std::uint8_t> highest_version_seen_;
    std::chrono::microseconds   deadline_{};
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
            const auto bitrate = ICANDriver::StandardBitrates.at(setting_index_ % ICANDriver::StandardBitrates.size());
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

    /// Heartbeats are exchanged at 1 Hz, so about one second should be enough to determine if the bit rate is
    /// correct.
    static constexpr std::chrono::microseconds ListeningPeriod{1'100'000};

    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;

    std::optional<ICANDriver::Mode> bus_mode_;
    std::size_t                     setting_index_ = ICANDriver::StandardBitrates.size() - 1U;
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
        if ((activity_ == nullptr) && can_bitrate &&     //
            uavcan_version && (*uavcan_version == 0) &&  //
            local_node_id && (*local_node_id > 0) && (*local_node_id < 128))
        {
            if (const auto bus_mode =
                    driver.configure(*can_bitrate, false, detail::makeAcceptanceFilter<0>(*local_node_id)))
            {
                (void) bus_mode;  // v0 doesn't care about mode because it only supports Classic CAN.
                activity_ = activity_allocator_.construct<detail::V0MainActivity>(activity_allocator_,
                                                                                  driver,
                                                                                  local_unique_id,
                                                                                  *local_node_id);
            }
        }
        if ((activity_ == nullptr) && can_bitrate &&     //
            uavcan_version && (*uavcan_version == 1) &&  //
            local_node_id && (*local_node_id < 128))
        {
            if (const auto bus_mode =
                    driver.configure(*can_bitrate, false, detail::makeAcceptanceFilter<1>(*local_node_id)))
            {
                activity_ = activity_allocator_.construct<detail::V1MainActivity>(activity_allocator_,
                                                                                  driver,
                                                                                  local_unique_id,
                                                                                  *bus_mode,
                                                                                  *local_node_id);
            }
        }
        if ((activity_ == nullptr) && can_bitrate && uavcan_version && (*uavcan_version == 0))
        {
            if (const auto bus_mode = driver.configure(*can_bitrate, false, detail::makeAcceptanceFilter<0>({})))
            {
                (void) bus_mode;  // v0 doesn't care about mode because it only supports Classic CAN.
                activity_ = activity_allocator_.construct<detail::V0NodeIDAllocationActivity>(activity_allocator_,
                                                                                              driver,
                                                                                              local_unique_id,
                                                                                              *can_bitrate);
            }
        }
        if ((activity_ == nullptr) && can_bitrate && uavcan_version && (*uavcan_version == 1))
        {
            if (const auto bus_mode = driver.configure(*can_bitrate, false, detail::makeAcceptanceFilter<1>({})))
            {
                activity_ = activity_allocator_.construct<detail::V1NodeIDAllocationActivity>(activity_allocator_,
                                                                                              driver,
                                                                                              local_unique_id,
                                                                                              *can_bitrate,
                                                                                              *bus_mode);
            }
        }
        if ((activity_ == nullptr) && can_bitrate)
        {
            if (const auto bus_mode =
                    driver.configure(*can_bitrate, true, CANAcceptanceFilterConfig::makePromiscuous()))
            {
                (void) bus_mode;  // The protocol version detection task doesn't care about the bus mode.
                activity_ = activity_allocator_.construct<detail::ProtocolVersionDetectionActivity>(activity_allocator_,
                                                                                                    driver,
                                                                                                    local_unique_id,
                                                                                                    *can_bitrate);
            }
        }
        if (activity_ == nullptr)
        {
            activity_ = activity_allocator_.construct<detail::BitrateDetectionActivity>(activity_allocator_,
                                                                                        driver,
                                                                                        local_unique_id);
        }
        assert(activity_ != nullptr);
    }

private:
    void poll(IReactor& reactor, const std::chrono::microseconds uptime) override
    {
        assert(uptime.count() >= 0);
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
