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
static constexpr std::uint8_t MaxNodeID = 127U;

/// Outgoing frames should be canceled if they could not be emitted in this time.
static constexpr std::chrono::microseconds SendTimeout(1'000'000);

/// Frames that are not extended data frames shall always be rejected.
struct CANAcceptanceFilterConfig
{
    std::uint32_t extended_can_id = AllSet;
    std::uint32_t mask            = AllSet;

    static constexpr std::uint32_t AllSet = (1ULL << 29U) - 1U;

    /// Constructs a filter that accepts all extended data frames.
    /// The mask=0 means that no bits matter. The ID is set to a non-zero value because some CAN drivers treat
    /// the zero-zero configuration as "reject everything" rather than "accept everything".
    [[nodiscard]] static auto makePromiscuous() -> CANAcceptanceFilterConfig { return {AllSet, 0}; }

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

    /// The length is rounded UP to the next valid DLC.
    constexpr static std::array<std::uint8_t, 65> LengthToDLC{{
        0,  1,  2,  3,  4,  5,  6,  7,  8,                               // 0-8
        9,  9,  9,  9,                                                   // 9-12
        10, 10, 10, 10,                                                  // 13-16
        11, 11, 11, 11,                                                  // 17-20
        12, 12, 12, 12,                                                  // 21-24
        13, 13, 13, 13, 13, 13, 13, 13,                                  // 25-32
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,  // 33-48
        15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,  // 49-64
    }};
    constexpr static std::array<std::uint8_t, 16> DLCToLength{{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64}};
};

/// This is an isolated component intended for use in embedded applications for queueing TX CAN frames.
/// Systems that leverage higher-level CAN backends (like SocketCAN in Zephyr/NuttX/Linux) will not need this.
/// Systems that run on bare metal probably will.
/// Assuming a constant-complexity heap, all methods are constant-complexity instead of push(),
/// which is linear of the queue depth. The recommended heap is https://github.com/pavel-kirienko/o1heap.
/// The template arguments are as follows:
///     MemAllocate(std::size_t) -> void*   // nullptr if out of memory
///     MemDeallocate(void*) -> void
template <typename MemAllocate, typename MemDeallocate>
class TxQueue final
{
public:
    struct Item
    {
        std::chrono::microseconds timestamp{};
        bool                      force_classic_can{};
        std::uint32_t             extended_can_id{};
        std::size_t               payload_size{};
        const std::uint8_t*       payload{};
    };

    TxQueue(const MemAllocate& alloc, const MemDeallocate& dealloc) : alloc_(alloc), dealloc_(dealloc) {}

    ~TxQueue() { clear(); }

    TxQueue(const TxQueue&) = delete;
    TxQueue(TxQueue&&)      = delete;
    auto operator=(const TxQueue&) -> TxQueue& = delete;
    auto operator=(TxQueue&&) -> TxQueue& = delete;

    /// The timestamp of the frame is the time when it is enqueued (created).
    /// It is kept in the queue to allow the caller to determine how long each frame was waiting for.
    /// When reading frames back using peek(), the caller should check if the frame has timed out by subtracting
    /// its timestamp from the current time. The default timeout of one second is adequate.
    /// Returns true on success, false on OOM.
    [[nodiscard]] auto push(const std::chrono::microseconds timestamp,
                            const bool                      force_classic_can,
                            const std::uint32_t             extended_can_id,
                            const std::size_t               payload_size,
                            const void* const               payload) -> bool
    {
        if (void* const raw_ptr = alloc_(std::size_t(sizeof(ListItem) + payload_size)))
        {
            auto* const payload_storage = static_cast<std::uint8_t*>(raw_ptr) + sizeof(ListItem);
            (void) std::memmove(payload_storage, payload, payload_size);
            auto* const item        = new (raw_ptr) ListItem{};
            item->timestamp         = timestamp;
            item->force_classic_can = force_classic_can;
            item->extended_can_id   = extended_can_id;
            item->payload_size      = payload_size;
            item->payload           = payload_storage;
            ListItem* li            = list_;
            if ((li == nullptr) || (li->extended_can_id > extended_can_id))
            {
                item->next = li;
                list_      = item;
            }
            else
            {
                while ((li != nullptr) && (li->next != nullptr) && (li->next->extended_can_id <= extended_can_id))
                {
                    li = li->next;
                }
                item->next = li->next;
                li->next   = item;
            }
            depth_++;
            return true;
        }
        return false;
    }

    /// View the next frame to transmit (top of the queue). Call pop() after you're done with this frame.
    /// Be sure to check the timestamp and discard the frame if it is already expired.
    /// Returns nullptr if the queue is empty.
    [[nodiscard]] auto peek() const -> const Item* { return list_; }

    /// Remove the frame returned by peek() from the top of the queue. Invalidates the pointer returned by peek().
    /// Has no effect if the queue is empty.
    void pop()
    {
        if (list_ != nullptr)
        {
            auto* const d = list_;
            list_         = list_->next;
            dealloc_(d);
            assert(depth_ > 0);
            depth_--;
        }
    }

    /// Remove all elements from the queue.
    void clear()
    {
        while (list_ != nullptr)
        {
            pop();
        }
    }

    /// Number of frames that are currently held in the queue.
    [[nodiscard]] auto size() const -> std::size_t { return depth_; }

private:
    struct ListItem final : public Item
    {
        ListItem* next = nullptr;
    };

    MemAllocate   alloc_;
    MemDeallocate dealloc_;
    ListItem*     list_  = nullptr;
    std::size_t   depth_ = 0;
};

namespace detail
{
using kocherga::detail::BitsPerByte;

static constexpr std::uint8_t TailByteStartOfTransfer = 0b1000'0000;
static constexpr std::uint8_t TailByteEndOfTransfer   = 0b0100'0000;
static constexpr std::uint8_t TailByteToggleBit       = 0b0010'0000;

static constexpr std::uint8_t MaxTransferID = 31;

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
    CRC64 crc;
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
    PortID                      subject_id = std::numeric_limits<PortID>::max();
    std::optional<std::uint8_t> source_node_id;
};

struct ServiceFrameModel : public FrameModel
{
    PortID       service_id           = std::numeric_limits<PortID>::max();
    std::uint8_t source_node_id       = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t destination_node_id  = std::numeric_limits<std::uint8_t>::max();
    bool         request_not_response = false;
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
    const bool         start_of_transfer = (tail & TailByteStartOfTransfer) != 0;
    const bool         end_of_transfer   = (tail & TailByteEndOfTransfer) != 0;
    const bool         toggle            = (tail & TailByteToggleBit) != 0;
    const std::uint8_t transfer_id       = (tail & MaxTransferID);
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

/// This function refers to the "data type ID" as "subject/service-ID" for reasons of unification with v1.
[[nodiscard]] inline auto parseFrameV0(const std::uint32_t extended_can_id,
                                       const std::size_t   payload_size,
                                       const void* const   payload)
    -> std::optional<std::variant<MessageFrameModel, ServiceFrameModel>>
{
    assert(payload != nullptr);
    if ((payload_size < 1) || (payload_size > 8))  // Legacy UAVCAN v0 is compatible only with Classic CAN.
    {                                              // This is because the low granularity of DLC in CAN FD breaks TAO.
        return {};
    }
    const auto         out_payload_size  = static_cast<std::uint8_t>(payload_size - 1U);
    const std::uint8_t tail              = *((static_cast<const uint8_t*>(payload)) + out_payload_size);  // NOSONAR
    const bool         start_of_transfer = (tail & TailByteStartOfTransfer) != 0;
    const bool         end_of_transfer   = (tail & TailByteEndOfTransfer) != 0;
    const bool         toggle            = (tail & TailByteToggleBit) != 0;
    const std::uint8_t transfer_id       = (tail & MaxTransferID);
    if (start_of_transfer && toggle)
    {
        return {};  // UAVCAN v1
    }
    const auto priority       = static_cast<std::uint8_t>((extended_can_id >> 24U) & 31U);
    const auto source_node_id = static_cast<std::uint8_t>(extended_can_id & 0x7FU);
    if (const auto is_message = (0 == (extended_can_id & (1ULL << 7U))); is_message)
    {
        MessageFrameModel out{};
        out.priority          = priority;
        out.transfer_id       = transfer_id;
        out.start_of_transfer = start_of_transfer;
        out.end_of_transfer   = end_of_transfer;
        out.toggle            = toggle;
        out.payload_size      = out_payload_size;
        out.payload           = static_cast<const uint8_t*>(payload);
        if (source_node_id != 0)
        {
            out.source_node_id = source_node_id;
            out.subject_id     = static_cast<std::uint16_t>((extended_can_id >> 8U) & 0xFFFFU);
        }
        else
        {
            if (!(start_of_transfer && end_of_transfer))
            {
                return {};  // Anonymous transfers can be only single-frame transfers.
            }
            out.subject_id = static_cast<std::uint16_t>((extended_can_id >> 8U) & 0b11U);  // Ignore the discriminator.
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
    out.service_id           = static_cast<std::uint16_t>((extended_can_id >> 16U) & 0xFFU);
    out.source_node_id       = source_node_id;
    out.destination_node_id  = static_cast<std::uint8_t>((extended_can_id >> 8U) & 0x7FU);
    out.request_not_response = 0 != (extended_can_id & (1ULL << 15U));
    if ((out.source_node_id == out.destination_node_id) || (out.source_node_id == 0) || (out.destination_node_id == 0))
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
            if ((source == source_node_id_) && (frame.transfer_id == transfer_id_))
            {
                return {};  // Drop the duplicate.
            }
            start(source, frame.transfer_id);
        }
        else
        {
            const bool match = (state_) &&                             //
                               (source == source_node_id_) &&          //
                               (frame.transfer_id == transfer_id_) &&  //
                               (frame.toggle == !state_->toggle);
            if (!match)
            {
                return {};
            }
            state_->toggle = !state_->toggle;
        }
        state_->crc.update(frame.payload_size, frame.payload);
        const auto sz = std::min(frame.payload_size, payload_.size() - state_->stored_payload_size);
        std::copy_n(frame.payload, sz, payload_.begin() + state_->stored_payload_size);
        state_->stored_payload_size += sz;
        state_->received_payload_size += frame.payload_size;
        if (frame.end_of_transfer)
        {
            const TransferState final = *state_;
            state_.reset();
            if (frame.start_of_transfer)  // This is a single-frame transfer.
            {
                return Result{final.stored_payload_size, payload_.data()};
            }
            if ((final.received_payload_size >= CRC16CCITT::Size) && final.crc.isResidueCorrect())
            {
                return Result{std::min(final.stored_payload_size, final.received_payload_size - CRC16CCITT::Size),
                              payload_.data()};
            }
        }
        return {};
    }

    std::array<std::uint8_t, Extent> payload_{};

private:
    void start(const std::uint8_t source_node_id, const std::uint8_t transfer_id)
    {
        state_.emplace();
        source_node_id_ = source_node_id;
        transfer_id_    = transfer_id;
    }

    std::uint8_t source_node_id_ = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t transfer_id_    = std::numeric_limits<std::uint8_t>::max();

    struct TransferState
    {
        std::size_t stored_payload_size   = 0;
        std::size_t received_payload_size = 0;
        CRC16CCITT  crc;
        bool        toggle = true;
    };
    std::optional<TransferState> state_;
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

/// This is like the above but for the legacy v0 protocol.
/// Unlike the v1 implementation, this one does not implement implicit payload truncation as it is not defined for v0.
template <std::size_t MaxPayloadSize>
class SimplifiedTransferReassemblerV0
{
public:
    using Result = std::pair<std::size_t, const std::uint8_t*>;

protected:
    explicit SimplifiedTransferReassemblerV0(const std::uint64_t signature) : signature_(signature) {}

    /// The payload pointer in the result remains valid until the next update.
    [[nodiscard]] auto updateImpl(const FrameModel& frame, const std::uint8_t source) -> std::optional<Result>
    {
        if (frame.start_of_transfer)
        {
            if ((source == source_node_id_) && (frame.transfer_id == transfer_id_))
            {
                return {};  // Drop the duplicate.
            }
            start(source, frame.transfer_id);
        }
        else
        {
            if (!((state_) && (source == source_node_id_) && (frame.transfer_id == transfer_id_) &&
                  (frame.toggle == !state_->toggle)))
            {
                return {};
            }
            state_->toggle = !state_->toggle;
        }
        if (frame.payload_size > (buffer_.size() - state_->payload_size))
        {
            state_.reset();  // Too much payload -- UAVCAN v0 does not define payload truncation.
            return {};
        }
        std::copy_n(frame.payload, frame.payload_size, buffer_.begin() + state_->payload_size);
        state_->payload_size += frame.payload_size;
        if (frame.end_of_transfer)
        {
            const TransferState final = *state_;
            state_.reset();
            if (frame.start_of_transfer)  // This is a single-frame transfer.
            {
                return Result{final.payload_size, buffer_.data()};
            }
            if (final.payload_size >= CRC16CCITT::Size)
            {
                CRC16CCITT crc;
                for (auto i = 0U; i < 8U; i++)
                {
                    crc.update(static_cast<std::uint8_t>((signature_ >> (i * 8U)) & 0xFFU));
                }
                crc.update(final.payload_size - CRC16CCITT::Size, buffer_.begin() + CRC16CCITT::Size);
                if ((buffer_.at(0) == (crc.get() & 0xFFU)) && (buffer_.at(1) == (crc.get() >> 8U)))
                {
                    return Result{final.payload_size - CRC16CCITT::Size, buffer_.begin() + CRC16CCITT::Size};
                }
            }
        }
        return {};
    }

    std::array<std::uint8_t, MaxPayloadSize + CRC16CCITT::Size> buffer_{};

private:
    void start(const std::uint8_t source_node_id, const std::uint8_t transfer_id)
    {
        state_.emplace();
        source_node_id_ = source_node_id;
        transfer_id_    = transfer_id;
    }

    std::uint8_t source_node_id_ = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t transfer_id_    = std::numeric_limits<std::uint8_t>::max();

    struct TransferState
    {
        std::size_t payload_size = 0;
        bool        toggle       = false;
    };
    std::optional<TransferState> state_;

    const std::uint64_t signature_;
};

/// The user of this class is responsible for checking the subject-ID on the received frames.
template <std::size_t MaxPayloadSize>
class SimplifiedMessageTransferReassemblerV0 : public SimplifiedTransferReassemblerV0<MaxPayloadSize>
{
    using Base = SimplifiedTransferReassemblerV0<MaxPayloadSize>;

public:
    using typename Base::Result;

    explicit SimplifiedMessageTransferReassemblerV0(const std::uint64_t signature) : Base(signature) {}

    /// The payload pointer in the result remains valid until the next update.
    [[nodiscard]] auto update(const MessageFrameModel& frame) -> std::optional<Result>
    {
        if (!frame.source_node_id)  // Anonymous frames accepted unconditionally.
        {
            const auto sz = std::min(frame.payload_size, buffer_.size());
            std::copy_n(frame.payload, sz, buffer_.data());
            return Result{sz, buffer_.data()};
        }
        return Base::updateImpl(frame, *frame.source_node_id);
    }

private:
    using Base::buffer_;
};

/// The user of this class is responsible for checking the service-ID and request/response flag on the received frames.
template <std::size_t MaxPayloadSize>
class SimplifiedServiceTransferReassemblerV0 : public SimplifiedTransferReassemblerV0<MaxPayloadSize>
{
    using Base = SimplifiedTransferReassemblerV0<MaxPayloadSize>;

public:
    using typename Base::Result;

    SimplifiedServiceTransferReassemblerV0(const std::uint64_t signature, const std::uint8_t local_node_id) :
        Base(signature), local_node_id_(local_node_id)
    {}

    /// The payload pointer in the result remains valid until the next update.
    [[nodiscard]] auto update(const ServiceFrameModel& frame) -> std::optional<Result>
    {
        if (local_node_id_ == frame.destination_node_id)
        {
            return Base::updateImpl(frame, frame.source_node_id);
        }
        return {};
    }

private:
    const std::uint8_t local_node_id_;
};

/// Send one UAVCAN/CAN v1 transfer. The push_frame callback is invoked per each transmitted frame; its type should be:
///     (std::size_t, const std::uint8_t*) -> bool
/// The return value is true on success, false otherwise.
/// The callback shall not be an std::function<> or std::bind<> to avoid heap allocation.
/// The MTU shall be representable as a valid DLC and be no less than 8 bytes.
/// The caller is responsible for computing the CAN ID (which shall be the same for all frames of this transfer).
template <typename Callback>
[[nodiscard]] static inline auto transmit(const Callback&           push_frame,
                                          const std::size_t         transport_layer_mtu,
                                          const std::uint8_t        transfer_id,
                                          const std::size_t         payload_length,
                                          const std::uint8_t* const payload) -> bool
{
    ICANDriver::PayloadBuffer buf{};
    if ((transport_layer_mtu < 8U) || (transport_layer_mtu > buf.size()) || (transfer_id > MaxTransferID) ||
        (ICANDriver::DLCToLength.at(ICANDriver::LengthToDLC.at(transport_layer_mtu)) != transport_layer_mtu) ||
        ((payload == nullptr) && (payload_length > 0)))
    {
        return false;
    }
    const std::size_t mtu = transport_layer_mtu - 1U;
    assert((mtu >= 7U) && (mtu <= 63U));

    // SINGLE-FRAME TRANSFER
    if (payload_length <= mtu)
    {
        std::copy_n(payload, payload_length, buf.begin());
        const auto dlc = ICANDriver::LengthToDLC.at(payload_length + 1);
        const auto len = ICANDriver::DLCToLength.at(dlc);
        assert(len > 0);
        buf.at(len - 1U) = static_cast<std::uint8_t>(static_cast<std::uint32_t>(TailByteStartOfTransfer) |
                                                     TailByteEndOfTransfer | TailByteToggleBit | transfer_id);
        return push_frame(len, buf.data());
    }

    // MULTI-FRAME TRANSFER
    CRC16CCITT crc;
    crc.update(payload_length, payload);
    std::size_t         remaining = payload_length;
    const std::uint8_t* ptr       = payload;
    bool                toggle    = true;
    while (remaining > mtu)
    {
        std::copy_n(ptr, mtu, buf.begin());
        const auto tail_byte =
            static_cast<std::uint8_t>(transfer_id |                        //
                                      (toggle ? TailByteToggleBit : 0U) |  //
                                      ((remaining == payload_length) ? TailByteStartOfTransfer : 0U));
        toggle      = !toggle;
        buf.at(mtu) = tail_byte;
        if (!push_frame(mtu + 1U, buf.data()))
        {
            return false;
        }
        ptr += mtu;
        remaining -= mtu;
    }
    // Last frame requires special treatment: the CRC may go into the same frame or be split off, this is convoluted.
    std::copy_n(ptr, remaining, buf.begin());
    if ((remaining + CRC16CCITT::Size) <= mtu)  // Remaining bytes plus CRC fit into one frame; padding may be needed.
    {
        auto*      dst = buf.begin() + remaining;
        const auto dlc = ICANDriver::LengthToDLC.at(remaining + CRC16CCITT::Size + 1U);
        const auto len = ICANDriver::DLCToLength.at(dlc);
        assert(len >= (remaining + CRC16CCITT::Size + 1U));
        const auto padding = len - remaining - CRC16CCITT::Size - 1U;
        assert(padding < 16);
        for (auto i = 0U; i < padding; i++)
        {
            crc.update(0);
            *dst++ = 0;
        }
        for (auto x : crc.getBytes())
        {
            *dst++ = x;
        }
        *dst = static_cast<std::uint8_t>(transfer_id | (toggle ? TailByteToggleBit : 0U) | TailByteEndOfTransfer);
        return push_frame(len, buf.data());
    }
    // Padding is not needed but one extra frame is required to contain (part of) the CRC.
    const auto  crc_bytes    = crc.getBytes();
    const auto* crc_bytes_it = crc_bytes.begin();
    assert(mtu >= remaining);
    for (auto i = remaining; i < mtu; i++)
    {
        buf.at(i) = *crc_bytes_it++;
    }
    buf.at(mtu) = static_cast<std::uint8_t>(transfer_id | (toggle ? TailByteToggleBit : 0U));
    toggle      = !toggle;
    if (!push_frame(mtu + 1U, buf.data()))
    {
        return false;
    }
    auto* buf_it = std::copy(crc_bytes_it, crc_bytes.end(), buf.begin());
    *buf_it++    = static_cast<std::uint8_t>(transfer_id | (toggle ? TailByteToggleBit : 0U) | TailByteEndOfTransfer);
    const auto size = buf_it - buf.begin();
    assert((size >= 2) && (size <= static_cast<std::int32_t>(CRC16CCITT::Size + 1U)));
    return push_frame(static_cast<std::uint8_t>(size), buf.data());
}

/// This is like transmit() but for the legacy v0 protocol.
/// It is substantially simpler because v0 does not need padding and the CRC is located in the first frame.
template <typename Callback>
[[nodiscard]] static inline auto transmitV0(const Callback&           push_frame,
                                            const std::uint64_t       signature,
                                            const std::uint8_t        transfer_id,
                                            const std::size_t         payload_length,
                                            const std::uint8_t* const payload) -> bool
{
    static constexpr auto              MTU = 7U;
    std::array<std::uint8_t, MTU + 1U> buf{};
    if ((transfer_id > MaxTransferID) || ((payload == nullptr) && (payload_length > 0)))
    {
        return false;
    }
    // SINGLE-FRAME TRANSFER
    if (payload_length <= MTU)
    {
        std::copy_n(payload, payload_length, buf.begin());
        buf.at(payload_length) = static_cast<std::uint8_t>(static_cast<std::uint32_t>(TailByteStartOfTransfer) |
                                                           TailByteEndOfTransfer | transfer_id);
        return push_frame(payload_length + 1, buf.data());
    }
    // MULTI-FRAME TRANSFER
    CRC16CCITT crc;
    for (auto i = 0U; i < 8U; i++)
    {
        crc.update(static_cast<std::uint8_t>((signature >> (i * 8U)) & 0xFFU));
    }
    crc.update(payload_length, payload);
    std::size_t         remaining = payload_length;
    const std::uint8_t* ptr       = payload;
    // First frame
    buf.at(0) = static_cast<std::uint8_t>(crc.get() >> 0U);
    buf.at(1) = static_cast<std::uint8_t>(crc.get() >> 8U);
    std::copy_n(ptr, MTU - CRC16CCITT::Size, buf.begin() + CRC16CCITT::Size);
    buf.at(MTU) = static_cast<std::uint8_t>(transfer_id | TailByteStartOfTransfer);
    ptr += MTU - CRC16CCITT::Size;
    remaining -= MTU - CRC16CCITT::Size;
    if (!push_frame(MTU + 1U, buf.data()))
    {
        return false;
    }
    // Middle frames
    bool toggle = true;
    while (remaining > MTU)
    {
        std::copy_n(ptr, MTU, buf.begin());
        buf.at(MTU) = static_cast<std::uint8_t>(transfer_id | (toggle ? TailByteToggleBit : 0U));
        if (!push_frame(MTU + 1U, buf.data()))
        {
            return false;
        }
        toggle = !toggle;
        ptr += MTU;
        remaining -= MTU;
    }
    // Last frame
    assert(remaining <= MTU);
    std::copy_n(ptr, remaining, buf.begin());
    buf.at(remaining) =
        static_cast<std::uint8_t>(transfer_id | (toggle ? TailByteToggleBit : 0U) | TailByteEndOfTransfer);
    return push_frame(static_cast<std::uint8_t>(remaining + 1U), buf.data());
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

/// The v0 main activity is a simplified translation layer between UAVCAN/CAN v1 and UAVCAN v0.
/// The following ports are supported:
///
///     Service                 RX                  TX
///     -----------------------------------------------------------------------------
///     FileRead                MFT response        MFT request
///     GetInfo                 SFT request         MFT response
///     ExecuteCommand          MFT request         SFT response
///     Heartbeat                                   SFT
class V0MainActivity : public IActivity
{
public:
    V0MainActivity(ICANDriver& driver, const std::uint8_t local_node_id) :
        driver_(driver), local_node_id_(local_node_id)
    {
        assert((local_node_id_ > 0) && (local_node_id_ <= MaxNodeID));
    }

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) uptime;
        ICANDriver::PayloadBuffer buf{};
        while (const auto transport_frame = driver_.pop(buf))
        {
            const auto [can_id, payload_size] = *transport_frame;
            if (const auto frame = detail::parseFrameV0(can_id, payload_size, buf.data()))
            {
                if (const auto* const s = std::get_if<ServiceFrameModel>(&*frame))
                {
                    processReceivedServiceFrame(reactor, *s);
                }
                // This implementation is not interested in accepting any message transfers.
            }
        }
        return nullptr;
    }

    [[nodiscard]] auto getLocalNodeID() const -> std::uint8_t { return local_node_id_; }

private:
    [[nodiscard]] auto sendRequest(const ServiceID           service_id,
                                   const NodeID              server_node_id,
                                   const TransferID          transfer_id,
                                   const std::size_t         payload_length,
                                   const std::uint8_t* const payload) -> bool override
    {
        if ((server_node_id > 0) && (server_node_id <= MaxNodeID))
        {
            if (service_id == ServiceID::FileRead)
            {
                return sendFileReadRequest(server_node_id,
                                           static_cast<std::uint8_t>(transfer_id & MaxTransferID),
                                           payload_length,
                                           payload);
            }
        }
        return false;  // Don't know how to translate this request v1 --> v0.
    }

    [[nodiscard]] auto sendResponse(const std::uint64_t       signature,
                                    const std::uint8_t        priority,
                                    const std::uint16_t       service_id,
                                    const std::uint8_t        client_node_id,
                                    const std::uint8_t        transfer_id,
                                    const std::size_t         payload_length,
                                    const std::uint8_t* const payload) -> bool
    {
        static constexpr std::uint32_t CANIDMask = 0b00000'00000000'0'0000000'1'0000000UL;
        //
        const auto can_id = CANIDMask |                                           //
                            (static_cast<std::uint32_t>(priority) << 24U) |       //
                            (static_cast<std::uint32_t>(service_id) << 16U) |     //
                            (static_cast<std::uint32_t>(client_node_id) << 8U) |  //
                            local_node_id_;
        return send(signature, can_id, transfer_id, payload_length, payload);
    }

    void cancelRequest() override { pending_request_meta_.reset(); }

    [[nodiscard]] auto publishMessage(const SubjectID           subject_id,
                                      const TransferID          transfer_id,
                                      const std::size_t         payload_length,
                                      const std::uint8_t* const payload) -> bool override
    {
        if (subject_id == SubjectID::NodeHeartbeat)
        {
            return publishHeartbeat(transfer_id, payload_length, payload);
        }
        if (subject_id == SubjectID::DiagnosticRecord)
        {
            return publishDiagnosticRecord(transfer_id, payload_length, payload);
        }
        return false;  // Don't know how to translate this message v1 --> v0.
    }

    void processReceivedServiceFrame(IReactor& reactor, const ServiceFrameModel& frame)
    {
        if (frame.request_not_response)
        {
            if (frame.service_id == static_cast<std::uint16_t>(ServiceTypeID::GetNodeInfo))
            {
                if (const auto req = rx_req_get_node_info_.update(frame))
                {
                    processGetNodeInfoRequest(reactor, frame, req->first, req->second);
                }
            }
            if (frame.service_id == static_cast<std::uint16_t>(ServiceTypeID::BeginFirmwareUpdate))
            {
                if (const auto req = rx_req_begin_fw_upd_.update(frame))
                {
                    processBeginFirmwareUpdateRequest(reactor, frame, req->first, req->second);
                }
            }
        }
        else
        {
            if (pending_request_meta_ &&                                            //
                (pending_request_meta_->server_node_id == frame.source_node_id) &&  //
                (pending_request_meta_->service_id == frame.service_id) &&          //
                (pending_request_meta_->transfer_id == frame.transfer_id))
            {
                if (frame.service_id == static_cast<std::uint16_t>(ServiceTypeID::FileRead))
                {
                    if (const auto res = rx_res_file_read_.update(frame))
                    {
                        pending_request_meta_.reset();
                        processFileReadResponse(reactor, res->first, res->second);
                    }
                }
                else
                {
                    assert(false);  // This means that we've sent a request for which there is no response listener.
                }
            }
        }
    }

    void processGetNodeInfoRequest(IReactor&                 reactor,
                                   const ServiceFrameModel&  frame,
                                   const std::size_t         request_size,
                                   const std::uint8_t* const request_data)
    {
        assert(frame.destination_node_id == local_node_id_);
        assert(frame.request_not_response);
        (void) request_size;  // No data is needed.
        std::array<std::uint8_t, MaxSerializedRepresentationSize> response_data{};
        if (const auto response_size = reactor.processRequest(static_cast<PortID>(ServiceID::NodeGetInfo),
                                                              frame.source_node_id,
                                                              0,
                                                              request_data,
                                                              response_data.data());
            response_size >= 31)
        {
            // Translate the response v1 -> v0.
            std::array<std::uint8_t, 128> buf{};
            buf.back()        = 0xAAU;  // Canary.
            std::uint8_t* ptr = buf.begin();
            (void) std::memcpy(ptr, last_node_status_.data(), last_node_status_.size());
            ptr += last_node_status_.size();
            *ptr++ = response_data.at(4);  // Software version
            *ptr++ = response_data.at(5);
            ptr += 13U;                    // Optional fields not available: no VCS hash, no CRC
            *ptr++ = response_data.at(2);  // Hardware version.
            *ptr++ = response_data.at(3);
            (void) std::memcpy(ptr, response_data.begin() + 14, 16);  // Unique-ID
            ptr += 17U;                                               // Skip UID and CoA
            const auto name_len = std::min<std::size_t>(response_data.at(30), 80);
            (void) std::memcpy(ptr, response_data.begin() + 31, name_len);
            assert(buf.back() == 0xAAU);  // Check the canary.
            (void) sendResponse(GetNodeInfoSignature,
                                frame.priority,
                                frame.service_id,
                                frame.source_node_id,
                                frame.transfer_id,
                                name_len + 41U,
                                buf.data());
        }
    }

    void processBeginFirmwareUpdateRequest(IReactor&                 reactor,
                                           const ServiceFrameModel&  frame,
                                           const std::size_t         request_size,
                                           const std::uint8_t* const request_data)
    {
        assert(frame.destination_node_id == local_node_id_);
        assert(frame.request_not_response);
        std::array<std::uint8_t, 210> scratchpad{};
        scratchpad.back() = 0xAA;  // Canary
        // Translate the request v0 --> v1, store into the scratchpad.
        scratchpad.at(0) = 0xFD;
        scratchpad.at(1) = 0xFF;
        if ((request_size >= 2) && (request_size <= 202))
        {
            const auto path_len = static_cast<std::uint8_t>(request_size - 1U);
            scratchpad.at(2)    = path_len;
            (void) std::memcpy(scratchpad.begin() + 3U, request_data + 1U, path_len);
            assert(scratchpad.back() == 0xAA);
            std::array<std::uint8_t, MaxSerializedRepresentationSize> response_data{};
            if (const auto response_size = reactor.processRequest(static_cast<PortID>(ServiceID::NodeExecuteCommand),
                                                                  frame.source_node_id,
                                                                  path_len + 3U,
                                                                  scratchpad.data(),
                                                                  response_data.data());
                response_size >= 1U)
            {
                // Translate the response v1 --> v0 re-using the same scratchpad.
                scratchpad.at(0) = (response_data.at(0) == 0) ? 0 : 255;  // Either ERROR_OK or ERROR_UNKNOWN.
                (void) sendResponse(BeginFirmwareUpdateSignature,
                                    frame.priority,
                                    frame.service_id,
                                    frame.source_node_id,
                                    frame.transfer_id,
                                    1U,
                                    scratchpad.data());
            }
        }
    }

    static void processFileReadResponse(IReactor&                 reactor,
                                        const std::size_t         response_size,
                                        const std::uint8_t* const response_data)
    {
        if (response_size >= 4)
        {
            std::array<std::uint8_t, 270> buf{};
            buf.back()     = 0xAA;
            buf.at(0)      = response_data[0];
            buf.at(1)      = response_data[1];
            const auto len = response_size - 2U;
            if (len <= 256U)
            {
                buf.at(2) = static_cast<std::uint8_t>(len >> 0U);
                buf.at(3) = static_cast<std::uint8_t>(len >> 8U);
                (void) std::memcpy(buf.begin() + 4, response_data + 2U, len);
                assert(buf.back() == 0xAA);
                reactor.processResponse(len + 4U, buf.data());
            }
        }
    }

    [[nodiscard]] auto sendFileReadRequest(const NodeID              server_node_id,
                                           const std::uint8_t        transfer_id,
                                           const std::size_t         payload_length,
                                           const std::uint8_t* const payload) -> bool
    {
        if (payload_length >= 6U)
        {
            // Convert v1 --> v0: apply TAO by removing the length field.
            std::array<std::uint8_t, 270> buf{};
            buf.back() = 0xAA;
            (void) std::memcpy(buf.data(), payload, 5U);
            const auto path_len = payload[5];
            (void) std::memcpy(buf.data() + 5U, payload + 6U, path_len);
            assert(buf.back() == 0xAA);
            static constexpr std::uint32_t CANIDMask       = 0b11110'00000000'1'0000000'1'0000000ULL;
            const std::uint32_t            extended_can_id = CANIDMask |
                                                  (static_cast<std::uint32_t>(ServiceTypeID::FileRead) << 16U) |
                                                  (static_cast<std::uint32_t>(server_node_id) << 8U) | local_node_id_;
            if (send(FileReadSignature, extended_can_id, transfer_id, path_len + 5U, buf.data()))
            {
                pending_request_meta_ = PendingRequestMetadata{static_cast<std::uint8_t>(server_node_id),
                                                               static_cast<std::uint16_t>(ServiceTypeID::FileRead),
                                                               transfer_id};
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] auto publishHeartbeat(const TransferID          transfer_id,
                                        const std::size_t         payload_length,
                                        const std::uint8_t* const payload) -> bool
    {
        if (payload_length < 7U)
        {
            return false;
        }
        // Translate the message from v1's Heartbeat into v0's NodeStatus.
        std::memcpy(last_node_status_.begin(), payload, 4);
        last_node_status_.at(4) = static_cast<std::uint8_t>(payload[4] << 6U) |  // Health
                                  static_cast<std::uint8_t>(payload[5] << 3U);   // Mode
        last_node_status_.at(5) = payload[6];                                    // Vendor-specific status code.
        // Publish the v0 NodeStatus
        static constexpr std::uint32_t CANIDMask = 0b10000'0000000101010101'0'0000000ULL;
        return send(NodeStatusSignature,
                    CANIDMask | local_node_id_,
                    transfer_id,
                    last_node_status_.size(),
                    last_node_status_.data());
    }

    [[nodiscard]] auto publishDiagnosticRecord(const TransferID          transfer_id,
                                               const std::size_t         payload_length,
                                               const std::uint8_t* const payload) -> bool
    {
        if (payload_length < 9U)
        {
            return false;
        }
        std::array<std::uint8_t, 128> buf{};
        buf.at(0)           = 0b010'00100U;
        buf.at(1)           = 'B';
        buf.at(2)           = 'o';
        buf.at(3)           = 'o';
        buf.at(4)           = 't';
        const auto text_len = std::min<std::size_t>(payload[8], 90);
        std::copy_n(&payload[9], text_len, buf.begin() + 5U);
        static constexpr std::uint32_t CANIDMask = 0b11111'0011111111111111'0'0000000ULL;
        return send(LogMessageSignature, CANIDMask | local_node_id_, transfer_id, text_len + 5U, buf.data());
    }

    [[nodiscard]] auto send(const std::uint64_t       signature,
                            const std::uint32_t       extended_can_id,
                            const TransferID          transfer_id,
                            const std::size_t         payload_length,
                            const std::uint8_t* const payload) -> bool
    {
        // This lambda is allocated on the stack, so that the closures do not require heap allocation.
        return detail::transmitV0(
            [this, extended_can_id](const std::size_t         frame_payload_size,
                                    const std::uint8_t* const frame_payload) -> bool {
                assert(frame_payload_size <= std::numeric_limits<std::uint8_t>::max());
                return driver_.push(true,  // Force Classic CAN mode.
                                    extended_can_id,
                                    static_cast<std::uint8_t>(frame_payload_size),
                                    frame_payload);
            },
            signature,
            static_cast<std::uint8_t>(transfer_id & detail::MaxTransferID),
            payload_length,
            payload);
    }

    enum class ServiceTypeID : std::uint8_t
    {
        GetNodeInfo         = 1,
        BeginFirmwareUpdate = 40,
        FileRead            = 48,
    };

    struct PendingRequestMetadata
    {
        std::uint8_t server_node_id{};
        PortID       service_id{};
        std::uint8_t transfer_id{};
    };

    static constexpr std::uint64_t NodeStatusSignature          = 0x0F0868D0C1A7C6F1ULL;
    static constexpr std::uint64_t LogMessageSignature          = 0xD654A48E0C049D75ULL;
    static constexpr std::uint64_t GetNodeInfoSignature         = 0xEE468A8121C46A9EULL;
    static constexpr std::uint64_t BeginFirmwareUpdateSignature = 0xB7D725DF72724126ULL;
    static constexpr std::uint64_t FileReadSignature            = 0x8DCDCA939F33F678ULL;

    ICANDriver&        driver_;
    const std::uint8_t local_node_id_;

    std::optional<PendingRequestMetadata> pending_request_meta_;

    std::array<std::uint8_t, 7> last_node_status_{};

    SimplifiedServiceTransferReassemblerV0<0>   rx_req_get_node_info_{GetNodeInfoSignature, local_node_id_};
    SimplifiedServiceTransferReassemblerV0<200> rx_req_begin_fw_upd_{BeginFirmwareUpdateSignature, local_node_id_};
    SimplifiedServiceTransferReassemblerV0<300> rx_res_file_read_{FileReadSignature, local_node_id_};
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
        if (const auto f = detail::parseFrameV0(can_id, payload_size, payload))
        {
            if (const auto* const mf = std::get_if<detail::MessageFrameModel>(&*f))
            {
                if (mf->subject_id == DataTypeID)
                {
                    if (const auto res = rx_.update(*mf))
                    {
                        return acceptAllocationResponseMessage(now, res->first, res->second);
                    }
                }
            }
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
        const auto node_id = static_cast<std::uint8_t>(message_data[0] >> 1U);
        if (node_id < 1)  // Bad allocation
        {
            return nullptr;
        }
        assert(node_id <= MaxNodeID);
        // Allocation done, full match.
        if (const auto bus_mode = driver_.configure(bitrate_, false, makeAcceptanceFilter<0>(node_id)))
        {
            (void) bus_mode;
            return allocator_.construct<V0MainActivity>(driver_, node_id);
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
        stage_ = 0;
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

    static constexpr std::uint16_t DataTypeID = 1;  // uavcan.protocol.dynamic_node_id.Allocation

    IAllocator&                allocator_;
    ICANDriver&                driver_;
    const SystemInfo::UniqueID local_uid_;
    const ICANDriver::Bitrate  bitrate_;

    std::optional<std::chrono::microseconds> deadline_;
    std::uint8_t                             stage_ = 0;

    std::uint8_t tx_transfer_id_ = 0;

    SimplifiedMessageTransferReassemblerV0<19> rx_{0x0B2A812620A11D40ULL};
};

class V1MainActivity : public IActivity
{
public:
    V1MainActivity(ICANDriver& driver, const ICANDriver::Mode bus_mode, const std::uint8_t local_node_id) :
        driver_(driver),
        bus_mode_(bus_mode),
        local_node_id_(local_node_id),
        rx_file_read_response_(local_node_id),
        rx_get_info_request_(local_node_id),
        rx_execute_command_request_(local_node_id)
    {}

    auto poll(IReactor& reactor, const std::chrono::microseconds uptime) -> IActivity* override
    {
        (void) uptime;
        ICANDriver::PayloadBuffer buf{};
        while (const auto transport_frame = driver_.pop(buf))
        {
            const auto [can_id, payload_size] = *transport_frame;
            if (const auto frame = detail::parseFrame(can_id, payload_size, buf.data()))
            {
                if (const auto* const s = std::get_if<ServiceFrameModel>(&*frame))
                {
                    processReceivedServiceFrame(reactor, *s);
                }
                // This implementation is not interested in accepting any message transfers.
            }
        }
        return nullptr;
    }

    [[nodiscard]] auto getLocalNodeID() const -> std::uint8_t { return local_node_id_; }

private:
    [[nodiscard]] auto sendRequest(const ServiceID           service_id,
                                   const NodeID              server_node_id,
                                   const TransferID          transfer_id,
                                   const std::size_t         payload_length,
                                   const std::uint8_t* const payload) -> bool override
    {
        static constexpr std::uint32_t CANIDMask = 0b110'11'0000000000'0000000'0000000UL;
        if (server_node_id <= MaxNodeID)
        {
            const auto can_id = CANIDMask |                                           //
                                (static_cast<std::uint32_t>(service_id) << 14U) |     //
                                (static_cast<std::uint32_t>(server_node_id) << 7U) |  //
                                local_node_id_;
            if (send(can_id, transfer_id, payload_length, payload))
            {
                pending_request_meta_ = PendingRequestMetadata{static_cast<std::uint8_t>(server_node_id),
                                                               static_cast<PortID>(service_id),
                                                               static_cast<std::uint8_t>(transfer_id & MaxTransferID)};
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] auto sendResponse(const std::uint8_t        priority,
                                    const std::uint16_t       service_id,
                                    const std::uint8_t        client_node_id,
                                    const std::uint8_t        transfer_id,
                                    const std::size_t         payload_length,
                                    const std::uint8_t* const payload) -> bool
    {
        static constexpr std::uint32_t CANIDMask = 0b000'10'0000000000'0000000'0000000UL;
        //
        const auto can_id = CANIDMask |                                           //
                            (static_cast<std::uint32_t>(priority) << 26U) |       //
                            (static_cast<std::uint32_t>(service_id) << 14U) |     //
                            (static_cast<std::uint32_t>(client_node_id) << 7U) |  //
                            local_node_id_;
        return send(can_id, transfer_id, payload_length, payload);
    }

    void cancelRequest() override { pending_request_meta_.reset(); }

    [[nodiscard]] auto publishMessage(const SubjectID           subject_id,
                                      const TransferID          transfer_id,
                                      const std::size_t         payload_length,
                                      const std::uint8_t* const payload) -> bool override
    {
        static constexpr std::uint32_t CANIDMask = 0b100'00'0110000000000000'0'0000000UL;
        const auto can_id = CANIDMask | (static_cast<std::uint32_t>(subject_id) << 8U) | local_node_id_;
        return send(can_id, transfer_id, payload_length, payload);
    }

    void processReceivedServiceFrame(IReactor& reactor, const ServiceFrameModel& frame)
    {
        if (frame.request_not_response)
        {
            if (frame.service_id == static_cast<std::uint16_t>(ServiceID::NodeGetInfo))
            {
                if (const auto req = rx_get_info_request_.update(frame))
                {
                    processServiceRequest(reactor, frame, req->first, req->second);
                }
            }
            if (frame.service_id == static_cast<std::uint16_t>(ServiceID::NodeExecuteCommand))
            {
                if (const auto req = rx_execute_command_request_.update(frame))
                {
                    processServiceRequest(reactor, frame, req->first, req->second);
                }
            }
        }
        else
        {
            if (pending_request_meta_ &&                                            //
                (pending_request_meta_->server_node_id == frame.source_node_id) &&  //
                (pending_request_meta_->service_id == frame.service_id) &&          //
                (pending_request_meta_->transfer_id == frame.transfer_id))
            {
                if (frame.service_id == static_cast<std::uint16_t>(ServiceID::FileRead))
                {
                    if (const auto res = rx_file_read_response_.update(frame))
                    {
                        processServiceResponse(reactor, res->first, res->second);
                    }
                }
                else
                {
                    assert(false);  // This means that we've sent a request for which there is no response listener.
                }
            }
        }
    }

    void processServiceRequest(IReactor&                 reactor,
                               const ServiceFrameModel&  frame,
                               const std::size_t         request_size,
                               const std::uint8_t* const request_data)
    {
        assert(frame.destination_node_id == local_node_id_);
        assert(frame.request_not_response);
        std::array<std::uint8_t, MaxSerializedRepresentationSize> response_data{};
        if (const auto response_size = reactor.processRequest(frame.service_id,
                                                              frame.source_node_id,
                                                              request_size,
                                                              request_data,
                                                              response_data.data()))
        {
            (void) sendResponse(frame.priority,
                                frame.service_id,
                                frame.source_node_id,
                                frame.transfer_id,
                                *response_size,
                                response_data.data());
        }
    }

    void processServiceResponse(IReactor&                 reactor,
                                const std::size_t         response_size,
                                const std::uint8_t* const response_data)
    {
        pending_request_meta_.reset();
        reactor.processResponse(response_size, response_data);
    }

    [[nodiscard]] auto send(const std::uint32_t       extended_can_id,
                            const TransferID          transfer_id,
                            const std::size_t         payload_length,
                            const std::uint8_t* const payload) -> bool
    {
        // This lambda is allocated on the stack, so that the closures do not require heap allocation.
        return detail::transmit(
            [this, extended_can_id](const std::size_t         frame_payload_size,
                                    const std::uint8_t* const frame_payload) -> bool {
                assert(frame_payload_size <= std::numeric_limits<std::uint8_t>::max());
                return driver_.push(false,
                                    extended_can_id,
                                    static_cast<std::uint8_t>(frame_payload_size),
                                    frame_payload);
            },
            (bus_mode_ == ICANDriver::Mode::Classic) ? 8U : 64U,
            static_cast<std::uint8_t>(transfer_id & detail::MaxTransferID),
            payload_length,
            payload);
    }

    struct PendingRequestMetadata
    {
        std::uint8_t server_node_id{};
        PortID       service_id{};
        std::uint8_t transfer_id{};
    };

    ICANDriver&            driver_;
    const ICANDriver::Mode bus_mode_;
    const std::uint8_t     local_node_id_;

    SimplifiedServiceTransferReassembler<300> rx_file_read_response_;
    SimplifiedServiceTransferReassembler<0>   rx_get_info_request_;
    SimplifiedServiceTransferReassembler<300> rx_execute_command_request_;

    std::optional<PendingRequestMetadata> pending_request_meta_;
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
        if (allocated_node_id > MaxNodeID)
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
        if ((allocated_node_id <= MaxNodeID) && (0 == std::memcmp(&body[2], local_uid_.data(), local_uid_.size())))
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
            return allocator_.construct<V1MainActivity>(driver_, *bus_mode, allocated_node_id);
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
                if (const auto uavcan_version = tryDetectProtocolVersionFromFrame(can_id, buf.at(payload_size - 1U)))
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
            const std::optional<NodeID>               local_node_id  = {})
    {
        if ((activity_ == nullptr) && can_bitrate &&     //
            uavcan_version && (*uavcan_version == 0) &&  //
            local_node_id && (*local_node_id > 0) && (*local_node_id <= MaxNodeID))
        {
            if (const auto bus_mode =
                    driver.configure(*can_bitrate,
                                     false,
                                     detail::makeAcceptanceFilter<0>(static_cast<std::uint8_t>(*local_node_id))))
            {
                (void) bus_mode;  // v0 doesn't care about mode because it only supports Classic CAN.
                activity_ =
                    activity_allocator_.construct<detail::V0MainActivity>(driver,
                                                                          static_cast<std::uint8_t>(*local_node_id));
            }
        }
        if ((activity_ == nullptr) && can_bitrate &&     //
            uavcan_version && (*uavcan_version == 1) &&  //
            local_node_id && (*local_node_id <= MaxNodeID))
        {
            if (const auto bus_mode =
                    driver.configure(*can_bitrate,
                                     false,
                                     detail::makeAcceptanceFilter<1>(static_cast<std::uint8_t>(*local_node_id))))
            {
                activity_ =
                    activity_allocator_.construct<detail::V1MainActivity>(driver,
                                                                          *bus_mode,
                                                                          static_cast<std::uint8_t>(*local_node_id));
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
        assert(activity_ != nullptr);
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
        assert(activity_ != nullptr);
        return activity_->sendRequest(service_id, server_node_id, transfer_id, payload_length, payload);
    }

    void cancelRequest() override { activity_->cancelRequest(); }

    [[nodiscard]] auto publishMessage(const SubjectID           subject_id,
                                      const TransferID          transfer_id,
                                      const std::size_t         payload_length,
                                      const std::uint8_t* const payload) -> bool override
    {
        assert(activity_ != nullptr);
        return activity_->publishMessage(subject_id, transfer_id, payload_length, payload);
    }

    detail::BlockAllocator<1024, 2> activity_allocator_;
    detail::IActivity*              activity_ = nullptr;
};

}  // namespace kocherga::can
