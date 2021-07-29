// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"
#include <cassert>
#include <cstdlib>

namespace kocherga::serial
{
namespace detail
{
using kocherga::detail::BitsPerByte;

constexpr std::uint8_t FrameDelimiter = 0x00;  ///< Zeros cannot occur in the stream thanks to COBS encoding.

/// Reference values to check the header against.
static constexpr std::uint8_t                FrameFormatVersion = 0;
static constexpr std::array<std::uint8_t, 4> FrameIndexEOTReference{0, 0, 0, 0x80};

/// Size-optimized implementation of CRC32-C (Castagnoli).
class CRC32C
{
public:
    static constexpr std::size_t Size = 4;

    void update(const std::uint8_t b)
    {
        value_ ^= static_cast<std::uint32_t>(b);
        for (auto i = 0U; i < BitsPerByte; i++)
        {
            value_ = ((value_ & 1U) != 0) ? ((value_ >> 1U) ^ ReflectedPoly) : (value_ >> 1U);  // NOLINT
        }
    }

    [[nodiscard]] auto get() const { return value_ ^ Xor; }

    [[nodiscard]] auto getBytes() const -> std::array<std::uint8_t, Size>
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

/// This is used for COBS coding. The buffer resides in one of two possible states: IN and OUT.
/// The initial state is IN, where push() can be used.
/// The first time pop() is called, the buffer switches into the OUT state and remains in it until empty.
/// While in the OUT state, pushing is not permitted.
class LookaheadFIFO  // NOLINT
{
public:
    /// Check size() before calling, otherwise the write pointer will wrap around to the beginning of the buffer.
    [[nodiscard]] auto push(const std::uint8_t val) { buf_.at(in_++) = val; }

    /// Returns empty if empty, meaning that the current state is PUSHING.
    [[nodiscard]] auto pop() -> std::optional<std::uint8_t>
    {
        if (out_ < in_)
        {
            return buf_.at(out_++);
        }
        out_ = 0;
        in_  = 0;
        return {};
    }

    /// The number of bytes that currently reside in the queue.
    [[nodiscard]] auto size() const { return in_; }

private:
    std::array<std::uint8_t, 256> buf_;
    std::uint8_t                  in_  = 0;
    std::uint8_t                  out_ = 0;
};

/// New instance shall be created per encoded frame.
/// ByteWriter is of type (std::uint8_t) -> bool, returns true on success.
template <typename ByteWriter>
class COBSEncoder
{
public:
    explicit COBSEncoder(ByteWriter byte_writer) : byte_writer_(byte_writer) {}

    /// Invoke this function once per byte to transmit COBS-encoded bytestream.
    /// The leading frame delimiter will be added automatically.
    /// The instance shall be discarded immediately if this method returns false.
    [[nodiscard]] auto push(const std::uint8_t b) -> bool
    {
        if (byte_count_ == 0)
        {
            if (!output(FrameDelimiter))
            {
                return false;
            }
        }
        byte_count_++;
        if (b != FrameDelimiter)
        {
            lookahead_.push(b);
        }
        if ((b == FrameDelimiter) || (lookahead_.size() >= (std::numeric_limits<std::uint8_t>::max() - 1)))
        {
            return flush();
        }
        return true;
    }

    /// This function shall be invoked at the end once.
    /// The trailing frame delimiter will be added automatically.
    /// The instance shall be discarded immediately if this method returns false.
    [[nodiscard]] auto end() -> bool { return flush() && output(FrameDelimiter); }

private:
    [[nodiscard]] auto output(const std::uint8_t b) const -> bool { return byte_writer_(b); }

    [[nodiscard]] auto flush() -> bool
    {
        const auto sz = lookahead_.size();
        assert(sz < std::numeric_limits<std::uint8_t>::max());
        if (!output(static_cast<std::uint8_t>(sz + 1U)))
        {
            return false;
        }
        while (auto b = lookahead_.pop())
        {
            if (!output(*b))
            {
                return false;
            }
        }
        assert(lookahead_.size() == 0);
        return true;
    }

    LookaheadFIFO lookahead_;
    std::size_t   byte_count_ = 0;
    ByteWriter    byte_writer_;
};

struct Transfer
{
    struct Metadata
    {
        static constexpr std::uint8_t DefaultPriority      = 6U;  // Second to lowest.
        static constexpr NodeID       AnonymousNodeID      = 0xFFFFU;
        static constexpr PortID       DataSpecServiceFlag  = 0x8000U;
        static constexpr PortID       DataSpecResponseFlag = 0x4000U;
        static constexpr auto         DataSpecServiceIDMask =
            static_cast<PortID>(~static_cast<PortID>(DataSpecServiceFlag | DataSpecResponseFlag));

        std::uint8_t  priority    = DefaultPriority;
        NodeID        source      = AnonymousNodeID;
        NodeID        destination = AnonymousNodeID;
        std::uint16_t data_spec{};
        TransferID    transfer_id{};

        [[nodiscard]] auto isRequest() const -> std::optional<PortID>
        {
            if (((data_spec & DataSpecServiceFlag) != 0) && ((data_spec & DataSpecResponseFlag) == 0))
            {
                return data_spec & DataSpecServiceIDMask;
            }
            return {};
        }

        [[nodiscard]] auto isResponse() const -> std::optional<PortID>
        {
            if (((data_spec & DataSpecServiceFlag) != 0) && ((data_spec & DataSpecResponseFlag) != 0))
            {
                return data_spec & DataSpecServiceIDMask;
            }
            return {};
        }
    };
    Metadata            meta{};
    std::size_t         payload_len = 0;
    const std::uint8_t* payload     = nullptr;
};

/// UAVCAN/serial stream parser. Extracts UAVCAN/serial frames from raw stream of bytes.
template <std::size_t MaxPayloadSize>
class StreamParser
{
public:
    /// If the byte completed a transfer, it will be returned.
    /// The returned object contains a pointer to the payload buffer memory. The memory is invalidated on the second
    /// call to update() after reception (the first call does not invalidate the memory).
    [[nodiscard]] auto update(const std::uint8_t stream_byte) -> std::optional<Transfer>
    {
        std::optional<Transfer> out;
        (void) stream_byte;
        // TODO FIXME implement
        return out;
    }

    void reset()
    {
        offset_ = 0;
        crc_    = {};
        meta_   = {};
    }

private:
    void acceptHeader(const std::uint8_t bt)
    {
        if ((OffsetVersion == offset_) && (bt != FrameFormatVersion))
        {
            reset();
        }
        if (OffsetPriority == offset_)
        {
            meta_.priority = bt;
        }
        acceptHeaderField(OffsetSource, meta_.source, bt);
        acceptHeaderField(OffsetDestination, meta_.destination, bt);
        acceptHeaderField(OffsetDataSpec, meta_.data_spec, bt);
        acceptHeaderField(OffsetTransferID, meta_.transfer_id, bt);
        if ((OffsetFrameIndexEOT.first <= offset_) && (offset_ <= OffsetFrameIndexEOT.second) &&
            (FrameIndexEOTReference.at(offset_ - OffsetFrameIndexEOT.first) != bt))
        {
            reset();
        }
        if (offset_ == (HeaderSize - 1U))
        {
            if (!crc_.isResidueCorrect())
            {
                reset();  // Header CRC error.
            }
            // At this point the header has been received and proven to be correct. Here, a generic implementation
            // would normally query the subscription list to see if the frame is interesting or it should be dropped;
            // also, the amount of dynamic memory that needs to be allocated for the payload would also be determined
            // at this moment. The main purpose of the header CRC is to permit such early-stage frame processing.
            // This specialized implementation requires none of that.
            crc_ = {};
        }
    }

    void acceptPayload(const std::uint8_t bt)
    {
        const auto buf_offset = offset_ - HeaderSize;
        if (buf_offset < buf_.size())
        {
            buf_.at(buf_offset) = bt;
        }
        else
        {
            reset();
        }
    }

    template <typename Field>
    void acceptHeaderField(const std::pair<std::size_t, std::size_t> range, Field& fld, const std::uint8_t bt) const
    {
        if ((range.first <= offset_) && (offset_ <= range.second))
        {
            if (const auto byte_index = offset_ - range.first; byte_index > 0)
            {
                fld |= static_cast<Field>(static_cast<Field>(bt) << (BitsPerByte * byte_index));
            }
            else
            {
                fld = static_cast<Field>(bt);
            }
        }
    }

    static constexpr std::size_t HeaderSize = 32;
    // Header field offsets.
    static constexpr std::size_t                         OffsetVersion  = 0;
    static constexpr std::size_t                         OffsetPriority = 1;
    static constexpr std::pair<std::size_t, std::size_t> OffsetSource{2, 3};
    static constexpr std::pair<std::size_t, std::size_t> OffsetDestination{4, 5};
    static constexpr std::pair<std::size_t, std::size_t> OffsetDataSpec{6, 7};
    static constexpr std::pair<std::size_t, std::size_t> OffsetTransferID{16, 23};
    static constexpr std::pair<std::size_t, std::size_t> OffsetFrameIndexEOT{24, 27};

    std::size_t offset_ = 0;
    CRC32C      crc_;

    Transfer::Metadata meta_;

    std::array<std::uint8_t, MaxPayloadSize + CRC32C::Size> buf_{};
};

/// Sends a transfer with minimal buffering (some buffering is required by COBS) to save memory and reduce latency.
/// Callback is of type (std::uint8_t) -> bool whose semantics reflects ISerialPort::send().
/// Callback shall not be an std::function<> to avoid heap allocation.
template <typename Callback>
[[nodiscard]] inline auto transmit(const Callback& send_byte, const Transfer& tr) -> bool
{
    COBSEncoder<const Callback&> encoder(send_byte);
    CRC32C                       crc;
    const auto                   out = [&crc, &encoder](const std::uint8_t b) -> bool {
        crc.update(b);
        return encoder.push(b);
    };
    const auto out2 = [&out](const std::uint16_t bb) -> bool {
        return out(static_cast<std::uint8_t>(bb)) && out(static_cast<std::uint8_t>(bb >> BitsPerByte));
    };
    bool ok = out(FrameFormatVersion) && out(tr.meta.priority) &&  //
              out2(tr.meta.source) && out2(tr.meta.destination) && out2(tr.meta.data_spec);
    for (auto i = 0U; i < sizeof(std::uint64_t); i++)
    {
        ok = ok && out(0);
    }
    auto tmp_transfer_id = tr.meta.transfer_id;
    for (auto i = 0U; i < sizeof(std::uint64_t); i++)
    {
        ok = ok && out(static_cast<std::uint8_t>(tmp_transfer_id));
        tmp_transfer_id >>= BitsPerByte;
    }
    for (const auto x : FrameIndexEOTReference)
    {
        ok = ok && out(x);
    }
    for (const auto x : crc.getBytes())
    {
        ok = ok && out(x);
    }
    crc             = {};  // Now it's the payload CRC.
    const auto* ptr = tr.payload;
    for (std::size_t i = 0U; i < tr.payload_len; i++)
    {
        ok = ok && out(*ptr);
        ++ptr;
        if (!ok)
        {
            break;
        }
    }
    for (const auto x : crc.getBytes())
    {
        ok = ok && out(x);
    }
    return ok && encoder.end();
}

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
    auto operator=(const ISerialPort&&) -> ISerialPort& = delete;
};

/// Kocherga node implementing the UAVCAN/serial transport.
class SerialNode : public kocherga::INode
{
public:
    SerialNode(ISerialPort& port, const SystemInfo::UniqueID& local_unique_id) :
        unique_id_(local_unique_id), port_(port)
    {}

    /// Set up the local node-ID manually instead of running PnP allocation.
    /// If a manual update is triggered, this shall be done beforehand.
    /// Do not set up the local node-ID more than once.
    void setLocalNodeID(const NodeID node_id) { local_node_id_ = node_id; }

    /// Resets the state of the frame parser. Call it when the communication channel is reinitialized.
    void reset() { stream_parser_.reset(); }

private:
    void poll(IReactor& reactor, const std::chrono::microseconds uptime) override
    {
        for (auto i = 0U; i < MaxBytesToProcessPerPoll; i++)
        {
            if (auto bt = port_.receive())
            {
                if (const auto tr = stream_parser_.update(*bt))
                {
                    processReceivedTransfer(reactor, *tr);
                }
            }
            else
            {
                break;
            }
        }
        if ((!local_node_id_) && (uptime >= pnp_next_request_at_))
        {
            using kocherga::detail::dsdl::PnPNodeIDAllocation;
            constexpr std::int64_t interval_usec =
                std::chrono::duration_cast<std::chrono::microseconds>(PnPNodeIDAllocation::MaxRequestInterval).count();
            const std::chrono::microseconds delay{(std::rand() * interval_usec) / RAND_MAX};  // NOSONAR rand() ok
            pnp_next_request_at_ = uptime + delay;
            std::array<std::uint8_t, PnPNodeIDAllocation::MessageSize_v2> buf{};
            std::uint8_t*                                                 ptr = buf.data();
            *ptr++ = std::numeric_limits<std::uint8_t>::max();
            *ptr++ = std::numeric_limits<std::uint8_t>::max();
            (void) std::memcpy(ptr, unique_id_.data(), unique_id_.size());
            (void) publishMessage(SubjectID::PnPNodeIDAllocationData_v2, pnp_transfer_id_, buf.size(), buf.data());
            ++pnp_transfer_id_;
        }
    }

    void processReceivedTransfer(IReactor& reactor, const detail::Transfer& tr)
    {
        if (const auto resp_id = tr.meta.isResponse())
        {
            if (pending_request_meta_ && local_node_id_)
            {
                const bool match = (resp_id == pending_request_meta_->service_id) &&
                                   (tr.meta.source == pending_request_meta_->server_node_id) &&
                                   (tr.meta.destination == (*local_node_id_)) &&
                                   (tr.meta.transfer_id == pending_request_meta_->transfer_id);
                if (match)
                {
                    reactor.processResponse(tr.payload_len, tr.payload);
                    pending_request_meta_.reset();
                }
            }
        }
        else if (const auto req_id = tr.meta.isRequest())
        {
            if (local_node_id_ && (tr.meta.destination == (*local_node_id_)))
            {
                std::array<std::uint8_t, MaxSerializedRepresentationSize> buf{};
                if (const auto size =
                        reactor.processRequest(*req_id, tr.meta.source, tr.payload_len, tr.payload, buf.data()))
                {
                    detail::Transfer::Metadata meta{};
                    meta.priority    = tr.meta.priority;
                    meta.source      = *local_node_id_;
                    meta.destination = tr.meta.source;
                    meta.data_spec   = static_cast<PortID>(*req_id) |
                                     static_cast<PortID>(detail::Transfer::Metadata::DataSpecServiceFlag |
                                                         detail::Transfer::Metadata::DataSpecResponseFlag);
                    meta.transfer_id = tr.meta.transfer_id;
                    (void) transmit({meta, *size, buf.data()});
                }
            }
        }
        else
        {
            if ((!local_node_id_) &&  // If node-ID is not yet allocated, check if this is an allocation response.
                (tr.meta.data_spec == static_cast<PortID>(SubjectID::PnPNodeIDAllocationData_v2)) &&
                (tr.meta.source == detail::Transfer::Metadata::AnonymousNodeID) &&
                (tr.meta.destination == detail::Transfer::Metadata::AnonymousNodeID))
            {
                const std::uint8_t* ptr     = tr.payload;
                NodeID              node_id = *ptr;
                ++ptr;
                node_id |= static_cast<NodeID>(static_cast<NodeID>(*ptr) << detail::BitsPerByte);
                ++ptr;
                const bool uid_match = std::equal(std::begin(unique_id_), std::end(unique_id_), ptr);
                if (uid_match)
                {
                    local_node_id_ = node_id;
                }
            }
        }
    }

    [[nodiscard]] auto sendRequest(const ServiceID           service_id,
                                   const NodeID              server_node_id,
                                   const TransferID          transfer_id,
                                   const std::size_t         payload_length,
                                   const std::uint8_t* const payload) -> bool override
    {
        if (local_node_id_)
        {
            detail::Transfer::Metadata meta{};
            meta.source      = *local_node_id_;
            meta.destination = server_node_id;
            meta.data_spec   = static_cast<PortID>(service_id) | detail::Transfer::Metadata::DataSpecServiceFlag;
            meta.transfer_id = transfer_id;
            if (transmit({meta, payload_length, payload}))
            {
                pending_request_meta_ = PendingRequestMetadata{
                    server_node_id,
                    static_cast<PortID>(service_id),
                    transfer_id,
                };
                return true;
            }
        }
        return false;
    }

    void cancelRequest() override { pending_request_meta_.reset(); }

    auto publishMessage(const SubjectID           subject_id,
                        const TransferID          transfer_id,
                        const std::size_t         payload_length,
                        const std::uint8_t* const payload) -> bool override
    {
        if (local_node_id_)
        {
            detail::Transfer::Metadata meta{};
            meta.source      = *local_node_id_;
            meta.data_spec   = static_cast<PortID>(subject_id);
            meta.transfer_id = transfer_id;
            return transmit({meta, payload_length, payload});
        }
        return false;
    }

    [[nodiscard]] auto transmit(const detail::Transfer& tr) -> bool
    {
        return detail::transmit([this](const std::uint8_t b) { return port_.send(b); }, tr);
    }

    struct PendingRequestMetadata
    {
        NodeID     server_node_id{};
        PortID     service_id{};
        TransferID transfer_id{};
    };

    static constexpr auto MaxBytesToProcessPerPoll = MaxSerializedRepresentationSize * 2U;

    const SystemInfo::UniqueID unique_id_;

    ISerialPort&                                          port_;
    detail::StreamParser<MaxSerializedRepresentationSize> stream_parser_;
    std::optional<NodeID>                                 local_node_id_;
    std::optional<PendingRequestMetadata>                 pending_request_meta_;

    std::chrono::microseconds pnp_next_request_at_{0};
    std::uint64_t             pnp_transfer_id_ = 0;
};

}  // namespace kocherga::serial