// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include "util.hpp"
#include <map>
#include <vector>

namespace mock
{
struct Transfer
{
    std::optional<kocherga::NodeID> remote_node_id;
    kocherga::TransferID            transfer_id{};
    std::vector<std::byte>          payload;

    Transfer() = default;
    Transfer(const kocherga::TransferID            arg_transfer_id,
             const std::vector<std::byte>&         arg_payload,
             const std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
        remote_node_id(arg_remote_node_id), transfer_id(arg_transfer_id), payload(arg_payload)
    {}
    Transfer(const kocherga::TransferID            arg_transfer_id,
             const std::vector<std::uint8_t>&      arg_payload,
             const std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
        Transfer(arg_transfer_id,
                 arg_payload.size(),
                 reinterpret_cast<const std::byte*>(arg_payload.data()),
                 arg_remote_node_id)
    {}
    Transfer(const kocherga::TransferID            arg_transfer_id,
             const std::size_t                     arg_payload_length,
             const std::byte* const                arg_payload,
             const std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
        remote_node_id(arg_remote_node_id), transfer_id(arg_transfer_id)
    {
        (void) std::copy_n(arg_payload, arg_payload_length, std::back_inserter(payload));
    }

    [[nodiscard]] auto toString() const -> std::string
    {
        return "remote_nid=" + (remote_node_id ? std::to_string(static_cast<std::uint64_t>(*remote_node_id)) : "?") +
               " tid=" + std::to_string(transfer_id) + " payload:\n" + util::makeHexDump(payload) + "\n";
    }

    auto operator==(const Transfer& rhs) const -> bool
    {
        return (remote_node_id == rhs.remote_node_id) && (transfer_id == rhs.transfer_id) &&
               std::equal(std::begin(payload), std::end(payload), std::begin(rhs.payload), std::end(rhs.payload));
    }
    auto operator!=(const Transfer& rhs) const -> bool { return !operator==(rhs); }
};

class Node : public kocherga::INode
{
public:
    enum class Output
    {
        ExecuteCommandResponse,
        NodeInfoResponse,
        FileReadRequest,
        HeartbeatMessage,
        LogRecordMessage,
    };

    enum class Input
    {
        ExecuteCommandRequest,
        NodeInfoRequest,
        FileReadResponse,
    };

    [[nodiscard]] auto getLastPollTime() const { return last_poll_at_; }

    void setFileReadResult(const bool value) { file_read_result_ = value; }

    /// Retrieve a previously received transfer under the specified session.
    /// Return an empty option if no such transfer was received since the last retrieval.
    /// The read is destructive -- the transfer is removed afterwards.
    [[nodiscard]] auto popOutput(const Output ses) -> std::optional<Transfer>
    {
        if (const auto it = outputs_.find(ses); it != std::end(outputs_))
        {
            const Transfer ret = it->second;
            outputs_.erase(it);
            return ret;
        }
        return {};
    }

    /// Store the transfer for reception when the next poll() is invoked.
    /// The invocation fails if such transfer is already pending.
    void pushInput(const Input ses, const Transfer& tr)
    {
        REQUIRE(inputs_.find(ses) == std::end(inputs_));
        inputs_[ses] = tr;
    }

private:
    void poll(kocherga::IReactor& reactor, const std::chrono::microseconds uptime) override
    {
        last_poll_at_ = uptime;
        for (auto [key, tr] : inputs_)
        {
            switch (key)
            {
            case Input::ExecuteCommandRequest:
            {
                std::vector<std::byte> buffer(kocherga::IReactor::ExecuteCommandResponseLength);
                reactor.processExecuteCommandRequest(*tr.remote_node_id,
                                                     tr.payload.size(),
                                                     tr.payload.data(),
                                                     buffer.data());
                store(Output::ExecuteCommandResponse, Transfer(tr.transfer_id, buffer, *tr.remote_node_id));
                break;
            }
            case Input::NodeInfoRequest:
            {
                std::vector<std::byte> buffer(kocherga::IReactor::NodeInfoResponseMaxLength);
                const auto             size = reactor.processNodeInfoRequest(buffer.data());
                buffer.resize(size);
                store(Output::NodeInfoResponse, Transfer(tr.transfer_id, buffer, *tr.remote_node_id));
                break;
            }
            case Input::FileReadResponse:
            {
                reactor.processFileReadResponse(std::size(tr.payload), tr.payload.data());
                break;
            }
            default:
            {
                FAIL("UNHANDLED CASE");
            }
            }
        }
        inputs_.clear();
    }

    [[nodiscard]] auto requestFileRead(const kocherga::NodeID     server_node_id,
                                       const kocherga::TransferID transfer_id,
                                       const std::size_t          payload_length,
                                       const std::byte* const     payload) -> bool override
    {
        store(Output::FileReadRequest, Transfer(transfer_id, payload_length, payload, server_node_id));
        return file_read_result_;
    }

    void publishHeartbeat(const kocherga::TransferID transfer_id, const std::byte* const payload) override
    {
        store(Output::HeartbeatMessage, Transfer(transfer_id, HeartbeatLength, payload));
    }

    void publishLogRecord(const kocherga::TransferID transfer_id,
                          const std::size_t          payload_length,
                          const std::byte* const     payload) override
    {
        store(Output::LogRecordMessage, Transfer(transfer_id, payload_length, payload));
    }

    void store(const Output ses, const Transfer& tr)
    {
        REQUIRE(outputs_.find(ses) == std::end(outputs_));
        outputs_[ses] = tr;
    }

    std::chrono::microseconds  last_poll_at_{};
    bool                       file_read_result_ = true;
    std::map<Output, Transfer> outputs_;
    std::map<Input, Transfer>  inputs_;
};

}  // namespace mock
