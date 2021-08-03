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
    std::vector<std::uint8_t>       payload;

    Transfer() = default;
    Transfer(const kocherga::TransferID            arg_transfer_id,
             const std::vector<std::uint8_t>&      arg_payload,
             const std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
        remote_node_id(arg_remote_node_id), transfer_id(arg_transfer_id), payload(arg_payload)
    {}
    Transfer(const kocherga::TransferID            arg_transfer_id,
             const std::size_t                     arg_payload_length,
             const std::uint8_t* const             arg_payload,
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

    [[nodiscard]] auto wasRequestCanceled()
    {
        const auto out    = request_canceled_;
        request_canceled_ = false;
        return out;
    }

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
        const auto proc = [&reactor, this](const Output ses, const kocherga::ServiceID service_id, const Transfer& tr) {
            std::vector<std::uint8_t> buffer(kocherga::MaxSerializedRepresentationSize);
            //
            const auto size = reactor.processRequest(static_cast<kocherga::PortID>(service_id),
                                                     *tr.remote_node_id,
                                                     tr.payload.size(),
                                                     tr.payload.data(),
                                                     buffer.data());
            if (size)
            {
                buffer.resize(*size);
                store(ses, Transfer(tr.transfer_id, buffer, *tr.remote_node_id));
            }
        };

        last_poll_at_ = uptime;
        for (auto [key, tr] : inputs_)
        {
            switch (key)
            {
            case Input::ExecuteCommandRequest:
            {
                proc(Output::ExecuteCommandResponse, kocherga::ServiceID::NodeExecuteCommand, tr);
                break;
            }
            case Input::NodeInfoRequest:
            {
                proc(Output::NodeInfoResponse, kocherga::ServiceID::NodeGetInfo, tr);
                break;
            }
            case Input::FileReadResponse:
            {
                reactor.processResponse(std::size(tr.payload), tr.payload.data());
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

    [[nodiscard]] auto sendRequest(const kocherga::ServiceID  service_id,
                                   const kocherga::NodeID     server_node_id,
                                   const kocherga::TransferID transfer_id,
                                   const std::size_t          payload_length,
                                   const std::uint8_t* const  payload) -> bool override
    {
        switch (service_id)
        {
        case kocherga::ServiceID::FileRead:
        {
            store(Output::FileReadRequest, Transfer(transfer_id, payload_length, payload, server_node_id));
            return file_read_result_;
        }
        case kocherga::ServiceID::NodeGetInfo:
        case kocherga::ServiceID::NodeExecuteCommand:
        default:
        {
            FAIL("UNEXPECTED SERVICE REQUEST");
            return false;
        }
        }
    }

    void cancelRequest() override { request_canceled_ = true; }

    [[nodiscard]] auto publishMessage(const kocherga::SubjectID  subject_id,
                                      const kocherga::TransferID transfer_id,
                                      const std::size_t          payload_length,
                                      const std::uint8_t* const  payload) -> bool override
    {
        bool out = false;
        switch (subject_id)
        {
        case kocherga::SubjectID::NodeHeartbeat:
        {
            store(Output::HeartbeatMessage, Transfer(transfer_id, payload_length, payload));
            out = true;
            break;
        }
        case kocherga::SubjectID::DiagnosticRecord:
        {
            store(Output::LogRecordMessage, Transfer(transfer_id, payload_length, payload));
            out = true;
            break;
        }
        case kocherga::SubjectID::PnPNodeIDAllocationData_v1:
        case kocherga::SubjectID::PnPNodeIDAllocationData_v2:
        default:
        {
            FAIL("UNEXPECTED MESSAGE");
            break;
        }
        }
        return out;
    }

    void store(const Output ses, const Transfer& tr)
    {
        INFO("output: " << static_cast<std::uint32_t>(ses));
        INFO("transfer: " << tr.toString());
        REQUIRE(outputs_.find(ses) == std::end(outputs_));
        outputs_[ses] = tr;
    }

    std::chrono::microseconds  last_poll_at_{};
    bool                       file_read_result_ = true;
    bool                       request_canceled_ = false;
    std::map<Output, Transfer> outputs_;
    std::map<Input, Transfer>  inputs_;
};

}  // namespace mock
