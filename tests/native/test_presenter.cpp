// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include "util.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

/// GENERATION OF REFERENCE SERIALIZED REPRESENTATIONS.
/// Prepare the DSDL-generated packages:
///     $ pyuavcan -vv dsdl-gen-pkg https://github.com/UAVCAN/public_regulated_data_types/archive/master.zip
/// Run PyUAVCAN:
///     >>> import pyuavcan, uavcan.node, uavcan.diagnostic, uavcan.file
///     >>> list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.Heartbeat_1_0(
///             mode=uavcan.node.Heartbeat_1_0.MODE_SOFTWARE_UPDATE,
///             health=uavcan.node.Heartbeat_1_0.HEALTH_ADVISORY,
///             uptime=1))))
///     [1, 0, 0, 0, 13, 0, 0]
namespace
{
struct Transfer
{
    std::optional<kocherga::NodeID> remote_node_id;
    kocherga::TransferID            transfer_id{};
    std::vector<std::byte>          payload;

    Transfer() = default;
    Transfer(const kocherga::TransferID      arg_transfer_id,
             std::vector<std::byte>          arg_payload,
             std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
        remote_node_id(arg_remote_node_id), transfer_id(arg_transfer_id), payload(std::move(arg_payload))
    {}
    Transfer(const kocherga::TransferID      arg_transfer_id,
             std::vector<std::uint8_t>       arg_payload,
             std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
        Transfer(arg_transfer_id,
                 arg_payload.size(),
                 reinterpret_cast<std::byte*>(arg_payload.data()),
                 arg_remote_node_id)
    {}
    Transfer(const kocherga::TransferID      arg_transfer_id,
             const std::size_t               arg_payload_length,
             const std::byte* const          arg_payload,
             std::optional<kocherga::NodeID> arg_remote_node_id = std::optional<kocherga::NodeID>()) :
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
};

class MockNode : public kocherga::INode
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

class MockController : public kocherga::detail::IController
{
public:
    [[nodiscard]] auto popRebootRequestedFlag()
    {
        const auto out    = reboot_requested_;
        reboot_requested_ = false;
        return out;
    }

    [[nodiscard]] auto popUpdateRequestedFlag()
    {
        const auto out    = update_requested_;
        update_requested_ = false;
        return out;
    }

    void setBeginUpdateResult(const bool value) { update_result_ = value; }

    void setAppInfo(const std::optional<kocherga::AppInfo>& value) { app_info_ = value; }

private:
    void reboot() override { reboot_requested_ = true; }

    auto beginUpdate() -> bool override
    {
        update_requested_ = true;
        return update_result_;
    }

    void handleFileReadResult(const std::optional<kocherga::detail::dsdl::File::ReadResponse> response) override
    {
        file_read_result_.emplace(response);
    }

    [[nodiscard]] auto getAppInfo() const -> std::optional<kocherga::AppInfo> override { return app_info_; }

    bool reboot_requested_ = false;
    bool update_requested_ = false;
    bool update_result_    = true;

    std::optional<kocherga::AppInfo>                                         app_info_;
    std::optional<std::optional<kocherga::detail::dsdl::File::ReadResponse>> file_read_result_;
};

}  // namespace

TEST_CASE("Presenter")
{
    std::array<std::byte, 222> coa{};
    for (auto i = 0U; i < coa.size(); i++)
    {
        coa.at(i) = static_cast<std::byte>(coa.size() - i);
    }

    kocherga::SystemInfo sys_info{
        kocherga::SemanticVersion{33, 11},
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
        "com.zubax.kocherga.test.presenter",
        222,
        coa.data(),
    };

    MockController                 controller;
    std::array<MockNode, 2>        nodes;
    kocherga::detail::Presenter<2> pres(sys_info, {&nodes.at(0), &nodes.at(1)}, controller);

    auto ts = std::chrono::microseconds{500'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }

    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.Heartbeat_1_0(
    //      mode=uavcan.node.Heartbeat_1_0.MODE_SOFTWARE_UPDATE,
    //      health=uavcan.node.Heartbeat_1_0.HEALTH_ADVISORY, uptime=1))))
    pres.setNodeHealth(kocherga::detail::dsdl::Heartbeat::Health::Advisory);
    ts = std::chrono::microseconds{1'500'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
        auto tr = *n.popOutput(MockNode::Output::HeartbeatMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(0, {1, 0, 0, 0, 13, 0, 0}));
    }

    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.diagnostic.Record_1_0(
    //      severity=uavcan.diagnostic.Severity_1_0(uavcan.diagnostic.Severity_1_0.NOTICE),
    //      text='Hello world!'))))
    pres.publishLogRecord(kocherga::detail::dsdl::Diagnostic::Severity::Notice, "Hello world!");
    for (auto& n : nodes)
    {
        auto tr = *n.popOutput(MockNode::Output::LogRecordMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr ==
                Transfer(0, {0, 0, 0, 0, 0, 0, 0, 3, 12, 72, 101, 108, 108, 111, 32, 119, 111, 114, 108, 100, 33}));
    }

    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.diagnostic.Record_1_0(
    //      severity=uavcan.diagnostic.Severity_1_0(uavcan.diagnostic.Severity_1_0.CRITICAL),
    //      text='We are going to die :)'))))
    pres.publishLogRecord(kocherga::detail::dsdl::Diagnostic::Severity::Critical, "We are going to die :)");
    for (auto& n : nodes)
    {
        auto tr = *n.popOutput(MockNode::Output::LogRecordMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(1, {0,   0,   0,   0,   0,   0,  0,   6,   22, 87,  101, 32,  97, 114, 101, 32,
                                   103, 111, 105, 110, 103, 32, 116, 111, 32, 100, 105, 101, 32, 58,  41}));
    }

    // Fails because the remote node-ID is not yet known.
    REQUIRE(!pres.requestFileRead(0xBAD'BAD'BADULL));

    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.ExecuteCommand_1_0.Request(
    //      uavcan.node.ExecuteCommand_1_0.Request.COMMAND_BEGIN_SOFTWARE_UPDATE, '/foo/bar/baz.app.bin'))))
    nodes.at(1).pushInput(MockNode::Input::ExecuteCommandRequest,
                          Transfer(123,
                                   {253, 255, 20,  47, 102, 111, 111, 47, 98, 97,  114, 47,
                                    98,  97,  122, 46, 97,  112, 112, 46, 98, 105, 110},
                                   3210));
    ts = std::chrono::microseconds{1'600'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(controller.popUpdateRequestedFlag());
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::ExecuteCommandResponse)) ==
            Transfer(123, {0, 0, 0, 0, 0, 0, 0}, 3210));

    // The file location is known now.
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.file.Read_1_0.Request(0xbadbadbad,
    //      uavcan.file.Path_1_0('/foo/bar/baz.app.bin')))))
    REQUIRE(pres.requestFileRead(0xBAD'BAD'BADULL));
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::FileReadRequest)) ==
            Transfer(0,
                     {173, 219, 186, 173, 11,  20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98,  97,  122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));
}
