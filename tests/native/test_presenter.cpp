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

    [[nodiscard]] auto popFileReadResult()
    {
        const auto out = file_read_result_;
        file_read_result_.reset();
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
    static constexpr auto               coa_capacity = 222U;
    std::array<std::byte, coa_capacity> coa{};
    for (auto i = 0U; i < coa_capacity; i++)
    {
        coa.at(i) = static_cast<std::byte>(coa.size() - i);
    }

    const kocherga::SystemInfo sys_info{
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
    for (const auto& n : nodes)
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
        const auto tr = *n.popOutput(MockNode::Output::HeartbeatMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(0, {1, 0, 0, 0, 13, 0, 0}));
    }

    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.diagnostic.Record_1_0(
    //      severity=uavcan.diagnostic.Severity_1_0(uavcan.diagnostic.Severity_1_0.NOTICE),
    //      text='Hello world!'))))
    pres.publishLogRecord(kocherga::detail::dsdl::Diagnostic::Severity::Notice, "Hello world!");
    for (auto& n : nodes)
    {
        const auto tr = *n.popOutput(MockNode::Output::LogRecordMessage);
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
        const auto tr = *n.popOutput(MockNode::Output::LogRecordMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(1, {0,   0,   0,   0,   0,   0,  0,   6,   22, 87,  101, 32,  97, 114, 101, 32,
                                   103, 111, 105, 110, 103, 32, 116, 111, 32, 100, 105, 101, 32, 58,  41}));
    }

    // Fails because the remote node-ID is not yet known.
    REQUIRE(!pres.requestFileRead(0xB'ADBA'DBADULL));

    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.ExecuteCommand_1_0.Request(
    //      uavcan.node.ExecuteCommand_1_0.Request.COMMAND_BEGIN_SOFTWARE_UPDATE, '/foo/bar/baz.app.bin'))))
    nodes.at(1).pushInput(MockNode::Input::ExecuteCommandRequest,
                          Transfer(123,
                                   {253, 255, 20,  47, 102, 111, 111, 47, 98, 97,  114, 47,
                                    98,  97,  122, 46, 97,  112, 112, 46, 98, 105, 110},
                                   3210));
    ts = std::chrono::microseconds{1'600'000};
    pres.poll(ts);
    for (const auto& n : nodes)
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
    REQUIRE(pres.requestFileRead(0xB'ADBA'DBADULL));
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::FileReadRequest)) ==
            Transfer(0,
                     {173, 219, 186, 173, 11,  20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98,  97,  122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));

    // Trigger failure. File location specifier not updated.
    controller.setBeginUpdateResult(false);
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.ExecuteCommand_1_0.Request(
    //      uavcan.node.ExecuteCommand_1_0.Request.COMMAND_BEGIN_SOFTWARE_UPDATE, '/ignored.bin'))))
    nodes.at(0).pushInput(MockNode::Input::ExecuteCommandRequest,
                          Transfer(333, {253, 255, 12, 47, 105, 103, 110, 111, 114, 101, 100, 46, 98, 105, 110}, 3210));
    ts = std::chrono::microseconds{1'700'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(controller.popUpdateRequestedFlag());
    REQUIRE((*nodes.at(0).popOutput(MockNode::Output::ExecuteCommandResponse)) ==
            Transfer(333, {6, 0, 0, 0, 0, 0, 0}, 3210));  // Internal error.
    REQUIRE(!nodes.at(1).popOutput(MockNode::Output::ExecuteCommandResponse));

    // Reboot request.
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.ExecuteCommand_1_0.Request(
    //      uavcan.node.ExecuteCommand_1_0.Request.COMMAND_RESTART, ''))))
    nodes.at(1).pushInput(MockNode::Input::ExecuteCommandRequest, Transfer(444, {255, 255, 0}, 2222));
    ts = std::chrono::microseconds{1'800'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(controller.popRebootRequestedFlag());
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::ExecuteCommandResponse)) ==
            Transfer(444, {0, 0, 0, 0, 0, 0, 0}, 2222));

    // Node info request; app info not available. SR generation not shown.
    nodes.at(0).pushInput(MockNode::Input::NodeInfoRequest, Transfer(555, std::vector<std::byte>{}, 3333));
    ts = std::chrono::microseconds{1'900'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(
        (*nodes.at(0).popOutput(MockNode::Output::NodeInfoResponse)) ==
        Transfer(555,
                 {1,   0,   33,  11,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   2,   3,   4,   5,   6,
                  7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  33,  99,  111, 109, 46,  122, 117, 98,  97,  120,
                  46,  107, 111, 99,  104, 101, 114, 103, 97,  46,  116, 101, 115, 116, 46,  112, 114, 101, 115, 101,
                  110, 116, 101, 114, 0,   222, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209,
                  208, 207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189,
                  188, 187, 186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169,
                  168, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149,
                  148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131, 130, 129,
                  128, 127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109,
                  108, 107, 106, 105, 104, 103, 102, 101, 100, 99,  98,  97,  96,  95,  94,  93,  92,  91,  90,  89,
                  88,  87,  86,  85,  84,  83,  82,  81,  80,  79,  78,  77,  76,  75,  74,  73,  72,  71,  70,  69,
                  68,  67,  66,  65,  64,  63,  62,  61,  60,  59,  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,
                  48,  47,  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35,  34,  33,  32,  31,  30,  29,
                  28,  27,  26,  25,  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13,  12,  11,  10,  9,
                  8,   7,   6,   5,   4,   3,   2,   1},
                 3333));
    REQUIRE(!nodes.at(1).popOutput(MockNode::Output::ExecuteCommandResponse));

    // Node info request; app info is available.
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.GetInfo_1_0.Response(
    //      protocol_version=uavcan.node.Version_1_0(1, 0), hardware_version=uavcan.node.Version_1_0(33, 11),
    //      software_version=uavcan.node.Version_1_0(3, 11), software_vcs_revision_id=0xDEADDEADDEADDEAD,
    //      unique_id=bytes(range(1, 17)), name='com.zubax.kocherga.test.presenter',
    //      software_image_crc=[0xBADC0FFEE0DDF00D], certificate_of_authenticity=bytes(range(1, 223)[::-1])))))
    controller.setAppInfo(kocherga::AppInfo{
        0xBADC'0FFE'E0DD'F00DULL,
        0xDE'AD'BULL,
        0xDEAD'DEAD'DEAD'DEADULL,
        0,
        2,
        {3, 11},
    });
    nodes.at(1).pushInput(MockNode::Input::NodeInfoRequest, Transfer(666, std::vector<std::byte>{}, 1111));
    ts = std::chrono::microseconds{1'910'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::ExecuteCommandResponse));
    const Transfer node_info_reference(666,
                                       {1,   0,   33,  11,  3,   11,  173, 222, 173, 222, 173, 222, 173, 222, 1,   2,
                                        3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  33,  99,
                                        111, 109, 46,  122, 117, 98,  97,  120, 46,  107, 111, 99,  104, 101, 114, 103,
                                        97,  46,  116, 101, 115, 116, 46,  112, 114, 101, 115, 101, 110, 116, 101, 114,
                                        1,   13,  240, 221, 224, 254, 15,  220, 186, 222, 222, 221, 220, 219, 218, 217,
                                        216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 206, 205, 204, 203, 202, 201,
                                        200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189, 188, 187, 186, 185,
                                        184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169,
                                        168, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153,
                                        152, 151, 150, 149, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137,
                                        136, 135, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123, 122, 121,
                                        120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105,
                                        104, 103, 102, 101, 100, 99,  98,  97,  96,  95,  94,  93,  92,  91,  90,  89,
                                        88,  87,  86,  85,  84,  83,  82,  81,  80,  79,  78,  77,  76,  75,  74,  73,
                                        72,  71,  70,  69,  68,  67,  66,  65,  64,  63,  62,  61,  60,  59,  58,  57,
                                        56,  55,  54,  53,  52,  51,  50,  49,  48,  47,  46,  45,  44,  43,  42,  41,
                                        40,  39,  38,  37,  36,  35,  34,  33,  32,  31,  30,  29,  28,  27,  26,  25,
                                        24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13,  12,  11,  10,  9,
                                        8,   7,   6,   5,   4,   3,   2,   1},
                                       1111);
    const auto     node_info_response = *nodes.at(1).popOutput(MockNode::Output::NodeInfoResponse);
    REQUIRE(node_info_reference == node_info_response);

    // It's time for another heartbeat.
    pres.setNodeHealth(kocherga::detail::dsdl::Heartbeat::Health::Warning);
    ts = std::chrono::microseconds{2'100'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
        const auto tr = *n.popOutput(MockNode::Output::HeartbeatMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(1, {2, 0, 0, 0, 15, 0, 0}));
    }

    // Deliver the file read response.
    // Remember that the second update request was rejected so the actual file location is the old one.
    // This one will be rejected because it's the wrong node.
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.file.Read_1_0.Response(uavcan.file.Error_1_0(0), b'The data.'))))
    nodes.at(0).pushInput(MockNode::Input::FileReadResponse,
                          Transfer(0, {0, 0, 9, 0, 84, 104, 101, 32, 100, 97, 116, 97, 46}, 3210));
    ts = std::chrono::microseconds{2'200'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!controller.popFileReadResult());
    // Correct node -- accepted.
    nodes.at(1).pushInput(MockNode::Input::FileReadResponse,
                          Transfer(0, {0, 0, 9, 0, 84, 104, 101, 32, 100, 97, 116, 97, 46}, 3210));
    ts = std::chrono::microseconds{2'200'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    const kocherga::detail::dsdl::File::ReadResponse read_result = **controller.popFileReadResult();
    REQUIRE(read_result.isSuccessful());
    REQUIRE(9 == read_result.data_length);  // The pointer is invalidated here, don't check it.
    REQUIRE(nullptr != read_result.data);
    // Further responses not accepted because no request is pending.
    nodes.at(1).pushInput(MockNode::Input::FileReadResponse,
                          Transfer(0, {0, 0, 9, 0, 84, 104, 101, 32, 100, 97, 116, 97, 46}, 3210));
    ts = std::chrono::microseconds{2'300'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!controller.popFileReadResult());

    // File read error handling.
    nodes.at(1).setFileReadResult(false);
    REQUIRE(!pres.requestFileRead(123'456));
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::FileReadRequest)) ==
            Transfer(1,
                     {64,  226, 1,  0,  0,   20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98, 97, 122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));

    // Successful request, but the response specifies a non-zero error code.
    nodes.at(1).setFileReadResult(true);
    REQUIRE(pres.requestFileRead(123'456));
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::FileReadRequest)) ==
            Transfer(2,
                     {64,  226, 1,  0,  0,   20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98, 97, 122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));
    nodes.at(1).pushInput(MockNode::Input::FileReadResponse,
                          Transfer(1, {1, 0, 9, 0, 84, 104, 101, 32, 100, 97, 116, 97, 46}, 3210));
    ts = std::chrono::microseconds{2'400'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!*controller.popFileReadResult());  // Empty option.

    // Successful request, but the response times out.
    nodes.at(1).setFileReadResult(true);
    REQUIRE(pres.requestFileRead(123'456));
    REQUIRE(!nodes.at(0).popOutput(MockNode::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(MockNode::Output::FileReadRequest)) ==
            Transfer(3,
                     {64,  226, 1,  0,  0,   20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98, 97, 122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));
    ts = std::chrono::microseconds{2'500'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!controller.popFileReadResult());  // Nothing yet.
    ts = std::chrono::microseconds{6'000'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!*controller.popFileReadResult());  // Empty option.
}
