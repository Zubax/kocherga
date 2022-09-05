// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include "mock.hpp"
#include <iostream>
#include <vector>

/// GENERATION OF REFERENCE SERIALIZED REPRESENTATIONS.
/// Prepare the DSDL-generated packages:
///     $ yakut compile https://github.com/OpenCyphal/public_regulated_data_types/archive/master.zip
/// Run PyCyphal:
///     >>> import pycyphal, uavcan.node, uavcan.diagnostic, uavcan.file
///     >>> list(b''.join(pycyphal.dsdl.serialize(uavcan.node.Heartbeat_1_0(
///             mode=uavcan.node.Mode_1_0(uavcan.node.Mode_1_0.SOFTWARE_UPDATE),
///             health=uavcan.node.Health_1_0(uavcan.node.Health_1_0.ADVISORY),
///             uptime=1))))
///     [1, 0, 0, 0, 1, 3, 0]
namespace
{
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

    void setAppInfo(const std::optional<kocherga::AppInfo>& value) { app_info_ = value; }

private:
    void reboot() override { reboot_requested_ = true; }

    void beginUpdate() override { update_requested_ = true; }

    void handleFileReadResult(const std::optional<kocherga::detail::dsdl::File::ReadResponse> response) override
    {
        file_read_result_.emplace(response);
    }

    [[nodiscard]] auto getAppInfo() const -> std::optional<kocherga::AppInfo> override { return app_info_; }

    bool reboot_requested_ = false;
    bool update_requested_ = false;

    std::optional<kocherga::AppInfo>                                         app_info_;
    std::optional<std::optional<kocherga::detail::dsdl::File::ReadResponse>> file_read_result_;
};

}  // namespace

TEST_CASE("Presenter")  // NOLINT NOSONAR complexity threshold
{
    using mock::Node;
    using mock::Transfer;

    static constexpr auto                  coa_capacity = 222U;
    std::array<std::uint8_t, coa_capacity> coa{};
    for (auto i = 0U; i < coa_capacity; i++)
    {
        coa.at(i) = static_cast<std::uint8_t>(coa.size() - i);
    }

    const kocherga::SystemInfo sys_info{
        kocherga::SemanticVersion{33, 11},
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
        "com.zubax.kocherga.test.presenter",
        222,
        coa.data(),
    };

    MockController                      controller;
    std::array<Node, 2>                 nodes;
    kocherga::detail::Presenter::Params params{};
    params.request_retry_limit = 1;
    kocherga::detail::Presenter pres(sys_info, controller, params);
    REQUIRE(pres.addNode(&nodes.at(0)));
    REQUIRE(pres.addNode(&nodes.at(1)));
    REQUIRE(!pres.addNode(&nodes.at(1)));  // Double registration has no effect.

    auto ts = std::chrono::microseconds{500'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }

    // list(b''.join(pycyphal.dsdl.serialize(uavcan.node.Heartbeat_1_0(
    //      mode=uavcan.node.Mode_1_0(uavcan.node.Mode_1_0.SOFTWARE_UPDATE),
    //      health=uavcan.node.Health_1_0(uavcan.node.Health_1_0.ADVISORY),
    //      uptime=1, vendor_specific_status_code=0x7d))))
    pres.setNodeHealth(kocherga::detail::dsdl::Heartbeat::Health::Advisory);
    pres.setNodeVSSC(125);
    ts = std::chrono::microseconds{1'500'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
        const auto tr = *n.popOutput(Node::Output::HeartbeatMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(0, {1, 0, 0, 0, 1, 3, 125}));
    }

    // list(b''.join(pycyphal.dsdl.serialize(uavcan.diagnostic.Record_1_1(
    //      severity=uavcan.diagnostic.Severity_1_0(uavcan.diagnostic.Severity_1_0.NOTICE),
    //      text='Hello world!'))))
    pres.publishLogRecord(kocherga::detail::dsdl::Diagnostic::Severity::Notice, "Hello world!");
    for (auto& n : nodes)
    {
        const auto tr = *n.popOutput(Node::Output::LogRecordMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr ==
                Transfer(0, {0, 0, 0, 0, 0, 0, 0, 3, 12, 72, 101, 108, 108, 111, 32, 119, 111, 114, 108, 100, 33}));
    }

    // list(b''.join(pycyphal.dsdl.serialize(uavcan.diagnostic.Record_1_1(
    //      severity=uavcan.diagnostic.Severity_1_0(uavcan.diagnostic.Severity_1_0.CRITICAL),
    //      text='We are going to die :)'))))
    pres.publishLogRecord(kocherga::detail::dsdl::Diagnostic::Severity::Critical, "We are going to die :)");
    for (auto& n : nodes)
    {
        const auto tr = *n.popOutput(Node::Output::LogRecordMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(1, {0,   0,   0,   0,   0,   0,  0,   6,   22, 87,  101, 32,  97, 114, 101, 32,
                                   103, 111, 105, 110, 103, 32, 116, 111, 32, 100, 105, 101, 32, 58,  41}));
    }

    // Fails because the remote node-ID is not yet known.
    REQUIRE(!pres.requestFileRead(0xB'ADBA'DBADULL));

    // list(b''.join(pycyphal.dsdl.serialize(uavcan.node.ExecuteCommand_1_1.Request(
    //      uavcan.node.ExecuteCommand_1_1.Request.COMMAND_BEGIN_SOFTWARE_UPDATE, '/foo/bar/baz.app.bin'))))
    nodes.at(1).pushInput(Node::Input::ExecuteCommandRequest,
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
    REQUIRE(!nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(123, {0, 0, 0, 0, 0, 0, 0}, 3210));

    // The file location is known now.
    // list(b''.join(pycyphal.dsdl.serialize(uavcan.file.Read_1_1.Request(0xbadbadbad,
    //      uavcan.file.Path_2_0('/foo/bar/baz.app.bin')))))
    REQUIRE(pres.requestFileRead(0xB'ADBA'DBADULL));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::FileReadRequest)) ==
            Transfer(1,
                     {173, 219, 186, 173, 11,  20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98,  97,  122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));

    // Invalid request serialization. File location specifier not updated.
    nodes.at(0).pushInput(Node::Input::ExecuteCommandRequest, Transfer(333, std::vector<std::uint8_t>(), 3210));
    ts = std::chrono::microseconds{1'700'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!controller.popUpdateRequestedFlag());
    REQUIRE((*nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(333, {6, 0, 0, 0, 0, 0, 0}, 3210));  // Internal error.
    REQUIRE(!nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse));

    // Reboot request.
    // list(b''.join(pycyphal.dsdl.serialize(uavcan.node.ExecuteCommand_1_1.Request(
    //      uavcan.node.ExecuteCommand_1_1.Request.COMMAND_RESTART, ''))))
    nodes.at(1).pushInput(Node::Input::ExecuteCommandRequest, Transfer(444, {255, 255, 0}, 2222));
    ts = std::chrono::microseconds{1'800'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(controller.popRebootRequestedFlag());
    REQUIRE(!nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(444, {0, 0, 0, 0, 0, 0, 0}, 2222));

    // Node info request; app info not available. SR generation not shown.
    nodes.at(0).pushInput(Node::Input::NodeInfoRequest, Transfer(555, std::vector<std::uint8_t>{}, 3333));
    ts = std::chrono::microseconds{1'900'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(
        (*nodes.at(0).popOutput(Node::Output::NodeInfoResponse)) ==
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
    REQUIRE(!nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse));

    // Node info request; app info is available.
    // list(b''.join(pycyphal.dsdl.serialize(uavcan.node.GetInfo_1_0.Response(
    //      protocol_version=uavcan.node.Version_1_0(1, 0), hardware_version=uavcan.node.Version_1_0(33, 11),
    //      software_version=uavcan.node.Version_1_0(3, 11), software_vcs_revision_id=0xDEADDEADDEADDEAD,
    //      unique_id=bytes(range(1, 17)), name='com.zubax.kocherga.test.presenter',
    //      software_image_crc=[0xBADC0FFEE0DDF00D], certificate_of_authenticity=bytes(range(1, 223)[::-1])))))
    controller.setAppInfo(kocherga::AppInfo{
        0xBADC'0FFE'E0DD'F00DULL,
        0xD'EADBULL,
        {},
        {3, 11},
        2,
        {},
        1234567890,
        0xDEAD'DEAD'DEAD'DEADULL,
        {},
    });
    nodes.at(1).pushInput(Node::Input::NodeInfoRequest, Transfer(666, std::vector<std::uint8_t>{}, 1111));
    ts = std::chrono::microseconds{1'910'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse));
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
    const auto     node_info_response = *nodes.at(1).popOutput(Node::Output::NodeInfoResponse);
    REQUIRE(node_info_reference == node_info_response);

    // It's time for another heartbeat.
    pres.setNodeHealth(kocherga::detail::dsdl::Heartbeat::Health::Warning);
    pres.setNodeVSSC(0x0000);
    ts = std::chrono::microseconds{2'100'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
        const auto tr = *n.popOutput(Node::Output::HeartbeatMessage);
        std::cout << tr.toString() << std::endl;
        REQUIRE(tr == Transfer(1, {2, 0, 0, 0, 3, 3, 0}));
    }

    // Deliver the file read response.
    // Remember that the second update request was rejected so the actual file location is the old one.
    // This one will be rejected because it's the wrong node.
    // list(b''.join(pycyphal.dsdl.serialize(uavcan.file.Read_1_1.Response(uavcan.file.Error_1_0(0), b'The data.'))))
    nodes.at(0).pushInput(Node::Input::FileReadResponse,
                          Transfer(0, {0, 0, 9, 0, 84, 104, 101, 32, 100, 97, 116, 97, 46}, 3210));
    ts = std::chrono::microseconds{2'200'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!controller.popFileReadResult());
    // Correct node -- accepted.
    nodes.at(1).pushInput(Node::Input::FileReadResponse,
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
    nodes.at(1).pushInput(Node::Input::FileReadResponse,
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
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::FileReadRequest)) ==
            Transfer(2,
                     {64,  226, 1,  0,  0,   20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98, 97, 122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));

    // Successful request, but the response specifies a non-zero error code.
    nodes.at(1).setFileReadResult(true);
    REQUIRE(pres.requestFileRead(123'456));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::FileReadRequest)) ==
            Transfer(3,
                     {64,  226, 1,  0,  0,   20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98, 97, 122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));
    nodes.at(1).pushInput(Node::Input::FileReadResponse,
                          Transfer(1, {1, 0, 9, 0, 84, 104, 101, 32, 100, 97, 116, 97, 46}, 3210));
    ts = std::chrono::microseconds{2'400'000};
    pres.poll(ts);
    for (const auto& n : nodes)
    {
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!*controller.popFileReadResult());  // Empty option.

    // Successful request, but the response times out after the configured number of retries.
    REQUIRE(1 == params.request_retry_limit);  // Ensure the configuration matches the expectations.
    nodes.at(1).setFileReadResult(true);
    REQUIRE(pres.requestFileRead(123'456));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    REQUIRE(nodes.at(1).popOutput(Node::Output::FileReadRequest).value() ==
            Transfer(4,
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
    ts = std::chrono::microseconds{9'000'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.popOutput(Node::Output::HeartbeatMessage));
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    REQUIRE(nodes.at(1).popOutput(Node::Output::FileReadRequest).value() ==
            Transfer(5,  // transfer-ID incremented
                     {64,  226, 1,  0,  0,   20, 47, 102, 111, 111, 47, 98,  97,
                      114, 47,  98, 97, 122, 46, 97, 112, 112, 46,  98, 105, 110},
                     3210));
    REQUIRE(!controller.popFileReadResult());
    REQUIRE(!nodes.at(0).wasRequestCanceled());
    REQUIRE(!nodes.at(1).wasRequestCanceled());  // Not yet!
    ts = std::chrono::microseconds{15'000'000};
    pres.poll(ts);
    for (auto& n : nodes)
    {
        REQUIRE(n.popOutput(Node::Output::HeartbeatMessage));
        REQUIRE(n.getLastPollTime() == ts);
    }
    REQUIRE(!controller.popFileReadResult().value());  // Empty option.
    REQUIRE(!nodes.at(0).wasRequestCanceled());
    REQUIRE(nodes.at(1).wasRequestCanceled());  // Given up.
}
