// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include "mock.hpp"
#include <iostream>

namespace
{
inline auto getSysInfo()
{
    static constexpr auto                  coa_capacity = 222U;
    std::array<std::uint8_t, coa_capacity> coa{};
    for (auto i = 0U; i < coa_capacity; i++)
    {
        coa.at(i) = static_cast<std::uint8_t>(coa.size() - i);
    }
    return kocherga::SystemInfo{
        kocherga::SemanticVersion{33, 11},
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
        "com.zubax.kocherga.test.core",
        222,
        coa.data(),
    };
}

template <std::size_t NumNodes>
inline auto checkHeartbeat(std::array<mock::Node, NumNodes>&               nodes,
                           const std::optional<kocherga::TransferID>       tid,
                           const std::uint32_t                             uptime,
                           const kocherga::detail::dsdl::Heartbeat::Health health,
                           const std::uint32_t                             vssc) -> bool
{
    using kocherga::detail::dsdl::Heartbeat;
    for (auto& n : nodes)
    {
        const mock::Transfer tr  = *n.popOutput(mock::Node::Output::HeartbeatMessage);
        const auto           hmv = (vssc << 5U) | static_cast<std::uint32_t>(Heartbeat::ModeSoftwareUpdate << 2U) |
                         static_cast<std::uint32_t>(health);
        const mock::Transfer ref(tid ? *tid : tr.transfer_id,
                                 {
                                     static_cast<std::uint8_t>(uptime >> 0U),
                                     static_cast<std::uint8_t>(uptime >> 8U),
                                     static_cast<std::uint8_t>(uptime >> 16U),
                                     static_cast<std::uint8_t>(uptime >> 24U),
                                     static_cast<std::uint8_t>(hmv >> 0U),
                                     static_cast<std::uint8_t>(hmv >> 8U),
                                     static_cast<std::uint8_t>(hmv >> 24U),
                                 });
        if (tr != ref)
        {
            std::cout << ref.toString() << tr.toString() << std::endl;
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("Bootloader-fast-boot")
{
    using std::chrono_literals::operator""ms;

    const auto           sys = getSysInfo();
    const auto           img = util::getImagePath("good-le-simple-3.1.badc0ffee0ddf00d.be8cb17ed02e7a88,debug.img");
    util::FileROMBackend rom(img);
    std::array<mock::Node, 3> nodes;
    kocherga::Bootloader<3>   bl(rom,
                               sys,
                               {&nodes.at(0), &nodes.at(1), &nodes.at(2)},
                               static_cast<std::size_t>(std::filesystem::file_size(img)),
                               false);

    REQUIRE(bl.poll(500ms) == kocherga::Final::BootApp);
    REQUIRE(bl.getState() == kocherga::State::BootDelay);

    auto ai = *bl.getAppInfo();
    REQUIRE(0xBE8C'B17E'D02E'7A88ULL == ai.image_crc);
    REQUIRE(0xBADC'0FFE'E0DD'F00DULL == ai.vcs_commit);
    REQUIRE(3 == ai.version.at(0));
    REQUIRE(1 == ai.version.at(1));
    REQUIRE(ai.isDebugBuild());
    REQUIRE(!ai.isDirtyBuild());
}

TEST_CASE("Bootloader-boot-delay")
{
    using std::chrono_literals::operator""s;
    using std::chrono_literals::operator""ms;

    const auto           sys = getSysInfo();
    const auto           img = util::getImagePath("good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img");
    util::FileROMBackend rom(img);
    mock::Node           node;
    kocherga::Bootloader<1> bl(rom, sys, {&node}, static_cast<std::size_t>(std::filesystem::file_size(img)), false, 1s);

    REQUIRE(!bl.poll(500ms));
    REQUIRE(bl.getState() == kocherga::State::BootDelay);

    auto ai = *bl.getAppInfo();
    REQUIRE(0x60CC'9645'68BF'B6B0ULL == ai.image_crc);
    REQUIRE(0x3333'3333'3333'3333ULL == ai.vcs_commit);
    REQUIRE(5 == ai.version.at(0));
    REQUIRE(6 == ai.version.at(1));
    REQUIRE(!ai.isDebugBuild());
    REQUIRE(ai.isDirtyBuild());

    REQUIRE(!bl.poll(900ms));
    REQUIRE(bl.getState() == kocherga::State::BootDelay);

    REQUIRE(bl.poll(1900ms) == kocherga::Final::BootApp);
    REQUIRE(bl.getState() == kocherga::State::BootDelay);
}

TEST_CASE("Bootloader-linger-reboot")
{
    using std::chrono_literals::operator""ms;
    using mock::Node;
    using mock::Transfer;

    const auto           sys = getSysInfo();
    const auto           img = util::getImagePath("good-le-simple-3.1.badc0ffee0ddf00d.be8cb17ed02e7a88,debug.img");
    util::FileROMBackend rom(img);
    std::array<mock::Node, 2> nodes;
    kocherga::Bootloader<2>   bl(rom,
                               sys,
                               {&nodes.at(0), &nodes.at(1)},
                               static_cast<std::size_t>(std::filesystem::file_size(img)),
                               true);

    REQUIRE(!bl.poll(500ms));
    REQUIRE(bl.getState() == kocherga::State::BootCanceled);  // LINGER -- NO BOOT

    auto ai = *bl.getAppInfo();
    REQUIRE(0xBE8C'B17E'D02E'7A88ULL == ai.image_crc);
    REQUIRE(0xBADC'0FFE'E0DD'F00DULL == ai.vcs_commit);
    REQUIRE(3 == ai.version.at(0));
    REQUIRE(1 == ai.version.at(1));
    REQUIRE(ai.isDebugBuild());
    REQUIRE(!ai.isDirtyBuild());

    nodes.at(1).pushInput(Node::Input::ExecuteCommandRequest, Transfer(444, {255, 255, 0}, 2222));
    REQUIRE(bl.poll(600ms) == kocherga::Final::Restart);
    REQUIRE(!nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(444, {0, 0, 0, 0, 0, 0, 0}, 2222));
}

TEST_CASE("Bootloader-update-valid")
{
    using std::chrono_literals::operator""ms;
    using mock::Node;
    using mock::Transfer;
    using kocherga::detail::dsdl::Heartbeat;

    const auto sys = getSysInfo();
    const auto img = util::getImagePath("good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img");
    REQUIRE(std::filesystem::copy_file(img, "rom.img.tmp", std::filesystem::copy_options::overwrite_existing));
    util::FileROMBackend      rom("rom.img.tmp");
    std::array<mock::Node, 2> nodes;
    kocherga::Bootloader<2>   bl(rom,
                               sys,
                               {&nodes.at(0), &nodes.at(1)},
                               static_cast<std::size_t>(std::filesystem::file_size(img)),
                               true);

    REQUIRE(!bl.poll(1'500ms));
    REQUIRE(bl.getState() == kocherga::State::BootCanceled);
    REQUIRE(checkHeartbeat(nodes, 0, 1, Heartbeat::Health::Advisory, 0));

    auto ai = *bl.getAppInfo();
    REQUIRE(0x60CC'9645'68BF'B6B0ULL == ai.image_crc);
    REQUIRE(0x3333'3333'3333'3333ULL == ai.vcs_commit);
    REQUIRE(5 == ai.version.at(0));
    REQUIRE(6 == ai.version.at(1));
    REQUIRE(!ai.isDebugBuild());
    REQUIRE(ai.isDirtyBuild());

    // REQUEST UPDATE
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.ExecuteCommand_1_0.Request(
    //    uavcan.node.ExecuteCommand_1_0.Request.COMMAND_BEGIN_SOFTWARE_UPDATE,
    //    'good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img'))))
    nodes.at(1).pushInput(Node::Input::ExecuteCommandRequest,
                          Transfer(444,
                                   {253, 255, 65,  103, 111, 111, 100, 45, 108, 101, 45,  51,  114, 100, 45,  101, 110,
                                    116, 114, 121, 45,  53,  46,  54,  46, 51,  51,  51,  51,  51,  51,  51,  51,  51,
                                    51,  51,  51,  51,  51,  51,  51,  46, 54,  48,  99,  99,  57,  54,  52,  53,  54,
                                    56,  98,  102, 98,  54,  98,  48,  44, 100, 105, 114, 116, 121, 46,  105, 109, 103},
                                   0));
    REQUIRE(!bl.poll(2'100ms));
    REQUIRE(checkHeartbeat(nodes, 1, 2, Heartbeat::Health::Nominal, 0));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse)) == Transfer(444, {0, 0, 0, 0, 0, 0, 0}, 0));

    // FIRST READ REQUEST
    REQUIRE(!bl.poll(2'200ms));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(nodes.at(1).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.file.Read_1_0.Request(0,
    //      uavcan.file.Path_1_0('good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img')))))
    const auto received = *nodes.at(1).popOutput(Node::Output::FileReadRequest);
    const auto reference =
        Transfer(1,
                 {0,   0,   0,   0,   0,   65, 103, 111, 111, 100, 45,  108, 101, 45, 51,  114, 100, 45,
                  101, 110, 116, 114, 121, 45, 53,  46,  54,  46,  51,  51,  51,  51, 51,  51,  51,  51,
                  51,  51,  51,  51,  51,  51, 51,  51,  46,  54,  48,  99,  99,  57, 54,  52,  53,  54,
                  56,  98,  102, 98,  54,  98, 48,  44,  100, 105, 114, 116, 121, 46, 105, 109, 103},
                 0);
    std::cout << received.toString() << reference.toString() << std::endl;
    REQUIRE(received == reference);

    // READ RESPONSE
    // The serialized representation was constructed manually from the binary file:
    //      good-le-simple-3.1.badc0ffee0ddf00d.be8cb17ed02e7a88,debug.img
    nodes.at(1).pushInput(Node::Input::FileReadResponse,
                          Transfer(0,
                                   {0,   0,   128, 0,   72,  101, 108, 108, 111, 32,  119, 111, 114, 108, 100, 33,  32,
                                    32,  32,  32,  199, 196, 192, 111, 20,  21,  68,  94,  136, 122, 46,  208, 126, 177,
                                    140, 190, 104, 0,   0,   0,   0,   0,   0,   0,   13,  240, 221, 224, 254, 15,  220,
                                    186, 0,   0,   0,   0,   1,   0,   3,   1,   0,   0,   0,   0,   0,   0,   1,   16,
                                    0,   0,   84,  104, 101, 32,  97,  112, 112, 108, 105, 99,  97,  116, 105, 111, 110,
                                    32,  105, 109, 97,  103, 101, 32,  103, 111, 101, 115, 32,  104, 101, 114, 101, 0,
                                    0,   0,   0,   0,   0,   0,   255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
                                   0));
    (void) bl.poll(2'300ms);  // Results will appear on the SECOND poll.
    REQUIRE(kocherga::Final::BootApp == *bl.poll(2'400ms));
    std::cout << nodes.at(0).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    std::cout << nodes.at(1).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    REQUIRE(bl.getState() == kocherga::State::BootDelay);
    REQUIRE(kocherga::Final::BootApp == *bl.poll(2'500ms));  // All subsequent calls yield the same Final.
    REQUIRE(kocherga::Final::BootApp == *bl.poll(2'600ms));  // Yep.

    // NEW APPLICATION IS NOW AVAILABLE
    ai = *bl.getAppInfo();
    REQUIRE(0xBE8C'B17E'D02E'7A88ULL == ai.image_crc);
    REQUIRE(0xBADC'0FFE'E0DD'F00DULL == ai.vcs_commit);
    REQUIRE(3 == ai.version.at(0));
    REQUIRE(1 == ai.version.at(1));
    REQUIRE(ai.isDebugBuild());
    REQUIRE(!ai.isDirtyBuild());
}

TEST_CASE("Bootloader-update-invalid")  // NOLINT NOSONAR complexity threshold
{
    using std::chrono_literals::operator""s;
    using std::chrono_literals::operator""ms;
    using mock::Node;
    using mock::Transfer;
    using kocherga::detail::dsdl::Heartbeat;

    const auto sys = getSysInfo();
    const auto img = util::getImagePath("good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img");
    REQUIRE(std::filesystem::copy_file(img, "rom.img.tmp", std::filesystem::copy_options::overwrite_existing));
    util::FileROMBackend      rom("rom.img.tmp");
    std::array<mock::Node, 1> nodes;
    kocherga::Bootloader<1>   bl(rom,
                               sys,
                               {&nodes.at(0)},
                               static_cast<std::size_t>(std::filesystem::file_size(img)),
                               false,
                               2s);

    REQUIRE(!bl.poll(1'100ms));
    REQUIRE(bl.getState() == kocherga::State::BootDelay);
    REQUIRE(checkHeartbeat(nodes, 0, 1, Heartbeat::Health::Nominal, 0));

    auto ai = *bl.getAppInfo();
    REQUIRE(0x60CC'9645'68BF'B6B0ULL == ai.image_crc);
    REQUIRE(0x3333'3333'3333'3333ULL == ai.vcs_commit);
    REQUIRE(5 == ai.version.at(0));
    REQUIRE(6 == ai.version.at(1));
    REQUIRE(!ai.isDebugBuild());
    REQUIRE(ai.isDirtyBuild());

    // REQUEST UPDATE
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.node.ExecuteCommand_1_0.Request(
    //    uavcan.node.ExecuteCommand_1_0.Request.COMMAND_BEGIN_SOFTWARE_UPDATE, 'bad-le-crc-x3.img'))))
    nodes.at(0).pushInput(Node::Input::ExecuteCommandRequest,
                          Transfer(111,
                                   {253, 255, 17, 98, 97,  100, 45, 108, 101, 45,
                                    99,  114, 99, 45, 120, 51,  46, 105, 109, 103},
                                   1111));
    REQUIRE(!bl.poll(1'600ms));
    REQUIRE((*nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(111, {0, 0, 0, 0, 0, 0, 0}, 1111));

    REQUIRE(!bl.poll(2'100ms));
    REQUIRE(checkHeartbeat(nodes, 1, 2, Heartbeat::Health::Nominal, 1));

    // FIRST READ REQUEST
    REQUIRE(!bl.poll(2'200ms));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.file.Read_1_0.Request(0,
    //      uavcan.file.Path_1_0('bad-le-crc-x3.img')))))
    auto received = *nodes.at(0).popOutput(Node::Output::FileReadRequest);
    auto reference =
        Transfer(1,
                 {0, 0, 0, 0, 0, 17, 98, 97, 100, 45, 108, 101, 45, 99, 114, 99, 45, 120, 51, 46, 105, 109, 103},
                 1111);
    std::cout << received.toString() << reference.toString() << std::endl;
    REQUIRE(received == reference);

    // READ RESPONSE
    // The serialized representation was constructed manually from the binary file: bad-le-crc-x3.img
    nodes.at(0).pushInput(Node::Input::FileReadResponse,
                          Transfer(0,
                                   {
                                       0x00, 0x00, 0x00, 0x01, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x77, 0x6f, 0x72,
                                       0x6c, 0x64, 0x3f, 0x20, 0x20, 0x20, 0x20, 0xc7, 0xc4, 0xc0, 0x6f, 0x14, 0x15,
                                       0x44, 0x5e, 0x88, 0x7a, 0x2e, 0xd0, 0x7e, 0xb1, 0x8c, 0xbe, 0xff, 0xff, 0xff,
                                       0xff, 0xff, 0x00, 0x00, 0x00, 0x0d, 0xf0, 0xdd, 0xe0, 0xfe, 0x0f, 0xdc, 0xba,
                                       0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x01, 0x10, 0x00, 0x00, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x48,
                                       0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x3f, 0x20, 0x20,
                                       0x20, 0x20, 0xc7, 0xc4, 0xc0, 0x6f, 0x14, 0x15, 0x44, 0x5e, 0x88, 0x7a, 0x2e,
                                       0xd0, 0x7e, 0xb1, 0x8c, 0xbe, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x0d, 0xf0, 0xdd, 0xe0, 0xfe, 0x0f, 0xdc, 0xba, 0x00, 0x00, 0x00, 0x00, 0x01,
                                       0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
                                       0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20,
                                       0x77, 0x6f, 0x72, 0x6c, 0x64, 0x3f, 0x20, 0x20, 0x20, 0x20, 0xc7, 0xc4, 0xc0,
                                       0x6f, 0x14, 0x15, 0x44, 0x5e, 0x88, 0x7a, 0x2e, 0xd0, 0x7e, 0xb1, 0x8c, 0xbe,
                                   },
                                   1111));
    REQUIRE(!bl.poll(3'100ms));
    REQUIRE(bl.getState() == kocherga::State::AppUpdateInProgress);
    REQUIRE(checkHeartbeat(nodes, 2, 3, Heartbeat::Health::Nominal, 1));
    REQUIRE(!bl.poll(3'200ms));
    REQUIRE(bl.getState() == kocherga::State::AppUpdateInProgress);

    // SECOND READ REQUEST
    REQUIRE(!bl.poll(3'300ms));
    received = *nodes.at(0).popOutput(Node::Output::FileReadRequest);
    reference =
        Transfer(2,
                 {0, 1, 0, 0, 0, 17, 98, 97, 100, 45, 108, 101, 45, 99, 114, 99, 45, 120, 51, 46, 105, 109, 103},
                 1111);
    std::cout << received.toString() << reference.toString() << std::endl;
    REQUIRE(received == reference);

    // READ TIMEOUT, NO VALID APPLICATION TO BOOT
    // Due to the large time leap, we will be getting a heartbeat every poll() from now on until the leap is corrected.
    REQUIRE(!bl.poll(10'100ms));
    REQUIRE(bl.getState() == kocherga::State::NoAppToBoot);
    REQUIRE(checkHeartbeat(nodes, 3, 10, Heartbeat::Health::Warning, 0));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));

    // RESPONSE ERROR CODE
    nodes.at(0).pushInput(Node::Input::ExecuteCommandRequest,
                          Transfer(112,
                                   {253, 255, 17, 98, 97,  100, 45, 108, 101, 45,
                                    99,  114, 99, 45, 120, 51,  46, 105, 109, 103},
                                   2222));
    REQUIRE(!bl.poll(10'200ms));
    REQUIRE((*nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(112, {0, 0, 0, 0, 0, 0, 0}, 2222));
    REQUIRE(checkHeartbeat(nodes, 4, 10, Heartbeat::Health::Nominal, 0));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    nodes.at(0).pushInput(Node::Input::FileReadResponse, Transfer(2, {0xAD, 0xDE, 0x00, 0x00}, 2222));
    REQUIRE(!bl.poll(10'300ms));
    REQUIRE(bl.getState() == kocherga::State::NoAppToBoot);
    REQUIRE(checkHeartbeat(nodes, 5, 10, Heartbeat::Health::Warning, 0));
    REQUIRE(nodes.at(0).popOutput(Node::Output::FileReadRequest));  // Not checked
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));

    // SEND REQUEST FAILURE
    nodes.at(0).setFileReadResult(false);
    nodes.at(0).pushInput(Node::Input::ExecuteCommandRequest,
                          Transfer(113,
                                   {253, 255, 17, 98, 97,  100, 45, 108, 101, 45,
                                    99,  114, 99, 45, 120, 51,  46, 105, 109, 103},
                                   3333));
    REQUIRE(!bl.poll(10'400ms));
    REQUIRE((*nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse)) ==
            Transfer(113, {0, 0, 0, 0, 0, 0, 0}, 3333));
    REQUIRE(checkHeartbeat(nodes, 6, 10, Heartbeat::Health::Nominal, 0));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(!bl.poll(10'500ms));
    REQUIRE(bl.getState() == kocherga::State::NoAppToBoot);
    (void) nodes.at(0).popOutput(Node::Output::HeartbeatMessage);
    REQUIRE(nodes.at(0).popOutput(Node::Output::FileReadRequest));  // Not checked
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));

    // ROM WRITE FAILURE
    nodes.at(0).setFileReadResult(true);
    rom.enableFailureInjection(true);
    nodes.at(0).pushInput(Node::Input::ExecuteCommandRequest,
                          Transfer(114,
                                   {253, 255, 17, 98, 97,  100, 45, 108, 101, 45,
                                    99,  114, 99, 45, 120, 51,  46, 105, 109, 103},
                                   3210));
    REQUIRE(!bl.poll(10'600ms));
    (void) nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse);
    (void) nodes.at(0).popOutput(Node::Output::HeartbeatMessage);
    (void) nodes.at(0).popOutput(Node::Output::LogRecordMessage);
    REQUIRE(!bl.poll(10'700ms));
    REQUIRE(bl.getState() == kocherga::State::AppUpdateInProgress);
    (void) nodes.at(0).popOutput(Node::Output::HeartbeatMessage);
    (void) nodes.at(0).popOutput(Node::Output::LogRecordMessage);
    REQUIRE(nodes.at(0).popOutput(Node::Output::FileReadRequest));
    nodes.at(0).pushInput(Node::Input::FileReadResponse,
                          Transfer(3,
                                   {0x00, 0x00, 0x09, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa},
                                   3210));
    REQUIRE(!bl.poll(10'800ms));
    (void) nodes.at(0).popOutput(Node::Output::HeartbeatMessage);
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(!bl.poll(11'100ms));
    REQUIRE(checkHeartbeat(nodes, {}, 11, Heartbeat::Health::Warning, 0));
    REQUIRE(bl.getState() == kocherga::State::NoAppToBoot);
    REQUIRE(!bl.getAppInfo());
}

TEST_CASE("Bootloader-trigger")
{
    using std::chrono_literals::operator""s;
    using std::chrono_literals::operator""ms;
    using mock::Node;
    using mock::Transfer;
    using kocherga::detail::dsdl::Heartbeat;

    const auto sys = getSysInfo();
    const auto img = util::getImagePath("good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img");
    REQUIRE(std::filesystem::copy_file(img, "rom.img.tmp", std::filesystem::copy_options::overwrite_existing));
    util::FileROMBackend      rom("rom.img.tmp");
    std::array<mock::Node, 3> nodes;
    kocherga::Bootloader<3>   bl(rom,
                               sys,
                               {&nodes.at(0), &nodes.at(1), &nodes.at(2)},
                               static_cast<std::size_t>(std::filesystem::file_size(img)),
                               false,
                               1s);

    REQUIRE(!bl.poll(100ms));
    REQUIRE(bl.getState() == kocherga::State::BootDelay);

    auto ai = *bl.getAppInfo();
    REQUIRE(0x60CC'9645'68BF'B6B0ULL == ai.image_crc);
    REQUIRE(0x3333'3333'3333'3333ULL == ai.vcs_commit);
    REQUIRE(5 == ai.version.at(0));
    REQUIRE(6 == ai.version.at(1));
    REQUIRE(!ai.isDebugBuild());
    REQUIRE(ai.isDirtyBuild());

    // MANUAL UPDATE TRIGGER
    const auto path =
        reinterpret_cast<const std::uint8_t*>("good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img");
    bl.trigger<1>(0, 65, path);
    REQUIRE(bl.getState() == kocherga::State::AppUpdateInProgress);
    REQUIRE(!bl.poll(1'100ms));
    REQUIRE(checkHeartbeat(nodes, 0, 1, Heartbeat::Health::Nominal, 1));

    // FIRST READ REQUEST
    REQUIRE(!bl.poll(1'200ms));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(nodes.at(1).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(nodes.at(2).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.file.Read_1_0.Request(0,
    //      uavcan.file.Path_1_0('good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img')))))
    const auto received = *nodes.at(1).popOutput(Node::Output::FileReadRequest);
    const auto reference =
        Transfer(1,
                 {0,   0,   0,   0,   0,   65, 103, 111, 111, 100, 45,  108, 101, 45, 51,  114, 100, 45,
                  101, 110, 116, 114, 121, 45, 53,  46,  54,  46,  51,  51,  51,  51, 51,  51,  51,  51,
                  51,  51,  51,  51,  51,  51, 51,  51,  46,  54,  48,  99,  99,  57, 54,  52,  53,  54,
                  56,  98,  102, 98,  54,  98, 48,  44,  100, 105, 114, 116, 121, 46, 105, 109, 103},
                 0);
    std::cout << received.toString() << reference.toString() << std::endl;
    REQUIRE(received == reference);

    // READ RESPONSE
    // The serialized representation was constructed manually from the binary file:
    //      good-le-simple-3.1.badc0ffee0ddf00d.be8cb17ed02e7a88,debug.img
    nodes.at(1).pushInput(Node::Input::FileReadResponse,
                          Transfer(0,
                                   {0,   0,   128, 0,   72,  101, 108, 108, 111, 32,  119, 111, 114, 108, 100, 33,  32,
                                    32,  32,  32,  199, 196, 192, 111, 20,  21,  68,  94,  136, 122, 46,  208, 126, 177,
                                    140, 190, 104, 0,   0,   0,   0,   0,   0,   0,   13,  240, 221, 224, 254, 15,  220,
                                    186, 0,   0,   0,   0,   1,   0,   3,   1,   0,   0,   0,   0,   0,   0,   1,   16,
                                    0,   0,   84,  104, 101, 32,  97,  112, 112, 108, 105, 99,  97,  116, 105, 111, 110,
                                    32,  105, 109, 97,  103, 101, 32,  103, 111, 101, 115, 32,  104, 101, 114, 101, 0,
                                    0,   0,   0,   0,   0,   0,   255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
                                   0));
    (void) bl.poll(1'300ms);  // Results will appear on the SECOND poll.
    REQUIRE(bl.getState() == kocherga::State::BootDelay);
    REQUIRE(kocherga::Final::BootApp == *bl.poll(2'400ms));
    std::cout << nodes.at(0).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    std::cout << nodes.at(1).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    std::cout << nodes.at(2).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    REQUIRE(kocherga::Final::BootApp == *bl.poll(2'500ms));  // All subsequent calls yield the same Final.
    REQUIRE(kocherga::Final::BootApp == *bl.poll(2'600ms));  // Yep.

    // NEW APPLICATION IS NOW AVAILABLE
    ai = *bl.getAppInfo();
    REQUIRE(0xBE8C'B17E'D02E'7A88ULL == ai.image_crc);
    REQUIRE(0xBADC'0FFE'E0DD'F00DULL == ai.vcs_commit);
    REQUIRE(3 == ai.version.at(0));
    REQUIRE(1 == ai.version.at(1));
    REQUIRE(ai.isDebugBuild());
    REQUIRE(!ai.isDirtyBuild());

    // BAD TRIGGER, GOOD TRIGGER
    REQUIRE(bl.trigger(&nodes.at(0), 0, 65, path));
    REQUIRE(bl.getState() == kocherga::State::AppUpdateInProgress);
    mock::Node stray_node;
    REQUIRE(!bl.trigger(&stray_node, 0, 65, path));
}
