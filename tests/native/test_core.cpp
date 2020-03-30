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
    static constexpr auto               coa_capacity = 222U;
    std::array<std::byte, coa_capacity> coa{};
    for (auto i = 0U; i < coa_capacity; i++)
    {
        coa.at(i) = static_cast<std::byte>(coa.size() - i);
    }
    return kocherga::SystemInfo{
        kocherga::SemanticVersion{33, 11},
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
        "com.zubax.kocherga.test.core",
        222,
        coa.data(),
    };
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
                               static_cast<std::size_t>(std::filesystem::file_size(img)));

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

TEST_CASE("Bootloader-update")
{
    using std::chrono_literals::operator""ms;
    using mock::Node;
    using mock::Transfer;

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

    REQUIRE(!bl.poll(500ms));
    REQUIRE(bl.getState() == kocherga::State::BootCanceled);

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
    REQUIRE(!bl.poll(600ms));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::ExecuteCommandResponse));
    REQUIRE((*nodes.at(1).popOutput(Node::Output::ExecuteCommandResponse)) == Transfer(444, {0, 0, 0, 0, 0, 0, 0}, 0));

    // FIRST READ REQUEST
    REQUIRE(!bl.poll(700ms));
    REQUIRE(nodes.at(0).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(nodes.at(1).popOutput(Node::Output::LogRecordMessage));
    REQUIRE(!nodes.at(0).popOutput(Node::Output::FileReadRequest));
    // list(b''.join(pyuavcan.dsdl.serialize(uavcan.file.Read_1_0.Request(0,
    //      uavcan.file.Path_1_0('good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img')))))
    const auto received = *nodes.at(1).popOutput(Node::Output::FileReadRequest);
    const auto reference =
        Transfer(0,
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
    REQUIRE(!bl.poll(800ms));  // Results will appear on the SECOND poll.
    REQUIRE(kocherga::Final::BootApp == *bl.poll(810ms));
    std::cout << nodes.at(0).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    std::cout << nodes.at(1).popOutput(Node::Output::LogRecordMessage)->toString() << std::endl;
    REQUIRE(bl.getState() == kocherga::State::BootDelay);

    // NEW APPLICATION IS NOW AVAILABLE
    ai = *bl.getAppInfo();
    REQUIRE(0xBE8C'B17E'D02E'7A88ULL == ai.image_crc);
    REQUIRE(0xBADC'0FFE'E0DD'F00DULL == ai.vcs_commit);
    REQUIRE(3 == ai.version.at(0));
    REQUIRE(1 == ai.version.at(1));
    REQUIRE(ai.isDebugBuild());
    REQUIRE(!ai.isDirtyBuild());
}
