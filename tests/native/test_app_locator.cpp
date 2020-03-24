// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "catch.hpp"
#include "kocherga.hpp"
#include "util.hpp"
#include <cstdlib>
#include <iostream>

namespace
{
auto getImagePath(const std::string& name) -> std::filesystem::path
{
    const auto out = util::getSourceDir() / "images" / name;
    std::cout << "Using image file: " << out << std::endl;
    return out;
}

}  // namespace

TEST_CASE("AppLocator-good-simple")
{
    const util::FileROMBackend rom(getImagePath("good-le-simple-3.1.badc0ffee0ddf00d.be8cb17ed02e7a88,debug.img"));

    const kocherga::detail::AppLocator loc_ok(rom, 1024U);

    auto info = loc_ok.identifyApplication();
    REQUIRE(info);
    REQUIRE(info->image_size == 104);
    REQUIRE(info->image_crc == 0xBE8C'B17E'D02E'7A88ULL);
    REQUIRE(info->vcs_commit == 0xBADC'0FFE'E0DD'F00DULL);
    REQUIRE(info->reserved == 0U);
    REQUIRE(info->isDebugBuild());
    REQUIRE(!info->isDirtyBuild());
    REQUIRE(info->version.at(0) == 3);
    REQUIRE(info->version.at(1) == 1);

    const kocherga::detail::AppLocator loc_too_small(rom, 64U);
    REQUIRE(!loc_too_small.identifyApplication());
}

TEST_CASE("AppLocator-good-3rd-entry")
{
    const util::FileROMBackend rom(getImagePath("good-le-3rd-entry-5.6.3333333333333333.60cc964568bfb6b0,dirty.img"));

    const kocherga::detail::AppLocator loc_ok(rom, 1024U);

    auto info = loc_ok.identifyApplication();
    REQUIRE(info);
    REQUIRE(info->image_size == 336);
    REQUIRE(info->image_crc == 0x60CC'9645'68BF'B6B0ULL);
    REQUIRE(info->vcs_commit == 0x3333'3333'3333'3333ULL);
    REQUIRE(info->reserved == 0U);
    REQUIRE(!info->isDebugBuild());
    REQUIRE(info->isDirtyBuild());
    REQUIRE(info->version.at(0) == 5);
    REQUIRE(info->version.at(1) == 6);

    const kocherga::detail::AppLocator loc_too_small(rom, 64U);
    REQUIRE(!loc_too_small.identifyApplication());
}

TEST_CASE("AppLocator-bad-crc-x3")
{
    const util::FileROMBackend rom(getImagePath("bad-le-crc-x3.img"));
    REQUIRE(!kocherga::detail::AppLocator(rom, 1024U).identifyApplication());
    REQUIRE(!kocherga::detail::AppLocator(rom, 100U).identifyApplication());
}

TEST_CASE("AppLocator-bad-short")
{
    const util::FileROMBackend rom(getImagePath("bad-le-short.img"));
    REQUIRE(!kocherga::detail::AppLocator(rom, 1024U).identifyApplication());
    REQUIRE(!kocherga::detail::AppLocator(rom, 10U).identifyApplication());
}