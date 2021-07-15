// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include "util.hpp"
#include <cstdlib>

TEST_CASE("AppLocator-good-simple")
{
    const util::FileROMBackend rom(
        util::getImagePath("good-le-simple-3.1.badc0ffee0ddf00d.452a4267971a3928.app.release.bin"));

    const kocherga::detail::AppLocator loc_ok(rom, 1024U);

    auto info = loc_ok.identifyApplication();
    REQUIRE(info);
    REQUIRE(info->image_size == 104);
    REQUIRE(info->image_crc == 0xBE8C'B17E'D02E'7A88ULL);
    REQUIRE(info->vcs_revision_id == 0xBADC'0FFE'E0DD'F00DULL);
    REQUIRE(info->isDebugBuild());
    REQUIRE(!info->isDirtyBuild());
    REQUIRE(info->version.at(0) == 3);
    REQUIRE(info->version.at(1) == 1);

    const kocherga::detail::AppLocator loc_too_small(rom, 64U);
    REQUIRE(!loc_too_small.identifyApplication());
}

TEST_CASE("AppLocator-good-3rd-entry")
{
    const util::FileROMBackend rom(
        util::getImagePath("good-le-3rd-entry-5.6.3333333333333333.8b61938ee5f90b1f.app.dirty.bin"));

    const kocherga::detail::AppLocator loc_ok(rom, 1024U);

    auto info = loc_ok.identifyApplication();
    REQUIRE(info);
    REQUIRE(info->image_size == 336);
    REQUIRE(info->image_crc == 0x60CC'9645'68BF'B6B0ULL);
    REQUIRE(info->vcs_revision_id == 0x3333'3333'3333'3333ULL);
    REQUIRE(!info->isDebugBuild());
    REQUIRE(info->isDirtyBuild());
    REQUIRE(info->version.at(0) == 5);
    REQUIRE(info->version.at(1) == 6);

    const kocherga::detail::AppLocator loc_too_small(rom, 64U);
    REQUIRE(!loc_too_small.identifyApplication());
}

TEST_CASE("AppLocator-bad-crc-x3")
{
    const util::FileROMBackend rom(util::getImagePath("bad-le-crc-x3.bin"));
    REQUIRE(!kocherga::detail::AppLocator(rom, 1024U).identifyApplication());
    REQUIRE(!kocherga::detail::AppLocator(rom, 100U).identifyApplication());
}

TEST_CASE("AppLocator-bad-short")
{
    const util::FileROMBackend rom(util::getImagePath("bad-le-short.bin"));
    REQUIRE(!kocherga::detail::AppLocator(rom, 1024U).identifyApplication());
    REQUIRE(!kocherga::detail::AppLocator(rom, 10U).identifyApplication());
}
