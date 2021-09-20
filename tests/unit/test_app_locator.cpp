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
    REQUIRE(info->image_size == 144);
    REQUIRE(info->image_crc == 0x452A'4267'971A'3928ULL);
    REQUIRE(info->timestamp_utc == 1234567890);
    REQUIRE(info->vcs_revision_id == 0xBADC'0FFE'E0DD'F00DULL);
    REQUIRE(info->isReleaseBuild());
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
    REQUIRE(info->image_size == 576);
    REQUIRE(info->image_crc == 0x8B61'938E'E5F9'0B1FULL);
    REQUIRE(info->timestamp_utc == 1234567890);
    REQUIRE(info->vcs_revision_id == 0x3333'3333'3333'3333ULL);
    REQUIRE(!info->isReleaseBuild());
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

TEST_CASE("AppLocator-legacy")
{
    const util::FileROMBackend rom(util::getImagePath("com.zubax.telega-1-0.3.68620b82.application.bin"));

    const kocherga::detail::AppLocator loc_ok(rom, 512UL * 1024U);

    REQUIRE(!loc_ok.identifyApplication());
    auto info = loc_ok.identifyApplication(true);
    REQUIRE(info);
    REQUIRE(info->image_size == 271'440U);
    REQUIRE(info->image_crc == 0x9EA4'7D98'DCC7'B58AULL);
    REQUIRE(info->version.at(0) == 0);
    REQUIRE(info->version.at(1) == 3);

    const kocherga::detail::AppLocator loc_too_small(rom, 128UL * 1024U);
    REQUIRE(!loc_too_small.identifyApplication());
}
