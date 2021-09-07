// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_serial.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include <numeric>

TEST_CASE("serial::CRC")
{
    kocherga::serial::detail::CRC32C crc;
    crc.update(static_cast<std::uint8_t>('1'));
    crc.update(static_cast<std::uint8_t>('2'));
    crc.update(static_cast<std::uint8_t>('3'));
    crc.update(static_cast<std::uint8_t>('4'));
    crc.update(static_cast<std::uint8_t>('5'));
    crc.update(static_cast<std::uint8_t>('6'));
    crc.update(static_cast<std::uint8_t>('7'));
    crc.update(static_cast<std::uint8_t>('8'));
    crc.update(static_cast<std::uint8_t>('9'));

    REQUIRE(0xE306'9283UL == crc.get());
    REQUIRE(crc.getBytes().at(0) == 0x83U);
    REQUIRE(crc.getBytes().at(1) == 0x92U);
    REQUIRE(crc.getBytes().at(2) == 0x06U);
    REQUIRE(crc.getBytes().at(3) == 0xE3U);

    REQUIRE(!crc.isResidueCorrect());
    crc.update(0x83U);
    crc.update(0x92U);
    crc.update(0x06U);
    crc.update(0xE3U);
    REQUIRE(crc.isResidueCorrect());
    REQUIRE(0xB798'B438UL == (~crc.get()));
}
