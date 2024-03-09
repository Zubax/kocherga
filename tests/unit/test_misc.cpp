// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include "util.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>

TEST_CASE("CRC64")
{
    kocherga::CRC64 crc;
    const char*     val = "12345";
    crc.update(reinterpret_cast<const std::uint8_t*>(val), 5);  // NOSONAR NOLINT reinterpret_cast
    crc.update(nullptr, 0);
    val = "6789";
    crc.update(reinterpret_cast<const std::uint8_t*>(val), 4);  // NOSONAR NOLINT reinterpret_cast

    REQUIRE(0x62EC'59E3'F1A4'F00AULL == crc.get());
    REQUIRE(crc.getBytes().at(0) == 0x62U);
    REQUIRE(crc.getBytes().at(1) == 0xECU);
    REQUIRE(crc.getBytes().at(2) == 0x59U);
    REQUIRE(crc.getBytes().at(3) == 0xE3U);
    REQUIRE(crc.getBytes().at(4) == 0xF1U);
    REQUIRE(crc.getBytes().at(5) == 0xA4U);
    REQUIRE(crc.getBytes().at(6) == 0xF0U);
    REQUIRE(crc.getBytes().at(7) == 0x0AU);

    REQUIRE(!crc.isResidueCorrect());
    crc.update(crc.getBytes().data(), 8);
    REQUIRE(crc.isResidueCorrect());
    REQUIRE(0xFCAC'BEBD'5931'A992ULL == (~crc.get()));
}

TEST_CASE("CRC16-CCITT")
{
    kocherga::detail::CRC16CCITT crc;
    crc.update(static_cast<std::uint8_t>('1'));
    crc.update(static_cast<std::uint8_t>('2'));
    crc.update(static_cast<std::uint8_t>('3'));
    crc.update(static_cast<std::uint8_t>('4'));
    crc.update(static_cast<std::uint8_t>('5'));
    crc.update(static_cast<std::uint8_t>('6'));
    crc.update(static_cast<std::uint8_t>('7'));
    crc.update(static_cast<std::uint8_t>('8'));
    crc.update(static_cast<std::uint8_t>('9'));

    REQUIRE(0x29B1U == crc.get());
    REQUIRE(crc.getBytes().at(0) == 0x29U);
    REQUIRE(crc.getBytes().at(1) == 0xB1U);

    REQUIRE(!crc.isResidueCorrect());
    crc.update(0x29U);
    crc.update(0xB1U);
    REQUIRE(crc.isResidueCorrect());
    REQUIRE(0x0000U == crc.get());
}

TEST_CASE("VolatileStorage")
{
    struct Data
    {
        std::uint64_t               a;
        std::uint8_t                b;
        std::array<std::uint8_t, 3> c;
    };
    static_assert(sizeof(Data) <= 16);
    static_assert(kocherga::VolatileStorage<Data>::StorageSize == (sizeof(Data) + 8U));

    std::array<std::uint8_t, kocherga::VolatileStorage<Data>::StorageSize> arena{};

    kocherga::VolatileStorage<Data> marshaller(arena.data());

    // The storage is empty, checking
    REQUIRE(!marshaller.take());

    // Writing zeros and checking the representation
    marshaller.store(Data());
    std::cout << util::makeHexDump(arena) << std::endl;
    REQUIRE(std::all_of(arena.begin(), arena.begin() + 12, [](auto x) { return x == 0; }));
    REQUIRE(std::any_of(arena.begin() + sizeof(Data), arena.end(), [](auto x) { return x != 0; }));

    // Reading and making sure it's erased afterwards
    auto rd = marshaller.take();
    REQUIRE(rd);
    REQUIRE(rd->a == 0);
    REQUIRE(rd->b == 0);
    REQUIRE(rd->c[0] == 0);
    REQUIRE(rd->c[1] == 0);
    REQUIRE(rd->c[2] == 0);

    std::cout << util::makeHexDump(arena) << std::endl;
    REQUIRE(std::all_of(arena.begin(), arena.end(), [](auto x) { return x == 0xCAU; }));
    REQUIRE(!marshaller.take());

    // Writing non-zeros and checking the representation
    marshaller.store({
        0x11AD'EADB'ADC0'FFEE,
        123,
        {{1, 2, 3}},
    });
    std::cout << util::makeHexDump(arena) << std::endl;

    // Reading and making sure it's erased afterwards
    rd = marshaller.take();
    REQUIRE(rd);
    REQUIRE(rd->a == 0x11AD'EADB'ADC0'FFEE);
    REQUIRE(rd->b == 123);
    REQUIRE(rd->c[0] == 1);
    REQUIRE(rd->c[1] == 2);
    REQUIRE(rd->c[2] == 3);
    std::cout << util::makeHexDump(arena) << std::endl;
    REQUIRE(std::all_of(arena.begin(), arena.end(), [](auto x) { return x == 0xCAU; }));
    REQUIRE(!marshaller.take());
}
