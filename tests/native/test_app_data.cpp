// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "catch.hpp"
#include "kocherga.hpp"
#include <numeric>

TEST_CASE("makeAppDataMarshaller_registers")
{
    struct Data
    {
        std::uint64_t               a = 0;
        std::uint8_t                b = 0;
        std::array<std::uint8_t, 3> c{};
    };
    static_assert(sizeof(Data) <= 16);

    struct RegisterBank
    {
        volatile std::uint64_t crc = 0;
        volatile std::uint64_t a   = 0;
        volatile std::uint8_t  b   = 0;
        volatile std::uint8_t  c0  = 0;
        volatile std::uint8_t  c1  = 0;
        volatile std::uint8_t  c2  = 0;
        volatile std::uint8_t  c3  = 0;
        volatile std::uint8_t  c4  = 0;
        volatile std::uint8_t  c5  = 0;
        volatile std::uint8_t  c6  = 0;
    } reg_bank;
    static_assert(sizeof(RegisterBank) <= 24);

    auto marshaller = kocherga::makeAppDataMarshaller<Data>(&reg_bank.crc,
                                                            &reg_bank.a,
                                                            &reg_bank.b,
                                                            &reg_bank.c0,
                                                            &reg_bank.c1,
                                                            &reg_bank.c2,
                                                            &reg_bank.c3,
                                                            &reg_bank.c4,
                                                            &reg_bank.c5,
                                                            &reg_bank.c6);
    // The storage is empty, checking
    REQUIRE(!marshaller.readAndErase());

    // Writing zeros and checking the representation
    marshaller.write(Data());
    REQUIRE(reg_bank.crc != 0);
    REQUIRE(reg_bank.a == 0);
    REQUIRE(reg_bank.b == 0);
    REQUIRE(reg_bank.c0 == 0);
    REQUIRE(reg_bank.c1 == 0);
    REQUIRE(reg_bank.c2 == 0);
    REQUIRE(reg_bank.c3 == 0);
    REQUIRE(reg_bank.c4 == 0);
    REQUIRE(reg_bank.c5 == 0);
    REQUIRE(reg_bank.c6 == 0);

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0);
        REQUIRE(rd->b == 0);
        REQUIRE(rd->c[0] == 0);
        REQUIRE(rd->c[1] == 0);
        REQUIRE(rd->c[2] == 0);
    }

    REQUIRE(reg_bank.crc == 0);
    REQUIRE(reg_bank.a == 0);
    REQUIRE(reg_bank.b == 0);
    REQUIRE(reg_bank.c0 == 0);
    REQUIRE(reg_bank.c1 == 0);
    REQUIRE(reg_bank.c2 == 0);
    REQUIRE(reg_bank.c3 == 0);
    REQUIRE(reg_bank.c4 == 0);
    REQUIRE(reg_bank.c5 == 0);
    REQUIRE(reg_bank.c6 == 0);

    REQUIRE(!marshaller.readAndErase());

    // Writing non-zeros and checking the representation
    marshaller.write({
        0x00AD'EADB'ADC0'FFEE,
        123,
        {{1, 2, 3}},
    });
    REQUIRE(reg_bank.crc != 0);
    REQUIRE(reg_bank.a == 0x00AD'EADB'ADC0'FFEE);
    REQUIRE(reg_bank.b == 123);
    REQUIRE(reg_bank.c0 == 1);
    REQUIRE(reg_bank.c1 == 2);
    REQUIRE(reg_bank.c2 == 3);
    REQUIRE(reg_bank.c3 == 0);
    REQUIRE(reg_bank.c4 == 0);
    REQUIRE(reg_bank.c5 == 0);
    REQUIRE(reg_bank.c6 == 0);

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0x00AD'EADB'ADC0'FFEE);
        REQUIRE(rd->b == 123);
        REQUIRE(rd->c[0] == 1);
        REQUIRE(rd->c[1] == 2);
        REQUIRE(rd->c[2] == 3);
    }

    REQUIRE(reg_bank.crc == 0);
    REQUIRE(reg_bank.a == 0);
    REQUIRE(reg_bank.b == 0);
    REQUIRE(reg_bank.c0 == 0);
    REQUIRE(reg_bank.c1 == 0);
    REQUIRE(reg_bank.c2 == 0);
    REQUIRE(reg_bank.c3 == 0);
    REQUIRE(reg_bank.c4 == 0);
    REQUIRE(reg_bank.c5 == 0);
    REQUIRE(reg_bank.c6 == 0);

    REQUIRE(!marshaller.readAndErase());
}

TEST_CASE("makeAppDataMarshaller_memory")
{
    struct Data
    {
        std::uint64_t               a = 0;
        std::uint8_t                b = 0;
        std::array<std::uint8_t, 3> c{};
    };
    static_assert(sizeof(Data) <= 16);

    std::array<std::uint8_t, 100> arena{};

    auto marshaller = kocherga::makeAppDataMarshaller<Data>(static_cast<void*>(arena.data()));

    // The storage is empty, checking
    REQUIRE(!marshaller.readAndErase());

    // Writing zeros and checking the representation
    marshaller.write(Data());
    REQUIRE(0 != std::accumulate(arena.begin(), arena.begin() + 8, 0ULL));  // First 8 bytes are CRC, non-zero
    REQUIRE(0 == std::accumulate(arena.begin() + 8, arena.end(), 0ULL));    // The payload is zeroed out

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0);
        REQUIRE(rd->b == 0);
        REQUIRE(rd->c[0] == 0);
        REQUIRE(rd->c[1] == 0);
        REQUIRE(rd->c[2] == 0);
    }

    REQUIRE(0 == std::accumulate(arena.begin(), arena.end(), 0ULL));
    REQUIRE(!marshaller.readAndErase());

    // Writing non-zeros and checking the representation
    marshaller.write({
        0x00AD'EADB'ADC0'FFEE,
        123,
        {{1, 2, 3}},
    });
    REQUIRE(0 != std::accumulate(arena.begin(), arena.end(), 0ULL));  // All non-zero

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0x00AD'EADB'ADC0'FFEE);
        REQUIRE(rd->b == 123);
        REQUIRE(rd->c[0] == 1);
        REQUIRE(rd->c[1] == 2);
        REQUIRE(rd->c[2] == 3);
    }

    REQUIRE(0 == std::accumulate(arena.begin(), arena.end(), 0ULL));
    REQUIRE(!marshaller.readAndErase());
}
