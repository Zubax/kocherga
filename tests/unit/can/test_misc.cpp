// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_can.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include <numeric>

TEST_CASE("can::CRC")
{
    kocherga::can::detail::CRC16CCITT crc;
    crc.update(3, reinterpret_cast<const std::uint8_t*>("123"));
    REQUIRE(0x5BCEU == crc.get());
    REQUIRE(crc.getBytes().at(0) == 0x5BU);
    REQUIRE(crc.getBytes().at(1) == 0xCEU);
    REQUIRE(!crc.isResidueCorrect());
    crc.update(0x5BU);
    crc.update(0xCEU);
    REQUIRE(crc.isResidueCorrect());
    REQUIRE(0 == crc.get());
}

TEST_CASE("can::BlockAllocator")
{
    kocherga::can::detail::BlockAllocator<8, 2> ba;

    static std::int64_t counter = 0;

    struct Foo final
    {
        const std::int64_t field;

        explicit Foo(const std::int64_t f) : field(f) { counter++; }
        ~Foo() { counter--; }

        [[maybe_unused]] Foo(const Foo&) = delete;
        [[maybe_unused]] Foo(Foo&&)      = delete;
        auto operator=(const Foo&) -> Foo& = delete;
        auto operator=(Foo&&) -> Foo& = delete;
    };

    REQUIRE(counter == 0);
    auto* a = ba.construct<Foo>(1234567890);
    REQUIRE(a);
    REQUIRE(a->field == 1234567890);
    REQUIRE(counter == 1);
    auto* b = ba.construct<Foo>(9876543210);
    REQUIRE(b);
    REQUIRE(b->field == 9876543210);
    REQUIRE(counter == 2);
    auto* c = ba.construct<Foo>(55555);
    REQUIRE(c == nullptr);

    ba.destroy(a);
    REQUIRE(counter == 1);
    c = ba.construct<Foo>(1111111);
    REQUIRE(counter == 2);
    REQUIRE(c);
    REQUIRE(c->field == 1111111);

    ba.destroy(b);
    REQUIRE(counter == 1);
    ba.destroy(c);
    REQUIRE(counter == 0);
}

TEST_CASE("can::parseFrame")
{
    //
}
