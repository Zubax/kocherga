// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "util.hpp"
#include "catch.hpp"

TEST_CASE("util::makeHexDump")
{
    REQUIRE("00000000  31 32 33                                          123             " ==
            util::makeHexDump(std::string("123")));

    REQUIRE("00000000  30 31 32 33 34 35 36 37  38 39 61 62 63 64 65 66  0123456789abcdef\n"
            "00000010  67 68 69 6a 6b 6c 6d 6e  6f 70 71 72 73 74 75 76  ghijklmnopqrstuv\n"
            "00000020  77 78 79 7a 41 42 43 44  45 46 47 48 49 4a 4b 4c  wxyzABCDEFGHIJKL\n"
            "00000030  4d 4e 4f 50 51 52 53 54  55 56 57 58 59 5a        MNOPQRSTUVWXYZ  " ==
            util::makeHexDump(std::string("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ")));
}

TEST_CASE("util::FileROMBackend")
{
    (void) std::filesystem::remove("test_rom.bin.tmp");

    util::FileROMBackend back("test_rom.bin.tmp", 1024, 0xFFU);

    const std::vector<std::byte> ref(1024, std::byte{0xFF});
    REQUIRE(back.isSameImage(ref.data(), ref.size()));

    const std::vector<std::byte> ref2(123, std::byte{0xFF});
    REQUIRE(back.isSameImage(ref2.data(), ref2.size()));

    const std::vector<std::byte> ref3(123, std::byte{0xAA});
    REQUIRE(!back.isSameImage(ref3.data(), ref3.size()));

    const util::FileROMBackend existing("test_rom.bin.tmp");
    REQUIRE(existing.getSize() == 1024U);
    REQUIRE(existing.isSameImage(ref.data(), ref.size()));

    kocherga::IROMBackend& interface = back;

    std::array<std::byte, 32'768> buf1{};

    REQUIRE(0 == back.getReadCount());
    REQUIRE(0 == back.getWriteCount());

    // Reading
    REQUIRE(32 == interface.read(0, buf1.data(), 32));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 32, [](auto x) { return x == std::byte{0xFF}; }));

    REQUIRE(1024 == interface.read(0, buf1.data(), 1024));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 1024, [](auto x) { return x == std::byte{0xFF}; }));

    REQUIRE(512 == interface.read(512, buf1.data(), 1024));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 512, [](auto x) { return x == std::byte{0xFF}; }));

    REQUIRE(128 == interface.read(512, buf1.data(), 128));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 1024, [](auto x) { return x == std::byte{0xFF}; }));

    REQUIRE(0 == interface.read(1024, buf1.data(), 1));

    REQUIRE(1024 == interface.read(0, buf1.data(), 2000));
    std::all_of(buf1.begin(), buf1.begin() + 1024, [](auto x) { return x == std::byte{0xFF}; });
    REQUIRE(buf1[1024] == std::byte{0});
    REQUIRE(buf1[1025] == std::byte{0});
    REQUIRE(buf1[1026] == std::byte{0});

    REQUIRE(6 == back.getReadCount());
    REQUIRE(0 == back.getWriteCount());

    // Writing
    REQUIRE_THROWS_AS(interface.write(0, buf1.data(), 123), std::runtime_error);
    REQUIRE_THROWS_AS(interface.endWrite(), std::runtime_error);
    interface.beginWrite();
    buf1.fill(std::byte{0xAA});
    REQUIRE(128 == *interface.write(0, buf1.data(), 128));
    REQUIRE(128 == *interface.write(512, buf1.data(), 128));
    interface.endWrite();

    REQUIRE(1024 == interface.read(0, buf1.data(), 2000));
    REQUIRE(std::all_of(buf1.begin() + 0, buf1.begin() + 128, [](auto x) { return x == std::byte{0xAA}; }));
    REQUIRE(std::all_of(buf1.begin() + 128, buf1.begin() + 512, [](auto x) { return x == std::byte{0xFF}; }));
    REQUIRE(std::all_of(buf1.begin() + 512, buf1.begin() + 640, [](auto x) { return x == std::byte{0xAA}; }));

    REQUIRE(1024 == static_cast<const kocherga::IROMBackend&>(existing).read(0, buf1.data(), 2000));
    REQUIRE(std::all_of(buf1.begin() + 0, buf1.begin() + 128, [](auto x) { return x == std::byte{0xAA}; }));
    REQUIRE(std::all_of(buf1.begin() + 128, buf1.begin() + 512, [](auto x) { return x == std::byte{0xFF}; }));
    REQUIRE(std::all_of(buf1.begin() + 512, buf1.begin() + 640, [](auto x) { return x == std::byte{0xAA}; }));

    REQUIRE(7 == back.getReadCount());
    REQUIRE(3 == back.getWriteCount());

    // Failure injection
    interface.beginWrite();
    back.enableFailureInjection(true);
    REQUIRE(!interface.write(0, buf1.data(), 128));
    back.enableFailureInjection(false);
    REQUIRE(128 == *interface.write(0, buf1.data(), 128));

    REQUIRE(7 == back.getReadCount());
    REQUIRE(5 == back.getWriteCount());
}