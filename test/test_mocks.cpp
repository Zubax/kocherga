/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Zubax Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

// We want to ensure that assertion checks are enabled when tests are run, for extra safety
#ifdef NDEBUG
# undef NDEBUG
#endif

// The tested header must be included first to make sure that it doesn't have any hidden include dependencies.
#include "mocks.hpp"

#include "catch.hpp"
#include <thread>
#include <algorithm>


TEST_CASE("Mocks-Platform")
{
    using namespace mocks;

    Platform pl;

    const auto do_lock_frenzy = [](kocherga::IPlatform& platform)
    {
        for (std::uint16_t i = 0; i < 100; i++)
        {
            platform.lockMutex();
        }
    };

    REQUIRE(!pl.isMutexLocked());
    REQUIRE(0 == pl.getMutexLockCount());
    static_cast<kocherga::IPlatform&>(pl).lockMutex();
    REQUIRE(pl.isMutexLocked());
    REQUIRE(1 == pl.getMutexLockCount());
    static_cast<kocherga::IPlatform&>(pl).unlockMutex();
    REQUIRE(!pl.isMutexLocked());
    REQUIRE(1 == pl.getMutexLockCount());

    REQUIRE_THROWS_AS(do_lock_frenzy(pl), BadUsageException);
    REQUIRE(pl.isMutexLocked());
    REQUIRE(10 <= pl.getMutexLockCount());
    while (pl.isMutexLocked())
    {
        static_cast<kocherga::IPlatform&>(pl).unlockMutex();
    }
    REQUIRE(!pl.isMutexLocked());

    // Time check
    {
        static_cast<kocherga::IPlatform&>(pl).lockMutex();

        const auto a = pl.getMonotonicUptime();
        REQUIRE(a.count() > 0);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        REQUIRE(a < pl.getMonotonicUptime());

        static_cast<kocherga::IPlatform&>(pl).unlockMutex();
    }
}


TEST_CASE("Mocks-ROM")
{
    using namespace mocks;

    FileMappedROMBackend back("mock-test-rom.tmp", 1024);

    {
        const std::vector<std::uint8_t> ref(1024, 0xFF);
        REQUIRE(back.isSameImage(ref.data(), ref.size()));

        const std::vector<std::uint8_t> ref2(123, 0xFF);
        REQUIRE(back.isSameImage(ref2.data(), ref2.size()));

        const std::vector<std::uint8_t> ref3(123, 0xAA);
        REQUIRE(!back.isSameImage(ref3.data(), ref3.size()));
    }

    auto& interface = static_cast<kocherga::IROMBackend&>(back);

    std::array<std::uint8_t, 32768> buf1{};

    REQUIRE(0 == back.getReadCount());
    REQUIRE(0 == back.getWriteCount());

    // Reading
    REQUIRE(32 == interface.read(0, buf1.data(), 32));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 32, [](auto x) { return x == 0xFF; }));

    REQUIRE(1024 == interface.read(0, buf1.data(), 1024));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 1024, [](auto x) { return x == 0xFF; }));

    REQUIRE(512 == interface.read(512, buf1.data(), 1024));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 512, [](auto x) { return x == 0xFF; }));

    REQUIRE(128 == interface.read(512, buf1.data(), 128));
    REQUIRE(std::all_of(buf1.begin(), buf1.begin() + 1024, [](auto x) { return x == 0xFF; }));

    REQUIRE(0 == interface.read(1024, buf1.data(), 1));

    back.setFailureInjector([](std::int16_t) { return -1; });
    REQUIRE(-1 == interface.read(0, buf1.data(), 1024));
    back.setFailureInjector({});

    REQUIRE(1024 == interface.read(0, buf1.data(), 2000));
    std::all_of(buf1.begin(), buf1.begin() + 1024, [](auto x) { return x == 0xFF; });
    REQUIRE(buf1[1024] == 0);
    REQUIRE(buf1[1025] == 0);
    REQUIRE(buf1[1026] == 0);

    REQUIRE_THROWS_AS(interface.read(0, buf1.data(), 32768), BadUsageException);

    REQUIRE(8 == back.getReadCount());
    REQUIRE(0 == back.getWriteCount());

    // Writing
    REQUIRE_THROWS_AS(interface.write(0, buf1.data(), 123), BadUsageException);
    REQUIRE_THROWS_AS(interface.endUpgrade(true), BadUsageException);
    REQUIRE_THROWS_AS(interface.endUpgrade(false), BadUsageException);
    REQUIRE(0 == interface.beginUpgrade());
    buf1.fill(0xAA);
    REQUIRE_THROWS_AS(interface.write(0, buf1.data(), 32768), BadUsageException);
    REQUIRE(128 == interface.write(0, buf1.data(), 128));
    REQUIRE(128 == interface.write(512, buf1.data(), 128));
    REQUIRE(0 == interface.endUpgrade(true));

    REQUIRE(1024 == interface.read(0, buf1.data(), 2000));
    REQUIRE(std::all_of(buf1.begin() + 0,   buf1.begin() + 128, [](auto x) { return x == 0xAA; }));
    REQUIRE(std::all_of(buf1.begin() + 128, buf1.begin() + 512, [](auto x) { return x == 0xFF; }));
    REQUIRE(std::all_of(buf1.begin() + 512, buf1.begin() + 640, [](auto x) { return x == 0xAA; }));

    REQUIRE(9 == back.getReadCount());
    REQUIRE(4 == back.getWriteCount());
}
