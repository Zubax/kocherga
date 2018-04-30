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

#include "catch.hpp"
#include "mocks.hpp"
#include <thread>


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
    static_cast<kocherga::IPlatform&>(pl).lockMutex();
    REQUIRE(pl.isMutexLocked());
    static_cast<kocherga::IPlatform&>(pl).unlockMutex();
    REQUIRE(!pl.isMutexLocked());

    REQUIRE_THROWS_AS(do_lock_frenzy(pl), BadUsageException);
    REQUIRE(pl.isMutexLocked());
    while (pl.isMutexLocked())
    {
        static_cast<kocherga::IPlatform&>(pl).unlockMutex();
    }
    REQUIRE(!pl.isMutexLocked());

    // Time check
    {
        const auto a = pl.getMonotonicUptime();
        REQUIRE(a.count() > 0);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        REQUIRE(a < pl.getMonotonicUptime());
    }
}
