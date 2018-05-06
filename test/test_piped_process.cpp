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

// The tested header must be included first to make sure that it doesn't have any hidden include dependencies.
#include "piped_process.hpp"

#include "catch.hpp"
#include <thread>
#include <iostream>


TEST_CASE("PipedProcess-Echo")
{
    auto p = piped_process::launch("echo 123");
    REQUIRE(p);
    REQUIRE(p->getPID() > 0);
    REQUIRE(p->getInputFD() > 0);
    REQUIRE(p->getOutputFD() > 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    char buf[256]{};
    const auto res = p->readOutput(&buf[0], 4);
    REQUIRE(res);
    REQUIRE(*res == 4);
    std::cout << "Echo: " << &buf[0] << std::endl;
    REQUIRE(std::string(&buf[0]) == "123\n");
}


TEST_CASE("PipedProcess-Tee")
{
    auto p = piped_process::launch("tee");
    REQUIRE(p);
    REQUIRE(p->getPID() > 0);
    REQUIRE(p->getInputFD() > 0);
    REQUIRE(p->getOutputFD() > 0);

    {
        const auto res = p->writeInput("12345\n", 6);
        REQUIRE(res);
        REQUIRE(*res == 6);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
        char buf[256]{};
        const auto res = p->readOutput(&buf[0], 6);
        REQUIRE(res);
        REQUIRE(*res == 6);
        std::cout << "Tee: " << &buf[0] << std::endl;
        REQUIRE(std::string(&buf[0]) == "12345\n");
    }
}
