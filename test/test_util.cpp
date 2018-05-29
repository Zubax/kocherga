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

#include "util.hpp"
#include "catch.hpp"


TEST_CASE("Util-HexDump")
{
    REQUIRE("00000000  31 32 33                                          123             "
            == util::makeHexDump(std::string("123")));

    REQUIRE("00000000  30 31 32 33 34 35 36 37  38 39 61 62 63 64 65 66  0123456789abcdef\n"
            "00000010  67 68 69 6a 6b 6c 6d 6e  6f 70 71 72 73 74 75 76  ghijklmnopqrstuv\n"
            "00000020  77 78 79 7a 41 42 43 44  45 46 47 48 49 4a 4b 4c  wxyzABCDEFGHIJKL\n"
            "00000030  4d 4e 4f 50 51 52 53 54  55 56 57 58 59 5a        MNOPQRSTUVWXYZ  "
            == util::makeHexDump(std::string("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ")));
}
