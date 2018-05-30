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

#pragma once

#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <cstdint>


namespace util
{

template <typename InputIterator>
inline std::string makeHexDump(InputIterator begin, const InputIterator end)
{
    static constexpr std::uint8_t BytesPerRow = 16;
    std::uint32_t offset = 0;
    std::ostringstream output;
    bool first = true;

    output << std::hex << std::setfill('0');

    do
    {
        if (first)
        {
            first = false;
        }
        else
        {
            output << std::endl;
        }

        output << std::setw(8) << offset << "  ";
        offset += BytesPerRow;

        {
            auto it = begin;
            for (std::uint8_t i = 0; i < BytesPerRow; ++i)
            {
                if (i == 8)
                {
                    output << ' ';
                }

                if (it != end)
                {
                    output << std::setw(2) << std::uint32_t(*it) << ' ';
                    ++it;
                }
                else
                {
                    output << "   ";
                }
            }
        }

        output << " ";
        for (std::uint8_t i = 0; i < BytesPerRow; ++i)
        {
            if (begin != end)
            {
                output << ((std::uint32_t(*begin) >= 32U && std::uint32_t(*begin) <= 126U) ? char(*begin) : '.');
                ++begin;
            }
            else
            {
                output << ' ';
            }
        }
    }
    while (begin != end);

    return output.str();
}


template <typename Container>
inline std::string makeHexDump(const Container& cont)
{
    return makeHexDump(std::begin(cont), std::end(cont));
}

}  // namespace util
