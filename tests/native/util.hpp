// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <utility>

namespace util
{
static constexpr std::uint8_t                          BitsPerByte = 8U;
static constexpr std::pair<std::uint8_t, std::uint8_t> PrintableASCIIRange{32, 126};

template <typename InputIterator>
inline auto makeHexDump(InputIterator begin, const InputIterator end) -> std::string
{
    static constexpr std::uint8_t BytesPerRow = BitsPerByte * 2;
    std::uint32_t                 offset      = 0;
    std::ostringstream            output;
    bool                          first = true;
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

        output << std::setw(BitsPerByte) << offset << "  ";
        offset += BytesPerRow;

        auto it = begin;
        for (std::uint8_t i = 0; i < BytesPerRow; ++i)
        {
            if (i == BitsPerByte)
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

        output << " ";
        for (std::uint8_t i = 0; i < BytesPerRow; ++i)
        {
            if (begin != end)
            {
                output << ((std::uint32_t(*begin) >= PrintableASCIIRange.first &&
                            std::uint32_t(*begin) <= PrintableASCIIRange.second)
                               ? char(*begin)
                               : '.');
                ++begin;
            }
            else
            {
                output << ' ';
            }
        }
    } while (begin != end);
    return output.str();
}

template <typename Container>
inline auto makeHexDump(const Container& cont)
{
    return makeHexDump(std::begin(cont), std::end(cont));
}

}  // namespace util
