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

#include <array>
#include <cstdint>

/**
 * The image definitions were generated with "xxd": https://stackoverflow.com/a/8707241/1007777
 * Example:
 *      xxd -c 16 -i demo-image-valid.bin
 */
namespace images
{
/**
 * A valid application image with the descriptor and the CRC signature.
 */
static constexpr std::uint8_t AppValidMajorVersion = 0;
static constexpr std::uint8_t AppValidMinorVersion = 1;
static constexpr std::uint32_t AppValidVCSCommit   = 0x5378b66fUL;
static constexpr std::array<std::uint8_t, 1024> AppValid{{
    0x00, 0x10, 0x00, 0x20, 0x61, 0xc3, 0x00, 0x08, 0x71, 0x45, 0x02, 0x08, 0x61, 0x45, 0x02, 0x08,
    0x51, 0x45, 0x02, 0x08, 0x41, 0x45, 0x02, 0x08, 0x31, 0x45, 0x02, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0xe1, 0x0b, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x81, 0x2f, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x31, 0x1a, 0x01, 0x08, 0xd1, 0xb2, 0x03, 0x08,
    0x31, 0xb0, 0x03, 0x08, 0x91, 0xad, 0x03, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0xa1, 0x2c, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x81, 0x2b, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x71, 0xb5, 0x03, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x01, 0xab, 0x03, 0x08,
    0x61, 0xa8, 0x03, 0x08, 0xc1, 0xa5, 0x03, 0x08, 0x62, 0xc3, 0x00, 0x08, 0xd1, 0x30, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x41, 0x50, 0x44, 0x65, 0x73, 0x63, 0x30, 0x30, 0xd3, 0x8d, 0xab, 0xe4, 0xe2, 0x0b, 0x56, 0x77,
    0x00, 0x04, 0x00, 0x00, 0x6f, 0xb6, 0x78, 0x53, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x31, 0xe6, 0x03, 0x08, 0xff, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0x72, 0xb6, 0x39, 0x48, 0x80, 0xf3, 0x08, 0x88, 0x38, 0x48, 0x80, 0xf3, 0x09, 0x88, 0x38, 0x48,
    0x4e, 0xf6, 0x08, 0x51, 0xce, 0xf2, 0x00, 0x01, 0x08, 0x60, 0x40, 0xf2, 0x00, 0x00, 0xcc, 0xf2,
    0x00, 0x00, 0x4e, 0xf6, 0x34, 0x71, 0xce, 0xf2, 0x00, 0x01, 0x08, 0x60, 0xbf, 0xf3, 0x4f, 0x8f,
    0xbf, 0xf3, 0x6f, 0x8f, 0x40, 0xf2, 0x00, 0x00, 0xc0, 0xf2, 0xf0, 0x00, 0x4e, 0xf6, 0x88, 0x51,
    0xce, 0xf2, 0x00, 0x01, 0x08, 0x60, 0xbf, 0xf3, 0x4f, 0x8f, 0xbf, 0xf3, 0x6f, 0x8f, 0x4f, 0xf0,
    0x00, 0x00, 0xe1, 0xee, 0x10, 0x0a, 0x4e, 0xf6, 0x3c, 0x71, 0xce, 0xf2, 0x00, 0x01, 0x08, 0x60,
    0x06, 0x20, 0x80, 0xf3, 0x14, 0x88, 0xbf, 0xf3, 0x6f, 0x8f, 0x0b, 0xf0, 0xa9, 0xfd, 0x02, 0xf0,
    0x3f, 0xfa, 0x4f, 0xf0, 0x55, 0x30, 0x1f, 0x49, 0x1b, 0x4a, 0x91, 0x42, 0x3c, 0xbf, 0x41, 0xf8,
    0x04, 0x0b, 0xfa, 0xe7, 0x1c, 0x49, 0x19, 0x4a, 0x91, 0x42, 0x3c, 0xbf, 0x41, 0xf8, 0x04, 0x0b,
    0xfa, 0xe7, 0x1a, 0x49, 0x1a, 0x4a, 0x1b, 0x4b, 0x9a, 0x42, 0x3e, 0xbf, 0x51, 0xf8, 0x04, 0x0b,
    0x42, 0xf8, 0x04, 0x0b, 0xf8, 0xe7, 0x00, 0x20, 0x17, 0x49, 0x18, 0x4a, 0x91, 0x42, 0x3c, 0xbf,
    0x41, 0xf8, 0x04, 0x0b, 0xfa, 0xe7, 0x0b, 0xf0, 0x3b, 0xfd, 0x0b, 0xf0, 0x79, 0xfd, 0x14, 0x4c,
    0x14, 0x4d, 0xac, 0x42, 0x03, 0xda, 0x54, 0xf8, 0x04, 0x1b, 0x88, 0x47, 0xf9, 0xe7, 0x1b, 0xf0,
    0xd7, 0xfd, 0x11, 0x4c, 0x11, 0x4d, 0xac, 0x42, 0x03, 0xda, 0x54, 0xf8, 0x04, 0x1b, 0x88, 0x47,
    0xf9, 0xe7, 0x0b, 0xf0, 0x5d, 0xbd, 0x00, 0x00, 0x00, 0x10, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20,
    0x00, 0xc0, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x10, 0x00, 0x20, 0x80, 0xd7, 0x05, 0x08,
    0x00, 0x20, 0x00, 0x20, 0x0c, 0x26, 0x00, 0x20, 0x40, 0x26, 0x00, 0x20, 0x0c, 0x8d, 0x01, 0x20,
    0x00, 0xc2, 0x00, 0x08, 0x04, 0xc2, 0x00, 0x08, 0x04, 0xc2, 0x00, 0x08, 0x04, 0xc2, 0x00, 0x08,
    0x6e, 0xe7, 0x18, 0xf0, 0x45, 0xff, 0xfe, 0xe7, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde,
    0x2d, 0xe9, 0xf0, 0x4f, 0x2d, 0xed, 0x10, 0x8a, 0xc1, 0xf8, 0x0c, 0xd0, 0xc3, 0x68, 0x9d, 0x46,
    0xbd, 0xec, 0x10, 0x8a, 0xbd, 0xe8, 0xf0, 0x8f, 0x04, 0xf0, 0xc2, 0xfd, 0x04, 0xf0, 0x80, 0xfc,
    0x00, 0x23, 0x83, 0xf3, 0x11, 0x88, 0x28, 0x46, 0xa0, 0x47, 0x00, 0x20, 0x09, 0xf0, 0x78, 0xfb,
    0x04, 0xf0, 0x86, 0xfc, 0x07, 0xf0, 0x94, 0xfd, 0x06, 0xf0, 0x12, 0xfa, 0x04, 0xf0, 0xb0, 0xfd,
    0x04, 0xf0, 0x6e, 0xfc, 0x00, 0xdf, 0xfe, 0xe7, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde,
    0xa2, 0xeb, 0x03, 0x00, 0x70, 0x47, 0x00, 0xbf, 0x02, 0x78, 0x0b, 0x78, 0x01, 0x2a, 0x28, 0xbf,
    0x9a, 0x42, 0xf5, 0xd1, 0x6d, 0xe9, 0x04, 0x45, 0x40, 0xea, 0x01, 0x04, 0xcd, 0xe9, 0x02, 0x67,
    0x6f, 0xf0, 0x00, 0x0c, 0x4f, 0xea, 0x44, 0x72, 0x12, 0xb3, 0x80, 0xea, 0x01, 0x04, 0x14, 0xf0,
    0x07, 0x0f, 0x6a, 0xd1, 0x00, 0xf0, 0x07, 0x04, 0x20, 0xf0, 0x07, 0x00, 0x04, 0xf0, 0x03, 0x05
}};

/**
 * An application image with a well-formed descriptor that contains invalid length and CRC.
 */
static constexpr std::array<std::uint8_t, 1024> AppWithInvalidDescriptor{{
    0x00, 0x10, 0x00, 0x20, 0x61, 0xc3, 0x00, 0x08, 0x71, 0x45, 0x02, 0x08, 0x61, 0x45, 0x02, 0x08,
    0x51, 0x45, 0x02, 0x08, 0x41, 0x45, 0x02, 0x08, 0x31, 0x45, 0x02, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0xe1, 0x0b, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x81, 0x2f, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x31, 0x1a, 0x01, 0x08, 0xd1, 0xb2, 0x03, 0x08,
    0x31, 0xb0, 0x03, 0x08, 0x91, 0xad, 0x03, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0xa1, 0x2c, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x81, 0x2b, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x71, 0xb5, 0x03, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x01, 0xab, 0x03, 0x08,
    0x61, 0xa8, 0x03, 0x08, 0xc1, 0xa5, 0x03, 0x08, 0x62, 0xc3, 0x00, 0x08, 0xd1, 0x30, 0x01, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08, 0x62, 0xc3, 0x00, 0x08,
    0x41, 0x50, 0x44, 0x65, 0x73, 0x63, 0x30, 0x30, 0x5b, 0x26, 0x9b, 0xd3, 0x98, 0xed, 0xdf, 0x96,
    0x90, 0x1d, 0x05, 0x00, 0x6f, 0xb6, 0x78, 0x53, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x31, 0xe6, 0x03, 0x08, 0xff, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0,
    0x72, 0xb6, 0x39, 0x48, 0x80, 0xf3, 0x08, 0x88, 0x38, 0x48, 0x80, 0xf3, 0x09, 0x88, 0x38, 0x48,
    0x4e, 0xf6, 0x08, 0x51, 0xce, 0xf2, 0x00, 0x01, 0x08, 0x60, 0x40, 0xf2, 0x00, 0x00, 0xcc, 0xf2,
    0x00, 0x00, 0x4e, 0xf6, 0x34, 0x71, 0xce, 0xf2, 0x00, 0x01, 0x08, 0x60, 0xbf, 0xf3, 0x4f, 0x8f,
    0xbf, 0xf3, 0x6f, 0x8f, 0x40, 0xf2, 0x00, 0x00, 0xc0, 0xf2, 0xf0, 0x00, 0x4e, 0xf6, 0x88, 0x51,
    0xce, 0xf2, 0x00, 0x01, 0x08, 0x60, 0xbf, 0xf3, 0x4f, 0x8f, 0xbf, 0xf3, 0x6f, 0x8f, 0x4f, 0xf0,
    0x00, 0x00, 0xe1, 0xee, 0x10, 0x0a, 0x4e, 0xf6, 0x3c, 0x71, 0xce, 0xf2, 0x00, 0x01, 0x08, 0x60,
    0x06, 0x20, 0x80, 0xf3, 0x14, 0x88, 0xbf, 0xf3, 0x6f, 0x8f, 0x0b, 0xf0, 0xa9, 0xfd, 0x02, 0xf0,
    0x3f, 0xfa, 0x4f, 0xf0, 0x55, 0x30, 0x1f, 0x49, 0x1b, 0x4a, 0x91, 0x42, 0x3c, 0xbf, 0x41, 0xf8,
    0x04, 0x0b, 0xfa, 0xe7, 0x1c, 0x49, 0x19, 0x4a, 0x91, 0x42, 0x3c, 0xbf, 0x41, 0xf8, 0x04, 0x0b,
    0xfa, 0xe7, 0x1a, 0x49, 0x1a, 0x4a, 0x1b, 0x4b, 0x9a, 0x42, 0x3e, 0xbf, 0x51, 0xf8, 0x04, 0x0b,
    0x42, 0xf8, 0x04, 0x0b, 0xf8, 0xe7, 0x00, 0x20, 0x17, 0x49, 0x18, 0x4a, 0x91, 0x42, 0x3c, 0xbf,
    0x41, 0xf8, 0x04, 0x0b, 0xfa, 0xe7, 0x0b, 0xf0, 0x3b, 0xfd, 0x0b, 0xf0, 0x79, 0xfd, 0x14, 0x4c,
    0x14, 0x4d, 0xac, 0x42, 0x03, 0xda, 0x54, 0xf8, 0x04, 0x1b, 0x88, 0x47, 0xf9, 0xe7, 0x1b, 0xf0,
    0xd7, 0xfd, 0x11, 0x4c, 0x11, 0x4d, 0xac, 0x42, 0x03, 0xda, 0x54, 0xf8, 0x04, 0x1b, 0x88, 0x47,
    0xf9, 0xe7, 0x0b, 0xf0, 0x5d, 0xbd, 0x00, 0x00, 0x00, 0x10, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20,
    0x00, 0xc0, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x10, 0x00, 0x20, 0x80, 0xd7, 0x05, 0x08,
    0x00, 0x20, 0x00, 0x20, 0x0c, 0x26, 0x00, 0x20, 0x40, 0x26, 0x00, 0x20, 0x0c, 0x8d, 0x01, 0x20,
    0x00, 0xc2, 0x00, 0x08, 0x04, 0xc2, 0x00, 0x08, 0x04, 0xc2, 0x00, 0x08, 0x04, 0xc2, 0x00, 0x08,
    0x6e, 0xe7, 0x18, 0xf0, 0x45, 0xff, 0xfe, 0xe7, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde,
    0x2d, 0xe9, 0xf0, 0x4f, 0x2d, 0xed, 0x10, 0x8a, 0xc1, 0xf8, 0x0c, 0xd0, 0xc3, 0x68, 0x9d, 0x46,
    0xbd, 0xec, 0x10, 0x8a, 0xbd, 0xe8, 0xf0, 0x8f, 0x04, 0xf0, 0xc2, 0xfd, 0x04, 0xf0, 0x80, 0xfc,
    0x00, 0x23, 0x83, 0xf3, 0x11, 0x88, 0x28, 0x46, 0xa0, 0x47, 0x00, 0x20, 0x09, 0xf0, 0x78, 0xfb,
    0x04, 0xf0, 0x86, 0xfc, 0x07, 0xf0, 0x94, 0xfd, 0x06, 0xf0, 0x12, 0xfa, 0x04, 0xf0, 0xb0, 0xfd,
    0x04, 0xf0, 0x6e, 0xfc, 0x00, 0xdf, 0xfe, 0xe7, 0xde, 0xad, 0xc0, 0xde, 0xde, 0xad, 0xc0, 0xde,
    0xa2, 0xeb, 0x03, 0x00, 0x70, 0x47, 0x00, 0xbf, 0x02, 0x78, 0x0b, 0x78, 0x01, 0x2a, 0x28, 0xbf,
    0x9a, 0x42, 0xf5, 0xd1, 0x6d, 0xe9, 0x04, 0x45, 0x40, 0xea, 0x01, 0x04, 0xcd, 0xe9, 0x02, 0x67,
    0x6f, 0xf0, 0x00, 0x0c, 0x4f, 0xea, 0x44, 0x72, 0x12, 0xb3, 0x80, 0xea, 0x01, 0x04, 0x14, 0xf0,
    0x07, 0x0f, 0x6a, 0xd1, 0x00, 0xf0, 0x07, 0x04, 0x20, 0xf0, 0x07, 0x00, 0x04, 0xf0, 0x03, 0x05
}};

}
