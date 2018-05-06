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

#define KOCHERGA_TRACE std::printf

// The library headers must be included first to make sure that they don't have any hidden include dependencies.
#include <kocherga_ymodem.hpp>

#include "catch.hpp"
#include "mocks.hpp"

#include <thread>
#include <numeric>
#include <functional>


namespace
{

class SerialPort final : public kocherga_ymodem::IYModemSerialPort
{
    Result emit(std::uint8_t byte, std::chrono::microseconds timeout) final
    {
        (void) byte;
        (void) timeout;
        return Result::Error;
    }

    Result receive(std::uint8_t& out_byte, std::chrono::microseconds timeout) final
    {
        (void) out_byte;
        (void) timeout;
        return Result::Error;
    }
};

}


TEST_CASE("YModem-Basic")
{
    static constexpr std::uint32_t ROMSize = 1024 * 1024;

    mocks::Platform platform;
    mocks::FileMappedROMBackend rom_backend("ymodem-rom.tmp", ROMSize);

    kocherga::BootloaderController blc(platform, rom_backend, ROMSize, std::chrono::seconds(1));

    SerialPort port;
    kocherga_ymodem::YModemProtocol ym(platform, port);

    (void) ym;
}
