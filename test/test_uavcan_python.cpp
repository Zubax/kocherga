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

#define KOCHERGA_TRACE          std::printf
#define KOCHERGA_UAVCAN_LOG     std::printf

// The library headers must be included first to make sure that they don't have any hidden include dependencies.
#include <kocherga_uavcan.hpp>

#include <drivers/socketcan/socketcan.h>

#include "catch.hpp"
#include "mocks.hpp"
#include "images.hpp"

#include <thread>
#include <numeric>
#include <functional>
#include <iostream>
#include <utility>
#include <sys/prctl.h>
#include <csignal>


namespace
{
/**
 * A CAN interface with this name MUST exist in the system in order for the test to succeed.
 */
const std::string IfaceName = "kocherga0";  // NOLINT

/**
 * Platform mock.
 * The API is not thread-safe.
 */
class Platform final : public kocherga_uavcan::IUAVCANPlatform
{
    static constexpr std::chrono::seconds WatchdogTimeout{3};  // NOLINT

    std::chrono::steady_clock::time_point last_watchdog_reset_at_ = std::chrono::steady_clock::now();

    bool should_exit_ = false;

    std::optional<SocketCANInstance> socketcan_;
    CANAcceptanceFilterConfig can_acceptance_filter_{};
    CANMode can_mode_{};

    kocherga::BootloaderController& blc_;

    std::function<bool ()> exit_checker_;


    void resetWatchdog() override
    {
        const auto n = std::chrono::steady_clock::now();
        const auto dif = n - last_watchdog_reset_at_;
        if (dif >= WatchdogTimeout)
        {
            throw std::logic_error("Watchdog would reset! Interval: " +
                                   std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(dif).count()) +
                                   " ms");
        }

        last_watchdog_reset_at_ = n;
    }

    void sleep(std::chrono::microseconds duration) const override
    {
        // Ensure that we're not sleeping for more than 1 second to avoid watchdog timeouts
        assert(duration < std::chrono::seconds(1));
        std::this_thread::sleep_for(duration);
    }

    std::uint64_t getRandomUnsignedInteger(std::uint64_t lower_bound,
                                           std::uint64_t upper_bound) const override
    {
        if (lower_bound < upper_bound)
        {
            const auto rnd = std::uint64_t(std::rand()) * std::uint64_t(std::rand());  // NOLINT
            const std::uint64_t out = lower_bound + rnd % (upper_bound - lower_bound);
            assert(out >= lower_bound);
            assert(out < upper_bound);
            return out;
        }
        else
        {
            assert(false);
            return lower_bound;
        }
    }

    std::int16_t configure(std::uint32_t bitrate,
                           CANMode mode,
                           const CANAcceptanceFilterConfig& acceptance_filter) override
    {
        (void) bitrate;
        KOCHERGA_TRACE("UAVCAN test: Configuring CAN; bitrate %u, mode %u, filter 0x%08x/0x%08x\n",
                       unsigned(bitrate),
                       unsigned(mode),
                       unsigned(acceptance_filter.id),
                       unsigned(acceptance_filter.mask));

        if (socketcan_)
        {
            (void) socketcanClose(&*socketcan_);
        }

        socketcan_.emplace();
        if (auto res = socketcanInit(&*socketcan_, IfaceName.c_str()); res < 0)
        {
            throw std::runtime_error("Could not init SocketCAN interface: errno=" + std::to_string(errno));
        }

        can_mode_ = mode;
        can_acceptance_filter_ = acceptance_filter;

        return 0;
    }

    std::int16_t send(const ::CanardCANFrame& frame, std::chrono::microseconds timeout) override
    {
        if (can_mode_ == CANMode::Silent)
        {
            throw std::logic_error("Attempting to send() while in silent mode!");
        }

        if (!socketcan_)
        {
            throw std::logic_error("Attempting to use a not configured CAN interface");
        }

        return std::int16_t(
            socketcanTransmit(&*socketcan_, &frame,
                              int(std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count())));
    }

    std::pair<std::int16_t, ::CanardCANFrame> receive(std::chrono::microseconds timeout) override
    {
        if (!socketcan_)
        {
            throw std::logic_error("Attempting to use a not configured CAN interface");
        }

        while (true)
        {
            ::CanardCANFrame frame{};
            const int res = socketcanReceive(&*socketcan_, &frame,
                                             int(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 timeout).count()));
            if (res > 0)
            {
                // Software acceptance filter emulation
                if (((frame.id & can_acceptance_filter_.mask) ^ can_acceptance_filter_.id) == 0)
                {
                    return {1, frame};
                }
            }
            else if (res == 0)
            {
                return {0, {}};
            }
            else
            {
                return {-1, {}};
            }
        }
    }

    bool shouldExit() const override
    {
        return should_exit_ || (blc_.getState() == kocherga::State::ReadyToBoot) || exit_checker_();
    }

    bool tryScheduleReboot() override
    {
        if (!should_exit_)
        {
            should_exit_ = true;
            return true;
        }
        else
        {
            return false;
        }
    }

public:
    Platform(kocherga::BootloaderController& blc,
             std::function<bool ()> exit_checker) :
        blc_(blc),
        exit_checker_(std::move(exit_checker))
    {
        if (!exit_checker_)
        {
            exit_checker_ = []() { return false; };
        }
    }
};

}  // namespace


TEST_CASE("UAVCAN-Python-slow")
{
    // Making sure the interface exists. This is how you add it manually (as root):
    //     modprobe can
    //     modprobe can_raw
    //     modprobe vcan
    //     ip link add dev kocherga0 type vcan
    //     ip link set up kocherga0
    //     ifconfig kocherga0 up
    REQUIRE(std::system(("ifconfig " + IfaceName).c_str()) == 0);

    // For debugging reasons it helps to have this printed
    std::cout << "KOCHERGA_TEST_SOURCE_DIR: " << KOCHERGA_TEST_SOURCE_DIR << std::endl;

    volatile bool should_exit = false;
    std::thread node_thread([&]() {
        while (!should_exit)
        {
            std::cout << "Node is (re)starting..." << std::endl;

            mocks::Platform platform;
            static constexpr std::uint32_t ROMSize = 1024 * 1024;
            mocks::FileMappedROMBackend rom_backend("uavcan-python-rom.tmp", ROMSize);
            kocherga::BootloaderController blc(platform, rom_backend, ROMSize);
            kocherga_uavcan::HardwareInfo hw_info;
            hw_info.major = 12;
            hw_info.minor = 34;
            hw_info.unique_id = {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};
            for (std::uint16_t i = 0; i < hw_info.certificate_of_authenticity.capacity(); i++)
            {
                hw_info.certificate_of_authenticity.push_back(std::uint8_t(i));
            }

            Platform uavcan_platform(blc, [&]() { return should_exit; });

            kocherga_uavcan::BootloaderNode node(blc, uavcan_platform, "com.zubax.kocherga.test", hw_info);
            node.run();
        }

        std::cout << "Node thread is exiting" << std::endl;
    });

    {
        std::string cmd;

        // Executable name
        cmd += KOCHERGA_TEST_SOURCE_DIR;
        cmd += "/uavcan_tester/main.py";
        cmd += " ";

        // CAN interface
        cmd += IfaceName;
        cmd += " ";

        // Unique ID - see above
        cmd += "000102030405060708090a0b0c0d0e0f";
        cmd += " ";

        // Valid images directory
        cmd += KOCHERGA_TEST_SOURCE_DIR;
        cmd += "/uavcan_tester/valid-images/";
        cmd += " ";

        // Invalid images directory
        cmd += KOCHERGA_TEST_SOURCE_DIR;
        cmd += "/uavcan_tester/invalid-images/";
        cmd += " ";

        // Test duration, in minutes
        cmd += "--duration-min=5";
        cmd += " ";

        // Tell the tester that we don't have any application
        cmd += "--no-application";
        cmd += " ";

        // Running the command; zero means test succeeded
        (void) ::prctl(PR_SET_PDEATHSIG, SIGKILL);       // Kill the child if the parent dies
        std::cout << "Executing: " << cmd << std::endl;
        REQUIRE(0 == std::system(cmd.c_str()));
    }

    std::cout << "Joining the node thread..." << std::endl;
    should_exit = true;
    if (node_thread.joinable())
    {
        node_thread.join();
    }

    std::cout << "Node thread joined, test finished." << std::endl;
}
