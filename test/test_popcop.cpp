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

// Skipping in case of Clang because it can't compile Popcop - the compiler is broken.
// See the bug report here: https://bugs.llvm.org/show_bug.cgi?id=31852
// TODO: This test should be re-enabled when a fixed version of Clang is available.
#ifndef __clang__

// We want to ensure that assertion checks are enabled when tests are run, for extra safety
#ifdef NDEBUG
# undef NDEBUG
#endif

#define KOCHERGA_TRACE          std::printf

// The library headers must be included first to make sure that they don't have any hidden include dependencies.
#include <kocherga_popcop.hpp>

#include "catch.hpp"
#include "mocks.hpp"
#include "images.hpp"
#include "util.hpp"

#include <thread>
#include <numeric>
#include <variant>
#include <functional>
#include <iostream>
#include <utility>
#include <sys/prctl.h>
#include <csignal>
#include <queue>


namespace
{

class DuplexQueue
{
    mutable std::recursive_mutex mutex_;
    std::queue<std::uint8_t> txq_;
    std::queue<std::uint8_t> rxq_;

    void pushAny(std::queue<std::uint8_t>& q, std::uint8_t byte)
    {
        std::lock_guard lock(mutex_);
        q.push(byte);
    }

    std::optional<std::uint8_t> popAny(std::queue<std::uint8_t>& q)
    {
        std::lock_guard lock(mutex_);
        if (!q.empty())
        {
            const auto ret = q.back();
            q.pop();
            return ret;
        }
        else
        {
            return {};
        }
    }

public:
    void pushTx(std::uint8_t byte) { pushAny(txq_, byte); }
    void pushRx(std::uint8_t byte) { pushAny(rxq_, byte); }

    std::optional<std::uint8_t> popTx() { return popAny(txq_); }
    std::optional<std::uint8_t> popRx() { return popAny(rxq_); }

    void reset()
    {
        std::lock_guard lock(mutex_);
        txq_ = std::queue<std::uint8_t>();
        rxq_ = std::queue<std::uint8_t>();
    }
};


class Platform : public kocherga_popcop::IPopcopPlatform
{
    static constexpr std::chrono::seconds WatchdogTimeout{3};  // NOLINT

    DuplexQueue& queue_;
    bool should_exit_ = false;
    std::vector<std::uint8_t> coa_;
    std::chrono::steady_clock::time_point last_watchdog_reset_at_ = std::chrono::steady_clock::now();
    kocherga::BootloaderController& blc_;
    std::function<bool ()> exit_checker_;


    void emit(std::uint8_t byte) override
    {
        queue_.pushTx(byte);
    }

    std::optional<std::uint8_t> receive() override
    {
        if (auto b = queue_.popRx())
        {
            return b;
        }

        std::this_thread::sleep_for(IOByteTimeout);
        return queue_.popRx();
    }

    void processUnhandledFrame(const popcop::transport::ParserOutput::Frame& frame) override
    {
        std::cout << "Unhandled frame " << std::int32_t(frame.type_code) << ":\n"
                  << util::makeHexDump(frame.payload) << std::endl;
    }

    void processExtraneousData(const std::uint8_t* data, std::size_t length) override
    {
        std::cout << "Extraneous data:\n" << util::makeHexDump(data, data + length) << std::endl;
    }

    popcop::standard::DeviceManagementCommandResponseMessage::Status processUnhandledDeviceManagementCommand(
        const popcop::standard::DeviceManagementCommandRequestMessage& request) override
    {
        switch (request.command)
        {
        case popcop::standard::DeviceManagementCommand::Restart:
        {
            should_exit_ = true;
            return popcop::standard::DeviceManagementCommandResponseMessage::Status::Ok;
        }

        case popcop::standard::DeviceManagementCommand::PowerOff:
        case popcop::standard::DeviceManagementCommand::FactoryReset:
        case popcop::standard::DeviceManagementCommand::PrintDiagnosticsBrief:
        case popcop::standard::DeviceManagementCommand::PrintDiagnosticsVerbose:
        {
            return popcop::standard::DeviceManagementCommandResponseMessage::Status::BadCommand;
        }

        case popcop::standard::DeviceManagementCommand::LaunchBootloader:
        {
            throw std::logic_error("The command LaunchBootloader has not been intercepted by the bootloader");
        }

        default:
        {
            throw std::logic_error("Invalid command: " + std::to_string(std::uint32_t(request.command)));
        }
        }
    }

    std::int16_t writeAndReadBackCertificateOfAuthenticity(const std::uint8_t* in_data,
                                                                 std::uint8_t* out_data,
                                                                 std::uint8_t length) override
    {
        coa_.resize(length);
        for (std::uint8_t i = 0; i < length; i++)
        {
            coa_.push_back(*(in_data + i));
            *(out_data + i) = *(in_data + i);
        }

        return length;
    }

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

    bool shouldExit() const override
    {
        return should_exit_ || (blc_.getState() == kocherga::State::ReadyToBoot) || exit_checker_();
    }

public:
    Platform(DuplexQueue& queue,
             kocherga::BootloaderController& blc,
             std::function<bool ()> exit_checker) :
        queue_(queue),
        blc_(blc),
        exit_checker_(std::move(exit_checker))
    {
        if (!exit_checker_)
        {
            exit_checker_ = []() { return false; };
        }
    }
};


using AnyMessage = std::variant<popcop::standard::EndpointInfoMessage,
                                popcop::standard::DeviceManagementCommandResponseMessage,
                                popcop::standard::BootloaderStatusResponseMessage,
                                popcop::standard::BootloaderImageDataResponseMessage>;

class Modem
{
    DuplexQueue& q_;
    popcop::transport::Parser<> parser_;

    template <std::uint16_t OptionIndex = 0>
    static std::optional<AnyMessage> decodeMessage(const popcop::transport::ParserOutput::Frame& frame)
    {
        if constexpr (OptionIndex < std::variant_size_v<AnyMessage>)
        {
            using MessageType = std::variant_alternative_t<OptionIndex, AnyMessage>;
            if (auto msg = MessageType::tryDecode(frame.payload.begin(), frame.payload.end()))
            {
                return AnyMessage(*msg);
            }
            else
            {
                return decodeMessage<OptionIndex + 1U>(frame);
            }
        }
        else
        {
            return {};
        }
    }

    void pushRxQueue(std::uint8_t byte)
    {
        q_.pushRx(byte);
    }

    template <typename R, typename P>
    std::optional<std::uint8_t> popTxQueue(std::chrono::duration<R, P> timeout)
    {
        if (auto b = q_.popTx())
        {
            return b;
        }

        std::this_thread::sleep_for(timeout);
        return q_.popTx();
    }
public:
    explicit Modem(DuplexQueue& queue) : q_(queue) { }

    std::size_t send(const AnyMessage& msg)
    {
        return std::visit([this](auto m) -> std::size_t {
            return m.encode(popcop::transport::StreamEmitter(popcop::presentation::StandardFrameTypeCode,
                                                             [this](std::uint8_t x) { pushRxQueue(x); }).begin());
        }, msg);
    }

    template <typename R, typename P>
    std::optional<AnyMessage> receive(std::chrono::duration<R, P> timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        do
        {
            if (auto ret = popTxQueue(std::chrono::milliseconds(1)))
            {
                const auto output = parser_.processNextByte(*ret);
                if (auto frame = output.getReceivedFrame())
                {
                    const auto msg = decodeMessage(*frame);
                    if (!msg)
                    {
                        throw std::logic_error("Unexpected frame with type code " +
                                               std::to_string(std::int32_t(frame->type_code)));
                    }

                    return msg;
                }
                else if (auto data = output.getExtraneousData())
                {
                    throw std::logic_error("Extraneous data not expected");
                }
                else
                {
                    ;   // Nothing to do
                }
            }
        }
        while (std::chrono::steady_clock::now() <= deadline);

        return {};
    }
};


}  // namespace


TEST_CASE("Popcop-Basic")
{
    volatile bool should_exit = false;

    DuplexQueue queue;

    std::thread endpoint_thread([&]() {
        while (!should_exit)
        {
            std::cout << "Endpoint is (re)starting..." << std::endl;

            mocks::Platform platform;
            static constexpr std::uint32_t ROMSize = 1024 * 1024;
            mocks::FileMappedROMBackend rom_backend("popcop-basic-rom.tmp", ROMSize);
            kocherga::BootloaderController blc(platform, rom_backend, ROMSize);

            popcop::standard::EndpointInfoMessage ep_info;

            // The entire software version struct will be overwritten by the protocol implementation
            ep_info.software_version.vcs_commit_id       = 0xdeadbeefUL;
            ep_info.software_version.image_crc           = 0xbadc0ffeULL;
            ep_info.software_version.dirty_build         = true;
            ep_info.software_version.release_build       = true;
            ep_info.software_version.build_timestamp_utc = 123456789UL;
            ep_info.software_version.major               = 12;
            ep_info.software_version.minor               = 34;

            ep_info.hardware_version.major = 56;
            ep_info.hardware_version.minor = 78;

            ep_info.endpoint_name                   = "com.zubax.test";
            ep_info.endpoint_description            = "Test Endpoint";
            ep_info.build_environment_description   = "Build Environment";
            ep_info.runtime_environment_description = "Runtime Environment";

            ep_info.mode = popcop::standard::EndpointInfoMessage::Mode::Normal;   // Will be reset to Bootloader

            ep_info.globally_unique_id = {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};

            for (std::uint16_t i = 0; i < ep_info.certificate_of_authenticity.capacity(); i++)
            {
                ep_info.certificate_of_authenticity.push_back(std::uint8_t(i));
            }

            queue.reset();      // <--- mighty reset!

            Platform popcop_platform(queue, blc, [&]() { return should_exit; });

            kocherga_popcop::PopcopProtocol endpoint(blc, popcop_platform, ep_info);
            endpoint.run();
        }

        std::cout << "Endpoint thread is exiting" << std::endl;
    });

    {
        Modem modem(queue);

        REQUIRE_FALSE(modem.receive(std::chrono::milliseconds(10)));
    }

    std::cout << "Joining the endpoint thread..." << std::endl;
    should_exit = true;
    if (endpoint_thread.joinable())
    {
        endpoint_thread.join();
    }

    std::cout << "Endpoint thread joined, test finished." << std::endl;
}

#endif // __clang__
