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
#include <kocherga.hpp>

#include "catch.hpp"
#include "mocks.hpp"
#include "images.hpp"

#include <thread>
#include <numeric>
#include <functional>


namespace
{
/**
 * A simple mock protocol that just downloads the specified image from memory.
 */
class MockProtocol : public kocherga::IProtocol
{
    static constexpr std::uint16_t BlockSize = 103;     ///< Using a weird prime block size intentionally

    const std::uint8_t* ptr_;
    std::size_t remaining_size_;
    const std::function<void ()> chunk_callback_;

    std::int16_t downloadImage(kocherga::IDownloadSink& sink) final
    {
        while (remaining_size_ > 0)
        {
            if (chunk_callback_)
            {
                chunk_callback_();
            }

            const std::uint16_t bs = std::uint16_t(std::min<std::size_t>(remaining_size_, BlockSize));

            const auto result = sink.handleNextDataChunk(ptr_, bs);
            if (result != bs)
            {
                if (result < 0)
                {
                    return result;
                }
                else
                {
                    return kocherga::ErrROMWriteFailure;
                }
            }

            ptr_ += bs;
            remaining_size_ -= bs;
        }

        return 0;
    }

public:
    MockProtocol(const void* data,
                 std::size_t size,
                 std::function<void ()> callback_per_chunk = {}) :
        ptr_(static_cast<const std::uint8_t*>(data)),
        remaining_size_(size),
        chunk_callback_(std::move(callback_per_chunk))
    { }
};

}


TEST_CASE("Core-Basic")
{
    static constexpr std::uint32_t ROMSize = 1024 * 1024;

    mocks::Platform platform;
    mocks::FileMappedROMBackend rom_backend("core-test-rom.tmp", ROMSize);

    kocherga::BootloaderController blc(platform, rom_backend, ROMSize, std::chrono::seconds(1));

    REQUIRE(1 == platform.getMutexLockCount());
    REQUIRE(!platform.isMutexLocked());

    REQUIRE(1 == platform.getMutexLockCount());
    REQUIRE(blc.getMonotonicUptime().count() > 0);
    REQUIRE(2 == platform.getMutexLockCount());

    // When verifying the image, we're reading it in 8-byte increments until the end.
    // The controller observes the last read request to fail, which indicates that the end of the ROM is reached.
    REQUIRE((ROMSize / 8) + 1 == rom_backend.getReadCount());
    REQUIRE(0 == rom_backend.getWriteCount());

    REQUIRE(2 == platform.getMutexLockCount());
    REQUIRE(kocherga::State::NoAppToBoot == blc.getState());
    REQUIRE(3 == platform.getMutexLockCount());
    REQUIRE(!blc.getAppInfo());
    REQUIRE(4 == platform.getMutexLockCount());

    // Boot request ignored - nothing to boot
    REQUIRE(4 == platform.getMutexLockCount());
    blc.requestBoot();
    REQUIRE(5 == platform.getMutexLockCount());
    REQUIRE(kocherga::State::NoAppToBoot == blc.getState());
    REQUIRE(6 == platform.getMutexLockCount());
    REQUIRE(!blc.getAppInfo());
    REQUIRE(7 == platform.getMutexLockCount());

    // Boot cancellation ignored - nothing to cancel
    REQUIRE(7 == platform.getMutexLockCount());
    blc.cancelBoot();
    REQUIRE(8 == platform.getMutexLockCount());
    REQUIRE(kocherga::State::NoAppToBoot == blc.getState());
    REQUIRE(9 == platform.getMutexLockCount());
    REQUIRE(!blc.getAppInfo());
    REQUIRE(10 == platform.getMutexLockCount());

    // Up to this point we did not write the ROM, making sure it's true
    REQUIRE(0 == rom_backend.getWriteCount());

    // Uploading a valid image now
    const auto upload_valid_image = [&]()
    {
        REQUIRE(!platform.isMutexLocked());

        MockProtocol proto(images::AppValid.data(),
                           images::AppValid.size(),
                           [&]() { REQUIRE(blc.getState() == kocherga::State::AppUpgradeInProgress); });
        REQUIRE(0 == blc.upgradeApp(proto));

        REQUIRE(12 < platform.getMutexLockCount());
        REQUIRE(!platform.isMutexLocked());

        REQUIRE(kocherga::State::BootDelay == blc.getState());

        const auto maybe_app_info = blc.getAppInfo();
        REQUIRE(maybe_app_info);
        const kocherga::AppInfo app_info = *maybe_app_info;
        REQUIRE(app_info.image_size    == images::AppValid.size());
        REQUIRE(app_info.major_version == images::AppValidMajorVersion);
        REQUIRE(app_info.minor_version == images::AppValidMinorVersion);
        REQUIRE(app_info.vcs_commit    == images::AppValidVCSCommit);
        REQUIRE(app_info.isReleaseBuild()    == images::AppValidReleaseBuild);   // NOLINT
        REQUIRE(app_info.isDirtyBuild()      == images::AppValidDirtyBuild);     // NOLINT
        REQUIRE(app_info.build_timestamp_utc == images::AppValidBuildTimestamp);
        REQUIRE_FALSE(app_info.isBuildTimestampValid());
    };
    upload_valid_image();

    // Boot cancellation
    blc.cancelBoot();
    REQUIRE(kocherga::State::BootCancelled == blc.getState());
    REQUIRE(blc.getAppInfo());      // Still valid!

    // Uploading a valid image but making it fail; the previously written image is VALID
    {
        REQUIRE(!platform.isMutexLocked());
        MockProtocol proto(images::AppValid.data(),
                           images::AppValid.size(),
                           [&]() { REQUIRE(blc.getState() == kocherga::State::AppUpgradeInProgress); });

        bool once = true;
        rom_backend.setFailureInjector([&](std::int16_t regular) -> std::int16_t {
            if (once)
            {
                once = false;
                return -123;
            }
            return regular;
        });

        REQUIRE(-123 == blc.upgradeApp(proto));
        REQUIRE(!platform.isMutexLocked());
        REQUIRE(kocherga::State::BootCancelled == blc.getState());      // The old one is still valid!
        rom_backend.setFailureInjector({});
    }

    // Uploading an invalid image now
    {
        REQUIRE(!platform.isMutexLocked());

        MockProtocol proto(images::AppWithInvalidDescriptor.data(),
                           images::AppWithInvalidDescriptor.size(),
                           [&]() { REQUIRE(blc.getState() == kocherga::State::AppUpgradeInProgress); });
        REQUIRE(0 == blc.upgradeApp(proto));

        REQUIRE(20 < platform.getMutexLockCount());
        REQUIRE(!platform.isMutexLocked());

        REQUIRE(kocherga::State::NoAppToBoot == blc.getState());
        REQUIRE(!blc.getAppInfo());
    }

    // Uploading a valid image but making it fail; the previously written image is INVALID
    // The failure is generated by returning larger size than requested
    {
        REQUIRE(!platform.isMutexLocked());
        MockProtocol proto(images::AppValid.data(),
                           images::AppValid.size(),
                           [&]() { REQUIRE(blc.getState() == kocherga::State::AppUpgradeInProgress); });

        rom_backend.setFailureInjector([&](std::int16_t regular) {
            if (regular > 0)
            {
                return std::int16_t(regular + 1);
            }
            return regular;
        });

        REQUIRE(-kocherga::ErrROMWriteFailure == blc.upgradeApp(proto));
        REQUIRE(!platform.isMutexLocked());
        REQUIRE(kocherga::State::NoAppToBoot == blc.getState());
        rom_backend.setFailureInjector({});
    }

    // Again uploading the valid image
    upload_valid_image();
    REQUIRE(kocherga::State::BootDelay == blc.getState());
    REQUIRE(blc.getAppInfo());

    // Waiting for more than 1 second to trigger boot timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    REQUIRE(kocherga::State::ReadyToBoot == blc.getState());

    // Great, now cancel it again, and re-trigger manually.
    blc.cancelBoot();
    REQUIRE(kocherga::State::BootCancelled == blc.getState());
    REQUIRE(blc.getAppInfo());      // Still valid!

    blc.requestBoot();
    REQUIRE(kocherga::State::ReadyToBoot == blc.getState());
}


TEST_CASE("Core-CRC64")
{
    kocherga::CRC64 crc;
    crc.add("123456789", 9);
    REQUIRE(crc.get() == 0x62EC59E3F1A4F00AULL);
}


TEST_CASE("Core-AppDataExchange-Registers")
{
    struct Data
    {
        std::uint64_t a = 0;
        std::uint8_t  b = 0;
        std::array<std::uint8_t, 3> c{};
    };
    static_assert(sizeof(Data) == 12);      // Including implicit padding; assuming 32-bit platform

    struct RegisterBank
    {
        volatile std::uint64_t crc = 0;
        volatile std::uint64_t a   = 0;
        volatile std::uint8_t b    = 0;
        volatile std::uint8_t c0   = 0;
        volatile std::uint8_t c1   = 0;
        volatile std::uint8_t c2   = 0;
    } reg_bank;
    static_assert(sizeof(RegisterBank) == 20);  // Padding made explicit plus CRC64

    auto marshaller = kocherga::makeAppDataExchangeMarshaller<Data>(
        &reg_bank.crc,
        &reg_bank.a,
        &reg_bank.b,
        &reg_bank.c0,
        &reg_bank.c1,
        &reg_bank.c2);

    // The storage is empty, checking
    REQUIRE(!marshaller.readAndErase());

    // Writing zeros and checking the representation
    marshaller.write(Data());
    REQUIRE(reg_bank.crc != 0);
    REQUIRE(reg_bank.a == 0);
    REQUIRE(reg_bank.b == 0);
    REQUIRE(reg_bank.c0 == 0);
    REQUIRE(reg_bank.c1 == 0);
    REQUIRE(reg_bank.c2 == 0);

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0);
        REQUIRE(rd->b == 0);
        REQUIRE(rd->c[0] == 0);
        REQUIRE(rd->c[1] == 0);
        REQUIRE(rd->c[2] == 0);
    }

    REQUIRE(reg_bank.crc == 0);
    REQUIRE(reg_bank.a == 0);
    REQUIRE(reg_bank.b == 0);
    REQUIRE(reg_bank.c0 == 0);
    REQUIRE(reg_bank.c1 == 0);
    REQUIRE(reg_bank.c2 == 0);

    REQUIRE(!marshaller.readAndErase());

    // Writing non-zeros and checking the representation
    marshaller.write({
        0x00AD'EADB'ADC0'FFEE,
        123,
        {{1, 2, 3}},
    });
    REQUIRE(reg_bank.crc != 0);
    REQUIRE(reg_bank.a == 0x00AD'EADB'ADC0'FFEE);
    REQUIRE(reg_bank.b == 123);
    REQUIRE(reg_bank.c0 == 1);
    REQUIRE(reg_bank.c1 == 2);
    REQUIRE(reg_bank.c2 == 3);

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0x00AD'EADB'ADC0'FFEE);
        REQUIRE(rd->b == 123);
        REQUIRE(rd->c[0] == 1);
        REQUIRE(rd->c[1] == 2);
        REQUIRE(rd->c[2] == 3);
    }

    REQUIRE(reg_bank.crc == 0);
    REQUIRE(reg_bank.a == 0);
    REQUIRE(reg_bank.b == 0);
    REQUIRE(reg_bank.c0 == 0);
    REQUIRE(reg_bank.c1 == 0);
    REQUIRE(reg_bank.c2 == 0);

    REQUIRE(!marshaller.readAndErase());
}


TEST_CASE("Core-AppDataExchange-Memory")
{
    struct Data
    {
        std::uint64_t a = 0;
        std::uint8_t  b = 0;
        std::array<std::uint8_t, 3> c{};
    };
    static_assert(sizeof(Data) == 12);      // Including implicit padding; assuming 32-bit platform

    std::array<std::uint8_t, 100> arena{};

    auto marshaller = kocherga::makeAppDataExchangeMarshaller<Data>(static_cast<void*>(arena.data()));

    // The storage is empty, checking
    REQUIRE(!marshaller.readAndErase());

    // Writing zeros and checking the representation
    marshaller.write(Data());
    REQUIRE(0 != std::accumulate(arena.begin(), arena.begin() + 8, 0ULL));      // First 8 bytes are CRC, non-zero
    REQUIRE(0 == std::accumulate(arena.begin() + 8, arena.end(), 0ULL));        // The payload is zeroed out

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0);
        REQUIRE(rd->b == 0);
        REQUIRE(rd->c[0] == 0);
        REQUIRE(rd->c[1] == 0);
        REQUIRE(rd->c[2] == 0);
    }

    REQUIRE(0 == std::accumulate(arena.begin(), arena.end(), 0ULL));
    REQUIRE(!marshaller.readAndErase());

    // Writing non-zeros and checking the representation
    marshaller.write({
        0x00AD'EADB'ADC0'FFEE,
        123,
        {{1, 2, 3}},
    });
    REQUIRE(0 != std::accumulate(arena.begin(), arena.end(), 0ULL));            // All non-zero

    // Reading and making sure it's erased afterwards
    {
        const auto rd = marshaller.readAndErase();
        REQUIRE(rd);
        REQUIRE(rd->a == 0x00AD'EADB'ADC0'FFEE);
        REQUIRE(rd->b == 123);
        REQUIRE(rd->c[0] == 1);
        REQUIRE(rd->c[1] == 2);
        REQUIRE(rd->c[2] == 3);
    }

    REQUIRE(0 == std::accumulate(arena.begin(), arena.end(), 0ULL));
    REQUIRE(!marshaller.readAndErase());
}
