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

#include <kocherga.hpp>
#include <mutex>
#include <vector>
#include <utility>
#include <fstream>
#include <functional>


namespace mocks
{
/**
 * This exception type represents a library error detected by the mock.
 */
class BadUsageException : public std::logic_error
{
public:
    explicit BadUsageException(const std::string& text) : std::logic_error(text) { }
};

/**
 * Generic platform mock for unit testing.
 * We use a mutex as well out of extra paranoia.
 */
class Platform : public kocherga::IPlatform
{
    std::uint64_t mutex_lock_count_ = 0;
    std::int64_t mutex_lock_nesting_ = 0;
    std::recursive_mutex mutex_;

    void lockMutex() final
    {
        mutex_.lock();
        mutex_lock_nesting_++;
        mutex_lock_count_++;

        if (mutex_lock_nesting_ > 10)
        {
            throw BadUsageException("Mutex usage bug: unhealthy locking habits");
        }
    }

    void unlockMutex() final
    {
        if (mutex_lock_nesting_ <= 0)
        {
            throw BadUsageException("Mutex usage bug: cannot unlock mutex that is not locked");
        }

        mutex_lock_nesting_--;
        mutex_.unlock();
    }

public:
    bool isMutexLocked() const { return mutex_lock_nesting_ > 0; }

    std::uint64_t getMutexLockCount() const { return mutex_lock_count_; }

    std::chrono::microseconds getMonotonicUptime() const final
    {
        // The library guarantees that the uptime can only be requested when the mutex is locked.
        // Making sure this is true.
        if (mutex_lock_nesting_ <= 0)
        {
            throw BadUsageException("Monotonic uptime usage bug: mutex not locked when querying the time.");
        }

        return std::chrono::duration_cast<std::chrono::microseconds>
            (std::chrono::steady_clock::now().time_since_epoch());
    }
};

/**
 * Maps ROM operations to a specified file.
 * The file can be used for inspection by the developer.
 */
class FileMappedROMBackend : public kocherga::IROMBackend
{
    const std::string file_name_;
    const std::uint32_t rom_size_;

    mutable std::uint64_t read_count_ = 0;
    std::uint64_t write_count_ = 0;

    bool upgrade_in_progress_ = false;
    mutable std::function<std::int16_t (std::int16_t)> failure_injector_;


    std::int16_t callFailureInjector(std::int16_t regular_error_code) const
    {
        if (failure_injector_)
        {
            return failure_injector_(regular_error_code);
        }
        else
        {
            return regular_error_code;
        }
    }

    void checkFileHealth() const
    {
        std::ifstream f(file_name_, std::ios::binary | std::ios::in | std::ios::ate);
        if (!f)
        {
            throw std::runtime_error("The ROM mapping file could not be open");
        }

        if (f.tellg() != rom_size_)
        {
            throw std::runtime_error("Invalid size of the ROM mapping file");
        }
    }

    std::int16_t beginUpgrade() override
    {
        checkFileHealth();

        if (upgrade_in_progress_)
        {
            throw BadUsageException("beginUpgrade() called twice");
        }

        if (const auto res = callFailureInjector(0); res < 0)
        {
            return res;
        }

        upgrade_in_progress_ = true;
        return 0;
    }

    std::int16_t endUpgrade(bool success) override
    {
        (void) success;
        checkFileHealth();

        if (!upgrade_in_progress_)
        {
            throw BadUsageException("endUpgrade() called twice");
        }

        upgrade_in_progress_ = false;
        return callFailureInjector(0);
    }

    std::int16_t write(std::size_t offset, const void* data, std::uint16_t size) override
    {
        write_count_++;

        if (size > kocherga::MaxDataBlockSize)
        {
            throw BadUsageException("Size is too big");
        }

        if (!upgrade_in_progress_)
        {
            throw BadUsageException("Upgrade is not in progress!");
        }

        if ((offset + size) > rom_size_)
        {
            size = std::uint16_t(rom_size_ - offset);
        }

        checkFileHealth();

        // Checking the failure generator before we modify the storage!
        if (const std::int16_t res = callFailureInjector(std::int16_t(size)); res != size)
        {
            return res;
        }

        // We need the in flag to prevent truncation
        std::ofstream f(file_name_, std::ios::binary | std::ios::out | std::ios::in);
        if (f)
        {
            f.seekp(std::streamoff(offset));
            f.write(static_cast<const char*>(data), size);
            f.flush();
            return std::int16_t(size);
        }
        else
        {
            throw std::runtime_error("Could not open the ROM mapping file for writing");
        }
    }

public:
    FileMappedROMBackend(std::string file_name,
                         std::uint32_t rom_size,
                         std::uint8_t empty_memory_fill = 0xFF) :
        file_name_(std::move(file_name)),
        rom_size_(rom_size)
    {
        std::ofstream f(file_name_, std::ios::binary | std::ios::out | std::ios::trunc);
        if (f)
        {
            const std::vector<std::uint8_t> empty_image(rom_size, empty_memory_fill);
            f.write(reinterpret_cast<const char*>(empty_image.data()), std::streamsize(empty_image.size()));
        }
        else
        {
            throw std::runtime_error("Could not init the ROM mapping file");
        }
    }

    void setFailureInjector(std::function<std::int16_t (std::int16_t)> injector)
    {
        failure_injector_ = injector;
    }

    bool isSameImage(const void* reference, std::size_t reference_size) const
    {
        std::vector<std::uint8_t> buffer(reference_size, 0);
        std::ifstream f(file_name_, std::ios::binary | std::ios::in);
        if (f)
        {
            f.seekg(0);
            f.read(reinterpret_cast<char*>(buffer.data()), std::streamsize(reference_size));
        }
        else
        {
            throw std::runtime_error("Could not open the ROM mapping file for verification");
        }

        return std::memcmp(reference, buffer.data(), reference_size) == 0;
    }

    std::int16_t read(std::size_t offset, void* data, std::uint16_t size) const override
    {
        read_count_++;

        if (size > kocherga::MaxDataBlockSize)
        {
            throw BadUsageException("Size is too big");
        }

        if ((offset + size) > rom_size_)
        {
            size = std::uint16_t(rom_size_ - offset);
        }

        checkFileHealth();

        std::ifstream f(file_name_, std::ios::binary | std::ios::in);
        if (f)
        {
            f.seekg(std::streamoff(offset));
            f.read(static_cast<char*>(data), size);
            return callFailureInjector(std::int16_t(size));
        }
        else
        {
            throw std::runtime_error("Could not open the ROM mapping file for reading");
        }
    }

    std::uint64_t getReadCount()  const { return read_count_; }
    std::uint64_t getWriteCount() const { return write_count_; }
};


static_assert(32767 == kocherga::MaxDataBlockSize);

}
