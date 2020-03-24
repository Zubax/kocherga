// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

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
                output << std::setw(2) << static_cast<std::uint32_t>(*it) << ' ';
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
                output << (((static_cast<std::uint32_t>(*begin) >= PrintableASCIIRange.first) &&
                            (static_cast<std::uint32_t>(*begin) <= PrintableASCIIRange.second))
                               ? static_cast<char>(*begin)
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

/// A mock ROM backend mapped onto a local file.
class FileROMBackend : public kocherga::IROMBackend
{
public:
    FileROMBackend(const std::string&  file_name,
                   const std::uint32_t rom_size,
                   const std::uint8_t  empty_memory_fill = 0xFF) :
        file_name_(file_name), rom_size_(rom_size)
    {
        std::ofstream f(file_name_, std::ios::binary | std::ios::out | std::ios::trunc);
        if (f)
        {
            const std::vector<char> empty_image(rom_size, static_cast<char>(empty_memory_fill));
            f.write(empty_image.data(), std::streamsize(empty_image.size()));
        }
        else
        {
            throw std::runtime_error("Could not init the ROM mapping file");
        }
    }

    void enableFailureInjection(const bool enabled) { trigger_failure_ = enabled; }

    auto isSameImage(const void* const reference, const std::size_t reference_size) const
    {
        checkFileHealth();
        if (std::ifstream f(file_name_, std::ios::binary | std::ios::in); f)
        {
            f.seekg(0);
            std::vector<char> buffer(reference_size, '\0');
            f.read(buffer.data(), std::streamsize(reference_size));
            return std::memcmp(reference, buffer.data(), reference_size) == 0;
        }
        throw std::runtime_error("Could not open the ROM mapping file for verification");
    }

    [[nodiscard]] auto read(const std::size_t offset, void* const out_data, const std::size_t size) const
        -> std::size_t override
    {
        read_count_++;
        const std::size_t out = ((offset + size) > rom_size_) ? (rom_size_ - offset) : size;
        assert(out <= size);
        assert((out + offset) <= rom_size_);
        checkFileHealth();
        if (std::ifstream f(file_name_, std::ios::binary | std::ios::in); f)
        {
            f.seekg(static_cast<std::streamoff>(offset));
            f.read(static_cast<char*>(out_data), static_cast<std::streamsize>(out));
            return out;
        }
        throw std::runtime_error("Could not open the ROM emulation file for reading");
    }

    auto getReadCount() const { return read_count_; }
    auto getWriteCount() const { return write_count_; }

private:
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

    [[nodiscard]] auto onBeforeFirstWrite() -> bool override
    {
        checkFileHealth();
        if (trigger_failure_)
        {
            return false;
        }
        if (upgrade_in_progress_)
        {
            throw std::runtime_error("Bad sequencing");
        }
        upgrade_in_progress_ = true;
        return true;
    }

    void onAfterLastWrite(const bool success) override
    {
        (void) success;
        checkFileHealth();
        if (!upgrade_in_progress_)
        {
            throw std::runtime_error("Bad sequencing");
        }
        upgrade_in_progress_ = false;
    }

    [[nodiscard]] auto write(const std::size_t offset, const void* const data, const std::size_t size)
        -> std::optional<std::size_t> override
    {
        write_count_++;
        if (!upgrade_in_progress_)
        {
            throw std::runtime_error("Bad sequencing");
        }
        checkFileHealth();
        if (trigger_failure_)
        {
            // Can't use {} because of a bug in GCC: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=92092
            return std::optional<std::size_t>(std::nullopt);
        }
        const std::size_t out = ((offset + size) > rom_size_) ? (rom_size_ - offset) : size;
        assert(out <= size);
        assert((out + offset) <= rom_size_);
        if (std::ofstream f(file_name_, std::ios::binary | std::ios::out | std::ios::in); f)
        {
            f.seekp(std::streamoff(offset));
            f.write(static_cast<const char*>(data), static_cast<std::streamsize>(out));
            f.flush();
            return out;
        }
        throw std::runtime_error("Could not open the ROM emulation file for writing");
    }

    const std::string   file_name_;
    const std::uint32_t rom_size_;

    mutable std::uint64_t read_count_          = 0;
    std::uint64_t         write_count_         = 0;
    bool                  upgrade_in_progress_ = false;
    bool                  trigger_failure_     = false;
};

}  // namespace util
