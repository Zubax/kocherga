// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include "kocherga.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <random>
#include <sstream>
#include <type_traits>
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
                               ? static_cast<char>(*begin)  // NOSONAR intentional conversion to plain char
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

/// A ROM backend mapped onto a local file.
class FileROMBackend : public kocherga::IROMBackend
{
public:
    class Error : public std::runtime_error
    {
    public:
        explicit Error(const std::string& x) : std::runtime_error(x) {}
    };

    /// This overload creates a new file filled as specified. If the file exists, it will be written over.
    FileROMBackend(const std::filesystem::path& file_path,
                   const std::size_t            rom_size,
                   const std::uint8_t           memory_fill = static_cast<std::uint8_t>((1U << BitsPerByte) - 1U)) :
        path_(file_path), rom_size_(rom_size)
    {
        std::ofstream f(path_, std::ios::binary | std::ios::out | std::ios::trunc);
        if (f)
        {
            const std::vector<char> empty_image(rom_size,
                                                static_cast<char>(
                                                    memory_fill));  // NOSONAR intentional conversion to plain char
            f.write(empty_image.data(), static_cast<std::streamsize>(empty_image.size()));
            f.flush();
        }
        else
        {
            throw Error("Could not init the ROM mapping file");
        }
    }

    /// This overload opens an existing file.
    explicit FileROMBackend(const std::filesystem::path& file_path) :
        path_(file_path), rom_size_(static_cast<std::size_t>(std::filesystem::file_size(file_path)))
    {
        checkFileHealth();
    }

    void enableFailureInjection(const bool enabled) { trigger_failure_ = enabled; }

    auto isSameImage(const std::byte* const reference, const std::size_t reference_size) const
    {
        checkFileHealth();
        if (std::ifstream f(path_, std::ios::binary | std::ios::in); f)
        {
            f.seekg(0);
            std::vector<char> buffer(reference_size, '\0');
            f.read(buffer.data(), static_cast<std::streamsize>(reference_size));
            return std::memcmp(reference, buffer.data(), reference_size) == 0;
        }
        throw Error("Could not open the ROM mapping file for verification");
    }

    [[nodiscard]] auto read(const std::size_t offset, std::byte* const out_data, const std::size_t size) const
        -> std::size_t override
    {
        read_count_++;
        const std::size_t out = ((offset + size) > rom_size_) ? (rom_size_ - offset) : size;
        assert(out <= size);
        assert((out + offset) <= rom_size_);
        checkFileHealth();
        if (std::ifstream f(path_, std::ios::binary | std::ios::in); f)
        {
            f.seekg(static_cast<std::streamoff>(offset));
            f.read(reinterpret_cast<char*>(out_data),  // NOLINT NOSONAR reinterpret_cast
                   static_cast<std::streamsize>(out));
            return out;
        }
        throw Error("Could not open the ROM emulation file for reading");
    }

    auto getReadCount() const { return read_count_; }
    auto getWriteCount() const { return write_count_; }

    auto getSize() const { return rom_size_; }

private:
    void checkFileHealth() const
    {
        if (std::filesystem::file_size(path_) != rom_size_)
        {
            throw Error("Invalid size of the ROM mapping file");
        }
    }

    void beginWrite() override
    {
        checkFileHealth();
        if (upgrade_in_progress_)
        {
            throw Error("Bad sequencing");
        }
        upgrade_in_progress_ = true;
    }

    void endWrite() override
    {
        checkFileHealth();
        if (!upgrade_in_progress_)
        {
            throw Error("Bad sequencing");
        }
        upgrade_in_progress_ = false;
    }

    [[nodiscard]] auto write(const std::size_t offset, const std::byte* const data, const std::size_t size)
        -> std::optional<std::size_t> override
    {
        write_count_++;
        if (!upgrade_in_progress_)
        {
            throw Error("Bad sequencing");
        }
        checkFileHealth();
        if (trigger_failure_)
        {
            // Can't use {} because of a bug in GCC: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=92092
            return std::optional<std::size_t>(std::nullopt);  // NOLINT
        }
        const std::size_t out = ((offset + size) > rom_size_) ? (rom_size_ - offset) : size;
        assert(out <= size);
        assert((out + offset) <= rom_size_);
        if (std::ofstream f(path_, std::ios::binary | std::ios::out | std::ios::in); f)
        {
            f.seekp(static_cast<std::streamoff>(offset));
            f.write(reinterpret_cast<const char*>(data),  // NOLINT NOSONAR reinterpret_cast
                    static_cast<std::streamsize>(out));
            f.flush();
            return out;
        }
        throw Error("Could not open the ROM emulation file for writing");
    }

    const std::filesystem::path path_;
    const std::size_t           rom_size_;

    mutable std::uint64_t read_count_          = 0;
    std::uint64_t         write_count_         = 0;
    bool                  upgrade_in_progress_ = false;
    bool                  trigger_failure_     = false;
};

class EnvironmentError : public std::runtime_error
{
public:
    explicit EnvironmentError(const std::string& x) : std::runtime_error(x) {}
};

/// A safe wrapper over the standard getenv() that returns an empty option if the variable is not set.
inline auto getEnvironmentVariableMaybe(const std::string& name) -> std::optional<std::string>
{
    if (auto* const st = std::getenv(name.c_str()); st != nullptr)  // NOSONAR getenv is considered unsafe.
    {
        return std::string(st);
    }
    return {};
}

/// Like above but throws EnvironmentError if the variable is missing.
inline auto getEnvironmentVariable(const std::string& name) -> std::string
{
    if (const auto v = getEnvironmentVariableMaybe(name))
    {
        return *v;
    }
    throw EnvironmentError("Environment variable is required but not set: " + name);
}

inline auto getSourceDir() -> std::filesystem::path
{
    return getEnvironmentVariable("SOURCE_DIR");
}

inline auto getImagePath(const std::string& name) -> std::filesystem::path
{
    return util::getSourceDir() / "images" / name;
}

template <typename T>
inline auto getRandomInteger() -> std::enable_if_t<std::is_unsigned_v<T>, T>
{
    return static_cast<T>((static_cast<std::uint64_t>(std::rand()) * std::numeric_limits<T>::max()) / RAND_MAX);
}

}  // namespace util
