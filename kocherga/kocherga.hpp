// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <tuple>
#include <type_traits>

namespace kocherga
{
/// Semantic version number pair: major then minor.
using SemanticVersion = std::array<std::uint8_t, 2>;

/// Version of the library, major and minor.
static constexpr SemanticVersion Version{{1, 0}};

/// Error codes.  These are returned from functions in negated form, e.g., -100 means error code 100.
static constexpr std::int8_t ErrInvalidParams    = 2;
static constexpr std::int8_t ErrROMWriteFailure  = 3;
static constexpr std::int8_t ErrAppImageTooLarge = 4;

// --------------------------------------------------------------------------------------------------------------------

/// The transport-specific node abstraction interface. Kocherga runs a separate node per transport interface.
/// If redundant transports are desired, they should be implemented in a custom implementation of INode.
class INode
{};

// --------------------------------------------------------------------------------------------------------------------

/// The structure is mapped to the ROM.
struct AppInfo
{
    static constexpr std::uint8_t Size = 32;

    std::uint64_t   image_crc;   ///< CRC-64-WE of the firmware padded to 8 bytes computed with this field =0.
    std::uint64_t   image_size;  ///< Size of the application image in bytes.
    std::uint64_t   vcs_commit;  ///< Version control system revision ID (e.g., git commit hash).
    std::uint32_t   reserved;    ///< Zero when writing, ignore when reading.
    std::uint16_t   flags;       ///< Flags; see the constants. Unused flags shall not be set.
    SemanticVersion version;     ///< Semantic version numbers, major then minor.

    /// Bit mask values of the flags field.
    struct Flags
    {
        static constexpr std::uint16_t DebugBuild = 1U;
        static constexpr std::uint16_t DirtyBuild = 2U;
    };

    [[nodiscard]] auto isDebugBuild() const { return (flags & Flags::DebugBuild) != 0; }
    [[nodiscard]] auto isDirtyBuild() const { return (flags & Flags::DirtyBuild) != 0; }
};
static_assert(std::is_trivial_v<AppInfo>, "Check your compiler.");
static_assert(AppInfo::Size == sizeof(AppInfo), "Check your compiler.");

// --------------------------------------------------------------------------------------------------------------------

/// Target platform services.
class IPlatform
{
public:
    IPlatform(const IPlatform&) = delete;
    IPlatform(IPlatform&&)      = delete;
    auto operator=(const IPlatform&) -> IPlatform& = delete;
    auto operator=(IPlatform &&) -> IPlatform& = delete;

    virtual ~IPlatform() = default;

    /// Returns the time since boot as a monotonic (i.e., steady) clock. The clock shall never overflow.
    [[nodiscard]] virtual auto getMonotonicUptime() const -> std::chrono::microseconds = 0;
};

// --------------------------------------------------------------------------------------------------------------------

/// This interface abstracts the target-specific ROM routines.
/// Upgrade scenario:
///  1. onBeforeFirstWrite()
///  2. write() repeated until finished.
///  3. onAfterLastWrite(success or not)
///
/// The read() method may be invoked at any time. Its performance is critical.
/// Slow access may lead to watchdog timeouts (assuming that the watchdog is used) and/or disruption of communications.
/// To avoid issues, ensure that the entirety of the image can be read x10 in less than the watchdog timeout interval.
///
/// The zero offset shall point to the beginning of the ROM segment dedicated to the application.
class IROMBackend
{
protected:
    IROMBackend() = default;

public:
    IROMBackend(const IROMBackend&) = delete;
    IROMBackend(IROMBackend&&)      = delete;
    auto operator=(const IROMBackend&) -> IROMBackend& = delete;
    auto operator=(IROMBackend &&) -> IROMBackend& = delete;

    virtual ~IROMBackend() = default;

    /// This hook allows the ROM driver to enable write operations, erase ROM, etc, depending on the hardware.
    /// False may be returned to indicate failure, the update is aborted in this case.
    /// @return True on success, False on error.
    [[nodiscard]] virtual auto onBeforeFirstWrite() -> bool { return true; }

    /// This hook allows the ROM driver to disable write operations or to perform other hardware-specific steps.
    /// The argument signifies whether the update process was successful. This operation cannot fail.
    virtual void onAfterLastWrite(const bool success) { (void) success; }

    /// @return Number of bytes written; a value less than size indicates an overflow; empty option indicates failure.
    [[nodiscard]] virtual auto write(const std::size_t offset, const std::byte* const data, const std::size_t size)
        -> std::optional<std::size_t> = 0;

    /// @return Number of bytes read; a value less than size indicates an overrun. This operation cannot fail.
    [[nodiscard]] virtual auto read(const std::size_t offset, std::byte* const out_data, const std::size_t size) const
        -> std::size_t = 0;
};

// --------------------------------------------------------------------------------------------------------------------

/// Internal use only.
namespace detail
{
/// This is used to verify integrity of the application and other data.
/// Note that the firmware CRC verification is a computationally expensive process that needs to be completed
/// in a limited time interval, which should be minimized. This class has been carefully manually optimized to
/// achieve the optimal balance between speed and ROM footprint.
/// The function is CRC-64/WE, see http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64.
class CRC64
{
public:
    static constexpr std::size_t Size = 8U;

    void add(const std::byte* const data, const std::size_t len)
    {
        auto bytes = data;
        for (auto remaining = len; remaining > 0; remaining--)
        {
            crc_ ^= static_cast<std::uint64_t>(*bytes) << InputShift;
            ++bytes;
            // Unrolled for performance reasons. This path directly affects the boot-up time, so it is very
            // important to keep it optimized for speed. Rolling this into a loop causes a significant performance
            // degradation at least with GCC since the compiler refuses to unroll the loop when size optimization
            // is selected (which is normally used for bootloaders).
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
        }
    }

    /// The current CRC value.
    [[nodiscard]] auto get() const { return crc_ ^ Xor; }

    /// The current CRC value represented as a big-endian sequence of bytes.
    /// This method is designed for inserting the computed CRC value after the data.
    [[nodiscard]] auto getBytes() const -> std::array<std::byte, Size>
    {
        auto                        x = get();
        std::array<std::byte, Size> out{};
        const auto                  rend = std::rend(out);
        for (auto it = std::rbegin(out); it != rend; ++it)
        {
            *it = static_cast<std::byte>(static_cast<std::uint8_t>(x));
            x >>= BitsPerByte;
        }
        return out;
    }

    /// True if the current CRC value is a correct residue (i.e., CRC verification successful).
    [[nodiscard]] auto isResidueCorrect() const { return crc_ == Residue; }

private:
    static constexpr auto Poly    = static_cast<std::uint64_t>(0x42F0'E1EB'A9EA'3693ULL);
    static constexpr auto Mask    = static_cast<std::uint64_t>(1) << 63U;
    static constexpr auto Xor     = static_cast<std::uint64_t>(0xFFFF'FFFF'FFFF'FFFFULL);
    static constexpr auto Residue = static_cast<std::uint64_t>(0xFCAC'BEBD'5931'A992ULL);

    static constexpr auto BitsPerByte = 8U;
    static constexpr auto InputShift  = 56U;

    std::uint64_t crc_ = Xor;
};

/// Detects the application in the ROM, verifies its integrity, and retrieves the information about it.
class AppLocator
{
public:
    AppLocator(const IROMBackend& backend, const std::uint32_t max_application_image_size) :
        max_application_image_size_(max_application_image_size), backend_(backend)
    {}

    /// Returns the AppInfo if the application is found and its integrity is intact. Otherwise, returns an empty option.
    [[nodiscard]] auto identifyApplication() const -> std::optional<AppInfo>
    {
        for (std::size_t offset = 0; offset < max_application_image_size_; offset += AppDescriptor::SignatureSize)
        {
            AppDescriptor desc{};
            if (sizeof(desc) == backend_.read(offset,
                                              reinterpret_cast<std::byte*>(&desc),  // NOLINT NOSONAR reinterpret_cast
                                              sizeof(desc)))
            {
                if (desc.isValid(max_application_image_size_) &&
                    validateImageCRC(offset + AppDescriptor::CRCOffset,
                                     static_cast<std::size_t>(desc.getAppInfo().image_size),
                                     desc.getAppInfo().image_crc))
                {
                    return desc.getAppInfo();
                }
            }
            else
            {
                break;
            }
        }
        return {};
    }

private:
    class AppDescriptor
    {
    public:
        static constexpr std::size_t SignatureSize = 8U;
        static constexpr std::size_t CRCOffset     = SignatureSize;

        [[nodiscard]] auto isValid(const std::uint32_t max_application_image_size) const -> bool
        {
            return (signature == ReferenceSignature) && (app_info.image_size > 0) &&
                   (app_info.image_size <= max_application_image_size) && ((app_info.image_size % SignatureSize) == 0);
        }

        [[nodiscard]] auto getAppInfo() const -> const AppInfo& { return app_info; }

    private:
        /// The signature is also used for byte order detection.
        /// The value of the signature was obtained from a random number generator, it does not mean anything.
        static constexpr std::uint64_t ReferenceSignature = 0x5E44'1514'6FC0'C4C7ULL;

        std::uint64_t signature;
        AppInfo       app_info;
    };
    static_assert(std::is_trivial_v<AppDescriptor>, "Check your compiler");
    static_assert((AppInfo::Size + AppDescriptor::SignatureSize) == sizeof(AppDescriptor), "Check your compiler");

    [[nodiscard]] auto validateImageCRC(const std::size_t   crc_storage_offset,
                                        const std::size_t   image_size,
                                        const std::uint64_t image_crc) const -> bool
    {
        std::array<std::byte, ROMBufferSize> buffer{};
        CRC64                                crc;
        std::size_t                          offset = 0U;
        // Read large chunks until the CRC field is reached (in most cases it will fit in just one chunk).
        while (offset < crc_storage_offset)
        {
            const auto res =
                backend_.read(offset, buffer.data(), std::min(std::size(buffer), crc_storage_offset - offset));
            if (res > 0)
            {
                offset += res;
                crc.add(buffer.data(), res);
            }
            else
            {
                return false;
            }
        }
        // Fill CRC with zero.
        static const std::array<std::byte, CRC64::Size> dummy{};
        offset += CRC64::Size;
        crc.add(dummy.data(), CRC64::Size);
        // Read the rest of the image in large chunks.
        while (offset < image_size)
        {
            const auto res = backend_.read(offset, buffer.data(), std::min(std::size(buffer), image_size - offset));
            if (res > 0)
            {
                offset += res;
                crc.add(buffer.data(), res);
            }
            else
            {
                return false;
            }
        }
        return crc.get() == image_crc;
    }

    static constexpr std::size_t ROMBufferSize = 256;

    const std::uint32_t max_application_image_size_;
    const IROMBackend&  backend_;
};

}  // namespace detail

// --------------------------------------------------------------------------------------------------------------------

/// The bootloader logic.
/// Larger buffer enables faster CRC verification, which is important, especially with large firmwares.
class Bootloader
{
public:
    /// The max application image size parameter is very important for performance reasons;
    /// without it, the bootloader may encounter an unrelated data structure in the ROM that looks like a
    /// valid app descriptor (by virtue of having the same signature, which is only 64 bit long),
    /// and it may spend a considerable amount of time trying to check the CRC that is certainly invalid.
    /// Having an upper size limit for the application image allows the bootloader to weed out too large
    /// values early, greatly improving the worst case boot time.
    Bootloader(IPlatform& platform, IROMBackend& rom_backend, const std::uint32_t max_application_image_size) :
        max_application_image_size_(max_application_image_size), platform_(platform), backend_(rom_backend)
    {}

    [[nodiscard]] auto run(const std::chrono::microseconds boot_delay = std::chrono::microseconds(0))
        -> std::optional<AppInfo>
    {
        detail::AppLocator locator(backend_, max_application_image_size_);
        auto               app_info = locator.identifyApplication();

        const std::chrono::microseconds boot_deadline = platform_.getMonotonicUptime() + boot_delay;
        (void) boot_deadline;
        (void) &Bootloader::run;
        return app_info;
    }

private:
    const std::uint32_t max_application_image_size_;
    IPlatform&          platform_;
    IROMBackend&        backend_;
};

// --------------------------------------------------------------------------------------------------------------------

/// This helper class allows the bootloader and the application to exchange arbitrary data in a robust way.
/// The data is stored in the specified memory location (usually it is a small dedicated area a few hundred bytes
/// large at the very end of the slowest RAM segment) together with a strong CRC64 hash to ensure its validity.
/// When one component (either the bootloader or the application) needs to pass data to another (e.g., when commencing
/// the upgrade process, the application needs to reboot into the bootloader and pass specific parameters to it),
/// the data is prepared in a particular application-specific data structure which is then passed into this class.
/// The class writes the data structure into the provided memory region and appends the CRC64 hash immediately
/// afterwards (no padding inserted). The other component then checks the memory region where the data is expected to
/// be found and validates its CRC; if the CRC matches, the data is reported to be found, otherwise it is reported
/// that there is no data to read (the latter typically occurs when the bootloader is started after power-on reset,
/// a power loss, or a hard reset).
///
/// The stored data type shall be a trivial type (see https://en.cppreference.com/w/cpp/named_req/TrivialType).
/// The storage space shall be large enough to accommodate an instance of the stored data type plus eight bytes
/// for the CRC (no padding inserted).
///
/// Here is a usage example. Initialization:
///
///     struct MyDataStructureForExchangeBetweenBootloaderAndApplication;
///     VolatileStorage<MyDataStructureForExchangeBetweenBootloaderAndApplication> storage(my_memory_location);
///
/// Reading the data from the storage (the storage is always erased when reading to prevent deja-vu after restart):
///
///     if (auto data = storage.take())
///     {
///         // Process the data...
///     }
///     else
///     {
///         // Data is not available (not stored)
///     }
///
/// Writing the data into the storage: storage.store(data).
template <typename Container>
class VolatileStorage
{
public:
    /// The amount of memory required to store the data. This is the size of the container plus 8 bytes.
    static constexpr auto StorageSize = sizeof(Container) + detail::CRC64::Size;

    explicit VolatileStorage(std::byte* const location) : ptr_(location) {}

    /// Checks if the data is available and reads it, then erases the storage to prevent deja-vu.
    /// Returns an empty option if no data is available (in that case the storage is not erased).
    [[nodiscard]] auto take() -> std::optional<Container>
    {
        detail::CRC64 crc;
        crc.add(ptr_, StorageSize);
        if (crc.isResidueCorrect())
        {
            Container out{};
            (void) std::memmove(&out, ptr_, sizeof(Container));
            (void) std::memset(ptr_, EraseFillValue, StorageSize);
            return out;
        }
        return {};
    }

    /// Writes the data into the storage with CRC64 protection.
    void store(const Container& data)
    {
        (void) std::memmove(ptr_, &data, sizeof(Container));
        detail::CRC64 crc;
        crc.add(ptr_, sizeof(Container));
        const auto crc_ptr = ptr_ + sizeof(Container);  // NOLINT NOSONAR pointer arithmetic
        (void) std::memmove(crc_ptr, crc.getBytes().data(), detail::CRC64::Size);
    }

protected:
    static_assert(std::is_trivial_v<Container>, "Container shall be a trivial type.");

    static constexpr std::uint8_t EraseFillValue = 0xCA;

    std::byte* const ptr_;
};

}  // namespace kocherga
