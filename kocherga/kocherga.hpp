// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <tuple>
#include <type_traits>

namespace kocherga
{
/// Version of the library, major and minor.
static constexpr std::pair<std::uint8_t, std::uint8_t> Version{1, 0};

/// Error codes.  These are returned from functions in negated form, e.g., -100 means error code 100.
static constexpr std::int8_t ErrInvalidParams    = 2;
static constexpr std::int8_t ErrROMWriteFailure  = 3;
static constexpr std::int8_t ErrAppImageTooLarge = 4;

// --------------------------------------------------------------------------------------------------------------------

/// The structure is mapped to the ROM. The fields are ordered to avoid padding.
struct AppInfo
{
    static constexpr std::uint8_t FlagReleaseBuild = 1U;
    static constexpr std::uint8_t FlagDirtyBuild   = 2U;

    /// Offset 0 bytes.
    /// CRC-64-WE of the firmware image padded to 8 bytes while this field zero.
    std::uint64_t image_crc = 0;

    /// Offset 8 bytes.
    /// Size of the application image in bytes.
    /// Version control system revision ID (e.g., git commit hash).
    std::uint32_t image_size = 0;
    std::uint32_t vcs_commit = 0;

    /// Offset 16 bytes.
    /// Semantic version numbers.
    /// Flags: 1 - release build, 2 - dirty build.
    std::uint8_t major_version = 0;
    std::uint8_t minor_version = 0;
    std::uint8_t flags         = 0;
    std::uint8_t _reserved_a_  = 0;

    /// Offset 20 bytes
    /// UTC Unix time in seconds when the application was built.
    std::uint32_t build_timestamp_utc = 0;

    [[nodiscard]] auto isReleaseBuild() const { return (flags & FlagReleaseBuild) != 0; }
    [[nodiscard]] auto isDirtyBuild() const { return (flags & FlagDirtyBuild) != 0; }
};
static_assert(std::is_standard_layout_v<AppInfo>, "AppInfo is not standard layout; check your compiler");

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
class IROMBackend
{
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
    [[nodiscard]] virtual auto write(const std::size_t offset, const void* const data, const std::size_t size)
        -> std::optional<std::size_t> = 0;

    /// @return Number of bytes read; a value less than size indicates an overrun. This operation cannot fail.
    [[nodiscard]] virtual auto read(const std::size_t offset, void* const out_data, const std::size_t size) const
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
/// The function is CRC-64-WE, see http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64.
class CRC64
{
    static constexpr auto Poly = static_cast<std::uint64_t>(0x42F0'E1EB'A9EA'3693ULL);
    static constexpr auto Mask = static_cast<std::uint64_t>(1) << 63U;
    static constexpr auto Xor  = static_cast<std::uint64_t>(0xFFFF'FFFF'FFFF'FFFFULL);

    static constexpr auto InputShift = 56U;

    std::uint64_t crc_ = Xor;

public:
    static constexpr std::size_t Size = 8U;

    void add(const std::uint8_t* const data, const std::size_t len)
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

    [[nodiscard]] auto get() const { return crc_ ^ Xor; }
};

/// Detects the application in the ROM, verifies it, and retrieves the information about it.
class AppLocator
{
public:
    AppLocator(IROMBackend& backend, const std::uint32_t max_application_image_size) :
        max_application_image_size_(max_application_image_size), backend_(backend)
    {}

    [[nodiscard]] auto identifyApplication() const -> std::optional<AppInfo>
    {
        for (std::size_t offset = 0; offset < max_application_image_size_; offset += AppDescriptor::SignatureSize)
        {
            AppDescriptor desc;
            if (sizeof(desc) == backend_.read(offset, &desc, sizeof(desc)))
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
        static constexpr std::size_t Size          = 32U;
        static constexpr std::size_t CRCOffset     = SignatureSize;

        [[nodiscard]] auto isValid(const std::uint32_t max_application_image_size) const -> bool
        {
            return std::equal(std::begin(signature), std::end(signature), std::begin(ReferenceSignature)) &&
                   (app_info.image_size > 0) && (app_info.image_size <= max_application_image_size) &&
                   ((app_info.image_size % SignatureSize) == 0);
        }

        [[nodiscard]] auto getAppInfo() const -> const AppInfo& { return app_info; }

    private:
        static constexpr std::array<std::uint8_t, SignatureSize> ReferenceSignature{
            {65, 80, 68, 101, 115, 99, 48, 48}};  // APDesc00

        alignas(SignatureSize) std::array<std::uint8_t, SignatureSize> signature{};
        alignas(SignatureSize) AppInfo app_info;
    };
    static_assert(sizeof(AppDescriptor) == AppDescriptor::Size, "Invalid packing");
    static_assert(std::is_standard_layout_v<AppDescriptor>, "Check your compiler");

    [[nodiscard]] auto validateImageCRC(const std::size_t   crc_storage_offset,
                                        const std::size_t   image_size,
                                        const std::uint64_t image_crc) const -> bool
    {
        std::array<std::uint8_t, ROMBufferSize> buffer{};
        CRC64                                   crc;
        std::size_t                             offset = 0U;
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
        static const std::array<std::uint8_t, CRC64::Size> dummy{};
        offset += std::size(dummy);
        crc.add(dummy.data(), std::size(dummy));
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
    IROMBackend&        backend_;
};

/// Refer to the factory function makeAppDataMarshaller().
template <typename Container, typename Pointers>
class AppDataMarshaller
{
    static_assert(std::is_standard_layout_v<Container>, "Container must be a standard layout type.");

    Pointers pointers_;

    struct ContainerWrapper
    {
        std::uint64_t crc = 0;
        Container     container{};

        ContainerWrapper() = default;

        explicit ContainerWrapper(const Container& c) : container(c)
        {
            detail::CRC64 crc_computer;
            crc_computer.add(reinterpret_cast<const std::uint8_t*>(&container),  // NOLINT NOSONAR reinterpret_cast<>()
                             sizeof(container));
            crc = crc_computer.get();
        }

        [[nodiscard]] auto isValid() const
        {
            detail::CRC64 crc_computer;
            crc_computer.add(reinterpret_cast<const std::uint8_t*>(&container),  // NOLINT NOSONAR reinterpret_cast<>()
                             sizeof(container));
            return crc == crc_computer.get();
        }
    };

    template <std::size_t N>
    using Size = std::integral_constant<std::size_t, N>;

    template <std::size_t MaxSize, typename T>  // Holy pants why auto doesn't work here
    [[nodiscard]] auto readOne(std::byte* const destination, const volatile T* const ptr)
        -> Size<std::min<std::size_t>(sizeof(T), MaxSize)>
    {
        const T x = *ptr;  // Guaranteeing proper pointer access
        std::memmove(destination, &x, std::min<std::size_t>(sizeof(T), MaxSize));
        return {};
    }

    template <std::size_t MaxSize>
    [[nodiscard]] auto readOne(std::byte* const destination, const void* const ptr) -> Size<MaxSize>
    {
        std::memmove(destination, ptr, MaxSize);
        return {};
    }

    template <std::size_t MaxSize, typename T>
    [[nodiscard]] auto writeOne(const std::byte* const source, volatile T* const ptr)
        -> Size<std::min<std::size_t>(sizeof(T), MaxSize)>
    {
        T x = T();
        std::memmove(&x, source, std::min<std::size_t>(sizeof(T), MaxSize));
        *ptr = x;  // Guaranteeing proper pointer access
        return {};
    }

    template <std::size_t MaxSize>
    [[nodiscard]] auto writeOne(const std::byte* const source, void* const ptr) -> Size<MaxSize>
    {
        std::memmove(ptr, source, MaxSize);
        return {};
    }

    template <bool WriteNotRead, std::size_t PtrIndex, std::size_t RemainingSize>
    auto unwindReadWrite(std::byte* const structure) -> std::enable_if_t<(RemainingSize > 0)>
    {
        static_assert(PtrIndex < std::tuple_size<Pointers>::value, "Storage is not large enough for the structure");
        const auto ret = WriteNotRead ? writeOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_))
                                      : readOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_));
        constexpr auto Increment = decltype(ret)::value;
        static_assert(RemainingSize >= Increment);
        unwindReadWrite<WriteNotRead, PtrIndex + 1U, RemainingSize - Increment>(
            structure + Increment);  // NOSONAR NOLINT pointer arithmetic
    }

    template <bool, std::size_t, std::size_t RemainingSize>
    auto unwindReadWrite(const std::byte* const structure) -> std::enable_if_t<(RemainingSize == 0)>
    {
        (void) structure;
        // Here we could implement a check whether all storage has been utilized.
    }

public:
    /// Do not instantiate this class manually. Use the factory makeAppDataMarshaller() instead.
    explicit AppDataMarshaller(const Pointers ps) : pointers_(std::move(ps)) {}

    /// Checks if the data is available and reads it; then erases the storage to prevent deja-vu.
    /// Returns an empty option if no data is available (in that case the storage is not erased).
    [[nodiscard]] auto readAndErase() -> std::optional<Container>
    {
        ContainerWrapper wrapper;
        unwindReadWrite<false, 0, sizeof(wrapper)>(
            reinterpret_cast<std::byte*>(&wrapper));  // NOLINT NOSONAR reinterpret_cast<>()
        if (wrapper.isValid())
        {
            ContainerWrapper empty;
            std::memset(&empty, 0, sizeof(empty));
            unwindReadWrite<true, 0, sizeof(empty)>(
                reinterpret_cast<std::byte*>(&empty));  // NOLINT NOSONAR reinterpret_cast<>()
            return wrapper.container;
        }
        return {};
    }

    /// Writes the data. This function cannot fail.
    void write(const Container& cont)
    {
        ContainerWrapper wrapper(cont);
        unwindReadWrite<true, 0, sizeof(wrapper)>(
            reinterpret_cast<std::byte*>(&wrapper));  // NOLINT NOSONAR reinterpret_cast<>()
    }
};

}  // namespace detail

// --------------------------------------------------------------------------------------------------------------------

/// The bootloader logic.
/// Larger buffer enables faster CRC verification, which is important, especially with large firmwares.
class Bootloader
{
    const std::uint32_t max_application_image_size_;
    IPlatform&          platform_;
    IROMBackend&        backend_;

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
};

// --------------------------------------------------------------------------------------------------------------------

/// Constructs an object that can be used to store and retrieve data for exchange with the application.
/// The data is CRC-64 protected to ensure its validity.
/// Two usage scenarios are supported:
///
///  1. Simple - the data is simply stored at a dedicated location in RAM. Normally this approach is recommended.
///     Normally the reserved area would be situated at the very end of the RAM.
///
///  2. Memory-efficient - the data is scattered across a set of special-function registers specified by the user.
///     This approach permits the user to avoid reserving any memory regions for data exchange, which is
///     useful in RAM-constrained systems.
///
/// Usage example:
///
///     // The list of registers can be replaced with a single void* pointer to a specific memory region.
///     auto marshaller = makeAppDataMarshaller<DataType>(&REG_A, &REG_B, &REG_C, &REG_D, &REG_E, &REG_F);
///     // Reading data (always destructively):
///     if (auto data = marshaller.readAndErase())
///     {
///         // Process the data...
///     }
///     else
///     {
///         // Data is not available (not stored)
///     }
///     // Writing data:
///     marshaller.write(the_data);
///
/// The following memory layout is used for the shared struct:
///
///      Offset  Length  Purpose
///      0       8       CRC64 of the following payload. Must be 8-byte aligned (or more if required by the platform).
///      8       >0      Payload written by the application or by the bootloader.
///
/// @tparam Container Payload data type, i.e., a structure that should be stored or read.
///
/// @param pointers   List of pointers to registers or memory where the structure will be stored or retrieved from.
///                   Pointer type defines access mode and size, e.g., a uint32_t pointer will be accessed in
///                   32-bit mode, and its memory block will be used to store exactly 4 bytes, etc.
///                   Supported pointer sizes are 8, 16, 32, and 64 bit. A single void pointer can be passed as well,
///                   in that case all of the data will be simply stored at that pointer using conventional memmove().
///
/// @return An instance of @ref AppDataMarshaller<>.
template <typename Container, typename... RegisterPointers>
[[nodiscard]] inline auto makeAppDataMarshaller(RegisterPointers... pointers)
{
    return detail::AppDataMarshaller<Container, std::tuple<RegisterPointers...>>(std::make_tuple(pointers...));
}

}  // namespace kocherga
