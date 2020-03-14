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
/// Error codes.  These are returned from functions in negated form, e.g., -100 means error code 100.
static constexpr std::int8_t ErrInvalidParams    = 2;
static constexpr std::int8_t ErrROMWriteFailure  = 3;
static constexpr std::int8_t ErrAppImageTooLarge = 4;

/// This is used to verify integrity of the application and other data.
/// Note that the firmware CRC verification is a computationally expensive process that needs to be completed
/// in a limited time interval, which should be minimized. This class has been carefully manually optimized to
/// achieve the optimal balance between speed and ROM footprint.
/// The function is CRC-64-WE, see http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64.
class CRC64
{
    static constexpr auto Poly = std::uint64_t(0x42F0E1EBA9EA3693ULL);
    static constexpr auto Mask = std::uint64_t(1) << 63U;
    static constexpr auto Xor  = std::uint64_t(0xFFFFFFFFFFFFFFFFULL);

    static constexpr auto InputShift = 56U;

    std::uint64_t crc_ = Xor;

public:
    static constexpr std::size_t Size = 8U;

    void add(const void* data, std::size_t len)
    {
        auto bytes = static_cast<const std::uint8_t*>(data);
        assert(bytes != nullptr);
        while (len-- > 0)
        {
            crc_ ^= std::uint64_t(*bytes++) << InputShift;
            // Unrolled for performance reasons. This path directly affects the boot-up time, so it is very
            // important to keep it optimized for speed. Rolling this into a loop causes a significant performance
            // degradation at least with GCC since the compiler refuses to unroll the loop when size optimization
            // is selected (which is normally used for bootloaders).
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = ((crc_ & Mask) != 0) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
        }
    }

    [[nodiscard]] auto get() const { return crc_ ^ Xor; }
};

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

/// This interface abstracts the target-specific ROM routines.
/// Upgrade scenario:
///  1. beginUpgrade()
///  2. write() repeated until finished.
///  3. endUpgrade(success or not)
///
/// The performance of the ROM reading routine is critical.
/// Slow access may lead to watchdog timeouts (assuming that the watchdog is used),
/// disruption of communications, and premature expiration of the boot timeout.
class IROMBackend
{
public:
    IROMBackend(const IROMBackend&) = delete;
    IROMBackend(IROMBackend&&)      = delete;
    auto operator=(const IROMBackend&) -> IROMBackend& = delete;
    auto operator=(IROMBackend &&) -> IROMBackend& = delete;

    virtual ~IROMBackend() = default;

    /// @return 0 on success, negative on error.
    [[nodiscard]] virtual auto beginUpgrade() -> std::int8_t = 0;

    /// @return number of bytes written; negative on error.
    [[nodiscard]] virtual auto write(std::size_t offset, const void* data, std::size_t size) -> std::intmax_t = 0;

    /// @return 0 on success, negative on error.
    [[nodiscard]] virtual auto endUpgrade(bool success) -> std::int8_t = 0;

    /// @return number of bytes read; negative on error.
    [[nodiscard]] virtual auto read(std::size_t offset, void* out_data, std::size_t size) const -> std::intmax_t = 0;
};

static constexpr std::size_t DefaultROMBufferSize = 1024U;

/// The bootloader logic.
/// Larger buffer enables faster CRC verification, which is important, especially with large firmwares.
template <std::size_t ROMBufferSize = DefaultROMBufferSize>
class Bootloader
{
    IROMBackend& backend_;

    const std::uint32_t             max_application_image_size_;
    const std::chrono::microseconds boot_delay_;
    std::chrono::microseconds       boot_delay_started_at_{};

    std::array<std::uint8_t, ROMBufferSize> rom_buffer_{};

    /// Refer to the Brickproof Bootloader specs.
    /// The structure shall be aligned at 8 bytes boundary, and the image shall be padded to 8 bytes!
    struct AppDescriptor
    {
        static constexpr std::size_t SignatureSize = 8U;
        static constexpr std::size_t Size          = 32U;

        alignas(SignatureSize) std::array<std::uint8_t, SignatureSize> signature{};
        alignas(SignatureSize) AppInfo app_info;

        static constexpr auto getSignatureValue() -> std::array<std::uint8_t, SignatureSize>
        {
            return {{'A', 'P', 'D', 'e', 's', 'c', '0', '0'}};
        }

        auto isValid(const std::uint32_t max_application_image_size) const
        {
            const auto sgn = getSignatureValue();
            return std::equal(std::begin(signature), std::end(signature), std::begin(sgn)) &&
                   (app_info.image_size > 0) && (app_info.image_size <= max_application_image_size) &&
                   ((app_info.image_size % SignatureSize) == 0);
        }
    };
    static_assert(sizeof(AppDescriptor) == AppDescriptor::Size, "Invalid packing");
    static_assert(std::is_standard_layout_v<AppDescriptor>, "AppInfo is not standard layout; check your compiler");
    static_assert(offsetof(AppDescriptor, app_info) + offsetof(AppInfo, image_crc) == AppDescriptor::SignatureSize);

    auto locateAppDescriptor() -> std::optional<AppDescriptor>
    {
        for (std::size_t offset = 0;; offset += AppDescriptor::SignatureSize)
        {
            // Reading the storage in 8 bytes increments until we've found the signature
            {
                std::array<std::uint8_t, AppDescriptor::SignatureSize> signature{};
                const auto res = backend_.read(offset, signature, sizeof(signature));
                if (res != std::int16_t(sizeof(signature)))
                {
                    break;
                }
                const auto reference = AppDescriptor::getSignatureValue();
                if (!std::equal(std::begin(signature), std::end(signature), std::begin(reference)))
                {
                    continue;
                }
            }

            // Reading the entire descriptor
            AppDescriptor desc;
            if (backend_.read(offset, &desc, sizeof(desc)) != std::int16_t(sizeof(desc)))
            {
                break;
            }
            if (!desc.isValid(max_application_image_size_))
            {
                continue;
            }

            // Checking firmware CRC.
            // This block is computationally expensive, so it has been carefully optimized for speed.
            {
                const auto crc_offset = offset + offsetof(AppDescriptor, app_info) + offsetof(AppInfo, image_crc);
                CRC64      crc;

                // Read large chunks until the CRC field is reached (in most cases it will fit in just one chunk)
                for (std::size_t i = 0; i < crc_offset;)
                {
                    const auto res =
                        backend_.read(i,
                                      rom_buffer_.data(),
                                      std::uint16_t(std::min<std::size_t>(rom_buffer_.size(), crc_offset - i)));
                    if (res > 0)
                    {
                        i += std::size_t(res);
                        crc.add(rom_buffer_.data(), std::size_t(res));
                    }
                    else
                    {
                        break;
                    }
                }

                // Fill CRC with zero
                {
                    static const std::array<std::uint8_t, CRC64::Size> dummy{0};
                    crc.add(&dummy[0], sizeof(dummy));
                }

                // Read the rest of the image in large chunks
                for (std::size_t i = crc_offset + CRC64::Size; i < desc.app_info.image_size;)
                {
                    const auto res = backend_.read(i,
                                                   rom_buffer_.data(),
                                                   std::uint16_t(std::min<std::size_t>(rom_buffer_.size(),
                                                                                       desc.app_info.image_size - i)));
                    if (res > 0)
                    {
                        i += std::size_t(res);
                        crc.add(rom_buffer_.data(), std::size_t(res));
                    }
                    else
                    {
                        break;
                    }
                }

                if (crc.get() != desc.app_info.image_crc)
                {
                    continue;  // Look further...
                }
            }

            // Returning if the descriptor is correct
            return {desc};
        }

        return {};
    }

public:
    /// Time since boot will be measured starting from the moment when the object was constructed.
    ///
    /// The max application image size parameter is very important for performance reasons;
    /// without it, the bootloader may encounter an unrelated data structure in the ROM that looks like a
    /// valid app descriptor (by virtue of having the same signature, which is only 64 bit long),
    /// and it may spend a considerable amount of time trying to check the CRC that is certainly invalid.
    /// Having an upper size limit for the application image allows the bootloader to weed out too large
    /// values early, greatly improving the worst case boot time.
    ///
    /// By default, the boot delay is set to zero; i.e., if the application is valid it will be launched immediately.
    explicit Bootloader(IROMBackend&  rom_backend,
                        std::uint32_t max_application_image_size = std::numeric_limits<std::uint32_t>::max(),
                        std::chrono::microseconds boot_delay     = std::chrono::microseconds(0)) :
        backend_(rom_backend), max_application_image_size_(max_application_image_size), boot_delay_(boot_delay)
    {}
};

/// This class allows the user to exchange arbitrary data between the bootloader and the application.
/// The data is CRC-64 protected to ensure its validity.
/// Two usage scenarios are supported:
///
///  1. Simple - the data is simply stored at a dedicated location in RAM. Normally this approach is recommended.
///     Normally the reserved RAM area would be situated at the very end of the RAM.
///
///  2. Memory-efficient - the data is scattered across a set of special-function registers specified by the user.
///     This approach permits the user to avoid reserving any memory regions for data exchange, which is
///     useful in RAM-constrained systems.
///
/// For more information, refer to the factory function @ref makeAppDataExchangeMarshaller().
template <typename Container, typename Pointers>
class AppDataExchangeMarshaller
{
    static_assert(std::is_standard_layout_v<Container>, "Container must be a standard layout type.");

    Pointers pointers_;

    class ContainerWrapper
    {
        std::uint64_t crc_ = 0;

    public:
        Container container;

        ContainerWrapper() : container() {}

        explicit ContainerWrapper(const Container& c) : container(c)
        {
            CRC64 crc_computer;
            crc_computer.add(&container, sizeof(container));
            crc_ = crc_computer.get();
        }

        auto isValid() const
        {
            CRC64 crc_computer;
            crc_computer.add(&container, sizeof(container));
            return crc_ == crc_computer.get();
        }
    };

    template <auto N>
    struct ValueAsType
    {
        static constexpr decltype(N) Value = N;
    };

    template <std::size_t MaxSize, typename T>  // Holy pants why auto doesn't work here
    auto readOne(void* destination, const volatile T* ptr) -> ValueAsType<std::min<std::size_t>(sizeof(T), MaxSize)>
    {
        const T x = *ptr;  // Guaranteeing proper pointer access
        std::memmove(destination, &x, std::min<std::size_t>(sizeof(T), MaxSize));
        return {};
    }

    template <std::size_t MaxSize>
    auto readOne(void* destination, const void* ptr) -> ValueAsType<MaxSize>
    {
        std::memmove(destination, ptr, MaxSize);  // Raw memory access
        return {};
    }

    template <std::size_t MaxSize, typename T>
    auto writeOne(const void* source, volatile T* ptr) -> ValueAsType<std::min<std::size_t>(sizeof(T), MaxSize)>
    {
        T x = T();
        std::memmove(&x, source, std::min<std::size_t>(sizeof(T), MaxSize));
        *ptr = x;  // Guaranteeing proper pointer access
        return {};
    }

    template <std::size_t MaxSize>
    auto writeOne(const void* source, void* ptr) -> ValueAsType<MaxSize>
    {
        std::memmove(ptr, source, MaxSize);  // Raw memory access
        return {};
    }

    template <bool WriteNotRead, std::size_t PtrIndex, std::size_t RemainingSize>
    auto unwindReadWrite(void* structure) -> std::enable_if_t<(RemainingSize > 0)>
    {
        static_assert(PtrIndex < std::tuple_size<Pointers>::value, "Storage is not large enough for the structure");
        const auto ret = WriteNotRead ? writeOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_))
                                      : readOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_));

        constexpr auto Increment = decltype(ret)::Value;
        static_assert(RemainingSize >= Increment, "Rock is dead");

        structure = static_cast<void*>(static_cast<std::uint8_t*>(structure) + Increment);
        unwindReadWrite<WriteNotRead, PtrIndex + 1U, RemainingSize - Increment>(structure);
    }

    template <bool, std::size_t PtrIndex, std::size_t RemainingSize>
    auto unwindReadWrite(void* structure) -> std::enable_if_t<(RemainingSize == 0)>
    {
        (void) structure;
        // Here we could implement a check whether all storage has been utilized.
    }

public:
    /// Do not instantiate this class manually, it's difficult.
    /// Use the factory method instead, @ref makeAppDataExchangeMarshaller().
    explicit AppDataExchangeMarshaller(const Pointers& ptrs) : pointers_(ptrs) {}

    /// Checks if the data is available and reads it; then erases the storage to prevent deja-vu.
    /// Returns an empty option if no data is available (in that case the storage is not erased).
    auto readAndErase() -> std::optional<Container>
    {
        ContainerWrapper wrapper;
        unwindReadWrite<false, 0, sizeof(wrapper)>(&wrapper);
        if (wrapper.isValid())
        {
            ContainerWrapper empty;
            std::memset(&empty, 0, sizeof(empty));
            unwindReadWrite<true, 0, sizeof(empty)>(&empty);
            return wrapper.container;
        }
        return {};
    }

    /// Writes the data. This function cannot fail.
    void write(const Container& cont)
    {
        ContainerWrapper wrapper(cont);
        unwindReadWrite<true, 0, sizeof(wrapper)>(&wrapper);
    }
};

/// Constructs an object that can be used to store and retrieve data for exchange with the application.
/// Usage example:
///
///     auto marshaller = makeAppDataExchangeMarshaller<DataType>(&REG_A, &REG_B, &REG_C, &REG_D, &REG_E, &REG_F);
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
/// The application should use the following memory layout for writing and reading the shared struct:
///
///      Offset  Length  Purpose
///      0       8       CRC64 of the following payload. Must be 8-byte aligned (or more if required by the platform).
///      8       >0      Payload written by the application or by the bootloader
///
/// @tparam Container Payload data type, i.e., a structure that should be stored or read.
///
/// @param pointers   List of pointers to registers or memory where the structure will be stored or retrieved from.
///                   Pointer type defines access mode and size, e.g., a uint32_t pointer will be accessed in
///                   32-bit mode, and its memory block will be used to store exactly 4 bytes, etc.
///                   Supported pointer sizes are 8, 16, 32, and 64 bit. A single void pointer can be passed as well,
///                   in that case all of the data will be simply stored at that pointer using conventional memmove().
///
/// @return An instance of @ref AppDataExchangeMarshaller<>.
template <typename Container, typename... RegisterPointers>
inline auto makeAppDataExchangeMarshaller(RegisterPointers... pointers)
{
    return AppDataExchangeMarshaller<Container, std::tuple<RegisterPointers...>>(std::make_tuple(pointers...));
}

}  // namespace kocherga
