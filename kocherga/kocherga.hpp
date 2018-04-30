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

#include <tuple>
#include <array>
#include <chrono>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <optional>
#include <type_traits>

/**
 * This macro can be defined by the application to provide debug output.
 * By default, all trace outputs are removed by the preprocessor.
 */
#ifndef KOCHERGA_TRACE
# define KOCHERGA_TRACE(...)        (void)0
#endif


namespace kocherga
{
/**
 * Error codes.
 * These are returned from functions in negated form, i.e. -1000 means error code 1000.
 */
static constexpr std::int16_t ErrOK                     = 0;
static constexpr std::int16_t ErrInvalidState           = 1001;
static constexpr std::int16_t ErrAppImageTooLarge       = 1002;
static constexpr std::int16_t ErrROMWriteFailure        = 1003;

/**
 * This is used to verify integrity of the application and other data.
 * Note that firmware CRC verification is a very computationally intensive process that needs to be completed
 * in a limited time interval, which should be minimized. Therefore, this class has been carefully manually
 * optimized to achieve the optimal balance between speed and ROM footprint.
 *
 * CRC-64-WE
 * Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
 * Initial value: 0xFFFFFFFFFFFFFFFF
 * Poly: 0x42F0E1EBA9EA3693
 * Reverse: no
 * Output xor: 0xFFFFFFFFFFFFFFFF
 * Check: 0x62EC59E3F1A4F00A
 */
class CRC64WE
{
    static constexpr std::uint64_t Poly = std::uint64_t(0x42F0E1EBA9EA3693ULL);
    static constexpr std::uint64_t Mask = std::uint64_t(1) << 63U;

    std::uint64_t crc_ = std::uint64_t(0xFFFFFFFFFFFFFFFFULL);

public:
    void add(const void* data, std::size_t len)
    {
        auto bytes = static_cast<const std::uint8_t*>(data);
        assert(bytes != nullptr);
        while (len --> 0)
        {
            crc_ ^= std::uint64_t(*bytes++) << 56U;

            // Do not fold this into loop! The difference in performance can be drastic.
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
            crc_ = (crc_ & Mask) ? (crc_ << 1U) ^ Poly : crc_ << 1U;
        }
    }

    std::uint64_t get() const { return crc_ ^ 0xFFFFFFFFFFFFFFFFULL; }
};

/**
 * Bootloader states. Some of the states are designed as commands to the outer logic, e.g. @ref ReadyToBoot
 * means that the application should be started.
 */
enum class State
{
    NoAppToBoot,
    BootDelay,
    BootCancelled,
    AppUpgradeInProgress,
    ReadyToBoot
};

/**
 * Returns human-readable name of the state.
 */
inline const char* convertStateToString(const State state)
{
    switch (state)
    {
    case State::NoAppToBoot:            return "NoAppToBoot";
    case State::BootDelay:              return "BootDelay";
    case State::BootCancelled:          return "BootCancelled";
    case State::AppUpgradeInProgress:   return "AppUpgradeInProgress";
    case State::ReadyToBoot:            return "ReadyToBoot";
    }

    return "INVALID_STATE";
}

/**
 * These fields are defined by the Brickproof Bootloader specification.
 * Observe that the fields are ordered from largest to smallest in order to avoid padding.
 */
struct AppInfo
{
    std::uint64_t image_crc     = 0;
    std::uint32_t image_size    = 0;
    std::uint32_t vcs_commit    = 0;
    std::uint8_t  major_version = 0;
    std::uint8_t  minor_version = 0;

    std::array<std::uint8_t, 6> _reserved{};    // Explicit padding to avoid compiler-specific padding/alignment
};

static_assert(sizeof(AppInfo) == 24, "AppInfo must be 24 bytes large, not including the signature");
static_assert(std::is_standard_layout_v<AppInfo>, "AppInfo is not standard layout; check your compiler");

/**
 * This interface abstracts the platform-specific functionality.
 * The implementation depends on the hardware and whether there is an operating system.
 */
class IPlatform
{
public:
    virtual ~IPlatform() = default;

    /**
     * Mutex lock/unlock functions are only meaningful in multi-threaded environments,
     * where the bootloader's instance can be accessed concurrently from multiple threads.
     * Single-thread environments and also environments where the bootloader's instance is not
     * shared across different threads should not implement these methods.
     * Note that the mutex must be recursive.
     */
    virtual void lockMutex() { }
    virtual void unlockMutex() { }

    /**
     * Returns the time since boot as a monotonic (i.e. steady) clock.
     */
    virtual std::chrono::microseconds getMonotonicUptime() const = 0;
};

/**
 * This interface abstracts the target-specific ROM routines.
 * Upgrade scenario:
 *  1. beginUpgrade()
 *  2. write() repeated until finished.
 *  3. endUpgrade(success or not)
 *
 * Please note that the performance of the ROM reading routine is critical.
 * Slow access may lead to watchdog timeouts (assuming that the watchdog is used),
 * disruption of communications, and premature expiration of the boot timeout.
 */
class IROMBackend
{
public:
    virtual ~IROMBackend() = default;

    /**
     * @return 0 on success, negative on error
     */
    virtual std::int16_t beginUpgrade() = 0;

    /**
     * The size cannot exceed 32767 bytes.
     * @return number of bytes written; negative on error
     */
    virtual std::int16_t write(std::size_t offset, const void* data, std::uint16_t size) = 0;

    /**
     * @return 0 on success, negative on error
     */
    virtual std::int16_t endUpgrade(bool success) = 0;

    /**
     * The size cannot exceed 32767 bytes.
     * @return number of bytes read; negative on error
     */
    virtual std::int16_t read(std::size_t offset, void* data, std::uint16_t size) const = 0;
};

/**
 * This interface proxies data received by the downloader into the bootloader.
 */
class IDownloadStreamSink
{
public:
    virtual ~IDownloadStreamSink() = default;

    /**
     * The data chunk length cannot exceed 32767 bytes.
     * @return Negative on error, non-negative on success.
     */
    virtual std::int16_t handleNextDataChunk(const void* data, std::uint16_t size) = 0;
};

/**
 * Inherit this class to implement firmware loading protocol, from remote to the local storage.
 */
class IDownloader
{
public:
    virtual ~IDownloader() = default;

    /**
     * Performs the download operation synchronously.
     * Every received data chunk is fed into the sink using the corresponding method (refer to the interface
     * definition). If the sink returns error, downloading will be aborted.
     * @return Negative on error, 0 on success.
     */
    virtual std::int16_t download(IDownloadStreamSink& sink) = 0;
};

/**
 * Main bootloader controller.
 * Beware that this class has a large buffer field used to cache ROM reads. Do not allocate it on the stack.
 */
class Bootloader final
{
    /**
     * RAII mutex manager.
     */
    class MutexLocker final
    {
        IPlatform& pl_;
    public:
        explicit MutexLocker(IPlatform& pl) : pl_(pl) { pl_.lockMutex(); }
        ~MutexLocker()                                { pl_.unlockMutex(); }
    };

    /**
     * A proxy that streams the data from the downloader into the application storage.
     * Note that every access to the storage backend is protected with the mutex!
     */
    class Sink : public IDownloadStreamSink
    {
        IPlatform& platform_;
        IROMBackend& backend_;
        const std::size_t max_image_size_;
        std::size_t offset_ = 0;

        std::int16_t handleNextDataChunk(const void* data, std::uint16_t size) override
        {
            MutexLocker mlock(platform_);

            if ((offset_ + size) <= max_image_size_)
            {
                const auto res = backend_.write(offset_, data, size);
                if ((res >= 0) && (res != int(size)))
                {
                    return -ErrROMWriteFailure;
                }

                offset_ += size;
                return res;
            }
            else
            {
                return -ErrAppImageTooLarge;
            }
        }

    public:
        Sink(IPlatform& pl,
             IROMBackend& back,
             std::size_t max_image_size) :
            platform_(pl),
            backend_(back),
            max_image_size_(max_image_size)
        { }
    };

    State state_{};
    IPlatform& platform_;
    IROMBackend& backend_;

    const std::uint32_t max_application_image_size_;
    const std::chrono::microseconds boot_delay_;
    std::chrono::microseconds boot_delay_started_at_{};

    std::uint8_t rom_buffer_[1024]{};          ///< Larger buffer enables faster CRC verification, which is important

    /// Caching is needed because app check can sometimes take a very long time (several seconds)
    std::optional<AppInfo> cached_app_info_;

    /**
     * Refer to the Brickproof Bootloader specs.
     * Note that the structure must be aligned at 8 bytes boundary, and the image must be padded to 8 bytes!
     */
    struct AppDescriptor
    {
        static constexpr std::size_t ImagePaddingBytes = 8;

        std::array<std::uint8_t, 8> signature{};
        AppInfo app_info;

        static constexpr std::array<std::uint8_t, 8> getSignatureValue()
        {
            return {'A','P','D','e','s','c','0','0'};
        }

        bool isValid(const std::uint32_t max_application_image_size) const
        {
            const auto sgn = getSignatureValue();
            return std::equal(std::begin(signature), std::end(signature), std::begin(sgn)) &&
                   (app_info.image_size > 0) &&
                   (app_info.image_size <= max_application_image_size) &&
                   ((app_info.image_size % ImagePaddingBytes) == 0);
        }
    };
    static_assert(sizeof(AppDescriptor) == 32, "Invalid packing");
    static_assert(std::is_standard_layout_v<AppDescriptor>, "AppInfo is not standard layout; check your compiler");

    std::optional<AppDescriptor> locateAppDescriptor()
    {
        constexpr auto Step = 8;

        for (std::size_t offset = 0;; offset += Step)
        {
            // Reading the storage in 8 bytes increments until we've found the signature
            {
                std::uint8_t signature[Step] = {};
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
            {
                const auto res = backend_.read(offset, &desc, sizeof(desc));
                if (res != std::int16_t(sizeof(desc)))
                {
                    break;
                }
                if (!desc.isValid(max_application_image_size_))
                {
                    continue;
                }
            }

            // Checking firmware CRC.
            // This block is very computationally intensive, so it has been carefully optimized for speed.
            {
                const auto crc_offset = offset + offsetof(AppDescriptor, app_info.image_crc);
                CRC64WE crc;

                // Read large chunks until the CRC field is reached (in most cases it will fit in just one chunk)
                for (std::size_t i = 0; i < crc_offset;)
                {
                    const auto res =
                        backend_.read(i, rom_buffer_,
                                      std::uint16_t(std::min<std::size_t>(sizeof(rom_buffer_), crc_offset - i)));
                    if (res > 0)
                    {
                        i += std::size_t(res);
                        crc.add(rom_buffer_, std::size_t(res));
                    }
                    else
                    {
                        break;
                    }
                }

                // Fill CRC with zero
                {
                    static const std::uint8_t dummy[8]{0};
                    crc.add(&dummy[0], sizeof(dummy));
                }

                // Read the rest of the image in large chunks
                for (std::size_t i = crc_offset + 8; i < desc.app_info.image_size;)
                {
                    const auto res = backend_.read(i, rom_buffer_,
                                                   std::uint16_t(std::min<std::size_t>(sizeof(rom_buffer_),
                                                                                       desc.app_info.image_size - i)));
                    if (res > 0)
                    {
                        i += std::size_t(res);
                        crc.add(rom_buffer_, std::size_t(res));
                    }
                    else
                    {
                        break;
                    }
                }

                if (crc.get() != desc.app_info.image_crc)
                {
                    KOCHERGA_TRACE("App descriptor found, but CRC is invalid\n");
                    continue;       // Look further...
                }
            }

            // Returning if the descriptor is correct
            KOCHERGA_TRACE("App descriptor located at offset %x\n", unsigned(offset));
            return {desc};
        }

        return {};
    }

    void verifyAppAndUpdateState(const State state_on_success)
    {
        if (const auto appdesc = locateAppDescriptor())
        {
            cached_app_info_ = appdesc->app_info;
            state_ = state_on_success;
            boot_delay_started_at_ =
                platform_.getMonotonicUptime();     // This only makes sense if the new state is BootDelay
            KOCHERGA_TRACE("App found; version %d.%d.%x, %d bytes\n",
                           appdesc_result.first.app_info.major_version,
                           appdesc_result.first.app_info.minor_version,
                           unsigned(appdesc->app_info.vcs_commit),
                           unsigned(appdesc->app_info.image_size));
        }
        else
        {
            cached_app_info_.reset();
            state_ = State::NoAppToBoot;
            KOCHERGA_TRACE("App not found\n");
        }
    }

public:
    /**
     * Time since boot will be measured starting from the moment when the object was constructed.
     *
     * The max application image size parameter is very important for performance reasons;
     * without it, the bootloader may encounter an unrelated data structure in the ROM that looks like a
     * valid app descriptor (by virtue of having the same signature, which is only 64 bit long),
     * and it may spend a considerable amount of time trying to check the CRC that is certainly invalid.
     * Having an upper size limit for the application image allows the bootloader to weed out too large
     * values early, greatly improving robustness.
     *
     * By default, the boot delay is set to zero; i.e. if the application is valid it will be launched immediately.
     */
    Bootloader(IPlatform& platform,
               IROMBackend& rom_backend,
               std::uint32_t max_application_image_size = 0xFFFFFFFFUL,
               std::chrono::microseconds boot_delay = std::chrono::microseconds(0)) :
        platform_(platform),
        backend_(rom_backend),
        max_application_image_size_(max_application_image_size),
        boot_delay_(boot_delay)
    {
        MutexLocker mlock(platform_);
        verifyAppAndUpdateState(State::BootDelay);
    }

    /**
     * @ref State.
     */
    State getState()
    {
        MutexLocker mlock(platform_);
        if ((state_ == State::BootDelay) &&
            ((platform_.getMonotonicUptime() - boot_delay_started_at_) >= boot_delay_))
        {
            KOCHERGA_TRACE("Boot delay expired\n");
            state_ = State::ReadyToBoot;
        }

        return state_;
    }

    /**
     * If there is a valid application in the ROM, returns info about it.
     * Otherwise returns an empty option.
     */
    std::optional<AppInfo> getAppInfo()
    {
        MutexLocker mlock(platform_);
        if (cached_app_info_)
        {
            return *cached_app_info_;
        }
        else
        {
            return {};
        }
    }

    /**
     * Switches the state to @ref BootCancelled, if allowed.
     */
    void cancelBoot()
    {
        MutexLocker mlock(platform_);
        switch (state_)
        {
        case State::BootDelay:
        case State::ReadyToBoot:
        {
            state_ = State::BootCancelled;
            KOCHERGA_TRACE("Boot cancelled\n");
            break;
        }
        case State::NoAppToBoot:
        case State::BootCancelled:
        case State::AppUpgradeInProgress:
        {
            break;
        }
        }
    }

    /**
     * Switches the state to @ref ReadyToBoot, if allowed.
     */
    void requestBoot()
    {
        MutexLocker mlock(platform_);

        switch (state_)
        {
        case State::BootDelay:
        case State::BootCancelled:
        {
            state_ = State::ReadyToBoot;
            KOCHERGA_TRACE("Boot requested\n");
            break;
        }
        case State::NoAppToBoot:
        case State::AppUpgradeInProgress:
        case State::ReadyToBoot:
        {
            break;
        }
        }
    }

    /**
     * Template method that implements all of the high-level steps of the application update procedure.
     */
    std::int16_t upgradeApp(IDownloader& downloader)
    {
        /*
         * Preparation stage.
         * Note that access to the backend and all members is always protected with the mutex, this is important.
         */
        {
            MutexLocker mlock(platform_);

            switch (state_)
            {
            case State::BootDelay:
            case State::BootCancelled:
            case State::NoAppToBoot:
            {
                break;      // OK, continuing below
            }
            case State::ReadyToBoot:
            case State::AppUpgradeInProgress:
            {
                return -ErrInvalidState;
            }
            }

            state_ = State::AppUpgradeInProgress;
            cached_app_info_.reset();                           // Invalidate now, as we're going to modify the storage

            const auto res = backend_.beginUpgrade();
            if (res < 0)
            {
                verifyAppAndUpdateState(State::BootCancelled);  // The backend could have modified the storage
                return res;
            }
        }

        KOCHERGA_TRACE("Starting app upgrade...\n");

        /*
         * Downloading stage.
         * New application is downloaded into the storage backend via the Sink proxy class.
         * Every write() via the Sink is mutex-protected.
         */
        Sink sink(platform_, backend_, max_application_image_size_);

        auto res = downloader.download(sink);
        KOCHERGA_TRACE("App download finished with status %d\n", res);

        /*
         * Finalization stage.
         * Checking if the downloader has succeeded, checking if the backend is able to finalize successfully.
         * Notice the mutex.
         */
        MutexLocker mlock(platform_);

        assert(state_ == State::AppUpgradeInProgress);
        state_ = State::NoAppToBoot;                // Default state until proven otherwise

        if (res < 0)                                // Download failed
        {
            (void)backend_.endUpgrade(false);       // Making sure the backend is finalized; error is irrelevant
            verifyAppAndUpdateState(State::BootCancelled);
            return res;
        }

        res = backend_.endUpgrade(true);
        if (res < 0)                                // Finalization failed
        {
            KOCHERGA_TRACE("App storage backend finalization failed (%d)\n", res);
            verifyAppAndUpdateState(State::BootCancelled);
            return res;
        }

        /*
         * Everything went well, checking if the application is valid and updating the state accordingly.
         * This method will report success even if the application image it just downloaded is not valid,
         * since that would be out of the scope of its responsibility.
         */
        verifyAppAndUpdateState(State::BootDelay);

        return ErrOK;
    }
};

/**
 * This option allows to erase the structure automatically once it's read
 */
enum class AutoErase
{
    EraseAfterRead,
    DoNotErase
};

/**
 * Implementation details, do not use directly
 */
namespace impl_
{

template <typename Container, typename Pointers>
class AppSharedMarshaller
{
    Pointers pointers_;

    struct ContainerWrapper
    {
        Container container;

    private:
        std::uint64_t crc_ = 0;

    public:
        ContainerWrapper() : container()  { }

        explicit ContainerWrapper(const Container& c) :
            container(c)
        {
            CRC64WE crc_computer;
            crc_computer.add(&container, sizeof(container));
            crc_ = crc_computer.get();
        }

        bool isValid() const
        {
            CRC64WE crc_computer;
            crc_computer.add(&container, sizeof(container));
            return crc_ == crc_computer.get();
        }
    };

    template <auto N>
    struct ValueAsType
    {
        static constexpr decltype(N) Value = N;
    };

    template <std::size_t MaxSize, typename T>                     // Holy pants why auto doesn't work here
    ValueAsType<std::min<std::size_t>(sizeof(T), MaxSize)> readOne(void* destination, const volatile T* ptr)
    {
        const T x = *ptr;                                       // Guaranteeing proper pointer access
        std::memmove(destination, &x, std::min<std::size_t>(sizeof(T), MaxSize));
        return {};
    }

    template <std::size_t MaxSize>
    ValueAsType<MaxSize> readOne(void* destination, const void* ptr)
    {
        std::memmove(destination, ptr, MaxSize);                // Raw memory access
        return {};
    }

    template <std::size_t MaxSize, typename T>
    ValueAsType<std::min<std::size_t>(sizeof(T), MaxSize)> writeOne(const void* source, volatile T* ptr)
    {
        T x = T();
        std::memmove(&x, source, std::min<std::size_t>(sizeof(T), MaxSize));
        *ptr = x;                                               // Guaranteeing proper pointer access
        return {};
    }

    template <std::size_t MaxSize>
    ValueAsType<MaxSize> writeOne(const void* source, void* ptr)
    {
        std::memmove(ptr, source, MaxSize);                     // Raw memory access
        return {};
    }

    template <bool WriteNotRead, std::size_t PtrIndex, std::size_t RemainingSize>
    typename std::enable_if<(RemainingSize > 0)>::type unwindReadWrite(void* structure)
    {
        static_assert(PtrIndex < std::tuple_size<Pointers>::value, "Storage is not large enough for the structure");
        const auto ret = WriteNotRead ?
                         writeOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_)) :
                         readOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_));

        constexpr auto Increment = decltype(ret)::Value;
        static_assert(RemainingSize >= Increment, "Rock is dead");

        structure = static_cast<void*>(static_cast<std::uint8_t*>(structure) + Increment);
        unwindReadWrite<WriteNotRead, PtrIndex + 1U, RemainingSize - Increment>(structure);
    }

    template <bool, std::size_t PtrIndex, std::size_t RemainingSize>
    typename std::enable_if<(RemainingSize == 0)>::type unwindReadWrite(void*)
    {
        // Here we could implement a check whether all storage has been utilized.
    }

public:
    explicit AppSharedMarshaller(const Pointers& ptrs) : pointers_(ptrs) { }

    /**
     * Checks if the data is available and reads it.
     * Returns an empty option if no data is available.
     */
    std::optional<Container> read(AutoErase auto_erase = AutoErase::DoNotErase)
    {
        ContainerWrapper wrapper;
        unwindReadWrite<false, 0, sizeof(wrapper)>(&wrapper);

        if (wrapper.isValid())
        {
            if (auto_erase == AutoErase::EraseAfterRead)
            {
                erase();
            }

            return wrapper.container;
        }
        else
        {
            return {};
        }
    }

    /**
     * Writes the data. This function cannot fail.
     */
    void write(const Container& cont)
    {
        ContainerWrapper wrapper(cont);
        unwindReadWrite<true, 0, sizeof(wrapper)>(&wrapper);
    }

    /**
     * Invalidates the stored data.
     */
    void erase()
    {
        ContainerWrapper wrapper;
        std::memset(&wrapper, 0, sizeof(wrapper));
        unwindReadWrite<true, 0, sizeof(wrapper)>(&wrapper);
    }
};

} // namespace impl_

/**
 * Constructs an object that can be used to store and retrieve data for exchange with the application.
 * Usage example:
 *
 *     auto marshaller = makeAppSharedMarshaller<DataType>(&REG_A, &REG_B, &REG_C, &REG_D, &REG_E, &REG_F);
 *     // Reading data:
 *     auto result = marshaller.read();
 *     if (result.second)
 *     {
 *         // Process the data...
 *     }
 *     else
 *     {
 *         // Data is not available (not stored)
 *     }
 *     // Writing data:
 *     marshaller.write(the_data);
 *     // Erasing data:
 *     marshaller.erase();
 *
 * @tparam Container                    Payload data type, i.e. a structure that should be stored or read.
 *
 * @param pointers                      List of pointers to registers or memory where the structure will be stored or
 *                                      retrieved from. Pointer type defines access mode and size, e.g. a uint32
 *                                      pointer will be accessed in 32-bit mode, and its memory block will be used to
 *                                      store exactly 4 bytes, etc. Supported pointer sizes are 8, 16, 32, and 64 bit.
 *
 * @return                              An instance of @ref impl_::AppSharedMarshaller<>.
 *                                      The returned instance supports methods read(), write(), and erase(), that can
 *                                      be used to read, write, and erase the storage, respectively.
 */
template <typename Container, typename... RegisterPointers>
inline auto makeAppSharedMarshaller(RegisterPointers... pointers)
{
    typedef impl_::AppSharedMarshaller<Container, decltype(std::make_tuple(pointers...))> Type;
    return Type(std::make_tuple(pointers...));
}

}
