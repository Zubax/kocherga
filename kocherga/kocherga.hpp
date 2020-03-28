// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <type_traits>

namespace kocherga
{
/// Semantic version number pair: major then minor.
using SemanticVersion = std::array<std::uint8_t, 2>;
using TransferID      = std::uint64_t;
using NodeID          = std::uint16_t;

/// Version of the library, major and minor.
static constexpr SemanticVersion Version{{1, 0}};

/// Version of the UAVCAN specification implemented by this library, major and minor.
static constexpr SemanticVersion UAVCANSpecificationVersion{{1, 0}};

/// The service response timeout used by the bootloader.
/// This value applies when the bootloader invokes uavcan.file.Read during the update.
static constexpr std::chrono::seconds ServiceResponseTimeout{3};

// --------------------------------------------------------------------------------------------------------------------

/// Bootloader controller states.
/// The state ReadyToBoot indicates that the application should be started.
///
/// The following state transition diagram illustrates the operating principles of the bootloader:
///
///     No valid application found ###################### Valid application found
///               /----------------# Bootloader started #----------+ /-------------------------------------------+
///               |                ######################          | |                                           |
///               v                                                v v  Boot delay expired                       |
///         +-------------+                               +-----------+  (typically zero)  +-------------+       |
///     /-->| NoAppToBoot |        /----------------------| BootDelay |------------------->| ReadyToBoot |       |
///     |   +-------------+       /                       +-----------+                    +-------------+       |
///     |          |             /                          |Boot cancelled,                  |ReadyToBoot is    |
///     |Update    |<-----------/                           |e.g., received a state transition|an auxiliary      /
///     |failed,   |Update requested,                       |request to BootCancelled.        |state, it is     /
///     |no valid  |e.g., received a state transition       v                                 |left automati-  /
///     |image is  |request to AppUpdateInProgress.  +---------------+                        |cally ASAP.    /
///     |now ava-  |<--------------------------------| BootCancelled |                        v              /
///     |ilable    |                                 +---------------+                ###############       /
///     |          v                                        ^                         # Booting the #      /
///     | +----------------------+ Update failed, but the   |                         # application #     /
///     +-| AppUpdateInProgress  |--------------------------/                         ###############    /
///       +----------------------+ existing valid image was not                                         /
///                |               altered and remains valid.                                          /
///                |                                                                                  /
///                | Update successful, received image is valid.                                     /
///                +--------------------------------------------------------------------------------/
enum class State
{
    NoAppToBoot,
    BootDelay,
    BootCancelled,
    AppUpdateInProgress,
    ReadyToBoot,
};

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

/// This information is provided by the bootloader host system during initialization. It does not change at runtime.
/// For documentation, please refer to uavcan.node.GetInfo.Response.
struct SystemInfo
{
    SemanticVersion hardware_version{};

    static constexpr std::size_t          UIDCapacity = 16;
    std::array<std::uint8_t, UIDCapacity> unique_id{};

    const char* node_name = "";

    /// CoA normally points into a specific region of ROM, but this is not required. Set 0/nullptr if not available.
    std::uint8_t     certificate_of_authenticity_length = 0;
    const std::byte* certificate_of_authenticity        = nullptr;
};

// --------------------------------------------------------------------------------------------------------------------

/// A standard IoC delegate for handling protocol events in the node.
/// The user code will INVOKE this interface, not implement it.
/// A reference to the reactor is supplied to INode implementations to let them delegate application-level activities
/// back to the bootloader core.
/// The methods accept raw serialized representation and return one as well.
/// Serialization/deserialization is done by the bootloader core because DSDL is transport-agnostic.
/// All methods are non-blocking and return immediately.
/// Implementations utilizing just a single transport should not incur any polymorphism-induced overhead because decent
/// C++ compilers are quite good at devirtualization.
class IReactor
{
public:
    static constexpr std::size_t ExecuteCommandResponseLength = 7;
    static constexpr std::size_t NodeInfoResponseMaxLength    = 313;

    /// Invoked when the node receives a uavcan.node.ExecuteResponse request.
    virtual void processExecuteCommandRequest(const NodeID           client_node_id,
                                              const std::size_t      request_length,
                                              const std::byte* const request,
                                              std::byte* const       out_response) = 0;

    /// Invoked when the node receives a uavcan.node.GetInfo request. The request does not have any useful payload.
    /// The client node-ID not provided because it doesn't matter.
    /// Returns the number of bytes written into out_response, which is never greater than NodeInfoResponseCapacity.
    [[nodiscard]] virtual auto processNodeInfoRequest(std::byte* const out_response) const -> std::size_t = 0;

    /// Invoked when the node receives the response to a previously sent uavcan.file.Read request.
    /// This service is actually used for transferring the application image when update is in progress.
    virtual void processFileReadResponse(const std::size_t response_length, const std::byte* const response) = 0;

    virtual ~IReactor()        = default;
    IReactor()                 = default;
    IReactor(const IReactor&)  = delete;
    IReactor(const IReactor&&) = delete;
    auto operator=(const IReactor&) -> IReactor& = delete;
    auto operator=(const IReactor &&) -> IReactor& = delete;
};

// --------------------------------------------------------------------------------------------------------------------

/// The transport-specific node abstraction interface. Kocherga runs a separate node per transport interface.
/// If redundant transports are desired, they should be implemented in a custom implementation of INode.
/// If the node implementation is unable to perform the requested action (for example, because a node-ID allocation
/// is still in progress), it shall ignore the commanded action or return an error if such possibility is provided.
class INode
{
public:
    static constexpr std::size_t HeartbeatLength = 7;

    /// The bootloader invokes this method every tick to let the node run background activities such as
    /// processing incoming transfers. If a reaction is required (such as responding to a service request),
    /// it is delegated to the bootloader core via the IoC IReactor.
    virtual void poll(IReactor& reactor, const std::chrono::microseconds uptime) = 0;

    /// Send a request uavcan.file.Read.
    /// The response will be delivered later asynchronously via IReactor.
    /// The return value is True on success and False if the request could not be sent (aborts the update process).
    [[nodiscard]] virtual auto requestFileRead(const NodeID           server_node_id,
                                               const TransferID       transfer_id,
                                               const std::size_t      payload_length,
                                               const std::byte* const payload) -> bool = 0;

    /// Publish a message uavcan.node.Heartbeat. Observe that the size of the payload is fixed so it is not passed.
    /// If an error occurred, it shall be ignored or reported using other means.
    /// Such lax error handling policy is implemented because the bootloader need not react to transient failures.
    virtual void publishHeartbeat(const TransferID transfer_id, const std::byte* const payload) = 0;

    /// Publish a message uavcan.diagnostic.Record.
    /// If an error occurred, it shall be ignored or reported using other means.
    virtual void publishLogRecord(const TransferID       transfer_id,
                                  const std::size_t      payload_length,
                                  const std::byte* const payload) = 0;

    virtual ~INode()     = default;
    INode()              = default;
    INode(const INode&)  = delete;
    INode(const INode&&) = delete;
    auto operator=(const INode&) -> INode& = delete;
    auto operator=(const INode &&) -> INode& = delete;
};

// --------------------------------------------------------------------------------------------------------------------

/// This interface abstracts the target-specific ROM routines.
/// App update scenario:
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
public:
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

    virtual ~IROMBackend()          = default;
    IROMBackend()                   = default;
    IROMBackend(const IROMBackend&) = delete;
    IROMBackend(IROMBackend&&)      = delete;
    auto operator=(const IROMBackend&) -> IROMBackend& = delete;
    auto operator=(IROMBackend &&) -> IROMBackend& = delete;
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
class AppLocator final
{
public:
    AppLocator(const IROMBackend& backend, const std::uint32_t max_app_size) :
        max_app_size_(max_app_size), backend_(backend)
    {}

    /// Returns the AppInfo if the app is found and its integrity is intact. Otherwise, returns an empty option.
    [[nodiscard]] auto identifyApplication() const -> std::optional<AppInfo>
    {
        for (std::size_t offset = 0; offset < max_app_size_; offset += AppDescriptor::MagicSize)
        {
            AppDescriptor desc{};
            if (sizeof(desc) == backend_.read(offset, reinterpret_cast<std::byte*>(&desc), sizeof(desc)))
            {
                if (desc.isValid(max_app_size_) &&
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
    class AppDescriptor final
    {
    public:
        static constexpr std::size_t MagicSize = 8U;
        static constexpr std::size_t CRCOffset = MagicSize;

        [[nodiscard]] auto isValid(const std::uint32_t max_app_size) const -> bool
        {
            return (magic == ReferenceMagic) && (app_info.image_size > 0) && (app_info.image_size <= max_app_size) &&
                   ((app_info.image_size % MagicSize) == 0);
        }

        [[nodiscard]] auto getAppInfo() const -> const AppInfo& { return app_info; }

    private:
        /// The magic is also used for byte order detection.
        /// The value of the magic was obtained from a random number generator, it does not mean anything.
        static constexpr std::uint64_t ReferenceMagic = 0x5E44'1514'6FC0'C4C7ULL;

        std::uint64_t magic;
        AppInfo       app_info;
    };
    static_assert(std::is_trivial_v<AppDescriptor>, "Check your compiler");
    static_assert((AppInfo::Size + AppDescriptor::MagicSize) == sizeof(AppDescriptor), "Check your compiler");

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

    const std::uint32_t max_app_size_;
    const IROMBackend&  backend_;
};

/// These DSDL-derived definitions substitute for the lack of code generation.
namespace dsdl
{
static constexpr std::size_t NameCapacity = 50;

struct Heartbeat
{
    enum class Health : std::uint8_t
    {
        Nominal  = 0,
        Advisory = 1,
        Warning  = 3,
    };
};

struct ExecuteCommand
{
    static constexpr std::size_t RequestSizeMin = 3;

    enum class Command : std::uint16_t
    {
        Restart             = 65535,
        BeginSoftwareUpdate = 65533,
        EmergencyStop       = 65531,
    };

    enum class Status : std::uint8_t
    {
        Success       = 0,
        BadCommand    = 3,
        InternalError = 6,
    };
};

struct Diagnostic
{
    enum class Severity : std::uint8_t
    {
        Notice   = 3,
        Critical = 6,
    };

    static constexpr std::size_t RecordSize = 121;
};

struct File
{
    static constexpr std::size_t PathCapacity             = 112;
    static constexpr std::size_t ReadRequestCapacity      = 118;
    static constexpr std::size_t ReadResponseSizeMin      = 4;
    static constexpr std::size_t ReadResponseDataCapacity = 256;

    /// If the server returned an error, data will be nullptr and the data length will be zero.
    struct ReadResponse
    {
        std::uint16_t    data_length = 0;
        const std::byte* data        = nullptr;

        [[nodiscard]] auto isSuccessful() const { return data != nullptr; }
    };
};

}  // namespace dsdl

/// A higher-level variant of IReactor that operates on application-level representations instead of raw transfers.
class IController
{
public:
    /// Remote node has commanded us to reboot (back into the bootloader). The command shall always be accepted.
    virtual void reboot() = 0;

    /// Begin software update.
    /// The remote node and path are stored in the presenter so that the controller does not need to manage that.
    virtual auto beginUpdate() -> bool = 0;

    /// Response from the file server received or timed out. In case of timeout, the argument is an empty option.
    virtual void handleFileReadResult(const std::optional<dsdl::File::ReadResponse> response) = 0;

    [[nodiscard]] virtual auto getAppInfo() const -> std::optional<AppInfo> = 0;

    virtual ~IController()           = default;
    IController()                    = default;
    IController(const IController&)  = delete;
    IController(const IController&&) = delete;
    auto operator=(const IController&) -> IController& = delete;
    auto operator=(const IController &&) -> IController& = delete;
};

/// Unifies multiple INode and performs DSDL serialization. Manages the network at the presentation layer.
template <std::uint8_t NumNodes>
class Presenter final : private IReactor
{
public:
    Presenter(const SystemInfo& system_info, const std::array<INode*, NumNodes>& nodes, IController& controller) :
        system_info_(system_info), nodes_{nodes}, controller_(controller)
    {}

    void poll(const std::chrono::microseconds uptime)
    {
        last_poll_at_ = uptime;

        current_node_index_ = 0;
        for (INode* node : nodes_)
        {
            node->poll(*this, uptime);
            ++current_node_index_;
        }

        if (uptime >= next_heartbeat_deadline_)
        {
            next_heartbeat_deadline_ += HeartbeatPeriod;
            publishHeartbeat(uptime);
        }

        if (file_loc_spec_)
        {
            FileLocationSpecifier& fls = *file_loc_spec_;
            if (fls.response_deadline_ && (uptime > *fls.response_deadline_))
            {
                fls.response_deadline_.reset();
                controller_.handleFileReadResult({});
            }
        }
    }

    void setNodeHealth(const dsdl::Heartbeat::Health value) { node_health_ = value; }

    /// The timeout will be managed by the presenter automatically.
    [[nodiscard]] auto requestFileRead(const std::uint64_t offset) -> bool
    {
        if (file_loc_spec_)
        {
            static constexpr auto  length_minus_path = 6U;
            FileLocationSpecifier& fls               = *file_loc_spec_;
            fls.response_deadline_                   = last_poll_at_ + ServiceResponseTimeout;
            std::array<std::uint8_t, dsdl::File::ReadRequestCapacity> buf{{
                static_cast<std::uint8_t>(offset >> ByteShiftFirst),
                static_cast<std::uint8_t>(offset >> ByteShiftSecond),
                static_cast<std::uint8_t>(offset >> ByteShiftThird),
                static_cast<std::uint8_t>(offset >> ByteShiftFourth),
                static_cast<std::uint8_t>(offset >> ByteShiftFifth),
            }};
            buf.at(length_minus_path - 1U) = fls.path_length;
            (void) std::memmove(&buf.at(length_minus_path), fls.path.data(), fls.path_length);
            const bool out = nodes_.at(fls.local_node_index)
                                 ->requestFileRead(fls.server_node_id,
                                                   fls.read_transfer_id,
                                                   fls.path_length + length_minus_path,
                                                   reinterpret_cast<const std::byte*>(buf.data()));
            fls.read_transfer_id++;
            return out;
        }
        return false;
    }

    void publishLogRecord(const dsdl::Diagnostic::Severity severity, const char* const text)
    {
        std::array<std::uint8_t, dsdl::Diagnostic::RecordSize> buf{};
        buf[7]                   = static_cast<std::uint8_t>(severity);
        std::uint8_t text_length = 0;
        const char*  ch          = text;
        for (auto it = std::begin(buf) + 9; (it != std::end(buf)) && (*ch != '\0'); ++it)
        {
            *it = static_cast<std::uint8_t>(*ch);
            ++ch;
            ++text_length;
        }
        buf[8] = text_length;
        for (INode* node : nodes_)
        {
            node->publishLogRecord(tid_log_record_, text_length + 9U, reinterpret_cast<const std::byte*>(buf.data()));
        }
        ++tid_log_record_;
    }

private:
    void processExecuteCommandRequest(const NodeID           client_node_id,
                                      const std::size_t      request_length,
                                      const std::byte* const request,
                                      std::byte* const       out_response) override
    {
        auto result = dsdl::ExecuteCommand::Status::InternalError;
        if (request_length >= dsdl::ExecuteCommand::RequestSizeMin)
        {
            auto ptr     = request;
            auto command = static_cast<std::uint16_t>(*ptr++);
            command      = static_cast<std::uint16_t>(
                command | static_cast<std::uint16_t>(static_cast<std::uint16_t>(*ptr++) << ByteShiftSecond));
            if ((command == static_cast<std::uint16_t>(dsdl::ExecuteCommand::Command::EmergencyStop)) ||
                (command == static_cast<std::uint16_t>(dsdl::ExecuteCommand::Command::Restart)))
            {
                controller_.reboot();
                result = dsdl::ExecuteCommand::Status::Success;
            }
            else if (command == static_cast<std::uint16_t>(dsdl::ExecuteCommand::Command::BeginSoftwareUpdate))
            {
                FileLocationSpecifier fls{};
                fls.local_node_index = current_node_index_;
                fls.server_node_id   = client_node_id;
                fls.path_length =
                    static_cast<std::uint8_t>(std::min(static_cast<std::size_t>(*ptr++), std::size(fls.path)));
                (void) std::memmove(fls.path.data(), ptr, fls.path_length);
                file_loc_spec_ = fls;
                if (controller_.beginUpdate())
                {
                    result = dsdl::ExecuteCommand::Status::Success;
                }
            }
            else
            {
                result = dsdl::ExecuteCommand::Status::BadCommand;
            }
        }
        *out_response = static_cast<std::byte>(result);
    }

    [[nodiscard]] auto processNodeInfoRequest(std::byte* const out_response) const -> std::size_t override
    {
        const auto app_info = controller_.getAppInfo();
        const auto base_ptr = reinterpret_cast<std::uint8_t*>(out_response);
        auto       ptr      = base_ptr;
        *ptr++              = UAVCANSpecificationVersion.at(0);
        *ptr++              = UAVCANSpecificationVersion.at(1);
        *ptr++              = system_info_.hardware_version.at(0);
        *ptr++              = system_info_.hardware_version.at(1);
        if (app_info)
        {
            *ptr++                   = app_info->version.at(0);
            *ptr++                   = app_info->version.at(1);
            std::uint64_t vcs_commit = app_info->vcs_commit;
            for (auto i = 0U; i < sizeof(vcs_commit); i++)
            {
                *ptr++ = static_cast<std::uint8_t>(vcs_commit);
                vcs_commit >>= ByteShiftSecond;
            }
        }
        else
        {
            *ptr++ = 0;
            *ptr++ = 0;
            for (auto i = 0U; i < sizeof(std::uint64_t); i++)
            {
                *ptr++ = 0;
            }
        }
        for (auto uid : system_info_.unique_id)
        {
            *ptr++ = uid;
        }
        auto&       name_length = *ptr++;
        const char* ch          = system_info_.node_name;
        for (auto i = 0U; (i < dsdl::NameCapacity) && (*ch != '\0'); i++)
        {
            name_length++;
            *ptr++ = static_cast<std::uint8_t>(*ch++);
        }
        if (app_info)
        {
            *ptr++            = 1;
            std::uint64_t crc = app_info->image_crc;
            for (auto i = 0U; i < sizeof(crc); i++)
            {
                *ptr++ = static_cast<std::uint8_t>(crc);
                crc >>= ByteShiftSecond;
            }
        }
        else
        {
            *ptr++ = 0;
        }
        *ptr++ = system_info_.certificate_of_authenticity_length;
        for (auto i = 0U; i < system_info_.certificate_of_authenticity_length; i++)
        {
            *ptr++ = static_cast<std::uint8_t>(system_info_.certificate_of_authenticity[i]);
        }
        return static_cast<std::size_t>(ptr - base_ptr);
    }

    void processFileReadResponse(const std::size_t response_length, const std::byte* const response) override
    {
        if (file_loc_spec_ && (response_length >= dsdl::File::ReadResponseSizeMin))
        {
            FileLocationSpecifier& fls = *file_loc_spec_;
            if (fls.response_deadline_ && (fls.local_node_index == current_node_index_))
            {
                fls.response_deadline_.reset();
                static const std::array<std::byte, 2> zero_error{};
                dsdl::File::ReadResponse              obj{};
                if (std::equal(std::begin(zero_error), std::end(zero_error), response))  // Error = OK
                {
                    obj.data_length =
                        static_cast<std::uint16_t>(static_cast<std::uint16_t>(response[2]) << ByteShiftFirst) |
                        static_cast<std::uint16_t>(static_cast<std::uint16_t>(response[3]) << ByteShiftSecond);
                    obj.data = &response[4];
                }
                if (obj.data_length <= dsdl::File::ReadResponseDataCapacity)
                {
                    controller_.handleFileReadResult(obj);
                }
            }
        }
    }

    void publishHeartbeat(const std::chrono::microseconds uptime)
    {
        const auto ut = static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(uptime).count());
        std::array<std::uint8_t, INode::HeartbeatLength> buf{{
            static_cast<std::uint8_t>(ut >> ByteShiftFirst),
            static_cast<std::uint8_t>(ut >> ByteShiftSecond),
            static_cast<std::uint8_t>(ut >> ByteShiftThird),
            static_cast<std::uint8_t>(ut >> ByteShiftFourth),
            static_cast<std::uint8_t>(static_cast<std::uint8_t>(NodeModeSoftwareUpdate << 2U) |
                                      static_cast<std::uint8_t>(node_health_)),
        }};
        for (INode* node : nodes_)
        {
            node->publishHeartbeat(tid_heartbeat_, reinterpret_cast<std::byte*>(buf.data()));
        }
        ++tid_heartbeat_;
    }

    struct FileLocationSpecifier
    {
        std::uint8_t                                       local_node_index{};
        NodeID                                             server_node_id{};
        TransferID                                         read_transfer_id{};
        std::uint8_t                                       path_length{};
        std::array<std::uint8_t, dsdl::File::PathCapacity> path{};
        std::optional<std::chrono::microseconds>           response_deadline_{};
    };

    const SystemInfo                   system_info_;
    const std::array<INode*, NumNodes> nodes_;
    IController&                       controller_;

    std::chrono::microseconds last_poll_at_{};

    std::uint8_t current_node_index_ = 0;

    std::optional<FileLocationSpecifier> file_loc_spec_;

    TransferID tid_heartbeat_  = 0;
    TransferID tid_log_record_ = 0;

    static constexpr std::uint8_t         NodeModeSoftwareUpdate = 3;
    static constexpr std::chrono::seconds HeartbeatPeriod{1};
    std::chrono::microseconds             next_heartbeat_deadline_{HeartbeatPeriod};
    dsdl::Heartbeat::Health               node_health_ = dsdl::Heartbeat::Health::Nominal;

    static constexpr std::uint8_t ByteShiftFirst  = 0;
    static constexpr std::uint8_t ByteShiftSecond = 8;
    static constexpr std::uint8_t ByteShiftThird  = 16;
    static constexpr std::uint8_t ByteShiftFourth = 24;
    static constexpr std::uint8_t ByteShiftFifth  = 32;
};

}  // namespace detail

// --------------------------------------------------------------------------------------------------------------------

/// The bootloader core.
///
/// The bootloader may run multiple nodes on different transports concurrently to support multi-transport functionality.
/// For example, a device may provide the firmware update capability via CAN and a serial port.
template <std::uint8_t NumNodes>
class Bootloader : private detail::IController
{
public:
    /// The max application image size parameter is very important for performance reasons;
    /// without it, the bootloader may encounter an unrelated data structure in the ROM that looks like a
    /// valid app descriptor (by virtue of having the same magic, which is only 64 bit long),
    /// and it may spend a considerable amount of time trying to check the CRC that is certainly invalid.
    /// Having an upper size limit for the application image allows the bootloader to weed out too large
    /// values early, greatly improving the worst case boot time.
    Bootloader(IROMBackend&                        rom_backend,
               const SystemInfo&                   system_info,
               const std::array<INode*, NumNodes>& nodes,
               const std::size_t                   max_app_size,
               const std::chrono::seconds          boot_delay = std::chrono::seconds(0)) :
        max_app_size_(max_app_size),
        boot_delay_(boot_delay),
        backend_(rom_backend),
        presentation_(system_info, nodes, *this)
    {}

    [[nodiscard]] auto poll(const std::chrono::microseconds uptime) -> State
    {
        if (!initialized_)
        {
            detail::AppLocator locator(backend_, max_app_size_);
            app_info_      = locator.identifyApplication();
            boot_deadline_ = uptime + boot_delay_;
            state_         = app_info_ ? State::BootDelay : State::NoAppToBoot;
            initialized_   = true;
        }

        presentation_.poll(uptime);

        if ((State::BootDelay == state_) && (uptime >= boot_deadline_))
        {
            state_ = State::ReadyToBoot;
        }

        return state_;
    }

    [[nodiscard]] auto getAppInfo() const override { return app_info_; }

private:
    const std::size_t          max_app_size_;
    const std::chrono::seconds boot_delay_;

    IROMBackend&                backend_;
    detail::Presenter<NumNodes> presentation_;

    bool                                     initialized_ = false;
    State                                    state_       = State::NoAppToBoot;
    std::optional<std::chrono::microseconds> boot_deadline_;
    std::optional<AppInfo>                   app_info_;
};

// --------------------------------------------------------------------------------------------------------------------

/// This helper class allows the bootloader and the application to exchange arbitrary data in a robust way.
/// The data is stored in the specified memory location (usually it is a small dedicated area a few hundred bytes
/// large at the very end of the slowest RAM segment) together with a strong CRC64 hash to ensure its validity.
/// When one component (either the bootloader or the application) needs to pass data to another (e.g., when commencing
/// the update process, the application needs to reboot into the bootloader and pass specific parameters to it),
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
