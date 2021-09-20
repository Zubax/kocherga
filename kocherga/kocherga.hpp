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

#define KOCHERGA_VERSION_MAJOR 0  // NOLINT
#define KOCHERGA_VERSION_MINOR 2  // NOLINT

namespace kocherga
{
/// Semantic version number pair: major then minor.
using SemanticVersion = std::array<std::uint8_t, 2>;

using TransferID = std::uint64_t;
using NodeID     = std::uint16_t;
using PortID     = std::uint16_t;

/// UAVCAN subjects used by Kocherga.
enum class SubjectID : PortID
{
    NodeHeartbeat              = 7509,
    PnPNodeIDAllocationData_v2 = 8165,
    PnPNodeIDAllocationData_v1 = 8166,
    DiagnosticRecord           = 8184,
};

/// UAVCAN services used by Kocherga.
enum class ServiceID : PortID
{
    FileRead           = 408,
    NodeGetInfo        = 430,
    NodeExecuteCommand = 435,
};

/// Version of the UAVCAN specification implemented by this library, major and minor.
static constexpr SemanticVersion UAVCANSpecificationVersion{{1, 0}};

/// The service response timeout used by the bootloader.
/// This value applies when the bootloader invokes uavcan.file.Read during the update.
static constexpr std::chrono::microseconds ServiceResponseTimeout{5'000'000};

/// The largest transfer size used by the bootloader. Use this to size the buffers in transport implementations.
static constexpr std::size_t MaxSerializedRepresentationSize = 600;

// --------------------------------------------------------------------------------------------------------------------

/// The structure is mapped to the ROM.
struct AppInfo
{
    static constexpr std::uint8_t Size = 48;

    template <std::size_t Size>
    using Skip = std::array<std::byte, Size>;

    std::uint64_t             image_crc;        ///< CRC-64-WE padded to 8 bytes computed with this field =0.
    std::uint32_t             image_size;       ///< Size of the application image in bytes.
    [[maybe_unused]] Skip<4>  _reserved_a;      ///< Used to contain 32-bit vcs_revision_id.
    SemanticVersion           version;          ///< Semantic version numbers, major then minor.
    std::uint8_t              flags;            ///< Flags; see the constants. Unused flags shall not be set.
    [[maybe_unused]] Skip<1>  _reserved_b;      ///< Write zero, ignore when reading.
    std::uint32_t             timestamp_utc;    ///< UTC UNIX timestamp when the application was built.
    std::uint64_t             vcs_revision_id;  ///< Version control system revision ID (e.g., git commit hash).
    [[maybe_unused]] Skip<16> _reserved_c;      ///< Write zero, ignore when reading.

    struct Flags
    {
        static constexpr std::uint8_t ReleaseBuild = 1U;
        static constexpr std::uint8_t DirtyBuild   = 2U;
    };

    [[nodiscard]] auto isReleaseBuild() const { return (flags & Flags::ReleaseBuild) != 0; }
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

    using UniqueID = std::array<std::uint8_t, 16>;
    UniqueID unique_id{};

    const char* node_name = "";

    /// CoA normally points into a specific region of ROM, but this is not required. Set 0/nullptr if not available.
    std::uint8_t        certificate_of_authenticity_len = 0;
    const std::uint8_t* certificate_of_authenticity     = nullptr;
};

// --------------------------------------------------------------------------------------------------------------------

/// A standard IoC delegate for handling protocol events in the node.
/// The user code will INVOKE this interface, not implement it.
/// A reference to the reactor is supplied to INode implementations to let them delegate transfer processing back
/// to the bootloader core.
/// The methods accept raw serialized representation and return one as well.
/// Serialization/deserialization is done by the bootloader core because DSDL is transport-agnostic.
/// All methods are non-blocking and return immediately.
/// Implementations utilizing just a single transport should not incur any polymorphism-induced overhead because decent
/// C++ compilers are quite good at devirtualization.
class IReactor
{
public:
    /// Returns the size of the response payload. Returns an empty option if no response should be sent.
    [[nodiscard]] virtual auto processRequest(const PortID              service_id,
                                              const NodeID              client_node_id,
                                              const std::size_t         request_length,
                                              const std::uint8_t* const request,
                                              std::uint8_t* const       out_response) -> std::optional<std::size_t> = 0;

    /// The service-ID is not communicated back because there may be at most one request pending at a time.
    /// Hence, the bootloader core knows what response it is by checking which request was sent last.
    virtual void processResponse(const std::size_t response_length, const std::uint8_t* const response) = 0;

    virtual ~IReactor()       = default;
    IReactor()                = default;
    IReactor(const IReactor&) = delete;
    IReactor(IReactor&&)      = delete;
    auto operator=(const IReactor&) -> IReactor& = delete;
    auto operator=(IReactor&&) -> IReactor& = delete;
};

// --------------------------------------------------------------------------------------------------------------------

/// The transport-specific node abstraction interface. Kocherga runs a separate node per transport interface.
/// If redundant transports are desired, they should be implemented in a custom implementation of INode.
/// If the node implementation is unable to perform the requested action (for example, because a node-ID allocation
/// is still in progress), it shall ignore the commanded action or return an error if such possibility is provided.
/// The priority of outgoing transfers should be the lowest, excepting the service responses -- those should use the
/// same priority level as the corresponding request transfer.
class INode
{
public:
    /// The bootloader invokes this method every tick to let the node run background activities such as
    /// processing incoming transfers. If a reaction is required (such as responding to a service request),
    /// it is delegated to the bootloader core via the IoC IReactor.
    virtual void poll(IReactor& reactor, const std::chrono::microseconds uptime) = 0;

    /// Send a request. The response will be delivered later asynchronously via IReactor.
    /// There may be AT MOST one pending request at a time.
    /// The return value is True on success and False if the request could not be sent (aborts the update process).
    [[nodiscard]] virtual auto sendRequest(const ServiceID           service_id,
                                           const NodeID              server_node_id,
                                           const TransferID          transfer_id,
                                           const std::size_t         payload_length,
                                           const std::uint8_t* const payload) -> bool = 0;

    /// Cancel the request that was previously set pending by sendRequest(); no longer expect the response.
    /// This method is invoked when the request has timed out.
    virtual void cancelRequest() = 0;

    /// Publish a message.
    /// The return value is True on success and False if the message could not be sent (does not abort the update).
    [[nodiscard]] virtual auto publishMessage(const SubjectID           subject_id,
                                              const TransferID          transfer_id,
                                              const std::size_t         payload_length,
                                              const std::uint8_t* const payload) -> bool = 0;

    virtual ~INode()    = default;
    INode()             = default;
    INode(const INode&) = delete;
    INode(INode&&)      = delete;
    auto operator=(const INode&) -> INode& = delete;
    auto operator=(INode&&) -> INode& = delete;
};

// --------------------------------------------------------------------------------------------------------------------

/// This interface abstracts the target-specific ROM routines.
/// App update scenario:
///  1. beginWrite()
///  2. write() repeated until finished.
///  3. endWrite() (the number of preceding writes may be zero)
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
    /// This operation cannot fail.
    virtual void beginWrite()
    {
        // No effect by default.
    }

    /// This hook allows the ROM driver to disable write operations or to perform other hardware-specific steps.
    /// This operation cannot fail.
    virtual void endWrite()
    {
        // No effect by default.
    }

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
    auto operator=(IROMBackend&&) -> IROMBackend& = delete;
};

// --------------------------------------------------------------------------------------------------------------------

/// This is used to verify integrity of the application and other data.
/// Note that the firmware CRC verification is a computationally expensive process that needs to be completed
/// in a limited time interval, which should be minimized. This class has been carefully manually optimized to
/// achieve the optimal balance between speed and ROM footprint.
/// The function is CRC-64/WE, see http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64.
class CRC64
{
public:
    static constexpr std::size_t Size = 8U;

    void update(const std::uint8_t* const data, const std::size_t len)
    {
        const auto* bytes = data;
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
    [[nodiscard]] auto getBytes() const -> std::array<std::uint8_t, Size>
    {
        auto                           x = get();
        std::array<std::uint8_t, Size> out{};
        const auto                     rend = std::rend(out);
        for (auto it = std::rbegin(out); it != rend; ++it)
        {
            *it = static_cast<std::uint8_t>(x);
            x >>= 8U;
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

    static constexpr auto InputShift = 56U;

    std::uint64_t crc_ = Xor;
};

// --------------------------------------------------------------------------------------------------------------------

/// Internal use only.
namespace detail
{
static constexpr auto BitsPerByte = 8U;

static constexpr std::chrono::microseconds DefaultTransferIDTimeout{2'000'000};  ///< Default taken from Specification.

/// Detects the application in the ROM, verifies its integrity, and retrieves the information about it.
class AppLocator final
{
public:
    AppLocator(const IROMBackend& backend, const std::size_t max_app_size) :
        max_app_size_(max_app_size), backend_(backend)
    {}

    /// Returns the AppInfo if the app is found and its integrity is intact. Otherwise, returns an empty option.
    /// If the allow_legacy parameter is set, legacy app descriptors will be accepted, too.
    [[nodiscard]] auto identifyApplication(const bool allow_legacy = false) const -> std::optional<AppInfo>
    {
        for (std::size_t offset = 0; offset < max_app_size_; offset += AppDescriptor::MagicSize)
        {
            AppDescriptor desc{};
            if (sizeof(desc) == backend_.read(offset, reinterpret_cast<std::byte*>(&desc), sizeof(desc)))
            {
                const bool match = desc.isValid(max_app_size_) || (allow_legacy && desc.isValidLegacy(max_app_size_));
                if (match && validateImageCRC(offset + AppDescriptor::CRCOffset,
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
        static constexpr std::size_t MagicSize     = 8U;
        static constexpr std::size_t SignatureSize = 8U;
        static constexpr std::size_t CRCOffset     = MagicSize + SignatureSize;

        [[nodiscard]] auto isValid(const std::size_t max_app_size) const -> bool
        {
            return (magic == ReferenceMagic) &&
                   std::equal(signature.begin(), signature.end(), ReferenceSignature.begin()) &&
                   (app_info.image_size > 0) && (app_info.image_size <= max_app_size) &&
                   ((app_info.image_size % MagicSize) == 0);
        }

        [[nodiscard]] auto isValidLegacy(const std::size_t max_app_size) const -> bool
        {
            static constexpr auto SizeAlignmentRequirement = 4U;  ///< Relaxed requirement to enhance compatibility.
            return std::equal(signature.begin(), signature.end(), ReferenceSignature.begin()) &&
                   (app_info.image_size > 0) && (app_info.image_size <= max_app_size) &&
                   ((app_info.image_size % SizeAlignmentRequirement) == 0);
        }

        [[nodiscard]] auto getAppInfo() const -> const AppInfo& { return app_info; }

    private:
        /// The magic is also used for byte order detection.
        /// The value of the magic was obtained from a random number generator, it does not mean anything.
        static constexpr std::uint64_t                           ReferenceMagic = 0x5E44'1514'6FC0'C4C7ULL;
        static constexpr std::array<std::uint8_t, SignatureSize> ReferenceSignature{
            {'A', 'P', 'D', 'e', 's', 'c', '0', '0'}};

        std::uint64_t                           magic;
        std::array<std::uint8_t, SignatureSize> signature;
        AppInfo                                 app_info;
    };
    static_assert(std::is_trivial_v<AppDescriptor>, "Check your compiler");
    static_assert((AppInfo::Size + AppDescriptor::MagicSize + AppDescriptor::SignatureSize) == sizeof(AppDescriptor),
                  "Check your compiler");
    static_assert(64 == sizeof(AppDescriptor), "Check your compiler");

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
            const auto res = backend_.read(offset,
                                           reinterpret_cast<std::byte*>(buffer.data()),
                                           std::min(std::size(buffer), crc_storage_offset - offset));
            if (res > 0)
            {
                offset += res;
                crc.update(buffer.data(), res);
            }
            else
            {
                return false;
            }
        }
        // Fill CRC with zero.
        static const std::array<std::uint8_t, CRC64::Size> dummy{};
        offset += CRC64::Size;
        crc.update(dummy.data(), CRC64::Size);
        // Read the rest of the image in large chunks.
        while (offset < image_size)
        {
            const auto res = backend_.read(offset,
                                           reinterpret_cast<std::byte*>(buffer.data()),
                                           std::min(std::size(buffer), image_size - offset));
            if (res > 0)
            {
                offset += res;
                crc.update(buffer.data(), res);
            }
            else
            {
                return false;
            }
        }
        return crc.get() == image_crc;
    }

    static constexpr std::size_t ROMBufferSize = 256;

    const std::size_t  max_app_size_;
    const IROMBackend& backend_;
};

/// These DSDL-derived definitions substitute for the lack of code generation.
namespace dsdl
{
static constexpr std::size_t NameCapacity = 255;

struct Heartbeat
{
    static constexpr std::size_t               Size = 7;
    static constexpr std::chrono::microseconds Period{1'000'000};

    static constexpr std::uint8_t ModeSoftwareUpdate = 3;

    enum class Health : std::uint8_t
    {
        Nominal  = 0,
        Advisory = 1,
        Warning  = 3,
    };
};

struct ExecuteCommand
{
    static constexpr std::size_t ResponseSize   = 7;
    static constexpr std::size_t RequestSizeMin = 3;

    enum class Command : std::uint16_t
    {
        Restart             = 65'535,
        BeginSoftwareUpdate = 65'533,
        EmergencyStop       = 65'531,
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
        Warning  = 4,
        Critical = 6,
    };

    static constexpr std::size_t RecordSize = 257;
};

struct File
{
    static constexpr std::size_t PathCapacity             = 255;
    static constexpr std::size_t ReadRequestCapacity      = PathCapacity + 6;
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

struct PnPNodeIDAllocation
{
    static constexpr std::chrono::microseconds MaxRequestInterval{2'000'000};

    static constexpr std::size_t MessageSize_v2 = 18;
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
    virtual void beginUpdate() = 0;

    /// Response from the file server received or timed out. In case of timeout, the argument is an empty option.
    virtual void handleFileReadResult(const std::optional<dsdl::File::ReadResponse> response) = 0;

    [[nodiscard]] virtual auto getAppInfo() const -> std::optional<AppInfo> = 0;

    virtual ~IController()           = default;
    IController()                    = default;
    IController(const IController&)  = delete;
    IController(const IController&&) = delete;
    auto operator=(const IController&) -> IController& = delete;
    auto operator=(const IController&&) -> IController& = delete;
};

/// Unifies multiple INode and performs DSDL serialization. Manages the network at the presentation layer.
class Presenter final : public IReactor
{
public:
    Presenter(const SystemInfo& system_info, IController& controller) :
        system_info_(system_info), controller_(controller)
    {}

    [[nodiscard]] auto addNode(INode* const node) -> bool
    {
        for (const auto* n : nodes_)
        {
            if (n == node)
            {
                return false;  // This node is already registered.
            }
        }
        if (num_nodes_ < nodes_.size())
        {
            nodes_.at(num_nodes_++) = node;
            return true;
        }
        return false;
    }

    [[nodiscard]] auto getNumberOfNodes() const -> std::uint8_t { return num_nodes_; }

    [[nodiscard]] auto trigger(const INode* const        node,
                               const NodeID              file_server_node_id,
                               const std::size_t         app_image_file_path_length,
                               const std::uint8_t* const app_image_file_path) -> bool
    {
        for (std::uint8_t i = 0U; i < num_nodes_; i++)
        {
            if (nodes_.at(i) == node)
            {
                beginUpdate(i, file_server_node_id, app_image_file_path_length, app_image_file_path);
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] auto trigger(const std::uint8_t        node_index,
                               const NodeID              file_server_node_id,
                               const std::size_t         app_image_file_path_length,
                               const std::uint8_t* const app_image_file_path) -> bool
    {
        if (node_index < num_nodes_)
        {
            beginUpdate(node_index, file_server_node_id, app_image_file_path_length, app_image_file_path);
            return true;
        }
        return false;
    }

    void poll(const std::chrono::microseconds uptime)
    {
        last_poll_at_ = uptime;

        current_node_index_ = 0;
        for (INode* const node : nodes_)
        {
            if (node != nullptr)
            {
                node->poll(*this, uptime);
            }
            ++current_node_index_;
        }

        if (file_loc_spec_)
        {
            FileLocationSpecifier& fls = *file_loc_spec_;
            if (fls.response_deadline && (uptime > *fls.response_deadline))
            {
                INode* const nd = nodes_.at(fls.local_node_index);
                nd->cancelRequest();
                fls.response_deadline.reset();
                controller_.handleFileReadResult({});
            }
        }

        if (uptime >= next_heartbeat_deadline_)  // Postpone heartbeat to reflect the latest state changes.
        {
            next_heartbeat_deadline_ += dsdl::Heartbeat::Period;
            publishHeartbeat(uptime);
        }
    }

    void setNodeHealth(const dsdl::Heartbeat::Health value) { node_health_ = value; }
    void setNodeVSSC(const std::uint8_t value) { node_vssc_ = value; }

    /// The timeout will be managed by the presenter automatically.
    [[nodiscard]] auto requestFileRead(const std::uint64_t offset) -> bool
    {
        if (file_loc_spec_)
        {
            std::array<std::uint8_t, dsdl::File::ReadRequestCapacity> buf{};

            auto of = offset;
            buf[0]  = static_cast<std::uint8_t>(of);
            of >>= BitsPerByte;
            buf[1] = static_cast<std::uint8_t>(of);
            of >>= BitsPerByte;
            buf[2] = static_cast<std::uint8_t>(of);
            of >>= BitsPerByte;
            buf[3] = static_cast<std::uint8_t>(of);
            of >>= BitsPerByte;
            buf[4] = static_cast<std::uint8_t>(of);

            static constexpr auto  length_minus_path = 6U;
            FileLocationSpecifier& fls               = *file_loc_spec_;
            buf.at(length_minus_path - 1U)           = static_cast<std::uint8_t>(fls.path_length);
            (void) std::memmove(&buf.at(length_minus_path), fls.path.data(), fls.path_length);
            INode* const node = nodes_.at(fls.local_node_index);

            read_transfer_id_++;
            const bool out = node->sendRequest(ServiceID::FileRead,
                                               fls.server_node_id,
                                               read_transfer_id_,
                                               fls.path_length + length_minus_path,
                                               buf.data());
            if (out)
            {
                fls.response_deadline = last_poll_at_ + ServiceResponseTimeout;
            }
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
        auto* const  buf_end     = std::end(buf);
        for (auto it = std::begin(buf) + 9; it != buf_end; ++it)  // NOLINT
        {
            if ('\0' == *ch)
            {
                break;
            }
            *it = static_cast<std::uint8_t>(*ch++);
            ++text_length;
        }
        buf[8] = text_length;
        for (INode* const node : nodes_)
        {
            if (node != nullptr)
            {
                // Ignore transient errors.
                (void) node->publishMessage(SubjectID::DiagnosticRecord, tid_log_record_, text_length + 9U, buf.data());
            }
        }
        ++tid_log_record_;
    }

private:
    [[nodiscard]] auto processRequest(const PortID              service_id,
                                      const NodeID              client_node_id,
                                      const std::size_t         request_length,
                                      const std::uint8_t* const request,
                                      std::uint8_t* const       out_response) -> std::optional<std::size_t> override
    {
        std::optional<std::size_t> out;
        switch (service_id)
        {
        case static_cast<PortID>(ServiceID::NodeExecuteCommand):
        {
            out = processExecuteCommandRequest(client_node_id, request_length, request, out_response);
            break;
        }
        case static_cast<PortID>(ServiceID::NodeGetInfo):
        {
            out = processNodeInfoRequest(out_response);
            break;
        }
        case static_cast<PortID>(ServiceID::FileRead):
        default:
        {
            break;  // Unknown services ignored.
        }
        }
        return out;
    }

    auto processExecuteCommandRequest(const NodeID              client_node_id,
                                      const std::size_t         request_length,
                                      const std::uint8_t* const request,
                                      std::uint8_t* const       out_response) -> std::size_t
    {
        auto result = dsdl::ExecuteCommand::Status::InternalError;
        if (request_length >= dsdl::ExecuteCommand::RequestSizeMin)
        {
            const auto* ptr     = request;
            auto        command = static_cast<std::uint16_t>(*ptr);
            ++ptr;
            command = static_cast<std::uint16_t>(
                command | static_cast<std::uint16_t>(static_cast<std::uint16_t>(*ptr) << BitsPerByte));
            ++ptr;
            if ((command == static_cast<std::uint16_t>(dsdl::ExecuteCommand::Command::EmergencyStop)) ||
                (command == static_cast<std::uint16_t>(dsdl::ExecuteCommand::Command::Restart)))
            {
                controller_.reboot();
                result = dsdl::ExecuteCommand::Status::Success;
            }
            else if (command == static_cast<std::uint16_t>(dsdl::ExecuteCommand::Command::BeginSoftwareUpdate))
            {
                const std::size_t path_length = *ptr++;
                beginUpdate(current_node_index_, client_node_id, path_length, ptr);
                result = dsdl::ExecuteCommand::Status::Success;
            }
            else
            {
                result = dsdl::ExecuteCommand::Status::BadCommand;
            }
        }
        (void) std::memset(out_response, 0, dsdl::ExecuteCommand::ResponseSize);
        *out_response = static_cast<std::uint8_t>(result);
        return dsdl::ExecuteCommand::ResponseSize;
    }

    [[nodiscard]] auto processNodeInfoRequest(std::uint8_t* const out_response) const -> std::size_t
    {
        const auto  app_info = controller_.getAppInfo();
        auto* const base_ptr = out_response;
        auto*       ptr      = base_ptr;
        *ptr++               = UAVCANSpecificationVersion.at(0);
        *ptr++               = UAVCANSpecificationVersion.at(1);
        *ptr++               = system_info_.hardware_version.at(0);
        *ptr++               = system_info_.hardware_version.at(1);
        if (app_info)
        {
            *ptr++                        = app_info->version.at(0);
            *ptr++                        = app_info->version.at(1);
            std::uint64_t vcs_revision_id = app_info->vcs_revision_id;
            for (auto i = 0U; i < sizeof(std::uint64_t); i++)
            {
                *ptr++ = static_cast<std::uint8_t>(vcs_revision_id);
                vcs_revision_id >>= BitsPerByte;
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
        for (auto i = 0U; i < dsdl::NameCapacity; i++)
        {
            if ('\0' == *ch)
            {
                break;
            }
            name_length++;
            *ptr++ = static_cast<std::uint8_t>(*ch++);
        }
        if (app_info)
        {
            *ptr++            = 1;
            std::uint64_t crc = app_info->image_crc;
            for (auto i = 0U; i < sizeof(std::uint64_t); i++)
            {
                *ptr++ = static_cast<std::uint8_t>(crc);
                crc >>= BitsPerByte;
            }
        }
        else
        {
            *ptr++ = 0;
        }
        *ptr++ = system_info_.certificate_of_authenticity_len;
        for (auto i = 0U; i < system_info_.certificate_of_authenticity_len; i++)
        {
            *ptr++ = static_cast<std::uint8_t>(system_info_.certificate_of_authenticity[i]);
        }
        return static_cast<std::size_t>(ptr - base_ptr);
    }

    void processResponse(const std::size_t response_length, const std::uint8_t* const response) override
    {
        if (file_loc_spec_ && (response_length >= dsdl::File::ReadResponseSizeMin))
        {
            FileLocationSpecifier& fls = *file_loc_spec_;
            if (fls.response_deadline && (fls.local_node_index == current_node_index_))
            {
                fls.response_deadline.reset();
                static const std::array<std::uint8_t, 2> zero_error{};
                std::optional<dsdl::File::ReadResponse>  argument;
                if (std::equal(std::begin(zero_error), std::end(zero_error), response))  // Error = OK
                {
                    argument = dsdl::File::ReadResponse{
                        static_cast<std::uint16_t>(
                            static_cast<std::uint16_t>(static_cast<std::uint16_t>(response[2])) |
                            static_cast<std::uint16_t>(static_cast<std::uint16_t>(response[3]) << BitsPerByte)),
                        reinterpret_cast<const std::byte*>(&response[4]),
                    };
                }
                if (argument && (argument->data_length > dsdl::File::ReadResponseDataCapacity))
                {
                    argument.reset();
                }
                controller_.handleFileReadResult(argument);
            }
        }
    }

    void publishHeartbeat(const std::chrono::microseconds uptime)
    {
        const auto ut = static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(uptime).count());
        std::array<std::uint8_t, dsdl::Heartbeat::Size> buf{{
            static_cast<std::uint8_t>(ut >> (BitsPerByte * 0U)),
            static_cast<std::uint8_t>(ut >> (BitsPerByte * 1U)),
            static_cast<std::uint8_t>(ut >> (BitsPerByte * 2U)),
            static_cast<std::uint8_t>(ut >> (BitsPerByte * 3U)),
            static_cast<std::uint8_t>(node_health_),
            static_cast<std::uint8_t>(dsdl::Heartbeat::ModeSoftwareUpdate),
            static_cast<std::uint8_t>(node_vssc_),
        }};
        for (INode* const node : nodes_)
        {
            if (node != nullptr)
            {
                // Ignore transient errors.
                (void) node->publishMessage(SubjectID::NodeHeartbeat, tid_heartbeat_, buf.size(), buf.data());
            }
        }
        ++tid_heartbeat_;
    }

    struct FileLocationSpecifier
    {
        std::uint8_t                                       local_node_index{};
        NodeID                                             server_node_id{};
        std::size_t                                        path_length{};
        std::array<std::uint8_t, dsdl::File::PathCapacity> path{};
        std::optional<std::chrono::microseconds>           response_deadline{};
    };

    void beginUpdate(const std::uint8_t        local_node_index,
                     const NodeID              file_server_node_id,
                     const std::size_t         app_image_file_path_length,
                     const std::uint8_t* const app_image_file_path)
    {
        FileLocationSpecifier fls{};
        fls.local_node_index = local_node_index;
        fls.server_node_id   = file_server_node_id;
        fls.path_length      = std::min(app_image_file_path_length, std::size(fls.path));
        (void) std::memmove(fls.path.data(), app_image_file_path, fls.path_length);

        if (file_loc_spec_ && file_loc_spec_->response_deadline)
        {
            nodes_.at(file_loc_spec_->local_node_index)->cancelRequest();
        }
        file_loc_spec_      = fls;
        current_node_index_ = fls.local_node_index;
        controller_.beginUpdate();
    }

    static constexpr std::uint8_t MaxNodes = 8;

    const SystemInfo             system_info_;
    std::array<INode*, MaxNodes> nodes_{};
    std::uint8_t                 num_nodes_ = 0;
    IController&                 controller_;

    std::chrono::microseconds last_poll_at_{};

    std::uint8_t current_node_index_ = 0;

    std::optional<FileLocationSpecifier> file_loc_spec_;
    TransferID                           read_transfer_id_ = 0;

    TransferID tid_heartbeat_  = 0;
    TransferID tid_log_record_ = 0;

    std::chrono::microseconds next_heartbeat_deadline_{dsdl::Heartbeat::Period};
    dsdl::Heartbeat::Health   node_health_ = dsdl::Heartbeat::Health::Nominal;
    std::uint8_t              node_vssc_   = 0;
};

}  // namespace detail

// --------------------------------------------------------------------------------------------------------------------

/// The following state transition diagram illustrates the operating principles of the bootloader.
///
///                                ######################
///               .----------------# Bootloader started #-------. Valid
///               | No valid       ######################       | application found.
///               v application found.                          v
///         +-------------+                               +-----------+ Boot delay expired. ################
///     .-->| NoAppToBoot |            .------------------| BootDelay |--------------------># Boot the app #
///     |   +-------------+           /                   +-----------+  (default delay 0)  ################
///     |              |             /                 Boot |       ^
///     |Update        |<-----------'              canceled.|       |
///     |failed,       |Update requested.                   |       |
///     |no valid      |                                    v       |
///     |image is now  |                        +--------------+    |
///     |available.    |<-----------------------| BootCanceled |    |
///     |              |                        +--------------+    |
///     |              v                                   ^        |
///     | +----------------------+ Update failed, but the  |        |Update successful,
///     '-| AppUpdateInProgress  |-------------------------/        |the received image
///       +----------------------+ existing valid image was not     |is valid.
///                |               altered and remains valid.       |
///                |                                                |
///                '------------------------------------------------'
///
/// The current state is communicated to the outside world via uavcan.node.Heartbeat. The mapping is as follows:
///
/// State                   Node mode           Node health     Vendor-specific status code
/// -----------------------------------------------------------------------------------------------
/// NoAppToBoot             SOFTWARE_UPDATE     WARNING         =0
/// BootDelay               SOFTWARE_UPDATE     NOMINAL         =0
/// BootCanceled            SOFTWARE_UPDATE     ADVISORY        =0
/// AppUpdateInProgress     SOFTWARE_UPDATE     NOMINAL         >0 (see below)
///
/// In the mode AppUpdateInProgress, the vendor-specific status code equals the number of uavcan.file.Read requests
/// sent since the update process was initiated; it is always greater than zero. This is used for progress reporting.
enum class State
{
    NoAppToBoot,
    BootDelay,
    BootCanceled,
    AppUpdateInProgress,
};

/// The bootloader instructs the outer logic what action needs to be taken when its execution has completed.
/// Once the final action is returned, the bootloader has terminated itself and need not be run anymore.
enum class Final
{
    BootApp,  ///< Jump to the application image. The bootloader has verified its correctness.
    Restart,  ///< Restart the bootloader itself.
};

/// The bootloader core.
///
/// The bootloader may run multiple nodes on different transports concurrently to support multi-transport functionality.
/// For example, a device may provide the firmware update capability via CAN and a serial port.
/// The nodes are registered using addNode() after the instance is constructed.
///
/// If the boot delay is zero and the valid application is found, the bootloader proceeds to start it immediately.
class Bootloader : public detail::IController
{
public:
    /// The max application image size parameter is very important for performance reasons.
    /// Without it, the bootloader may encounter an unrelated data structure in the ROM that looks like a
    /// valid app descriptor (by virtue of having the same magic, which is only 64 bit long),
    /// and it may spend a considerable amount of time trying to check the CRC that is certainly invalid.
    /// Having an upper size limit for the application image allows the bootloader to weed out too large
    /// values early, greatly improving the worst case boot time.
    ///
    /// SystemInfo is used for responding to uavcan.node.GetInfo requests.
    ///
    /// If the linger flag is set, the bootloader will not boot the application after the initial verification.
    /// If the application is valid, then the initial state will be BootCanceled instead of BootDelay.
    /// If the application is invalid, the flag will have no effect.
    /// It is designed to support the common use case where the application commands the bootloader to start and
    /// sit idle until instructed otherwise, or if the application itself commands the bootloader to begin the update.
    /// The flag affects only the initial verification and has no effect on all subsequent checks; for example,
    /// after the application is updated and validated, it will be booted after BootDelay regardless of this flag.
    ///
    /// If the allow_legacy_app_descriptors option is set, the bootloader will also accept legacy descriptors alongside
    /// the new format. This option should be set only if the bootloader is introduced to a product that was using
    /// the old app descriptor format in the past; refer to the PX4 Brickproof Bootloader for details. If you are not
    /// sure, leave the default value.
    Bootloader(IROMBackend&               rom_backend,
               const SystemInfo&          system_info,
               const std::size_t          max_app_size,
               const bool                 linger,
               const std::chrono::seconds boot_delay                   = std::chrono::seconds(0),
               const bool                 allow_legacy_app_descriptors = false) :
        max_app_size_(max_app_size),
        boot_delay_(boot_delay),
        backend_(rom_backend),
        presentation_(system_info, *this),
        linger_(linger),
        allow_legacy_app_descriptors_(allow_legacy_app_descriptors)
    {}

    /// Nodes shall be registered using this method after the instance is constructed.
    /// The return value is true on success, false if there are too many nodes already or this node is already
    /// registered (no effect in this case).
    [[nodiscard]] auto addNode(INode* const node) -> bool { return presentation_.addNode(node); }

    /// The number of nodes added with addNode(). Zero by default (obviously).
    [[nodiscard]] auto getNumberOfNodes() const -> std::uint8_t { return presentation_.getNumberOfNodes(); }

    /// Non-blocking periodic state update.
    /// The outer logic should invoke this method after any hardware event (for example, if WFE/WFI is used on an
    /// ARM platform), and periodically at least once per second. Typically, it would be invoked from the main loop.
    /// The watchdog, if used, should be reset before or after each invocation.
    [[nodiscard]] auto poll(const std::chrono::microseconds time_since_boot) -> std::optional<Final>
    {
        uptime_ = time_since_boot;
        if (!inited_)
        {
            inited_ = true;
            reset(!linger_);
        }

        if ((State::AppUpdateInProgress == state_) && request_read_)
        {
            request_read_ = false;
            ++read_request_count_;
            presentation_.setNodeVSSC(static_cast<std::uint8_t>(read_request_count_));  // Indicate progress.
            if (!presentation_.requestFileRead(rom_offset_))
            {
                presentation_.publishLogRecord(detail::dsdl::Diagnostic::Severity::Critical,
                                               "Could not send request uavcan.file.Read, abort");
                reset(false);
            }
        }

        if ((State::BootDelay == state_) && (uptime_ >= boot_deadline_))  // Fast boot path.
        {
            final_ = Final::BootApp;
        }

        if (!final_)
        {
            // If the application is valid and the boot delay is zero, the nodes will never be polled by design.
            // This avoids unnecessary delays at startup. In many embedded systems fast boot-up is critical.
            presentation_.poll(uptime_);
        }

        if (pending_log_)
        {
            // Send logs as late as possible to take into account the new entries from this poll().
            presentation_.publishLogRecord(pending_log_->first, pending_log_->second);
            pending_log_.reset();
        }

        return final_;
    }

    /// Manual trigger: commence the application update process without waiting for an external node to trigger it.
    /// This is normally used when the application commands the bootloader to begin the update directly.
    /// Returns false if the node reference is invalid; otherwise true.
    /// The path is truncated if too long.
    auto trigger(const INode* const        node,
                 const NodeID              file_server_node_id,
                 const std::size_t         app_image_file_path_length,
                 const std::uint8_t* const app_image_file_path) -> bool
    {
        return presentation_.trigger(node, file_server_node_id, app_image_file_path_length, app_image_file_path);
    }
    auto trigger(const std::uint8_t        node_index,
                 const NodeID              file_server_node_id,
                 const std::size_t         app_image_file_path_length,
                 const std::uint8_t* const app_image_file_path) -> bool
    {
        return presentation_.trigger(node_index, file_server_node_id, app_image_file_path_length, app_image_file_path);
    }

    /// Returns the information about the installed application, if there is one. Otherwise, returns an empty option.
    /// The return value may not match the actual state of the application before the first poll() is executed.
    [[nodiscard]] auto getAppInfo() const -> std::optional<AppInfo> override { return app_info_; }

    /// This method is mostly intended for state indication purposes, like driving the status LEDs.
    /// The return value may not match the actual state of the application before the first poll() is executed.
    [[nodiscard]] auto getState() const { return state_; }

private:
    /// Execution may take several seconds due to the ROM scanning and CRC verification.
    /// The argument is true if the application should be automatically launched if present.
    void reset(const bool auto_boot)
    {
        if (State::AppUpdateInProgress == state_)
        {
            backend_.endWrite();
        }
        app_info_ = detail::AppLocator(backend_, max_app_size_).identifyApplication(allow_legacy_app_descriptors_);
        final_.reset();
        if (app_info_)
        {
            if (auto_boot)
            {
                state_         = State::BootDelay;
                boot_deadline_ = uptime_ + boot_delay_;
                presentation_.setNodeHealth(detail::dsdl::Heartbeat::Health::Nominal);
            }
            else
            {
                state_ = State::BootCanceled;
                presentation_.setNodeHealth(detail::dsdl::Heartbeat::Health::Advisory);
            }
        }
        else
        {
            state_ = State::NoAppToBoot;
            presentation_.setNodeHealth(detail::dsdl::Heartbeat::Health::Warning);
        }
        presentation_.setNodeVSSC(0);
    }

    void reboot() override { final_ = Final::Restart; }

    void beginUpdate() override
    {
        // If an update was already in progress, it is simply interrupted and we begin from scratch.
        if (State::AppUpdateInProgress == state_)
        {
            backend_.endWrite();  // Cycle the state to re-init ROM if needed.
            pending_log_ = {detail::dsdl::Diagnostic::Severity::Warning, "Ongoing software update restarted"};
        }
        else
        {
            pending_log_ = {detail::dsdl::Diagnostic::Severity::Notice, "Software update started"};
        }
        state_              = State::AppUpdateInProgress;
        rom_offset_         = 0;
        read_request_count_ = 0;
        request_read_       = true;
        backend_.beginWrite();
        presentation_.setNodeHealth(detail::dsdl::Heartbeat::Health::Nominal);
    }

    void handleFileReadResult(const std::optional<detail::dsdl::File::ReadResponse> response) override
    {
        if (!response)
        {
            pending_log_ = {detail::dsdl::Diagnostic::Severity::Critical,
                            "Software image file request timeout or file server error"};
            reset(false);
        }
        else
        {
            const auto result = backend_.write(rom_offset_, response->data, response->data_length);
            const bool ok     = result && (result == response->data_length);
            if (ok && (response->data_length >= detail::dsdl::File::ReadResponseDataCapacity))
            {
                rom_offset_ += response->data_length;
                request_read_ = true;
            }
            else
            {
                if (!ok)
                {
                    pending_log_ = {detail::dsdl::Diagnostic::Severity::Critical, "ROM write failure"};
                }
                reset(true);
            }
        }
    }

    const std::size_t          max_app_size_;
    const std::chrono::seconds boot_delay_;

    IROMBackend&      backend_;
    detail::Presenter presentation_;
    const bool        linger_;
    const bool        allow_legacy_app_descriptors_;

    std::chrono::microseconds uptime_{};
    bool                      inited_ = false;
    State                     state_  = State::NoAppToBoot;
    std::optional<Final>      final_;
    std::chrono::microseconds boot_deadline_{};
    std::size_t               rom_offset_         = 0;
    std::uint32_t             read_request_count_ = 0;
    bool                      request_read_       = false;
    std::optional<AppInfo>    app_info_;

    std::optional<std::pair<detail::dsdl::Diagnostic::Severity, const char*>> pending_log_;
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
/// that there is no data to read (the latter occurs when the bootloader is started after power-on reset,
/// a power loss, or a hard reset).
///
/// The stored data type shall be a trivial type (see https://en.cppreference.com/w/cpp/named_req/TrivialType).
/// The storage space shall be large enough to accommodate an instance of the stored data type plus eight bytes
/// for the CRC (no padding inserted).
///
/// Here is a usage example. Initialization:
///
///     struct MyData;
///     VolatileStorage<MyData> storage(my_memory_location);
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
    /// The amount of memory required to store the data. This is the size of the container plus 8 bytes for the CRC.
    static constexpr auto StorageSize = sizeof(Container) + CRC64::Size;  // NOLINT

    explicit VolatileStorage(std::uint8_t* const location) : ptr_(location) {}

    /// Checks if the data is available and reads it, then erases the storage to prevent deja-vu.
    /// Returns an empty option if no data is available (in that case the storage is not erased).
    [[nodiscard]] auto take() -> std::optional<Container>
    {
        CRC64 crc;
        crc.update(ptr_, StorageSize);
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
        CRC64 crc;
        crc.update(ptr_, sizeof(Container));
        const auto crc_ptr = ptr_ + sizeof(Container);  // NOLINT NOSONAR pointer arithmetic
        (void) std::memmove(crc_ptr, crc.getBytes().data(), CRC64::Size);
    }

protected:
    static constexpr std::uint8_t EraseFillValue = 0xCA;

    std::uint8_t* const ptr_;
};

}  // namespace kocherga
