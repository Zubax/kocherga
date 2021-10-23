// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_can.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "util.hpp"          // NOLINT include order
#include "catch.hpp"
#include <algorithm>
#include <deque>
#include <iostream>

namespace
{
using Bitrate = kocherga::can::ICANDriver::Bitrate;
using kocherga::can::CANAcceptanceFilterConfig;
using Allocator = kocherga::can::detail::BlockAllocator<1024, 2>;

class CANDriverMock : public kocherga::can::ICANDriver
{
public:
    struct Config
    {
        Bitrate                   bitrate{};
        bool                      silent{};
        CANAcceptanceFilterConfig filter{};
    };

    struct Frame
    {
        std::uint32_t             extended_can_id{};
        std::vector<std::uint8_t> payload;
    };

    struct TxFrame : public Frame
    {
        bool force_classic_can{};
    };

    void setMode(const std::optional<Mode> m) { mode_ = m; }

    [[nodiscard]] auto getConfig() const -> std::optional<Config> { return config_; }

    [[nodiscard]] auto popTx() -> std::optional<TxFrame>
    {
        if (!tx_.empty())
        {
            const TxFrame f = tx_.front();
            tx_.pop_front();
            return f;
        }
        return {};
    }

    void pushRx(const Frame& fr) { rx_.push_back(fr); }

    [[nodiscard]] auto configure(const Bitrate& bitrate, const bool silent, const CANAcceptanceFilterConfig& filter)
        -> std::optional<Mode> override
    {
        config_ = {bitrate, silent, filter};
        return mode_;
    }

private:
    [[nodiscard]] auto push(const bool          force_classic_can,
                            const std::uint32_t extended_can_id,
                            const std::uint8_t  payload_size,
                            const void* const   payload) -> bool override
    {
        if (!mode_)
        {
            return false;
        }
        TxFrame f{};
        f.force_classic_can = force_classic_can;
        f.extended_can_id   = extended_can_id;
        std::copy_n(static_cast<const std::uint8_t*>(payload), payload_size, std::back_insert_iterator(f.payload));
        tx_.emplace_back(f);
        return true;
    }

    [[nodiscard]] auto pop(PayloadBuffer& payload_buffer)
        -> std::optional<std::pair<std::uint32_t, std::uint8_t>> override
    {
        if (mode_ && !rx_.empty())
        {
            const Frame f = rx_.front();
            rx_.pop_front();
            std::copy(f.payload.begin(), f.payload.end(), payload_buffer.begin());
            return {{f.extended_can_id, f.payload.size()}};
        }
        const std::optional<std::pair<std::uint32_t, std::uint8_t>> empty{};  // Suppress bogus warning from GCC.
        return empty;
    }

    std::optional<Mode>   mode_;
    std::optional<Config> config_;
    std::deque<TxFrame>   tx_;
    std::deque<Frame>     rx_;
};

class ReactorMock : public kocherga::IReactor
{
public:
    struct IncomingRequest
    {
        kocherga::PortID          service_id = 0xFFFF;
        std::uint8_t              client_node_id{};
        std::vector<std::uint8_t> data;
    };

    /// Accepts request, returns the serialized response payload or nothing if no response should be sent.
    using IncomingRequestHandler = std::function<std::optional<std::vector<std::uint8_t>>(const IncomingRequest&)>;

    void setIncomingRequestHandler(const IncomingRequestHandler& irh) { request_handler_ = irh; }

    [[nodiscard]] auto popPendingResponse() -> std::optional<std::vector<std::uint8_t>>
    {
        if (!pending_responses_.empty())
        {
            const auto out = pending_responses_.front();
            pending_responses_.pop_front();
            return out;
        }
        return {};
    }

private:
    [[nodiscard]] auto processRequest(const kocherga::PortID    service_id,
                                      const kocherga::NodeID    client_node_id,
                                      const std::size_t         request_length,
                                      const std::uint8_t* const request,
                                      std::uint8_t* const       out_response) -> std::optional<std::size_t> override
    {
        const auto client_node_id_uint8 = static_cast<std::uint8_t>(client_node_id);
        REQUIRE(client_node_id_uint8 == client_node_id);
        IncomingRequest ir{service_id, client_node_id_uint8, {}};
        std::copy_n(request, request_length, std::back_insert_iterator(ir.data));
        REQUIRE(request_handler_);
        if (const auto response_payload = request_handler_(ir))
        {
            std::copy(response_payload->begin(), response_payload->end(), out_response);
            return response_payload->size();
        }
        return {};
    }

    void processResponse(const std::size_t response_length, const std::uint8_t* const response) override
    {
        pending_responses_.emplace_back();
        std::copy_n(response, response_length, std::back_insert_iterator(pending_responses_.back()));
    }

    IncomingRequestHandler                request_handler_;
    std::deque<std::vector<std::uint8_t>> pending_responses_;
};

}  // namespace

TEST_CASE("can::detail::BitrateDetectionActivity")
{
    using kocherga::can::detail::IActivity;
    using kocherga::can::detail::BitrateDetectionActivity;
    using kocherga::can::detail::VersionDetectionActivity;
    Allocator                  alloc;
    CANDriverMock              driver;
    ReactorMock                reactor;
    std::shared_ptr<IActivity> act;

    driver.setMode(CANDriverMock::Mode::FD);

    act = std::make_shared<BitrateDetectionActivity>(alloc, driver, kocherga::SystemInfo::UniqueID{});
    REQUIRE(!driver.getConfig());
    REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000)));
    REQUIRE(driver.getConfig()->bitrate == Bitrate{1'000'000, 4'000'000});
    REQUIRE(driver.getConfig()->silent);
    REQUIRE(driver.getConfig()->filter == CANAcceptanceFilterConfig{0x1FFFFFFF, 0});
    REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'200'000)));
    REQUIRE(driver.getConfig()->bitrate == Bitrate{500'000, 2'000'000});
    REQUIRE(!act->poll(reactor, std::chrono::microseconds(2'400'000)));
    REQUIRE(driver.getConfig()->bitrate == Bitrate{250'000, 1'000'000});
    REQUIRE(!act->poll(reactor, std::chrono::microseconds(3'600'000)));
    REQUIRE(driver.getConfig()->bitrate == Bitrate{125'000, 500'000});
    REQUIRE(!act->poll(reactor, std::chrono::microseconds(4'800'000)));
    REQUIRE(driver.getConfig()->bitrate == Bitrate{1'000'000, 4'000'000});  // Loop over!
    driver.pushRx(CANDriverMock::Frame{123456, {}});
    IActivity* const proto_ver_act = act->poll(reactor, std::chrono::microseconds(5'000'000));
    REQUIRE(proto_ver_act);
    REQUIRE(driver.getConfig()->bitrate == Bitrate{1'000'000, 4'000'000});  // This is the detected bitrate.
    REQUIRE(driver.getConfig()->silent);
    REQUIRE(driver.getConfig()->filter == CANAcceptanceFilterConfig{0x1FFFFFFF, 0});
    REQUIRE(dynamic_cast<VersionDetectionActivity*>(proto_ver_act));
}

TEST_CASE("can::detail::VersionDetectionActivity")
{
    using kocherga::can::detail::IActivity;
    using kocherga::can::detail::VersionDetectionActivity;
    using kocherga::can::detail::V0NodeIDAllocationActivity;
    using kocherga::can::detail::V1NodeIDAllocationActivity;
    Allocator                  alloc;
    CANDriverMock              driver;
    ReactorMock                reactor;
    std::shared_ptr<IActivity> act;
    driver.setMode(CANDriverMock::Mode::FD);
    const Bitrate br{1'000'000, 4'000'000};
    // Detect v1 if both v0 and v1 are present at the same time.
    {
        REQUIRE(driver.configure(br, true, CANAcceptanceFilterConfig::makePromiscuous()));
        act = std::make_shared<VersionDetectionActivity>(alloc, driver, kocherga::SystemInfo::UniqueID{}, br);
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000)));
        // No matter how long we wait, if there are no valid frames, the protocol version detection will never end.
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(2'000'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(3'000'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(4'000'000)));
        driver.pushRx(CANDriverMock::Frame{123456, {}});  // This is not an acceptable UAVCAN frame (either version).
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(5'000'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(6'000'000)));
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0}});  // Not a start frame, no version info in it.
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(7'000'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(8'000'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(9'000'000)));
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0b1000'0000}});  // UAVCAN/CAN v0
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0b1010'0000}});  // UAVCAN/CAN v1
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0b1000'0000}});  // UAVCAN/CAN v0
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0b1010'0000}});  // UAVCAN/CAN v1
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(10'000'000)));     // Detection timeout not yet expired
        IActivity* const v1_pnp_act = act->poll(reactor, std::chrono::microseconds(12'000'000));
        REQUIRE(v1_pnp_act);
        REQUIRE(dynamic_cast<V1NodeIDAllocationActivity*>(v1_pnp_act));
        // The silent flag must be reset before invoking the next activity!
        REQUIRE(driver.getConfig()->bitrate == br);
        REQUIRE(!driver.getConfig()->silent);                                                       // Not silent!
        REQUIRE(driver.getConfig()->filter == kocherga::can::detail::makeAcceptanceFilter<1>({}));  // v1!
    }
    // Detect v0 if only v0 is present.
    {
        REQUIRE(driver.configure(br, true, CANAcceptanceFilterConfig::makePromiscuous()));
        act = std::make_shared<VersionDetectionActivity>(alloc, driver, kocherga::SystemInfo::UniqueID{}, br);
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000)));
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000'000)));
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0b1000'0000}});  // UAVCAN/CAN v0
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(2'000'000)));
        IActivity* const v0_pnp_act = act->poll(reactor, std::chrono::microseconds(4'000'000));
        REQUIRE(v0_pnp_act);
        REQUIRE(dynamic_cast<V0NodeIDAllocationActivity*>(v0_pnp_act));
        // The silent flag must be reset before invoking the next activity!
        REQUIRE(driver.getConfig()->bitrate == br);
        REQUIRE(!driver.getConfig()->silent);                                                       // Not silent!
        REQUIRE(driver.getConfig()->filter == kocherga::can::detail::makeAcceptanceFilter<0>({}));  // v0!
    }
}

TEST_CASE("can::detail::V0NodeIDAllocationActivity")
{
    using kocherga::can::detail::IActivity;
    using kocherga::can::detail::V0NodeIDAllocationActivity;
    using kocherga::can::detail::V0MainActivity;
    using std::chrono_literals::operator""us;
    using Buf = std::vector<std::uint8_t>;

    Allocator     alloc;
    CANDriverMock driver;
    ReactorMock   reactor;
    driver.setMode(CANDriverMock::Mode::FD);
    const Bitrate                        br{1'000'000, 4'000'000};
    const kocherga::SystemInfo::UniqueID uid{
        {0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x43, 0x00, 0x00, 0x00, 0x00}};

    static const auto compute_pnp_request_can_id = [](const std::vector<std::uint8_t>& frame_payload) -> std::uint32_t {
        REQUIRE(!frame_payload.empty());
        REQUIRE(frame_payload.size() <= 8);
        kocherga::can::detail::CRC16CCITT discriminator_crc;
        discriminator_crc.update(frame_payload.size(), frame_payload.data());
        const std::uint32_t discriminator = discriminator_crc.get() & ((1UL << 14U) - 1U);
        return 0x1E'0001'00UL | (discriminator << 10U);
    };

    // Successful allocation.
    {
        V0NodeIDAllocationActivity act(alloc, driver, uid, br);
        REQUIRE(act.getDeadline().count() == 0);
        REQUIRE(act.getStage() == 0);
        // First poll -- set up the deadline with randomization.
        REQUIRE(!act.poll(reactor, 1'000'000us));
        const auto deadline_a = act.getDeadline();
        REQUIRE(deadline_a >= 1'600'000us);
        REQUIRE(deadline_a <= 2'000'000us);
        REQUIRE(act.getStage() == 0);
        REQUIRE(!act.poll(reactor, deadline_a - 1us));  // No change yet
        REQUIRE(act.getDeadline() == deadline_a);
        REQUIRE(act.getStage() == 0);
        REQUIRE(!driver.popTx());
        // Deadline reached, state updated; first stage allocation request sent.
        REQUIRE(!act.poll(reactor, deadline_a));
        auto fr = driver.popTx();
        REQUIRE(fr);
        REQUIRE(!driver.popTx());
        REQUIRE(fr->force_classic_can);
        Buf payload{0x01, 0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0b1100'0000U};
        REQUIRE(fr->extended_can_id == compute_pnp_request_can_id(payload));
        REQUIRE(fr->payload == payload);
        // If we don't send a response, the allocatee will continue re-sending the first stage request forever.
        const auto deadline_b = act.getDeadline();
        REQUIRE(deadline_b >= deadline_a + 600'000us);
        REQUIRE(deadline_b <= deadline_a + 1'000'000us);
        REQUIRE(!act.poll(reactor, deadline_b - 2us));
        REQUIRE(!act.poll(reactor, deadline_b - 1us));
        REQUIRE(!driver.popTx());
        REQUIRE(!act.poll(reactor, deadline_b));
        fr = driver.popTx();
        REQUIRE(fr);
        REQUIRE(!driver.popTx());
        REQUIRE(fr->force_classic_can);
        payload = {0x01, 0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0b1100'0001U};
        REQUIRE(fr->extended_can_id == compute_pnp_request_can_id(payload));
        REQUIRE(fr->payload == payload);
        // Send 1st stage allocation response matching this UID, prompting the 2nd stage request.
        driver.pushRx({0x14'0001'7FUL, {0x00, 0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0b1110'0000U}});  // UAVCAN v1 ignore
        driver.pushRx({0x14'0001'7FUL, {0x00, 0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0b1100'0000U}});  // Valid accepted
        REQUIRE(0 == act.getStage());
        REQUIRE(!act.poll(reactor, deadline_b + 1000us));
        REQUIRE(1 == act.getStage());  // It's a match!
        const auto deadline_c = act.getDeadline();
        REQUIRE(deadline_c >= deadline_b + 1000us);
        REQUIRE(deadline_c <= deadline_b + 401'000us);
        REQUIRE(!act.poll(reactor, deadline_c - 1us));
        REQUIRE(!driver.popTx());
        REQUIRE(!act.poll(reactor, deadline_c));
        fr = driver.popTx();
        REQUIRE(fr);
        REQUIRE(!driver.popTx());
        REQUIRE(fr->force_classic_can);
        payload = {0x00, 0x31, 0x34, 0x61, 0x41, 0x23, 0x43, 0b1100'0010U};
        REQUIRE(fr->extended_can_id == compute_pnp_request_can_id(payload));
        REQUIRE(fr->payload == payload);
        // After the 2nd stage request is sent, the schedule should be armed to send the 1st stage request on timeout.
        REQUIRE(0 == act.getStage());
        const auto deadline_d = act.getDeadline();
        REQUIRE(deadline_d >= deadline_c + 600'000us);
        REQUIRE(deadline_d <= deadline_c + 1'000'000us);
        // Let the timeout expire and ensure that another 1st stage request is out.
        REQUIRE(!act.poll(reactor, deadline_d));
        fr = driver.popTx();
        REQUIRE(fr);
        REQUIRE(!driver.popTx());
        REQUIRE(fr->force_classic_can);
        payload = {0x01, 0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0b1100'0011U};
        REQUIRE(fr->extended_can_id == compute_pnp_request_can_id(payload));
        REQUIRE(fr->payload == payload);
        REQUIRE(0 == act.getStage());
        // Regardless of the current stage, reception of a matching UID sub-sequence will advance the process.
        driver.pushRx({0x1E'01FD'FFUL, {0xC0}});  // Ignore service transfer (this is a GetNodeInfo request)
        driver.pushRx({0x14'0001'7FUL, {0x9E, 0x3D, 0x00, 0x35, 0xFF, 0xD5, 0x05, 0x81}});
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});  // Ignore duplicate
        driver.pushRx({0x14'0002'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});  // Ignore unrelated
        driver.pushRx({0x14'0001'77UL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});  // Ignore wrong sender
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x22}});  // Ignore wrong transfer-ID
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x41}});
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x41}});  // Ignore duplicate
        REQUIRE(!act.poll(reactor, deadline_d));
        REQUIRE(!driver.popTx());
        REQUIRE(2 == act.getStage());
        const auto deadline_e = act.getDeadline();
        std::cout << "deadline_e=" << deadline_e.count() << " deadline_d=" << deadline_d.count() << std::endl;
        REQUIRE(deadline_e >= deadline_d + 0us);
        REQUIRE(deadline_e <= deadline_d + 400'000us);
        REQUIRE(!act.poll(reactor, deadline_e));
        REQUIRE(0 == act.getStage());  // Reset back to zero again after publication.
        REQUIRE(act.getDeadline() >= deadline_e + 600'000us);
        REQUIRE(act.getDeadline() <= deadline_e + 1'000'000us);
        fr = driver.popTx();
        REQUIRE(fr);
        REQUIRE(!driver.popTx());
        REQUIRE(fr->force_classic_can);
        payload = {0x00, 0x00, 0x00, 0x00, 0x00, 0b1100'0100U};  // 3rd stage request
        REQUIRE(fr->extended_can_id == compute_pnp_request_can_id(payload));
        REQUIRE(fr->payload == payload);
        // Receive a matching response, which completes the process.
        driver.pushRx({0x14'0001'7FUL, {0xFF, 0xFF, 0xEA, 0x35, 0xFF, 0xD5, 0x05, 0x82}});  // Bad CRC! Ignored.
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x22}});
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x00, 0x00, 0x00, 0x00, 0x42}});
        REQUIRE(!act.poll(reactor, deadline_e));
        REQUIRE(!driver.popTx());
        driver.pushRx({0x14'0001'7FUL, {0x57, 0x76, 0x00, 0x35, 0xFF, 0xD5, 0x05, 0x83}});  // Bad node-ID value!
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x23}});
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x00, 0x00, 0x00, 0x00, 0x43}});
        REQUIRE(!act.poll(reactor, deadline_e));
        REQUIRE(!driver.popTx());
        driver.pushRx({0x14'0001'7FUL, {0x8C, 0x7A, 0xFA, 0x35, 0xFF, 0xD5, 0x05, 0x84}});  // Correct CRC here.
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x24}});
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x00, 0x00, 0x00, 0x00, 0x44}});
        auto* const new_act = act.poll(reactor, deadline_e);
        REQUIRE(new_act != nullptr);
        REQUIRE(!driver.popTx());
        REQUIRE(dynamic_cast<V0MainActivity*>(new_act));
        REQUIRE(dynamic_cast<V0MainActivity*>(new_act)->getLocalNodeID() == 125);
    }
}

TEST_CASE("can::detail::V1NodeIDAllocationActivity")
{
    using kocherga::SubjectID;
    using kocherga::can::detail::IActivity;
    using kocherga::can::detail::V1NodeIDAllocationActivity;
    using kocherga::can::detail::V1MainActivity;
    using std::chrono_literals::operator""us;
    using Buf = std::vector<std::uint8_t>;

    Allocator                            alloc;
    CANDriverMock                        driver;
    ReactorMock                          reactor;
    const Bitrate                        br{1'000'000, 4'000'000};
    const kocherga::SystemInfo::UniqueID uid{
        {0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x43, 0x00, 0x00, 0x00, 0x00}};
    // The pseudo-UID is computed as CRC64WE of the UID using PyUAVCAN.
    const std::array<std::uint8_t, 6> pseudo_uid_bytes{50, 191, 169, 46, 145, 226};

    std::optional<V1NodeIDAllocationActivity> act;
    std::chrono::microseconds                 uptime(0);

    const auto spin = [&reactor, &driver, &act, &uptime]() -> std::variant<IActivity*, CANDriverMock::TxFrame> {
        const auto uptime_increment = 1000us;
        const auto deadline         = uptime + 3'000'000us + uptime_increment;
        while (uptime <= deadline)
        {
            uptime += uptime_increment;
            if (auto* const res = act->poll(reactor, uptime))
            {
                return res;
            }
            if (const auto f = driver.popTx())
            {
                REQUIRE(!driver.popTx());  // Ensure there are no unexpected frames left.
                return *f;
            }
        }
        FAIL("TX FRAME NOT EMITTED BEFORE THE DEADLINE");
        return nullptr;  // Unreachable
    };

    const auto ensure_subject_id = [](const std::uint32_t extended_can_id, const SubjectID sid) -> bool {
        constexpr auto mask = 0b110'01'0110000000000000'0'1111111UL;
        return (extended_can_id | 0x7FU) == (mask | (static_cast<std::uint32_t>(sid) << 8U));
    };

    // FD-capable bus; the allocator responds using v2
    {
        driver.setMode(CANDriverMock::Mode::FD);
        act.emplace(alloc, driver, uid, br, CANDriverMock::Mode::FD);

        // v2 is sent first because it is supposed to take precedence.
        auto f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(!f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v2));
        {
            Buf ref(20, 0);  // We only need 19 bytes; +1 is due to CAN FD DLC padding.
            ref.at(0) = std::numeric_limits<std::uint8_t>::max();
            ref.at(1) = std::numeric_limits<std::uint8_t>::max();
            std::copy(uid.begin(), uid.end(), &ref.at(2));
            ref.back() = 0b1110'0000U;
            REQUIRE(f.payload == ref);
        }

        // v1 send afterwards
        f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v1));
        {
            Buf ref(8, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 0;
            ref.at(7) = 0b1110'0000U;
            REQUIRE(f.payload == ref);
        }
        REQUIRE(!driver.popTx());  // Ensure there are no unexpected frames left.

        // Wait for the next request pair to ensure the transfer-ID is being properly incremented and v1/v2 alternate.
        f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(!f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v2));
        {
            Buf ref(20, 0);  // We only need 19 bytes; +1 is due to CAN FD DLC padding.
            ref.at(0) = std::numeric_limits<std::uint8_t>::max();
            ref.at(1) = std::numeric_limits<std::uint8_t>::max();
            std::copy(uid.begin(), uid.end(), &ref.at(2));
            ref.back() = 0b1110'0001U;  // TID incremented!
            REQUIRE(f.payload == ref);
        }
        f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v1));
        {
            Buf ref(8, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 0;
            ref.at(7) = 0b1110'0001U;
            REQUIRE(f.payload == ref);
        }
        REQUIRE(!driver.popTx());  // Ensure there are no unexpected frames left.

        // Send v1 response with invalid node-ID to ensure it is ignored.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 1;
            ref.at(7) = 0xFF;  // Not a valid node-ID
            ref.at(8) = 0xFF;  // Second byte
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        // Send v1 response with mismatching UID to ensure it is ignored.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(5) = static_cast<std::uint8_t>(~ref.at(5));  // Change one byte
            ref.at(6) = 1;
            ref.at(7) = 3;  // LSB
            ref.at(8) = 0;  // MSB
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        // Send v1 response with bad array length to ensure it is ignored.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 2;  // Shall be either 0 (if request) or 1 (if response)
            ref.at(7) = 3;  // LSB
            ref.at(8) = 0;  // MSB
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        // Send malformed v1 response to ensure it is ignored.
        {
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, {0b1111'1111U}});
        }
        // Send v2 response with invalid node-ID to ensure it is ignored.
        {
            Buf ref(20, 0);  // We only need 19 bytes; +1 is due to CAN FD DLC padding.
            ref.at(0) = 0xAA;
            ref.at(1) = 0xAA;
            std::copy(uid.begin(), uid.end(), &ref.at(2));
            ref.back() = 0b1111'1111U;
            driver.pushRx({0b110'00'0111111111100101'0'1111110UL, ref});
        }
        // Send v2 response with mismatching UID to ensure it is ignored.
        {
            Buf ref(20, 0);  // We only need 19 bytes; +1 is due to CAN FD DLC padding.
            ref.at(0) = 3;
            ref.at(1) = 0;
            std::copy(uid.begin(), uid.end(), &ref.at(2));
            ref.at(17) = static_cast<std::uint8_t>(~ref.at(17));  // Change one byte
            ref.back() = 0b1111'1111U;
            driver.pushRx({0b110'00'0111111111100101'0'1111110UL, ref});
        }
        // Send malformed v2 response to ensure it is ignored.
        {
            driver.pushRx({0b110'00'0111111111100101'0'1111110UL, {0b1111'1111U}});
        }
        // Yup, we simply get the next request -- the bad responses are ignored.
        f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(!f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v2));
        // Send the correct v2 response and ensure it is accepted.
        {
            Buf ref(20, 0);  // We only need 19 bytes; +1 is due to CAN FD DLC padding.
            ref.at(0) = 7;
            ref.at(1) = 0;
            std::copy(uid.begin(), uid.end(), &ref.at(2));
            ref.back() = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100101'0'1111110UL, ref});
        }
        const auto result = spin();
        REQUIRE(std::holds_alternative<IActivity*>(result));
        const auto* const main_act = dynamic_cast<V1MainActivity*>(std::get<IActivity*>(result));
        REQUIRE(main_act);
        REQUIRE(main_act->getLocalNodeID() == 7);  // Ensure the allocated value is extracted correctly.
    }

    // Classic CAN bus
    {
        driver.setMode(CANDriverMock::Mode::Classic);
        act.emplace(alloc, driver, uid, br, CANDriverMock::Mode::Classic);

        auto f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v1));
        {
            Buf ref(8, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 0;
            ref.at(7) = 0b1110'0000U;
            REQUIRE(f.payload == ref);
        }
        REQUIRE(!driver.popTx());  // Ensure there are no unexpected frames left.

        // Wait for the next request to ensure the transfer-ID is being properly incremented.
        f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v1));
        {
            Buf ref(8, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 0;
            ref.at(7) = 0b1110'0001U;  // Incremented!
            REQUIRE(f.payload == ref);
        }
        REQUIRE(!driver.popTx());  // Ensure there are no unexpected frames left.

        // Send v1 response with invalid node-ID to ensure it is ignored.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 1;
            ref.at(7) = 0xFF;  // Not a valid node-ID
            ref.at(8) = 0xFF;  // Second byte
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        // Send v1 response with mismatching UID to ensure it is ignored.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(5) = static_cast<std::uint8_t>(~ref.at(5));  // Change one byte
            ref.at(6) = 1;
            ref.at(7) = 3;  // LSB
            ref.at(8) = 0;  // MSB
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        // Send v1 response with bad array length to ensure it is ignored.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 2;  // Shall be either 0 (if request) or 1 (if response)
            ref.at(7) = 3;  // LSB
            ref.at(8) = 0;  // MSB
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        // Send malformed v1 response to ensure it is ignored.
        {
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, {0b1111'1111U}});
        }
        // Yup, we simply get the next request -- the bad responses are ignored.
        f = std::get<CANDriverMock::TxFrame>(spin());
        REQUIRE(f.force_classic_can);
        REQUIRE(ensure_subject_id(f.extended_can_id, SubjectID::PnPNodeIDAllocationData_v1));
        // Send the correct v1 response and ensure it is accepted.
        {
            Buf ref(10, 0);
            std::copy(pseudo_uid_bytes.begin(), pseudo_uid_bytes.end(), ref.begin());
            ref.at(6) = 1;
            ref.at(7) = 15;  // LSB
            ref.at(8) = 0;   // MSB
            ref.at(9) = 0b1110'0000U;
            driver.pushRx({0b110'00'0111111111100110'0'1111110UL, ref});
        }
        const auto result = spin();
        REQUIRE(std::holds_alternative<IActivity*>(result));
        const auto* const main_act = dynamic_cast<V1MainActivity*>(std::get<IActivity*>(result));
        REQUIRE(main_act);
        REQUIRE(main_act->getLocalNodeID() == 15);  // Ensure the allocated value is extracted correctly.
    }
}

TEST_CASE("can::CANNode v1")
{
    using kocherga::SubjectID;
    using kocherga::ServiceID;
    using kocherga::can::CANNode;
    using std::chrono_literals::operator""us;
    using Buf = std::vector<std::uint8_t>;

    CANDriverMock             driver;
    ReactorMock               reactor;
    std::chrono::microseconds uptime(0);

    const Bitrate                        br{1'000'000, 4'000'000};
    const kocherga::SystemInfo::UniqueID uid{
        {0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x43, 0x00, 0x00, 0x00, 0x00}};

    driver.setMode(kocherga::can::ICANDriver::Mode::FD);
    CANNode node(driver, uid, br, 1, 123);

    const auto poll = [&reactor, &driver, &node, &uptime](const std::chrono::microseconds            uptime_delta,
                                                          const std::optional<CANDriverMock::Frame>& rx_frame = {})
        -> std::optional<CANDriverMock::TxFrame> {
        if (rx_frame)
        {
            driver.pushRx(*rx_frame);
        }
        static_cast<kocherga::INode&>(node).poll(reactor, uptime);
        uptime += uptime_delta;
        if (const auto f = driver.popTx())
        {
            return *f;
        }
        return {};
    };

    // Run a few dry polls, kick the wheels.
    REQUIRE(!poll(1000us));
    REQUIRE(!poll(1000us));
    REQUIRE(!poll(1000us));
    REQUIRE(uptime == 3000us);
    REQUIRE(!reactor.popPendingResponse());

    // Check service requests.
    reactor.setIncomingRequestHandler([](const ReactorMock::IncomingRequest& req) -> std::optional<Buf> {
        REQUIRE(req.client_node_id == 42);
        switch (req.service_id)
        {
        case static_cast<kocherga::PortID>(ServiceID::NodeGetInfo):
        {
            return Buf{1, 2, 3};
        }
        default:
        {
            FAIL("Unexpected service request");
            break;
        }
        }
        return {};
    });
    // Send GetInfo request.
    auto tx_frame = poll(1000us,
                         CANDriverMock::Frame{
                             (0b110'11'0000000000'0000000'0000000UL |                        // Request
                              (static_cast<std::uint32_t>(ServiceID::NodeGetInfo) << 14U) |  // Service-ID
                              (static_cast<std::uint32_t>(123) << 7U) |                      // Destination node-ID
                              (static_cast<std::uint32_t>(42) << 0U)),                       // Source node-ID
                             {
                                 0b1110'0000U,  // Tail byte
                             },
                         });
    REQUIRE(tx_frame);
    REQUIRE(!tx_frame->force_classic_can);
    REQUIRE(tx_frame->payload == Buf{1, 2, 3, 0b1110'0000U});

    // Do a request, send response back.
    REQUIRE(static_cast<kocherga::INode&>(node)
                .sendRequest(ServiceID::FileRead, 42, 15, 3, reinterpret_cast<const std::uint8_t*>("\x04\x05\x06")));
    tx_frame = poll(1000us);
    REQUIRE(tx_frame);
    REQUIRE(!tx_frame->force_classic_can);
    REQUIRE(tx_frame->payload == Buf{4, 5, 6, 0b1110'1111U});
    REQUIRE(!poll(1000us));
    REQUIRE(!reactor.popPendingResponse());
    REQUIRE(!poll(1000us,
                  CANDriverMock::Frame{
                      (0b110'10'0000000000'0000000'0000000UL |                     // Response
                       (static_cast<std::uint32_t>(ServiceID::FileRead) << 14U) |  // Service-ID
                       (static_cast<std::uint32_t>(123) << 7U) |                   // Destination node-ID
                       (static_cast<std::uint32_t>(42) << 0U)),                    // Source node-ID
                      {
                          3, 2, 1, 0b1110'1111U,  // Tail byte at the end
                      },
                  }));
    auto resp = reactor.popPendingResponse();
    REQUIRE(resp);
    REQUIRE(*resp == Buf{3, 2, 1});
    REQUIRE(!reactor.popPendingResponse());

    // Same, but cancel the pending request midway, ensure response is ignored.
    REQUIRE(static_cast<kocherga::INode&>(node)
                .sendRequest(ServiceID::FileRead, 42, 31, 3, reinterpret_cast<const std::uint8_t*>("\x04\x05\x06")));
    tx_frame = poll(1000us);
    REQUIRE(tx_frame);
    REQUIRE(!tx_frame->force_classic_can);
    REQUIRE(tx_frame->payload == Buf{4, 5, 6, 0b1111'1111U});
    REQUIRE(!poll(1000us));
    REQUIRE(!reactor.popPendingResponse());
    static_cast<kocherga::INode&>(node).cancelRequest();
    REQUIRE(!poll(1000us,
                  CANDriverMock::Frame{
                      (0b110'10'0000000000'0000000'0000000UL |                     // Response
                       (static_cast<std::uint32_t>(ServiceID::FileRead) << 14U) |  // Service-ID
                       (static_cast<std::uint32_t>(123) << 7U) |                   // Destination node-ID
                       (static_cast<std::uint32_t>(42) << 0U)),                    // Source node-ID
                      {
                          3, 2, 1, 0b1111'1111U,  // Tail byte at the end
                      },
                  }));
    REQUIRE(!reactor.popPendingResponse());

    // Publish a message.
    REQUIRE(static_cast<kocherga::INode&>(node).publishMessage(SubjectID::DiagnosticRecord,
                                                               7,
                                                               3,
                                                               reinterpret_cast<const std::uint8_t*>("\x07\x08\x09")));
    tx_frame = poll(1000us);
    REQUIRE(tx_frame);
    REQUIRE(!tx_frame->force_classic_can);
    REQUIRE(tx_frame->payload == Buf{7, 8, 9, 0b1110'0111U});
    REQUIRE(!poll(1000us));
}

TEST_CASE("can::CANNode v0")
{
    using kocherga::PortID;
    using kocherga::SubjectID;
    using kocherga::ServiceID;
    using kocherga::can::CANNode;
    using kocherga::can::detail::BasicTransferReasmV0;
    using kocherga::can::detail::BasicMessageTransferReasmV0;
    using kocherga::can::detail::BasicServiceTransferReasmV0;
    using kocherga::can::detail::MessageFrameModel;
    using kocherga::can::detail::ServiceFrameModel;
    using std::chrono_literals::operator""us;
    using Buf = std::vector<std::uint8_t>;

    CANDriverMock             driver;
    ReactorMock               reactor;
    std::chrono::microseconds uptime(0);

    const Bitrate                        br{1'000'000, 4'000'000};
    const kocherga::SystemInfo::UniqueID uid{
        {0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x43, 0x00, 0x00, 0x00, 0x00}};

    driver.setMode(kocherga::can::ICANDriver::Mode::FD);
    CANNode node(driver, uid, br, 0, 123);

    struct Transfer
    {
        std::uint8_t priority    = std::numeric_limits<std::uint8_t>::max();
        std::uint8_t transfer_id = std::numeric_limits<std::uint8_t>::max();
        Buf          payload;

        Transfer() = default;
        Transfer(std::uint8_t priority, std::uint8_t transfer_id, const Buf& payload) :
            priority(priority), transfer_id(transfer_id), payload(payload)
        {}

        [[nodiscard]] virtual auto makeCANID() const -> std::uint32_t = 0;

        virtual ~Transfer()                        = default;
        [[maybe_unused]] Transfer(const Transfer&) = delete;
        [[maybe_unused]] Transfer(Transfer&&)      = delete;
        auto operator=(const Transfer&) -> Transfer& = delete;
        auto operator=(Transfer&&) -> Transfer& = delete;
    };

    struct MessageTransfer : public Transfer
    {
        PortID                      subject_id = std::numeric_limits<PortID>::max();
        std::optional<std::uint8_t> source_node_id;

        MessageTransfer() = default;
        MessageTransfer(std::uint8_t                priority,
                        std::uint8_t                transfer_id,
                        PortID                      subject_id,
                        std::optional<std::uint8_t> source_node_id,
                        const Buf&                  payload) :
            Transfer(priority, transfer_id, payload), subject_id(subject_id), source_node_id(source_node_id)
        {}

        [[nodiscard]] auto makeCANID() const -> std::uint32_t override
        {
            if (source_node_id)
            {
                return (static_cast<std::uint32_t>(priority) << 24U) | (static_cast<std::uint32_t>(subject_id) << 8U) |
                       *source_node_id;
            }
            FAIL("Anonymous transfers not supported in this implementation");
            return 0;
        }
    };

    struct ServiceTransfer : public Transfer
    {
        PortID       service_id           = std::numeric_limits<PortID>::max();
        std::uint8_t source_node_id       = std::numeric_limits<std::uint8_t>::max();
        std::uint8_t destination_node_id  = std::numeric_limits<std::uint8_t>::max();
        bool         request_not_response = false;

        ServiceTransfer() = default;
        ServiceTransfer(std::uint8_t priority,
                        std::uint8_t transfer_id,
                        PortID       service_id,
                        std::uint8_t source_node_id,
                        std::uint8_t destination_node_id,
                        const bool   request_not_response,
                        const Buf&   payload) :
            Transfer(priority, transfer_id, payload),
            service_id(service_id),
            source_node_id(source_node_id),
            destination_node_id(destination_node_id),
            request_not_response(request_not_response)
        {}

        [[nodiscard]] auto makeCANID() const -> std::uint32_t override
        {
            return (static_cast<std::uint32_t>(priority) << 24U) |                         // Priority
                   (static_cast<std::uint32_t>(service_id) << 16U) |                       // Service-ID
                   static_cast<std::uint32_t>(request_not_response ? (1UL << 15U) : 0U) |  // Request not response
                   static_cast<std::uint32_t>(1UL << 7U) |                                 // Service not message
                   (static_cast<std::uint32_t>(destination_node_id) << 8U) |               // To
                   static_cast<std::uint32_t>(source_node_id);                             // From
        }
    };

    std::unordered_map<PortID, std::shared_ptr<BasicMessageTransferReasmV0<1024>>> ra_msg;
    ra_msg[341]   = std::make_shared<BasicMessageTransferReasmV0<1024>>(0x0F0868D0C1A7C6F1ULL);
    ra_msg[16383] = std::make_shared<BasicMessageTransferReasmV0<1024>>(0XD654A48E0C049D75ULL);
    std::unordered_map<PortID, std::shared_ptr<BasicServiceTransferReasmV0<1024>>> ra_req;
    ra_req[48] = std::make_shared<BasicServiceTransferReasmV0<1024>>(0x8DCDCA939F33F678ULL, 42);
    std::unordered_map<PortID, std::shared_ptr<BasicServiceTransferReasmV0<1024>>> ra_res;
    ra_res[1]  = std::make_shared<BasicServiceTransferReasmV0<1024>>(0xEE468A8121C46A9EULL, 42);
    ra_res[40] = std::make_shared<BasicServiceTransferReasmV0<1024>>(0xB7D725DF72724126ULL, 42);

    using SignatureTransferPair = std::pair<std::uint64_t, std::shared_ptr<Transfer>>;
    // At the network layer, we are dealing with v0-translated transfers, not v1.
    const auto poll = [&](const std::chrono::microseconds             uptime_delta,
                          const std::optional<SignatureTransferPair>& rx_transfer = {}) -> std::shared_ptr<Transfer> {
        if (rx_transfer)
        {
            const auto [sig, tr]       = *rx_transfer;
            const std::uint32_t can_id = tr->makeCANID();
            REQUIRE(kocherga::can::detail::transmitV0(
                [&driver, can_id](const std::size_t frame_payload_size, const std::uint8_t* const frame_payload) {
                    std::cout << "RX " << std::hex << can_id << std::dec << " "
                              << util::makeHexDump(frame_payload, frame_payload + frame_payload_size) << std::endl;
                    driver.pushRx(CANDriverMock::Frame{can_id, Buf{frame_payload, frame_payload + frame_payload_size}});
                    return true;
                },
                sig,
                tr->transfer_id,
                tr->payload.size(),
                tr->payload.data()));
        }
        static_cast<kocherga::INode&>(node).poll(reactor, uptime);
        uptime += uptime_delta;
        while (const auto raw_frame = driver.popTx())
        {
            std::cout << "TX " << std::hex << raw_frame->extended_can_id << std::dec << " "
                      << util::makeHexDump(raw_frame->payload) << std::endl;
            REQUIRE(raw_frame->force_classic_can);  // v0 shall use Classic only.
            const auto frame = kocherga::can::detail::parseFrameV0(raw_frame->extended_can_id,
                                                                   raw_frame->payload.size(),
                                                                   raw_frame->payload.data())
                                   .value();
            if (const auto* const f_m = std::get_if<MessageFrameModel>(&frame))
            {
                if (const auto res = ra_msg.at(f_m->subject_id)->update(*f_m))
                {
                    return std::make_shared<MessageTransfer>(f_m->priority,
                                                             f_m->transfer_id,
                                                             f_m->subject_id,
                                                             f_m->source_node_id,
                                                             Buf(res->second, res->second + res->first));
                }
            }
            else if (const auto* const f_s = std::get_if<ServiceFrameModel>(&frame))
            {
                auto& ra = f_s->request_not_response ? ra_req : ra_res;
                if (const auto res = ra.at(f_s->service_id)->update(*f_s))
                {
                    return std::make_shared<ServiceTransfer>(f_s->priority,
                                                             f_s->transfer_id,
                                                             f_s->service_id,
                                                             f_s->source_node_id,
                                                             f_s->destination_node_id,
                                                             f_s->request_not_response,
                                                             Buf(res->second, res->second + res->first));
                }
            }
            else
            {
                throw std::logic_error("Unexpected option");
            }
        }
        return {};
    };

    REQUIRE(!poll(1000us));
    REQUIRE(!poll(1000us));
    REQUIRE(!poll(1000us));
    REQUIRE(uptime == 3000us);

    // Can't publish unknown messages.
    REQUIRE(!static_cast<kocherga::INode&>(node).publishMessage(SubjectID::PnPNodeIDAllocationData_v2, 0, 0, nullptr));

    // Publish heartbeat, check translation.
    REQUIRE(static_cast<kocherga::INode&>(node).publishMessage(  //
        SubjectID::NodeHeartbeat,
        5,
        7,
        reinterpret_cast<const std::uint8_t*>("\x03\x00\x00\x00\x03\x03\x00")));
    auto tx = poll(1000us);
    REQUIRE(tx);
    REQUIRE(tx->transfer_id == 5);
    REQUIRE(tx->priority == 16);
    REQUIRE(tx->payload == Buf{0x03, 0x00, 0x00, 0x00, 0xd8, 0x00, 0x00});  // This is v0 format.
    REQUIRE(dynamic_cast<MessageTransfer&>(*tx).subject_id == 341);         // v0 NodeStatus, not v1 Heartbeat
    REQUIRE(*dynamic_cast<MessageTransfer&>(*tx).source_node_id == 123);

    // Publish log message, check translation.
    REQUIRE(
        static_cast<kocherga::INode&>(node).publishMessage(SubjectID::DiagnosticRecord,
                                                           14,
                                                           23,
                                                           reinterpret_cast<const std::uint8_t*>(
                                                               "\x00\x00\x00\x00\x00\x00\x00\x03\x0EUpdate started")));
    tx = poll(1000us);
    REQUIRE(tx);
    REQUIRE(tx->transfer_id == 14);
    REQUIRE(tx->priority == 31);
    REQUIRE(
        tx->payload ==
        Buf{0b010'00100U, 'B', 'o', 'o', 't', 'U', 'p', 'd', 'a', 't', 'e', ' ', 's', 't', 'a', 'r', 't', 'e', 'd'});
    REQUIRE(dynamic_cast<MessageTransfer&>(*tx).subject_id == 16383);
    REQUIRE(*dynamic_cast<MessageTransfer&>(*tx).source_node_id == 123);

    // Check service calls.
    REQUIRE(!reactor.popPendingResponse());
    reactor.setIncomingRequestHandler([](const ReactorMock::IncomingRequest& req) -> std::optional<Buf> {
        REQUIRE(req.client_node_id == 42);
        switch (req.service_id)
        {
        case static_cast<kocherga::PortID>(ServiceID::NodeGetInfo):
        {
            return Buf{
                0x01, 0x00,                                      // Protocol version
                0x0A, 0x1E,                                      // Hardware version
                0x07, 0x99,                                      // Software version
                0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,  // Software VCS revision ID
                0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61,  // UID
                0x23,  // Name length: "com.zubax.kocherga.test.integration"
                0x63, 0x6F, 0x6D, 0x2E, 0x7A, 0x75, 0x62, 0x61, 0x78, 0x2E, 0x6B, 0x6F, 0x63, 0x68, 0x65, 0x72,
                0x67, 0x61, 0x2E, 0x74, 0x65, 0x73, 0x74, 0x2E, 0x69, 0x6E, 0x74, 0x65, 0x67, 0x72, 0x61, 0x74,
                0x69, 0x6F, 0x6E,  // End of name
                0x00,              // No image CRC
                0x15,              // CoA length, then the CoA itself
                0x74, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, 0x61, 0x20, 0x63, 0x65, 0x72, 0x74, 0x69, 0x66,
                0x69, 0x63, 0x61, 0x74, 0x65,
            };
        }
        case static_cast<kocherga::PortID>(ServiceID::NodeExecuteCommand):
        {
            REQUIRE(req.data == Buf{
                                    0xFD, 0xFF,  // Command == COMMAND_BEGIN_SOFTWARE_UPDATE
                                    0x5C,        // Path length
                                    0x63, 0x6F, 0x6D, 0x2E, 0x7A, 0x75, 0x62, 0x61, 0x78, 0x2E, 0x6B, 0x6F, 0x63, 0x68,
                                    0x65, 0x72, 0x67, 0x61, 0x2E, 0x74, 0x65, 0x73, 0x74, 0x2E, 0x69, 0x6E, 0x74, 0x65,
                                    0x67, 0x72, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x2D, 0x31, 0x30, 0x2D, 0x33, 0x2E, 0x31,
                                    0x2E, 0x62, 0x61, 0x64, 0x63, 0x30, 0x66, 0x66, 0x65, 0x65, 0x30, 0x64, 0x64, 0x66,
                                    0x30, 0x30, 0x64, 0x2E, 0x34, 0x35, 0x32, 0x61, 0x34, 0x32, 0x36, 0x37, 0x39, 0x37,
                                    0x31, 0x61, 0x33, 0x39, 0x32, 0x38, 0x2E, 0x61, 0x70, 0x70, 0x2E, 0x72, 0x65, 0x6C,
                                    0x65, 0x61, 0x73, 0x65, 0x2E, 0x62, 0x69, 0x6E,
                                });
            return Buf{1};  // FAILURE will be translated into ERROR_UNKNOWN.
        }
        default:
        {
            FAIL("Unexpected service request");
            break;
        }
        }
        return {};
    });

    // Validate GetInfo -- ensure the service is operational and v1/v0 translation is correct.
    tx = poll(1000us,
              SignatureTransferPair{0xEE468A8121C46A9EULL,
                                    std::make_shared<ServiceTransfer>(4, 15, 1, 42, 123, true, Buf{})});
    REQUIRE(tx);
    REQUIRE(tx->priority == 4);
    REQUIRE(tx->transfer_id == 15);
    const Buf translated_node_info{
        0x03, 0x00, 0x00, 0x00, 0xd8, 0x00, 0x00,        // NodeStatus
        0x07, 0x99,                                      // Software version major.minor
        0x00,                                            // Optional field flags (none available)
        0x00, 0x00, 0x00, 0x00,                          // VCS commit
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Image CRC
        0x0a, 0x1e,                                      // Hardware version major minor
        0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61,  // UID
        0x00,  // CoA is not translated for simplicity; the next field is the name
        0x63, 0x6f, 0x6d, 0x2e, 0x7a, 0x75, 0x62, 0x61, 0x78, 0x2e, 0x6b, 0x6f, 0x63, 0x68, 0x65, 0x72, 0x67, 0x61,
        0x2e, 0x74, 0x65, 0x73, 0x74, 0x2e, 0x69, 0x6e, 0x74, 0x65, 0x67, 0x72, 0x61, 0x74, 0x69, 0x6f, 0x6e,
    };
    REQUIRE(tx->payload == translated_node_info);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).service_id == 1);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).source_node_id == 123);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).destination_node_id == 42);
    REQUIRE(!dynamic_cast<ServiceTransfer&>(*tx).request_not_response);

    // Validate firmware update request.
    const Buf update_request_v0{
        0x00,  // Source node-ID unset, meaning that the file should be downloaded from the caller.
        0x63, 0x6F, 0x6D, 0x2E, 0x7A, 0x75, 0x62, 0x61, 0x78, 0x2E, 0x6B, 0x6F, 0x63, 0x68, 0x65, 0x72,
        0x67, 0x61, 0x2E, 0x74, 0x65, 0x73, 0x74, 0x2E, 0x69, 0x6E, 0x74, 0x65, 0x67, 0x72, 0x61, 0x74,
        0x69, 0x6F, 0x6E, 0x2D, 0x31, 0x30, 0x2D, 0x33, 0x2E, 0x31, 0x2E, 0x62, 0x61, 0x64, 0x63, 0x30,
        0x66, 0x66, 0x65, 0x65, 0x30, 0x64, 0x64, 0x66, 0x30, 0x30, 0x64, 0x2E, 0x34, 0x35, 0x32, 0x61,
        0x34, 0x32, 0x36, 0x37, 0x39, 0x37, 0x31, 0x61, 0x33, 0x39, 0x32, 0x38, 0x2E, 0x61, 0x70, 0x70,
        0x2E, 0x72, 0x65, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x2E, 0x62, 0x69, 0x6E,
    };
    tx = poll(1000us,
              SignatureTransferPair{0xB7D725DF72724126ULL,
                                    std::make_shared<ServiceTransfer>(31, 31, 40, 42, 123, true, update_request_v0)});
    REQUIRE(tx);
    REQUIRE(tx->priority == 31);
    REQUIRE(tx->transfer_id == 31);
    REQUIRE(tx->payload == Buf{255});
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).service_id == 40);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).source_node_id == 123);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).destination_node_id == 42);
    REQUIRE(!dynamic_cast<ServiceTransfer&>(*tx).request_not_response);

    // Validate file read request/response.
    const Buf file_read_request_v1{
        0x00, 0x00, 0x00, 0x00, 0x00, 0x5C, 0x63, 0x6F, 0x6D, 0x2E, 0x7A, 0x75, 0x62, 0x61, 0x78, 0x2E, 0x6B,
        0x6F, 0x63, 0x68, 0x65, 0x72, 0x67, 0x61, 0x2E, 0x74, 0x65, 0x73, 0x74, 0x2E, 0x69, 0x6E, 0x74, 0x65,
        0x67, 0x72, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x2D, 0x31, 0x30, 0x2D, 0x33, 0x2E, 0x31, 0x2E, 0x62, 0x61,
        0x64, 0x63, 0x30, 0x66, 0x66, 0x65, 0x65, 0x30, 0x64, 0x64, 0x66, 0x30, 0x30, 0x64, 0x2E, 0x34, 0x35,
        0x32, 0x61, 0x34, 0x32, 0x36, 0x37, 0x39, 0x37, 0x31, 0x61, 0x33, 0x39, 0x32, 0x38, 0x2E, 0x61, 0x70,
        0x70, 0x2E, 0x72, 0x65, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x2E, 0x62, 0x69, 0x6E,
    };
    // Same but without the length field (removed by TAO):
    const Buf file_read_request_v0{
        0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x6F, 0x6D, 0x2E, 0x7A, 0x75, 0x62, 0x61, 0x78, 0x2E, 0x6B, 0x6F,
        0x63, 0x68, 0x65, 0x72, 0x67, 0x61, 0x2E, 0x74, 0x65, 0x73, 0x74, 0x2E, 0x69, 0x6E, 0x74, 0x65, 0x67,
        0x72, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x2D, 0x31, 0x30, 0x2D, 0x33, 0x2E, 0x31, 0x2E, 0x62, 0x61, 0x64,
        0x63, 0x30, 0x66, 0x66, 0x65, 0x65, 0x30, 0x64, 0x64, 0x66, 0x30, 0x30, 0x64, 0x2E, 0x34, 0x35, 0x32,
        0x61, 0x34, 0x32, 0x36, 0x37, 0x39, 0x37, 0x31, 0x61, 0x33, 0x39, 0x32, 0x38, 0x2E, 0x61, 0x70, 0x70,
        0x2E, 0x72, 0x65, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x2E, 0x62, 0x69, 0x6E,
    };
    REQUIRE(static_cast<kocherga::INode&>(node)
                .sendRequest(ServiceID::FileRead, 42, 22, file_read_request_v1.size(), file_read_request_v1.data()));
    tx = poll(1000us);
    REQUIRE(tx);
    REQUIRE(tx->transfer_id == 22);
    REQUIRE(tx->priority == 30);
    REQUIRE(tx->payload == file_read_request_v0);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).service_id == 48);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).source_node_id == 123);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).destination_node_id == 42);
    REQUIRE(dynamic_cast<ServiceTransfer&>(*tx).request_not_response);
    const Buf file_read_response_v0{
        0x00, 0x00,  // Error code; no length prefix due to TAO.
        0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x20, 0x77, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x20, 0x20, 0x20, 0x20, 0xC7, 0xC4,
        0xC0, 0x6F, 0x14, 0x15, 0x44, 0x5E, 0x41, 0x50, 0x44, 0x65, 0x73, 0x63, 0x30, 0x30, 0x28, 0x39, 0x1A, 0x97,
        0x67, 0x42, 0x2A, 0x45, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x01, 0x00, 0xD2, 0x02,
        0x96, 0x49, 0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x68, 0x65, 0x20, 0x61, 0x70, 0x70, 0x6C, 0x69, 0x63,
        0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x69, 0x6D, 0x61, 0x67, 0x65, 0x20, 0x67, 0x6F, 0x65, 0x73, 0x20, 0x68,
        0x65, 0x72, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
    };
    // Translated response v0 --> v1.
    const Buf file_read_response_v1{
        0x00, 0x00,  // Error code.
        0x90, 0x00,  // Length prefix.
        0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x20, 0x77, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x20, 0x20, 0x20, 0x20, 0xC7, 0xC4,
        0xC0, 0x6F, 0x14, 0x15, 0x44, 0x5E, 0x41, 0x50, 0x44, 0x65, 0x73, 0x63, 0x30, 0x30, 0x28, 0x39, 0x1A, 0x97,
        0x67, 0x42, 0x2A, 0x45, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x01, 0x00, 0xD2, 0x02,
        0x96, 0x49, 0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x68, 0x65, 0x20, 0x61, 0x70, 0x70, 0x6C, 0x69, 0x63,
        0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x69, 0x6D, 0x61, 0x67, 0x65, 0x20, 0x67, 0x6F, 0x65, 0x73, 0x20, 0x68,
        0x65, 0x72, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
    };
    REQUIRE(!poll(1000us));
    REQUIRE(!reactor.popPendingResponse());
    REQUIRE(!poll(1000us,
                  SignatureTransferPair{0x8DCDCA939F33F678ULL,
                                        std::make_shared<
                                            ServiceTransfer>(30, 22, 48, 42, 123, false, file_read_response_v0)}));
    auto response = reactor.popPendingResponse();
    REQUIRE(response);
    REQUIRE(*response == file_read_response_v1);  // Translated correctly.

    // Validate unknown service.
    REQUIRE(!static_cast<kocherga::INode&>(node).sendRequest(ServiceID::NodeExecuteCommand, 124, 22, 0, nullptr));
    REQUIRE(!poll(1000us));
}
