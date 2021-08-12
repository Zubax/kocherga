// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_can.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include <algorithm>
#include <deque>

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
        return {};
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
    using IncomingRequestHandler = std::function<std::optional<std::vector<std::uint8_t>>(IncomingRequest)>;

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
    using kocherga::can::detail::ProtocolVersionDetectionActivity;
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
    REQUIRE(driver.getConfig()->filter == CANAcceptanceFilterConfig{0, 0});
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
    REQUIRE(driver.getConfig()->filter == CANAcceptanceFilterConfig{0, 0});
    REQUIRE(dynamic_cast<ProtocolVersionDetectionActivity*>(proto_ver_act));
}

TEST_CASE("can::detail::ProtocolVersionDetectionActivity")
{
    using kocherga::can::detail::IActivity;
    using kocherga::can::detail::ProtocolVersionDetectionActivity;
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
        act = std::make_shared<ProtocolVersionDetectionActivity>(alloc, driver, kocherga::SystemInfo::UniqueID{}, br);
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
        act = std::make_shared<ProtocolVersionDetectionActivity>(alloc, driver, kocherga::SystemInfo::UniqueID{}, br);
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
        driver.pushRx({0x14'0001'7FUL, {0x00, 0x35, 0xFF, 0xD5, 0x05, 0x50, 0x59, 0b1100'0000U}});
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
        driver.pushRx({0x14'0001'7FUL, {0x9E, 0x3D, 0x00, 0x35, 0xFF, 0xD5, 0x05, 0x81}});
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});  // Ignore duplicate
        driver.pushRx({0x14'0002'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});  // Ignore unrelated
        driver.pushRx({0x14'0001'7EUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x21}});  // Ignore wrong sender
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x22}});  // Ignore wrong transfer-ID
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x41}});
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x41}});  // Ignore duplicate
        REQUIRE(!act.poll(reactor, deadline_d));
        REQUIRE(!driver.popTx());
        REQUIRE(2 == act.getStage());
        const auto deadline_e = act.getDeadline();
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
        driver.pushRx({0x14'0001'7FUL, {0x8C, 0x7A, 0xFA, 0x35, 0xFF, 0xD5, 0x05, 0x82}});  // Correct CRC here.
        driver.pushRx({0x14'0001'7FUL, {0x50, 0x59, 0x31, 0x34, 0x61, 0x41, 0x23, 0x22}});
        driver.pushRx({0x14'0001'7FUL, {0x43, 0x00, 0x00, 0x00, 0x00, 0x42}});
        auto* const new_act = act.poll(reactor, deadline_e);
        REQUIRE(new_act != nullptr);
        REQUIRE(!driver.popTx());
        REQUIRE(dynamic_cast<V0MainActivity*>(new_act));
    }
}