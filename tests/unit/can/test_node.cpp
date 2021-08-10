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

private:
    [[nodiscard]] auto configure(const Bitrate& bitrate, const bool silent, const CANAcceptanceFilterConfig& filter)
        -> std::optional<Mode> override
    {
        config_ = {bitrate, silent, filter};
        return mode_;
    }

    [[nodiscard]] auto push(const bool          force_classic_can,
                            const std::uint32_t extended_can_id,
                            const std::uint8_t  payload_size,
                            const void* const   payload) -> bool override
    {
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
        if (!rx_.empty())
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
    // Detect v1 if both v0 and v1 are present at the same time.
    {
        act = std::make_shared<ProtocolVersionDetectionActivity>(alloc,
                                                                 driver,
                                                                 kocherga::SystemInfo::UniqueID{},
                                                                 Bitrate{1'000'000, 4'000'000});
        REQUIRE(!driver.getConfig());
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000)));
        REQUIRE(driver.getConfig()->bitrate == Bitrate{1'000'000, 4'000'000});
        REQUIRE(driver.getConfig()->silent);
        REQUIRE(driver.getConfig()->filter == CANAcceptanceFilterConfig{0, 0});
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
    }
    // Detect v0 if only v0 is present.
    {
        act = std::make_shared<ProtocolVersionDetectionActivity>(alloc,
                                                                 driver,
                                                                 kocherga::SystemInfo::UniqueID{},
                                                                 Bitrate{1'000'000, 4'000'000});
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000)));
        REQUIRE(driver.getConfig()->bitrate == Bitrate{1'000'000, 4'000'000});
        REQUIRE(driver.getConfig()->silent);
        REQUIRE(driver.getConfig()->filter == CANAcceptanceFilterConfig{0, 0});
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(1'000'000)));
        driver.pushRx(CANDriverMock::Frame{123456, {0, 1, 2, 3, 0b1000'0000}});  // UAVCAN/CAN v0
        REQUIRE(!act->poll(reactor, std::chrono::microseconds(2'000'000)));
        IActivity* const v0_pnp_act = act->poll(reactor, std::chrono::microseconds(4'000'000));
        REQUIRE(v0_pnp_act);
        REQUIRE(dynamic_cast<V0NodeIDAllocationActivity*>(v0_pnp_act));
    }
}
