// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_serial.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include <deque>
#include <functional>

namespace
{
class SerialPortMock : public kocherga::serial::ISerialPort
{
public:
    void pushRx(const kocherga::serial::detail::Transfer& transfer)
    {
        REQUIRE(kocherga::serial::detail::transmit(
            [this](const std::uint8_t bt) {
                rx_.push_back(bt);
                return true;
            },
            transfer));
    }

    /// The payload pointer of the returned transfer is invalidated on the 32-nd call to ISerialPort::send().
    [[nodiscard]] auto popTx() -> std::optional<kocherga::serial::detail::Transfer>
    {
        while (!tx_.empty())
        {
            const auto bt = tx_.front();
            tx_.pop_front();
            if (const auto tr = stream_parser_.update(bt))
            {
                return tr;
            }
        }
        return {};
    }

private:
    [[nodiscard]] auto receive() -> std::optional<std::uint8_t> override
    {
        if (!rx_.empty())
        {
            const auto out = rx_.front();
            rx_.pop_front();
            return out;
        }
        return {};
    }

    [[nodiscard]] auto send(const std::uint8_t b) -> bool override
    {
        tx_.push_back(b);
        return true;
    }

    std::deque<std::uint8_t>                     tx_;
    std::deque<std::uint8_t>                     rx_;
    kocherga::serial::detail::StreamParser<1024> stream_parser_;
};

class ReactorMock : public kocherga::IReactor
{
public:
    struct IncomingRequest
    {
        kocherga::PortID          service_id     = 0xFFFF;
        kocherga::NodeID          client_node_id = 0xFFFF;
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
        IncomingRequest ir{service_id, client_node_id, {}};
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

TEST_CASE("kocherga_serial::SerialNode service")
{
    SerialPortMock               port;
    kocherga::serial::SerialNode node(port, {});
    node.setLocalNodeID(2222);
    ReactorMock reactor;

    // Send a request transfer and then pop a response.
    {
        REQUIRE(static_cast<kocherga::INode&>(node).sendRequest(kocherga::ServiceID::NodeExecuteCommand,
                                                                1111,
                                                                0xCAFE'CAFE,
                                                                3,
                                                                reinterpret_cast<const std::uint8_t*>("\x03\x03\x03")));
        const auto request = port.popTx();
        REQUIRE(request);
        REQUIRE(request->meta.source == 2222);
        REQUIRE(request->meta.destination == 1111);
        REQUIRE(request->meta.data_spec == (static_cast<std::uint32_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                            kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag));
        REQUIRE(request->meta.transfer_id == 0xCAFE'CAFE);
        REQUIRE(request->payload_len == 3);
        REQUIRE(0 == std::memcmp(request->payload, "\x03\x03\x03", 3));
        REQUIRE(!port.popTx());  // Ensure no extra requests are sent.
        // Ensure non-matching responses are simply ignored.
        {
            kocherga::serial::detail::Transfer response;
            // BAD SOURCE
            response.meta.source      = 1110;
            response.meta.destination = 2222;
            response.meta.data_spec   = static_cast<std::uint32_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecResponseFlag;
            response.meta.transfer_id = 0xCAFE'CAFE;
            port.pushRx(response);
            static_cast<kocherga::INode&>(node).poll(reactor, std::chrono::microseconds(1'000));
            REQUIRE(!reactor.popPendingResponse());
            // BAD DESTINATION
            response.meta.source      = 1111;
            response.meta.destination = 2221;
            response.meta.data_spec   = static_cast<std::uint32_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecResponseFlag;
            response.meta.transfer_id = 0xCAFE'CAFE;
            port.pushRx(response);
            static_cast<kocherga::INode&>(node).poll(reactor, std::chrono::microseconds(1'000));
            REQUIRE(!reactor.popPendingResponse());
            // BAD DATA SPEC
            response.meta.source      = 1111;
            response.meta.destination = 2222;
            response.meta.data_spec   = static_cast<std::uint32_t>(kocherga::ServiceID::NodeGetInfo) |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecResponseFlag;
            response.meta.transfer_id = 0xCAFE'CAFE;
            port.pushRx(response);
            static_cast<kocherga::INode&>(node).poll(reactor, std::chrono::microseconds(1'000));
            REQUIRE(!reactor.popPendingResponse());
            // BAD TRANSFER ID
            response.meta.source      = 1111;
            response.meta.destination = 2222;
            response.meta.data_spec   = static_cast<std::uint32_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag |
                                      kocherga::serial::detail::Transfer::Metadata::DataSpecResponseFlag;
            response.meta.transfer_id = 0xCAFE'CAFA;
            port.pushRx(response);
            static_cast<kocherga::INode&>(node).poll(reactor, std::chrono::microseconds(1'000));
            REQUIRE(!reactor.popPendingResponse());
        }
        // Receive the matching response transfer.
        kocherga::serial::detail::Transfer response;
        response.meta.source      = 1111;
        response.meta.destination = 2222;
        response.meta.data_spec   = static_cast<std::uint32_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                  kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag |
                                  kocherga::serial::detail::Transfer::Metadata::DataSpecResponseFlag;
        response.meta.transfer_id = 0xCAFE'CAFE;
        response.payload_len      = 6;
        response.payload          = reinterpret_cast<const std::uint8_t*>("\x05\x04\x03\x02\x01\x00");
        port.pushRx(response);  // First response shall be processed.
        port.pushRx(response);  // Deterministic data loss mitigation; shall be ignored.
        port.pushRx(response);  // Same here.
        static_cast<kocherga::INode&>(node).poll(reactor, std::chrono::microseconds(9'000));
        const auto rp = reactor.popPendingResponse();
        REQUIRE(rp);
        REQUIRE(*rp == std::vector<std::uint8_t>{5, 4, 3, 2, 1, 0});
        REQUIRE(!reactor.popPendingResponse());  // Ensure duplicates are removed.
    }

    // Send a request multiple times emulating the deterministic data loss mitigation, ensure repetitions are ignored.
    {
        kocherga::serial::detail::Transfer request;
        request.meta.source      = 1111;
        request.meta.destination = 2222;
        request.meta.data_spec   = static_cast<std::uint16_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                 kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag;
        request.meta.transfer_id = 0xCAFE'BABE;
        request.payload_len      = 6;
        request.payload          = reinterpret_cast<const std::uint8_t*>("\x05\x04\x03\x02\x01\x00");
        port.pushRx(request);  // First request shall be processed.
        port.pushRx(request);  // Deterministic data loss mitigation; shall be ignored.
        port.pushRx(request);  // Same here.
        auto num_requests = 0;
        reactor.setIncomingRequestHandler(
            [&num_requests](const ReactorMock::IncomingRequest& ir) -> std::optional<std::vector<std::uint8_t>> {
                num_requests++;
                REQUIRE(ir.service_id == static_cast<std::uint16_t>(kocherga::ServiceID::NodeExecuteCommand));
                REQUIRE(ir.client_node_id == 1111);
                REQUIRE(ir.data == std::vector<std::uint8_t>{5, 4, 3, 2, 1, 0});
                return std::vector<std::uint8_t>{1, 2, 3, 4, 5};
            });
        static_cast<kocherga::INode&>(node).poll(reactor, std::chrono::microseconds(1'000));
        REQUIRE(num_requests == 1);  // Ensure extras are ignored.
        const auto response = port.popTx();
        REQUIRE(response);
        REQUIRE(response->meta.source == 2222);
        REQUIRE(response->meta.destination == 1111);
        REQUIRE(response->meta.data_spec == (static_cast<std::uint32_t>(kocherga::ServiceID::NodeExecuteCommand) |
                                             kocherga::serial::detail::Transfer::Metadata::DataSpecServiceFlag |
                                             kocherga::serial::detail::Transfer::Metadata::DataSpecResponseFlag));
        REQUIRE(response->meta.transfer_id == 0xCAFE'BABE);
        REQUIRE(response->payload_len == 5);
        REQUIRE(0 == std::memcmp(response->payload, "\x01\x02\x03\x04\x05", 5));
        REQUIRE(!port.popTx());  // Ensure no extra responses are sent.
    }
}
