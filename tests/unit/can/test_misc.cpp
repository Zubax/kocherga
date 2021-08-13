// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_can.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "catch.hpp"
#include <numeric>

TEST_CASE("can::CRC")
{
    kocherga::can::detail::CRC16CCITT crc;
    crc.update(3, reinterpret_cast<const std::uint8_t*>("123"));
    REQUIRE(0x5BCEU == crc.get());
    REQUIRE(crc.getBytes().at(0) == 0x5BU);
    REQUIRE(crc.getBytes().at(1) == 0xCEU);
    REQUIRE(!crc.isResidueCorrect());
    crc.update(0x5BU);
    crc.update(0xCEU);
    REQUIRE(crc.isResidueCorrect());
    REQUIRE(0 == crc.get());
}

TEST_CASE("can::BlockAllocator")
{
    kocherga::can::detail::BlockAllocator<8, 2> ba;

    static std::int64_t counter = 0;

    struct Foo final
    {
        const std::int64_t field;

        explicit Foo(const std::int64_t f) : field(f) { counter++; }
        ~Foo() { counter--; }

        [[maybe_unused]] Foo(const Foo&) = delete;
        [[maybe_unused]] Foo(Foo&&)      = delete;
        auto operator=(const Foo&) -> Foo& = delete;
        auto operator=(Foo&&) -> Foo& = delete;
    };

    REQUIRE(counter == 0);
    auto* a = ba.construct<Foo>(1234567890);
    REQUIRE(a);
    REQUIRE(a->field == 1234567890);
    REQUIRE(counter == 1);
    auto* b = ba.construct<Foo>(9876543210);
    REQUIRE(b);
    REQUIRE(b->field == 9876543210);
    REQUIRE(counter == 2);
    auto* c = ba.construct<Foo>(55555);
    REQUIRE(c == nullptr);

    ba.destroy(a);
    REQUIRE(counter == 1);
    c = ba.construct<Foo>(1111111);
    REQUIRE(counter == 2);
    REQUIRE(c);
    REQUIRE(c->field == 1111111);

    ba.destroy(b);
    REQUIRE(counter == 1);
    ba.destroy(c);
    REQUIRE(counter == 0);
}

TEST_CASE("can::parseFrame")
{
    using kocherga::can::detail::parseFrame;
    using kocherga::can::detail::FrameModel;
    using kocherga::can::detail::MessageFrameModel;
    using kocherga::can::detail::ServiceFrameModel;

    static const auto parse = [](const std::uint32_t extended_can_id, const std::vector<std::uint8_t>& payload)
        -> std::optional<std::variant<MessageFrameModel, ServiceFrameModel>> {
        static std::vector<std::uint8_t> payload_storage;
        payload_storage = payload;
        return parseFrame(extended_can_id, payload_storage.size(), payload_storage.data());
    };

    static const auto cmp_payload = [](const FrameModel& fm, const std::vector<std::uint8_t>& reference) {
        return (fm.payload_size == reference.size()) &&
               (0 == std::memcmp(fm.payload, reference.data(), fm.payload_size));
    };
    // Message
    {
        const auto m = std::get<MessageFrameModel>(*parse(0U, {0, 1, 2, 3, 4, 5, 6, 7}));
        REQUIRE(m.priority == 0);
        REQUIRE(m.subject_id == 0);
        REQUIRE(*m.source_node_id == 0);
        REQUIRE(m.transfer_id == 7);
        REQUIRE(!m.start_of_transfer);
        REQUIRE(!m.end_of_transfer);
        REQUIRE(!m.toggle);
        REQUIRE(cmp_payload(m, {0, 1, 2, 3, 4, 5, 6}));
        // Similar but invalid.
        REQUIRE(!parse(0U, {}));                     // No tail byte.
        REQUIRE(!parse(0U, {0}));                    // Multi-frame transfer frames require payload.
        REQUIRE(!parse(0U, {0, 1, 2, 3, 4, 5, 6}));  // Non-last frame of MFT cannot have fewer than 7 bytes of payload.
    }
    // Message
    {
        const auto m = std::get<MessageFrameModel>(
            *parse(0b001'00'0'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
        REQUIRE(m.priority == 1);
        REQUIRE(m.subject_id == 0b0110011001100U);
        REQUIRE(*m.source_node_id == 0b0100111U);
        REQUIRE(m.transfer_id == 23);
        REQUIRE(m.start_of_transfer);
        REQUIRE(!m.end_of_transfer);
        REQUIRE(m.toggle);
        REQUIRE(cmp_payload(m, {0, 1, 2, 3, 4, 5, 6}));
        // Similar but invalid:
        // No tail byte
        REQUIRE(!parse(0b001'00'0'11'0110011001100'0'0100111U, {}));
        // Bad toggle (UAVCAN v0)
        REQUIRE(!parse(0b001'00'0'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b100'00000U | 23U}));
        // Bad reserved r07
        REQUIRE(!parse(0b001'00'0'11'0110011001100'1'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
        // Bad reserved r23
        REQUIRE(!parse(0b001'00'1'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
        // Bad reserved r07 r23
        REQUIRE(!parse(0b001'00'1'11'0110011001100'1'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
        // Anon transfer is not single frame
        REQUIRE(!parse(0b001'01'0'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
    }
    // Anonymous message
    {
        {
            const auto m =
                std::get<MessageFrameModel>(*parse(0b010'01'0'00'0110011001101'0'0100111U, {0b111'00000U | 0U}));
            REQUIRE(m.priority == 2);
            REQUIRE(m.subject_id == 0b0110011001101U);
            REQUIRE(!m.source_node_id);
            REQUIRE(m.transfer_id == 0);
            REQUIRE(m.start_of_transfer);
            REQUIRE(m.end_of_transfer);
            REQUIRE(m.toggle);
            REQUIRE(cmp_payload(m, {}));
        }
        // SAME BUT RESERVED 21 22 SET (and ignored)
        const auto m2 =
            std::get<MessageFrameModel>(*parse(0b010'01'0'11'0110011001101'0'0100111U, {0b111'00000U | 0U}));
        REQUIRE(m2.subject_id == 0b0110011001101U);  // Yup, parsed correctly.
        // Similar but invalid
        REQUIRE(!parse(0b010'01'0'11'0110011001100'0'0100111U, {}));                   // No tail byte
        REQUIRE(!parse(0b010'01'0'11'0110011001100'0'0100111U, {0b110'00000U | 0U}));  // Bad toggle
        REQUIRE(!parse(0b010'01'0'11'0110011001100'1'0100111U, {0b111'00000U | 0U}));  // Bad reserved 07
        REQUIRE(!parse(0b010'01'1'11'0110011001100'0'0100111U, {0b111'00000U | 0U}));  // Bad reserved 23
        REQUIRE(!parse(0b010'01'1'11'0110011001100'1'0100111U, {0b111'00000U | 0U}));  // Bad reserved 07 23
    }
    // Request
    {
        const auto model =
            std::get<ServiceFrameModel>(*parse(0b011'11'0000110011'0011010'0100111U, {0, 1, 2, 3, 0b011'00000U | 31U}));
        REQUIRE(model.priority == 3);
        REQUIRE(model.service_id == 0b0000110011U);
        REQUIRE(model.request_not_response);
        REQUIRE(model.source_node_id == 0b0100111U);
        REQUIRE(model.destination_node_id == 0b0011010U);
        REQUIRE(model.transfer_id == 31U);
        REQUIRE(!model.start_of_transfer);
        REQUIRE(model.end_of_transfer);
        REQUIRE(model.toggle);
        REQUIRE(cmp_payload(model, {0, 1, 2, 3}));
        // Similar but invalid
        REQUIRE(!parse(0b011'11'0000110011'0011010'0100111U, {}));                                // No tail byte
        REQUIRE(!parse(0b011'11'0000110011'0011010'0100111U, {0, 1, 2, 3, 0b110'00000U | 31U}));  // Bad toggle
        REQUIRE(!parse(0b011'11'1000110011'0011010'0100111U, {0, 1, 2, 3, 0b011'00000U | 31U}));  // Bad reserved
        REQUIRE(!parse(0b011'11'0000110011'0100111'0100111U, {0, 1, 2, 3, 0b011'00000U | 31U}));  // Src == destination
    }
    // Response
    {
        const auto model =
            std::get<ServiceFrameModel>(*parse(0b100'10'0000110011'0100111'0011010U, {255, 0b010'00000U | 1U}));
        REQUIRE(model.priority == 4);
        REQUIRE(!model.request_not_response);
        REQUIRE(model.service_id == 0b0000110011U);
        REQUIRE(model.source_node_id == 0b0011010U);
        REQUIRE(model.destination_node_id == 0b0100111U);
        REQUIRE(model.transfer_id == 1U);
        REQUIRE(!model.start_of_transfer);
        REQUIRE(model.end_of_transfer);
        REQUIRE(!model.toggle);
        REQUIRE(cmp_payload(model, {255}));
        // Similar but invalid
        REQUIRE(!parse(0b100'10'0000110011'0100111'0011010U, {}));                        // No tail byte
        REQUIRE(!parse(0b100'10'0000110011'0100111'0011010U, {255, 0b100'00000U | 1U}));  // Bad toggle
        REQUIRE(!parse(0b100'10'1000110011'0100111'0011010U, {255, 0b010'00000U | 1U}));  // Bad reserved
        REQUIRE(!parse(0b100'10'0000110011'0011010'0011010U, {255, 0b010'00000U | 1U}));  // Src == destination
    }
}
