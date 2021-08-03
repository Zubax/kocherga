// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_serial.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "../util.hpp"
#include "catch.hpp"
#include <iostream>
#include <numeric>

TEST_CASE("serial::COBSEncoder")
{
    using kocherga::serial::detail::COBSEncoder;
    using Buf = std::vector<std::uint8_t>;

    struct Rig
    {
        Rig() :
            enc([this](const std::uint8_t x) {
                this->out.push_back(x);
                return true;
            })
        {}

        Buf                                            out;
        COBSEncoder<std::function<bool(std::uint8_t)>> enc;
    };

    {  // import cobs.cobs; cobs.cobs.encode(b'\x00')
        Rig rig;
        REQUIRE(rig.enc.push(0));
        REQUIRE(rig.enc.end());
        REQUIRE(rig.out == Buf{0, 1, 1, 0});
    }

    {  // import cobs.cobs; cobs.cobs.encode(b'\x01')
        Rig rig;
        REQUIRE(rig.enc.push(1));
        REQUIRE(rig.enc.end());
        REQUIRE(rig.out == Buf{0, 2, 1, 0});
    }

    {  // Reference taken from https://github.com/cmcqueen/cobs-c
        Rig rig;
        REQUIRE(rig.enc.push(0x2F));
        REQUIRE(rig.enc.push(0xA2));
        REQUIRE(rig.enc.push(0x00));
        REQUIRE(rig.enc.push(0x92));
        REQUIRE(rig.enc.push(0x73));
        REQUIRE(rig.enc.push(0x02));
        REQUIRE(rig.enc.end());
        REQUIRE(rig.out == Buf{0x00, 0x03, 0x2F, 0xA2, 0x04, 0x92, 0x73, 0x02, 0x00});
    }

    {  // import cobs.cobs; cobs.cobs.encode(b'\x01' * 600)
        Rig rig;
        for (auto i = 0; i < 600; i++)
        {
            REQUIRE(rig.enc.push(1));
        }
        REQUIRE(rig.enc.end());
        const Buf ref{
            0,    // head delimiter
            255,  // length code
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  //
            255,                                                                                         // length code
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  //
            93,                                                                                          // length code
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  //
            0,                                                                                     // tail delimiter
        };
        assert(ref.size() == 603 + 2);
        REQUIRE(rig.out == ref);
    }
}

TEST_CASE("serial::transmit")
{
    using kocherga::serial::detail::transmit;
    using kocherga::serial::detail::Transfer;
    using Buf = std::vector<std::uint8_t>;

    // The reference dump has been obtained as follows:
    // import pyuavcan.serial
    // tr = pyuavcan.transport.serial.SerialTransport('loop://', local_node_id=1234, baudrate=115200)
    // pm = pyuavcan.transport.PayloadMetadata(1024)
    // ds = pyuavcan.transport.MessageDataSpecifier(2345)
    // pub = tr.get_output_session(pyuavcan.transport.OutputSessionSpecifier(ds, None), pm)
    // caps = []
    // tr.begin_capture(caps.append)
    // pub.send(pyuavcan.transport.Transfer(pyuavcan.transport.Timestamp.now(), pyuavcan.transport.Priority.LOW,
    //                                      1111, fragmented_payload=[]),
    //          tr.loop.time() + 1.0)
    // print(caps)
    {
        const Buf reference = {
            0x00,                                            // starting delimiter
            0x01,                                            // COBS starting stuff byte, next byte zero
            0x08,                                            // version 0, next zero 8 bytes later
            0x05,                                            // priority low
            0xd2, 0x04,                                      // src node-ID  1234
            0xff, 0xff,                                      // dst node-ID  broadcast
            0x29, 0x09,                                      // subject-ID   2345
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03,  // reserved zeros
            0x57, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,  // transfer-ID  1111
            0x01, 0x01, 0x06, 0x80,                          // frame index EOT 0x80000000 (single-frame transfer)
            0x02, 0xf4, 0x6f, 0x2a,                          // header CRC  0x2a6ff402
            0x01, 0x01, 0x01, 0x01,                          // payload CRC 0x00000000
            0x00                                             // final delimiter
        };
        Buf history;
        REQUIRE(transmit(
            [&history](const auto bt) {
                history.push_back(bt);
                return true;
            },
            Transfer{
                Transfer::Metadata{
                    5,
                    1234,
                    Transfer::Metadata::AnonymousNodeID,  // Broadcast
                    2345,
                    1111,
                },
                0,
                nullptr,
            }));
        REQUIRE(reference == history);
    }

    // The second test is like above but with the payload set to b'\x00\x01\x02'
    {
        const Buf reference = {
            0x00,                                            // starting delimiter
            0x01,                                            // COBS starting stuff byte, next byte zero
            0x08,                                            // version 0, next zero 8 bytes later
            0x05,                                            // priority low
            0xd2, 0x04,                                      // src node-ID  1234
            0xff, 0xff,                                      // dst node-ID  broadcast
            0x29, 0x09,                                      // subject-ID   2345
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03,  // reserved zeros
            0x57, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,  // transfer-ID  1111
            0x01, 0x01, 0x06, 0x80,                          // frame index EOT 0x80000000 (single-frame transfer)
            0x02, 0xf4, 0x6f, 0x2a,                          // header CRC  0x2a6ff402
            0x07, 0x01, 0x02,                                // payload [0, 1, 2]
            0xfa, 0x4b, 0xfd, 0x92,                          // payload CRC
            0x00,                                            // final delimiter
        };
        Buf history;
        REQUIRE(transmit(
            [&history](const auto bt) {
                history.push_back(bt);
                return true;
            },
            Transfer{
                Transfer::Metadata{
                    5,
                    1234,
                    Transfer::Metadata::AnonymousNodeID,  // Broadcast
                    2345,
                    1111,
                },
                3,
                reinterpret_cast<const std::uint8_t*>("\x00\x01\x02"),
            }));
        REQUIRE(reference == history);
    }

    // Failure case
    {
        std::array<std::uint8_t, 300> buf{};
        std::uint32_t                 fail_after = 0;
        std::uint32_t                 num_sent   = 0;
        const auto                    feed_abort = [&fail_after, &num_sent](const auto bt) {
            (void) bt;
            if (fail_after > 0)
            {
                --fail_after;
                ++num_sent;
                return true;
            }
            return false;
        };
        for (auto i = 0U; i < 300U; i++)
        {
            num_sent   = 0;
            fail_after = i;
            REQUIRE(!transmit(feed_abort,
                              Transfer{
                                  Transfer::Metadata{
                                      static_cast<std::uint8_t>(i % 8U),
                                      static_cast<std::uint16_t>(i | 0x8E00U),
                                      static_cast<std::uint16_t>(i | 0x9E00U),
                                      0,
                                      0,
                                  },
                                  std::min<std::size_t>(buf.size(), i),
                                  buf.data(),
                              }));
            REQUIRE(num_sent == i);
        }
    }
}

TEST_CASE("serial::COBSDecoder")
{
    using kocherga::serial::detail::COBSDecoder;
    using Buf = std::vector<std::uint8_t>;

    COBSDecoder dec;

    // If we only feed delimiters, we get empty packets out.
    REQUIRE(std::holds_alternative<COBSDecoder::Delimiter>(dec.feed(0)));
    REQUIRE(std::holds_alternative<COBSDecoder::Delimiter>(dec.feed(0)));

    const auto decode = [&dec](const auto encoded) -> std::optional<Buf> {
        Buf out;
        for (std::uint8_t bt : encoded)
        {
            const auto res = dec.feed(bt);
            if (const auto* out_byte = std::get_if<std::uint8_t>(&res))
            {
                out.push_back(*out_byte);
            }
            else if (std::holds_alternative<COBSDecoder::Delimiter>(res))
            {
                return out;
            }
            else
            {
                assert(std::holds_alternative<COBSDecoder::Nothing>(res));
            }
        }
        return {};  // Return empty because unterminated.
    };

    // Valid packets.
    REQUIRE(*decode(Buf{1, 1, 0}) == Buf{0});
    REQUIRE(*decode(Buf{2, 1, 0}) == Buf{1});
    REQUIRE(decode(Buf{0})->empty());
    REQUIRE(*decode(Buf{0x03, 0x2F, 0xA2, 0x04, 0x92, 0x73, 0x02, 0x00}) == Buf{0x2F, 0xA2, 0x00, 0x92, 0x73, 0x02});
    {
        const Buf input{
            255,  // length code
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  //
            255,                                                                                         // length code
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  //
            93,                                                                                          // length code
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  //
            0,                                                                                     // tail delimiter
        };
        assert(input.size() == 603 - 1 + 2);
        const Buf ref(600, 1);
        REQUIRE(*decode(input) == ref);
    }
    // Malformed packets are not detected, best effort handling.
    REQUIRE(decode(Buf{3, 0})->empty());
    // Successful recovery.
    REQUIRE(*decode(Buf{0x03, 0x2F, 0xA2, 0x04, 0x92, 0x73, 0x02, 0x00}) == Buf{0x2F, 0xA2, 0x00, 0x92, 0x73, 0x02});
}

TEST_CASE("serial::COBS* roundtrip")
{
    using kocherga::serial::detail::COBSDecoder;
    using kocherga::serial::detail::COBSEncoder;
    using Buf = std::vector<std::uint8_t>;

    static const auto synth_buf = []() -> Buf {
        Buf out(util::getRandomInteger<std::uint16_t>());
        for (auto& x : out)
        {
            x = util::getRandomInteger<std::uint8_t>();
        }
        return out;
    };

    // Generate many random packets.
    std::vector<Buf> original_packets(100);
    for (auto& x : original_packets)
    {
        x = synth_buf();
    }

    // Serialize all packets into one stream representing the transmission medium.
    Buf encoded_stream;
    for (const auto& in : original_packets)
    {
        COBSEncoder<std::function<bool(std::uint8_t)>> enc([&encoded_stream](const std::uint8_t x) {
            encoded_stream.push_back(x);
            return true;
        });
        for (auto x : in)
        {
            REQUIRE(enc.push(x));
        }
        REQUIRE(enc.end());
    }
    std::cout << "Encoded stream length: " << encoded_stream.size() << std::endl;

    // Deserialize the packets back from the stream.
    std::vector<Buf> decoded_packets;
    COBSDecoder      dec;
    Buf              out;
    for (const auto& bt : encoded_stream)
    {
        const auto res = dec.feed(bt);
        if (const auto* out_byte = std::get_if<std::uint8_t>(&res))
        {
            out.push_back(*out_byte);
        }
        else if (std::holds_alternative<COBSDecoder::Delimiter>(res))
        {
            if (!out.empty())
            {
                decoded_packets.push_back(out);
                out.clear();
            }
        }
        else
        {
            assert(std::holds_alternative<COBSDecoder::Nothing>(res));
        }
    }

    // Compare the packets we've just deserialized against the original ones.
    REQUIRE(decoded_packets.size() == original_packets.size());
    REQUIRE(decoded_packets == original_packets);
}

TEST_CASE("serial::StreamParser basic")
{
    using kocherga::serial::detail::StreamParser;
    using kocherga::serial::detail::Transfer;
    using Buf = std::vector<std::uint8_t>;

    StreamParser<10> sp;

    const auto feed = [&sp](const auto& data) {
        std::optional<Transfer> out;
        for (const std::uint8_t x : data)
        {
            REQUIRE(!out);
            out = sp.update(x);
        }
        return out;
    };

    // VALID MESSAGE
    // crc = lambda x: pyuavcan.transport.commons.crc.CRC32C.new(x).value_as_bytes
    // ', '.join(f'0x{x:02x}' for x in cobs.cobs.encode(bytes(hdr) + crc(hdr) + bytes(payload) + crc(bytes(payload))))
    {
        const Buf chunk{
            0x01,                                            // COBS starting overhead byte
            0x03,                                            // Version           0
            0x05,                                            // Priority          5
            0x7b, 0x15,                                      // Source NID        123
            0xff, 0xff,                                      // Destination NID   456
            0xe1, 0x10,                                      // Data specifier    4321
            0x0d, 0xf0, 0xdd, 0xe0, 0xfe, 0x0f, 0xdc, 0xba,  //
            0xd2, 0x0a, 0x1f, 0xeb, 0x8c, 0xa9, 0x54, 0xab,  // Transfer ID       12345678901234567890
            0x01, 0x01, 0x0f, 0x80,                          // Frame index, EOT  0 with EOT flag set
            0xad, 0x13, 0xce, 0xc6,                          // Header CRC computed with the help of PyUAVCAN
            0x01, 0x02, 0x03, 0x04, 0x05,                    // Payload 1 2 3 4 5
            0xab, 0x8f, 0x51, 0x53,                          // Payload CRC
        };
        REQUIRE(!feed(Buf{0}));
        REQUIRE(!feed(chunk));
        const auto tr = feed(Buf{0});
        REQUIRE(tr);
        REQUIRE(tr->payload_len == 5);
        REQUIRE(0 == std::memcmp("\x01\x02\x03\x04\x05", tr->payload, 5));
        REQUIRE(tr->meta.priority == 5);
        REQUIRE(tr->meta.source == 123);
        REQUIRE(tr->meta.destination == Transfer::Metadata::AnonymousNodeID);
        REQUIRE(tr->meta.data_spec == 4321);
        REQUIRE(tr->meta.transfer_id == 12'345'678'901'234'567'890ULL);
        REQUIRE(!tr->meta.isRequest());
        REQUIRE(!tr->meta.isResponse());
    }

    // VALID RESPONSE
    {
        const Buf chunk = {
            0x01,
            0x18,                                            // Version           0
            0x02,                                            // Priority          1
            0x8e, 0x01,                                      // Source NID        398
            0x11, 0x01,                                      // Destination NID   273
            0x9e, 0xc0,                                      // Data specifier    response 158
            0x0d, 0xf0, 0xdd, 0xe0, 0xfe, 0x0f, 0xdc, 0xba,  //
            0xd2, 0x0a, 0x1f, 0xeb, 0x8c, 0xa9, 0x54, 0xab,  // Transfer ID       12345678901234567890
            0x01, 0x01, 0x0f, 0x80,                          // Frame index, EOT  0 with EOT flag set
            0x50, 0x42, 0x9b, 0x1f,                          // Header CRC
            0x01, 0x02, 0x03, 0x04, 0x05,                    // Payload           1 2 3 4 5
            0xab, 0x8f, 0x51, 0x53                           // Payload CRC
        };
        // Empty frames ignored.
        REQUIRE(!feed(Buf{0}));
        REQUIRE(!feed(Buf{0}));
        REQUIRE(!feed(chunk));
        const auto tr = feed(Buf{0});
        REQUIRE(tr);
        REQUIRE(tr->payload_len == 5);
        REQUIRE(0 == std::memcmp("\x01\x02\x03\x04\x05", tr->payload, 5));
        REQUIRE(tr->meta.priority == 2);
        REQUIRE(tr->meta.source == 398);
        REQUIRE(tr->meta.destination == 273);
        REQUIRE(tr->meta.data_spec == 0xC09E);
        REQUIRE(tr->meta.transfer_id == 12'345'678'901'234'567'890ULL);
        REQUIRE(!tr->meta.isRequest());
        REQUIRE(tr->meta.isResponse() == 158);
    }

    // BAD HEADER CRC
    {
        const Buf chunk = {
            0x01,
            0x18,                                            // Version           0
            0x02,                                            // Priority          1
            0x8e, 0x01,                                      // Source NID        398
            0x11, 0x01,                                      // Destination NID   273
            0x9e, 0xc0,                                      // Data specifier    response 158
            0x0d, 0xf0, 0xdd, 0xe0, 0xfe, 0x0f, 0xdc, 0xba,  //
            0xd2, 0x0a, 0x1f, 0xeb, 0x8c, 0xa9, 0x54, 0xab,  // Transfer ID       12345678901234567890
            0x01, 0x01, 0x0f, 0x80,                          // Frame index, EOT  0 with EOT flag set
            0x50, 0x42, 0x9b, 0x0f,                          // Header CRC        MSB FLIP ERROR
            0x01, 0x02, 0x03, 0x04, 0x05,                    // Payload           1 2 3 4 5
            0xab, 0x8f, 0x51, 0x53                           // Payload CRC
        };
        REQUIRE(!feed(Buf{0}));
        REQUIRE(!feed(chunk));
        REQUIRE(!feed(Buf{0}));  // CRC ERROR
    }

    // BAD PAYLOAD CRC
    {
        const Buf chunk = {
            0x01,
            0x18,                                            // Version           0
            0x02,                                            // Priority          1
            0x8e, 0x01,                                      // Source NID        398
            0x11, 0x01,                                      // Destination NID   273
            0x9e, 0xc0,                                      // Data specifier    response 158
            0x0d, 0xf0, 0xdd, 0xe0, 0xfe, 0x0f, 0xdc, 0xba,  //
            0xd2, 0x0a, 0x1f, 0xeb, 0x8c, 0xa9, 0x54, 0xab,  // Transfer ID       12345678901234567890
            0x01, 0x01, 0x0f, 0x80,                          // Frame index, EOT  0 with EOT flag set
            0x50, 0x42, 0x9b, 0x1f,                          // Header CRC
            0x01, 0x02, 0x03, 0x04, 0x05,                    // Payload           1 2 3 4 5
            0xaa, 0x8f, 0x51, 0x53                           // Payload CRC       LSB FLIP ERROR
        };
        REQUIRE(!feed(Buf{0}));
        REQUIRE(!feed(chunk));
        REQUIRE(!feed(Buf{0}));  // CRC ERROR
    }
}

TEST_CASE("serial::StreamParser error")
{
    using kocherga::serial::detail::StreamParser;
    using kocherga::serial::detail::Transfer;
    using Buf = std::vector<std::uint8_t>;

    StreamParser<10> sp;

    const auto feed = [&sp](const Buf& header, const Buf& payload) {
        std::optional<Transfer>               out;
        kocherga::serial::detail::COBSEncoder enc([&out, &sp](const std::uint8_t x) {
            REQUIRE(!out);
            out = sp.update(x);
            return true;
        });
        const auto                            inject = [&enc](const auto& data) {
            kocherga::serial::detail::CRC32C crc_computer;
            for (const std::uint8_t x : data)
            {
                crc_computer.update(x);
                REQUIRE(enc.push(x));
            }
            for (const std::uint8_t x : crc_computer.getBytes())
            {
                REQUIRE(enc.push(x));
            }
        };
        inject(header);
        inject(payload);
        REQUIRE(enc.end());
        return out;
    };

    // Self-test with max length payload
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0x8a, 0x0c,                                      // Destination NID
            0x4d, 0x81,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9});
        REQUIRE(tr);
        REQUIRE(tr->meta.priority == 7);
        REQUIRE(tr->meta.source == 1234);
        REQUIRE(tr->meta.destination == 3210);
        REQUIRE(tr->meta.data_spec == (Transfer::Metadata::DataSpecServiceFlag | 333U));
        REQUIRE(tr->meta.transfer_id == 0x1234'5678'9012'3456ULL);
        REQUIRE(tr->payload_len == 10);
        REQUIRE(0 == std::memcmp(tr->payload, "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09", 10));
        REQUIRE(tr->meta.isRequest() == 333);
        REQUIRE(!tr->meta.isResponse());
    }

    // Self-test with zero length payload
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0x8a, 0x0c,                                      // Destination NID
            0x4d, 0xc1,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {});
        REQUIRE(tr);
        REQUIRE(tr->meta.priority == 7);
        REQUIRE(tr->meta.source == 1234);
        REQUIRE(tr->meta.destination == 3210);
        REQUIRE(tr->meta.data_spec ==
                (333U | Transfer::Metadata::DataSpecServiceFlag | Transfer::Metadata::DataSpecResponseFlag));
        REQUIRE(tr->meta.transfer_id == 0x1234'5678'9012'3456ULL);
        REQUIRE(tr->payload_len == 0);
        REQUIRE(!tr->meta.isRequest());
        REQUIRE(tr->meta.isResponse() == 333);
    }

    // Payload one byte too long
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0x8a, 0x0c,                                      // Destination NID
            0x4d, 0x81,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
        REQUIRE(!tr);
    }

    // Bad version
    {
        const Buf header{
            0x01,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0x8a, 0x0c,                                      // Destination NID
            0x4d, 0x81,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {});
        REQUIRE(!tr);
    }

    // UAVCAN/serial transfers cannot be multi-frame
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0x8a, 0x0c,                                      // Destination NID
            0x4d, 0x81,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x00,                          // Frame index, EOT (cleared)
        };
        const auto tr = feed(header, {});
        REQUIRE(!tr);
    }

    // Service transfers cannot be broadcast
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0xff, 0xff,                                      // Destination NID
            0x4d, 0xc1,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {});
        REQUIRE(!tr);
    }

    // Service transfers cannot be anonymous
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xff, 0xff,                                      // Source NID
            0x01, 0x01,                                      // Destination NID
            0x4d, 0xc1,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {});
        REQUIRE(!tr);
    }

    // Message transfers cannot be unicast
    {
        const Buf header{
            0x00,                                            // Version
            0x07,                                            // Priority
            0xd2, 0x04,                                      // Source NID
            0x8a, 0x0c,                                      // Destination NID
            0x4d, 0x01,                                      // Data specifier
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
            0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12,  // Transfer ID
            0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT
        };
        const auto tr = feed(header, {});
        REQUIRE(!tr);
    }
}
