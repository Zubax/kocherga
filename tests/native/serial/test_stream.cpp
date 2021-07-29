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

        std::vector<std::uint8_t>                      out;
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
        const std::vector<std::uint8_t> reference = {
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
        std::vector<std::uint8_t> history;
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
        const std::vector<std::uint8_t> reference = {
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
        std::vector<std::uint8_t> history;
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

TEST_CASE("serial::StreamParser")
{
    using kocherga::serial::detail::StreamParser;
    using kocherga::serial::detail::Transfer;

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

    const auto get_crc = [](const std::vector<std::uint8_t>& data) {
        kocherga::serial::detail::CRC32C crc;
        for (const auto x : data)
        {
            crc.update(x);
        }
        return crc;
    };

    // VALID MESSAGE
    std::vector<std::uint8_t> chunk = {
        0,                                               // Version
        5,                                               // Priority
        0x7B, 0x00,                                      // Source NID        123
        0xC8, 0x01,                                      // Destination NID   456
        0xE1, 0x10,                                      // Data specifier    4321
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    chunk = {1, 2, 3, 4, 5};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    auto tr = feed(std::vector<std::uint8_t>{0x9E});
    REQUIRE(tr);
    REQUIRE(tr->payload_len == 5);
    REQUIRE(std::equal(std::begin(chunk), std::end(chunk), tr->payload));
    REQUIRE(tr->meta.priority == 5);
    REQUIRE(tr->meta.source == 123);
    REQUIRE(tr->meta.destination == 456);
    REQUIRE(tr->meta.data_spec == 4321);
    REQUIRE(tr->meta.transfer_id == 12'345'678'901'234'567'890ULL);
    REQUIRE(!tr->meta.isRequest());
    REQUIRE(!tr->meta.isResponse());

    // VALID RESPONSE
    chunk = {
        0,                                               // Version
        2,                                               // Priority
        0x8E, 0x71, 0x01,                                // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0x61, 0xC0,                                // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    // No frame delimiter here.
    REQUIRE(!feed(chunk));
    chunk = {
        // Same as above but unescaped to compute the CRC.
        0,                                               // Version
        2,                                               // Priority
        0x8E, 0x01,                                      // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x9E, 0xC0,                                      // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    chunk = {0x8E, 0x61};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc({0x9E}).getBytes()));
    tr = feed(std::vector<std::uint8_t>{0x9E});
    REQUIRE(tr);
    REQUIRE(tr->payload_len == 1);
    REQUIRE((*tr->payload) == 0x9E);  // Escaped
    REQUIRE(tr->meta.priority == 2);
    REQUIRE(tr->meta.source == 398);
    REQUIRE(tr->meta.destination == tr->meta.AnonymousNodeID);
    REQUIRE(tr->meta.data_spec == 0xC09E);
    REQUIRE(tr->meta.transfer_id == 12'345'678'901'234'567'890ULL);
    REQUIRE(!tr->meta.isRequest());
    REQUIRE(tr->meta.isResponse() == 158);

    // BAD VERSION
    chunk = {
        123,                                             // Version
        3,                                               // Priority
        0x8E, 0x71, 0x01,                                // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0x61, 0xC0,                                // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    // No frame delimiter here.
    REQUIRE(!feed(chunk));
    chunk = {
        // Same as above but unescaped to compute the CRC.
        123,                                             // Version
        3,                                               // Priority
        0x8E, 0x02,                                      // Source NID        (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0xC0,                                      // Data specifier    response (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    chunk = {0x8E, 0x61};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc({0x9E}).getBytes()));
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));

    // MULTIFRAME TRANSFERS NOT SUPPORTED
    chunk = {
        0,                                               // Version
        4,                                               // Priority
        0x8E, 0x71, 0x01,                                // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0x61, 0xC0,                                // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x00,                          // Frame index, EOT  0
    };
    // No frame delimiter here.
    REQUIRE(!feed(chunk));
    chunk = {
        // Same as above but unescaped to compute the CRC.
        0,                                               // Version
        4,                                               // Priority
        0x8E, 0x01,                                      // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x9E, 0xC0,                                      // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x00,                          // Frame index, EOT  0
    };
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    chunk = {0x8E, 0x61};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc({0x9E}).getBytes()));
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));

    // PAYLOAD TOO LARGE
    chunk = {
        0,                                               // Version
        6,                                               // Priority
        0x8E, 0x71, 0x01,                                // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0x61, 0xC0,                                // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    // No frame delimiter here.
    REQUIRE(!feed(chunk));
    chunk = {
        // Same as above but unescaped to compute the CRC.
        0,                                               // Version
        6,                                               // Priority
        0x8E, 0x01,                                      // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x9E, 0xC0,                                      // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    chunk = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));

    // DOUBLE ESCAPING
    chunk = {
        0,                                               // Version
        7,                                               // Priority
        0x8E, 0x8E, 0x71, 0x01,                          // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0x61, 0xC0,                                // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    // No frame delimiter here.
    REQUIRE(!feed(chunk));
    chunk = {
        // Same as above but unescaped to compute the CRC.
        0,                                               // Version
        7,                                               // Priority
        0x8E, 0x01,                                      // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x9E, 0xC0,                                      // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
    };
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    chunk = {};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));

    // HEADER CRC ERROR
    chunk = {
        0,                                               // Version
        5,                                               // Priority
        0x8E, 0x71, 0x01,                                // Source NID        398 (escaped)
        0xFF, 0xFF,                                      // Destination NID   broadcast
        0x8E, 0x61, 0xC0,                                // Data specifier    response 158 (escaped)
        0x0D, 0xF0, 0xDD, 0xE0, 0xFE, 0x0F, 0xDC, 0xBA,  // Data type hash    0xbad_c0ffee_0dd_f00d
        0xD2, 0x0A, 0x1F, 0xEB, 0x8C, 0xA9, 0x54, 0xAB,  // Transfer ID       12345678901234567890
        0x00, 0x00, 0x00, 0x80,                          // Frame index, EOT  0 with EOT flag set
        0,    0,    0,    0,                             // Invalid header CRC
    };
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));  // Frame delimiter
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));  // Or maybe two.
    REQUIRE(!feed(chunk));
    chunk = {};
    REQUIRE(!feed(chunk));
    REQUIRE(!feed(get_crc(chunk).getBytes()));
    REQUIRE(!feed(std::vector<std::uint8_t>{0x9E}));
}
