// This software is distributed under the terms of the MIT License.
// Copyright (c) 2020 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga_serial.hpp"  // NOLINT include order: include Kocherga first to ensure no headers are missed.
#include "../util.hpp"
#include "catch.hpp"
#include <iostream>
#include <numeric>

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

TEST_CASE("serial::transmit")
{
    using kocherga::serial::detail::StreamParser;
    using kocherga::serial::detail::transmit;
    using kocherga::serial::detail::Transfer;

    StreamParser<10>             sp;
    std::array<std::uint8_t, 90> buf{};
    std::optional<Transfer>      rx;
    std::vector<std::uint8_t>    history;

    const auto feed = [&sp, &rx, &history](const auto bt) {
        if (!rx)
        {
            history.push_back(bt);
            rx = sp.update(bt);
            return true;
        }
        return false;
    };

    buf = {5, 4, 3, 2, 1};
    REQUIRE(transmit(feed,
                     Transfer{
                         Transfer::Metadata{
                             4,
                             0x8E9E,
                             0x9E8E,
                             0x8042,
                             12'345'678'901'234'567'890ULL,
                         },
                         5,
                         buf.data(),
                     }));
    REQUIRE(rx);
    REQUIRE(rx->meta.priority == 4);
    REQUIRE(rx->meta.source == 0x8E9E);
    REQUIRE(rx->meta.destination == 0x9E8E);
    REQUIRE(rx->meta.data_spec == 0x8042);
    REQUIRE(rx->meta.transfer_id == 12'345'678'901'234'567'890ULL);
    REQUIRE(0x42 == (*rx->meta.isRequest()));
    REQUIRE(!rx->meta.isResponse());
    REQUIRE(std::equal(std::begin(buf), std::begin(buf) + 5, rx->payload));
    std::cout << "transmit:" << std::endl << util::makeHexDump(history) << std::endl;
    const std::vector<std::uint8_t> reference{
        0x9e,                                            // Head delimiter
        0x00,                                            // Version
        0x04,                                            // Priority
        0x8e, 0x61, 0x8e, 0x71,                          // Source with escaping
        0x8e, 0x71, 0x8e, 0x61,                          // Destination with escaping
        0x42, 0x80,                                      // Data specifier
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Data type hash (zero)
        0xd2, 0x0a, 0x1f, 0xeb, 0x8c, 0xa9, 0x54, 0xab,  // Transfer-ID
        0x00, 0x00, 0x00, 0x80,                          // Frame index EOT
        0x9f, 0x8d, 0xa1, 0x1d,                          // Header CRC
        0x05, 0x04, 0x03, 0x02, 0x01,                    // Payload
        0x44, 0xe5, 0x04, 0xd9,                          // Payload CRC
        0x9e,                                            // Tail delimiter
    };
    REQUIRE_THAT(history, Catch::Matchers::Equals(reference));

    // Transmission fails on the first byte because the result is not cleared.
    REQUIRE(!transmit(feed, *rx));
    history.clear();
    rx = {};

    std::uint8_t fail_after = 0;
    const auto   feed_abort = [&sp, &rx, &fail_after](const auto bt) {
        rx = sp.update(bt);
        if (fail_after > 0)
        {
            --fail_after;
            return true;
        }
        return false;
    };

    for (std::uint8_t i = 0U; i < 100U; i++)
    {
        fail_after = i;
        REQUIRE(!transmit(feed_abort,
                          Transfer{
                              Transfer::Metadata{
                                  static_cast<std::uint8_t>(i % 8U),
                                  static_cast<std::uint16_t>(i | 0x8E00U),
                                  static_cast<std::uint16_t>(i | 0x9E00U),
                                  0x8E9EU,
                                  0x9E8E'9E8EU,
                              },
                              std::min<std::size_t>(buf.size(), i),
                              buf.data(),
                          }));
        REQUIRE(!rx);
    }
}
