// This software is distributed under the terms of the MIT License.
// Copyright (c) 2021 Zubax Robotics.
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "kocherga.hpp"
#include "kocherga_serial.hpp"
#include "util.hpp"
#include <cstdint>
#include <iostream>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
/// Tunnels serial port connection via TCP socket as-is without any wrapping.
/// Per Kocherga's API contracts, the API is fully non-blocking.
class TCPSerialPort : public kocherga::serial::ISerialPort
{
public:
    explicit TCPSerialPort(const int sock_fd) : fd_(sock_fd) {}

    ~TCPSerialPort() override { (void) ::close(fd_); }

    TCPSerialPort(const TCPSerialPort&) = delete;
    TCPSerialPort(TCPSerialPort&&)      = delete;
    auto operator=(const TCPSerialPort&) -> TCPSerialPort& = delete;
    auto operator=(TCPSerialPort&&) -> TCPSerialPort& = delete;

    static auto connect(const char* const remote_host, const std::uint16_t remote_port)
        -> std::shared_ptr<TCPSerialPort>
    {
        const ::hostent* const he = gethostbyname(remote_host);
        if (he == nullptr)
        {
            return {};
        }
        ::sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_port   = ::htons(remote_port);
        sa.sin_addr   = *static_cast<const in_addr*>(static_cast<const void*>(he->h_addr));

        const int fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (fd < 0)
        {
            return {};
        }
        if (::connect(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sockaddr)) < 0)
        {
            (void) ::close(fd);
            return {};
        }
        return std::make_shared<TCPSerialPort>(fd);
    }

    [[nodiscard]] auto receive() -> std::optional<std::uint8_t> override
    {
        std::uint8_t out{};
        if (::recv(fd_, &out, 1, MSG_DONTWAIT) > 0)
        {
            return out;
        }
        return {};
    }

    [[nodiscard]] auto send(const std::uint8_t b) -> bool override { return ::send(fd_, &b, 1, MSG_DONTWAIT) > 0; }

private:
    const int fd_;
};

auto initSerialPort() -> std::shared_ptr<kocherga::serial::ISerialPort>
{
    const auto               iface_env = util::getEnvironmentVariable("UAVCAN__SERIAL__IFACE");
    static const std::string Prefix    = "socket://";
    if (iface_env.find(Prefix) != 0)
    {
        throw std::invalid_argument("Expected serial port prefix: " + Prefix);
    }
    const auto endpoint  = iface_env.substr(Prefix.size());
    const auto colon_pos = endpoint.find(':');
    if ((colon_pos == std::string::npos) || ((colon_pos + 1) >= endpoint.size()))
    {
        throw std::invalid_argument("Invalid serial port name format: " + endpoint);
    }
    const auto host     = endpoint.substr(0, colon_pos);
    const auto port_str = endpoint.substr(colon_pos + 1);
    const auto port_raw = std::stoull(port_str, nullptr, 0);
    const auto port     = static_cast<std::uint16_t>(port_raw);
    if (port != port_raw)
    {
        throw std::invalid_argument("Port number invalid: " + port_str);
    }
    return TCPSerialPort::connect(host.c_str(), port);
}

}  // namespace

auto main() -> int
{
    try
    {
        util::FileROMBackend                           file_rom("rom.bin", 2048);
        std::shared_ptr<kocherga::serial::ISerialPort> serial_port = initSerialPort();
        std::cout << "Hello world!" << std::endl;
    }
    catch (std::exception& ex)
    {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
