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
#include <thread>
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
            throw std::runtime_error(std::string("Could not resolve host: ") + remote_host);
        }
        ::sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_port   = ::htons(remote_port);
        sa.sin_addr   = *static_cast<const in_addr*>(static_cast<const void*>(he->h_addr));

        const auto fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (fd < 0)
        {
            throw std::runtime_error("Could not open socket");
        }
        if (::connect(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sockaddr)) < 0)
        {
            (void) ::close(fd);
            throw std::runtime_error("Could not connect to remote endpoint at: " + std::string(remote_host) + ":" +
                                     std::to_string(static_cast<std::uint32_t>(remote_port)));
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

auto getSystemInfo() -> kocherga::SystemInfo
{
    kocherga::SystemInfo system_info{};
    system_info.node_name = "com.zubax.kocherga.test.integration";
    {
        auto       hw_ver = util::getEnvironmentVariable("UAVCAN__NODE__HARDWARE_VERSION");
        const auto maj    = std::stoull(hw_ver.substr(0, hw_ver.find(' ')));
        const auto min    = std::stoull(hw_ver.substr(hw_ver.find(' ') + 1));
        if (maj > std::numeric_limits<std::uint8_t>::max() || min > std::numeric_limits<std::uint8_t>::max())
        {
            throw std::invalid_argument("Hardware version numbers out of range");
        }
        system_info.hardware_version = {static_cast<std::uint8_t>(maj), static_cast<std::uint8_t>(min)};
    }
    {
        const auto uid = util::getEnvironmentVariable("UAVCAN__NODE__UNIQUE_ID");
        if (uid.length() > system_info.unique_id.size())
        {
            throw std::runtime_error("Invalid value length of register uavcan.node.unique_id");
        }
        std::copy(uid.begin(), uid.end(), system_info.unique_id.begin());
    }
    {
        static const auto coa = util::getEnvironmentVariableMaybe("UAVCAN__NODE__CERTIFICATE_OF_AUTHENTICITY");
        if (coa)
        {
            system_info.certificate_of_authenticity_len = static_cast<std::uint8_t>(coa->size());
            system_info.certificate_of_authenticity     = reinterpret_cast<const std::uint8_t*>(coa->data());
        }
    }
    return system_info;
}

}  // namespace

auto main(const int argc, char* const argv[]) -> int
{
    (void) argc;
    try
    {
        const bool linger       = util::getEnvironmentVariable("BOOTLOADER__LINGER") != "0";
        const auto rom_file     = util::getEnvironmentVariable("BOOTLOADER__ROM_FILE");
        const auto rom_size     = std::stoul(util::getEnvironmentVariable("BOOTLOADER__ROM_SIZE"));
        const auto max_app_size = std::stoul(util::getEnvironmentVariable("BOOTLOADER__MAX_APP_SIZE"));

        util::FileROMBackend rom(rom_file, rom_size);

        const auto system_info = getSystemInfo();

        std::shared_ptr<kocherga::serial::ISerialPort> serial_port = initSerialPort();
        kocherga::serial::SerialNode                   serial_node(*serial_port, system_info.unique_id);

        kocherga::Bootloader<1> boot(rom, system_info, {&serial_node}, max_app_size, linger);
        const auto              started_at = std::chrono::steady_clock::now();
        std::clog << "Bootloader started" << std::endl;
        while (true)
        {
            const auto uptime = std::chrono::steady_clock::now() - started_at;
            if (const auto fin = boot.poll(std::chrono::duration_cast<std::chrono::microseconds>(uptime)))
            {
                std::clog << "Final state reached: " << static_cast<std::uint32_t>(*fin) << std::endl;
                if (*fin == kocherga::Final::BootApp)
                {
                    std::clog << "Booting the application" << std::endl;
                    break;
                }
                if (*fin == kocherga::Final::Restart)
                {
                    std::clog << "Restarting the bootloader; using executable " << argv[0] << std::endl;
                    return -::execve(argv[0], argv, ::environ);
                }
                assert(false);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    catch (std::exception& ex)
    {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
