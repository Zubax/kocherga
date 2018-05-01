# Kochergá

[![Join the chat at https://gitter.im/Zubax/general](https://img.shields.io/badge/GITTER-join%20chat-green.svg)](https://gitter.im/Zubax/general)
[![Travis CI build status](https://travis-ci.org/Zubax/kocherga.svg?branch=master)](https://travis-ci.org/Zubax/kocherga)

**Kochergá is a robust portable multi-protocol bootloader for deeply embedded systems.**

## Features

### Portability

Kochergá has no external dependencies
(although protocol implementations may require their own dependencies).

Kocherga is written in standard C++17 and is distributed as a header-only library.

### Robustness

Kochergá is brick-proof.

The firmware update process can be interrupted at any point (e.g. by turning off the power supply
or by disconnecting the interface), and it is guaranteed that the device will always end up in
a known valid state.
No matter what happens during the update process, Kochergá won't let the user brick the device.

Even if a misbehaving firmware image was uploaded, Kochergá always can take control and let the user replace it.

### Security

Kochergá verifies the correctness of the firmware image with a strong 64-bit hash function
before every boot.

### Supported protocols

Kochergá supports several communication interfaces and protocols:

Interface           | Protocols
--------------------|------------------------------------------------------------------------------
Serial (USB/UART)   | XMODEM, YMODEM, XMODEM-1K, Popcop
CAN bus             | UAVCAN

## Usage

*Come back later.*

![Kocherga State Machine Diagram](state_machine.svg "Kocherga State Machine Diagram")

## License

Kochergá is available under the terms of the MIT License.
