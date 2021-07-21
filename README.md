# Kochergá

[![Forum](https://img.shields.io/discourse/https/forum.zubax.com/users.svg)](https://forum.zubax.com)

**Kochergá is a robust [UAVCAN](https://uavcan.org) bootloader for deeply embedded systems.**

## Features

- **Portability** -- Kochergá is written in standard C++17 and is distributed as a header-only library
with no external dependencies.

- **Robustness** -- Kochergá is brick-proof.
The application (i.e., firmware) update process can be interrupted at any point (e.g., by turning off the power supply
or by disconnecting the interface), and it is guaranteed that the device will always end up in a known valid state.
If a dysfunctional application image is uploaded, Kochergá can regain control after a watchdog reset.

- **Safety** -- Kochergá verifies the correctness of the application image with a 64-bit hash before every boot.

### Supported transports

- **UAVCAN/CAN** (WIP) -- supports both v1 and v0, the protocol version is auto-detected at runtime.
- **UAVCAN/serial** (WIP)

## Usage

The entire library is contained in the header file `kocherga.hpp`;
protocol implementations are provided each in a separate header file named `kocherga_*.hpp`.
Kocherga does not have any compilation units of its own.

To integrate Kocherga into your application, just include this repository as a git subtree/submodule,
or simply copy-paste the required header files into your source tree.

The bootloader looks for an instance of the `AppInfo` structure located in the ROM image of the
application at every boot.
Only if a valid `AppInfo` structure is found the application will be launched.
It is recommended to allocate the structure closer to the beginning of the image in order to speed up its verification.
The structure is defined as follows:

Offset | Type     | Description
-------|----------|-----------------------------------------------------------------------------------------------------
-16    |`uint64`  | Constant value 0x5E4415146FC0C4C7 used for locating the descriptor and detecting the byte order.
-8     |`uint8[8]`| Set to `APDesc00`; used for compatibility with legacy deployments.
0      |`uint64`  | CRC-64-WE of the entire application image when this field itself is set to zero.
8      |`uint32`  | Size of the application image, in bytes. Note that the image must be padded to eight bytes.
12     |`void32`  | Reserved. Used to contain the 32-bit version control system revision ID; see replacement below.
16     |`uint8[2]`| Major and minor semantic version numbers.
18     |`uint8`   | Flags: 1 - this is a release build; 2 - this is a dirty build (uncommitted changes present).
19     |`void8`   | Reserved; set to 0.
20     |`uint32`  | UNIX UTC build timestamp; i.e., the number of seconds since 1970-01-01T00:00:00Z.
24     |`uint64`  | Version control system (VCS) revision ID (e.g., the git commit hash).
32     |`void64`  | Reserved.
40     |`void64`  | Reserved.

When computing the application image CRC, the process will eventually encounter the location where the CRC itself
is stored.
In order to avoid recursive dependency, the CRC storage location must be replaced with zero bytes
when computing/verifying the CRC.
The parameters of the CRC-64 algorithm are the following:
* Initial value: 0xFFFF'FFFF'FFFF'FFFF
* Polynomial: 0x42F0'E1EB'A9EA'3693
* Reverse: no
* Output xor: 0xFFFF'FFFF'FFFF'FFFF
* Check: 0x62EC'59E3'F1A4'F00A

The CRC and size fields cannot be populated until after the application binary is compiled and linked.
One possible way to populate these fields is to initialize them with zeroes in the source code,
and then use the script `tools/kocherga_image.py` after the binary is generated to update the fields
with their actual values.
The script can be invoked from the build system (e.g., from a Makefile rule) trivially as follows:

```sh
kocherga_image.py application-name-goes-here.bin
```

The following diagram documents the state machine of the bootloader:

![Kocherga State Machine Diagram](docs/state_machine.svg "Kocherga State Machine Diagram")

The bootloader states are mapped onto UAVCAN node states as follows:

Bootloader state     | Node mode       | Node health| Vendor-specific status code
---------------------|-----------------|------------|-------------------------------
NoAppToBoot          |`SOFTWARE_UPDATE`| `WARNING`  | 0
BootDelay            |`SOFTWARE_UPDATE`| `NOMINAL`  | 0
BootCancelled        |`SOFTWARE_UPDATE`| `ADVISORY` | 0
AppUpdateInProgress  |`SOFTWARE_UPDATE`| `NOMINAL`  | number of read requests, always >0

## Revisions

### v1.0

Work in progress.
