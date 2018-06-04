#!/usr/bin/env python3
#
# Copyright (c) 2017-2018 Pavel Kirienko <pavel.kirienko@zubax.com>
#
# This program is distributed under the terms of the MIT software license.
#

import os
import sys
import time
import glob

sys.path.append(os.path.join(os.path.dirname(__file__), '../popcop/python'))

import popcop

try:
    PORT_NAME = sys.argv[1]
except IndexError:
    print('Port name or glob not provided', file=sys.stderr)
    sys.exit(1)

try:
    FIRMWARE_FILE_NAME = sys.argv[2]
except IndexError:
    print('Firmware file name not provided', file=sys.stderr)
    sys.exit(1)


def main():
    print('Reading the file...')
    with open(glob.glob(FIRMWARE_FILE_NAME)[0], 'rb') as f:
        firmware_image = f.read()

    print('Waiting for the device to start...')
    ei = None
    while ei is None or ei.mode != ei.Mode.BOOTLOADER:
        try:
            port_name = glob.glob(PORT_NAME)[0]
            port = popcop.physical.serial_multiprocessing.Channel(port_name, max_payload_size=2048)
        except Exception:
            print('Could not open port')
            time.sleep(1)
            continue

        port.send_standard(popcop.standard.endpoint_info.EndpointInfoMessage)
        while True:
            try:
                ei = port.receive(1)
                if not ei:
                    break
            except popcop.physical.serial_multiprocessing.ChannelException:
                break

            if isinstance(ei, popcop.standard.endpoint_info.EndpointInfoMessage):
                break

        if ei is not None and ei.mode != ei.Mode.BOOTLOADER:
            print('Launching the bootloader...')
            port.send_standard(popcop.standard.device_management.CommandRequestMessage(
                popcop.standard.device_management.Command.LAUNCH_BOOTLOADER))
            time.sleep(1)

    print('Endpoint info: ', ei)

    # Enter the FW update mode
    port.send_standard(popcop.standard.bootloader.StatusRequestMessage(
        popcop.standard.bootloader.State.APP_UPGRADE_IN_PROGRESS))
    response = port.receive(1)
    if not isinstance(response, popcop.standard.bootloader.StatusResponseMessage):
        raise Exception('Expected BL status response, got:' + repr(response))

    print('Bootloader status:', response)
    if response.state != popcop.standard.bootloader.State.APP_UPGRADE_IN_PROGRESS:
        raise Exception('Unexpected response: ' + repr(response))

    # Set the CoA
    print('Reading CoA...')
    port.send_standard(popcop.standard.bootloader.ImageDataRequestMessage(
        image_offset=0,
        image_type=popcop.standard.bootloader.ImageType.CERTIFICATE_OF_AUTHENTICITY,
        image_data=b''))

    response = port.receive(3)
    if not isinstance(response, popcop.standard.bootloader.ImageDataResponseMessage):
        raise Exception('Expected BL data response (CoA read), got:' + repr(response))

    if not response.image_data:
        coa = os.urandom(255)
        print('CoA is not set! Writing this:', coa)

        port.send_standard(popcop.standard.bootloader.ImageDataRequestMessage(
            image_offset=0,
            image_type=popcop.standard.bootloader.ImageType.CERTIFICATE_OF_AUTHENTICITY,
            image_data=coa))

        response = port.receive(3)
        if not isinstance(response, popcop.standard.bootloader.ImageDataResponseMessage):
            raise Exception('Expected BL data response (CoA write), got:' + repr(response))

        if response.image_data != coa:
            raise Exception('CoA data mismatch: ' + repr(response))
    else:
        print('Stored CoA:', response.image_data)

    # Upload the FW
    offset = 0
    for chunk in chunkized(firmware_image, 256):
        print('Sending chunk of', len(chunk), ' bytes at offset', offset)

        port.send_standard(popcop.standard.bootloader.ImageDataRequestMessage(
            image_offset=offset,
            image_type=popcop.standard.bootloader.ImageType.APPLICATION,
            image_data=chunk))

        response = port.receive(3)
        if not isinstance(response, popcop.standard.bootloader.ImageDataResponseMessage):
            raise Exception('Expected BL data response, got:' + repr(response))

        if response.image_data != chunk:
            raise Exception('Response data mismatch: ' + repr(response))

        offset += len(chunk)

    # Finalization
    port.send_standard(popcop.standard.bootloader.ImageDataRequestMessage(
        image_offset=offset,
        image_type=popcop.standard.bootloader.ImageType.APPLICATION,
        image_data=b''))

    # Wait for boot
    print('Waiting for boot...')
    del port
    time.sleep(6)

    # Check
    try:
        port_name = glob.glob(PORT_NAME)[0]
    except IndexError:
        raise Exception('Port not found') from None

    port = popcop.physical.serial_multiprocessing.Channel(port_name, max_payload_size=2048)
    port.send_standard(popcop.standard.endpoint_info.EndpointInfoMessage)
    while True:
        response = port.receive(1)
        if isinstance(response, (bytes, str)):
            continue
        else:
            break

    if not isinstance(response, popcop.standard.endpoint_info.EndpointInfoMessage):
        raise Exception('Expected endpoing info response, got:' + repr(response))

    print('Endpoint info: ', response)
    if response.mode != response.Mode.NORMAL:
        raise Exception('Application did not start!')

    print('Success!')


def chunkized(sequence, chunk_size: int):
    for i in range(0, len(sequence), chunk_size):
        yield sequence[i:i + chunk_size]


if __name__ == '__main__':
    sys.exit(main() or 0)
