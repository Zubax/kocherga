#!/usr/bin/env python3
#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
#   Copyright (c) 2018-2020 Zubax Robotics <info@zubax.com>. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Based on make_can_boot_descriptor.py originally created by Ben Dyer, David Sidrane and Pavel Kirienko for PX4.
# See https://github.com/PX4/Firmware/blob/nuttx_next/Tools/make_can_boot_descriptor.py.
# This script has no external dependencies.
#

import os
import enum
import typing
import struct
import optparse

PADDING_BYTE = b'\x00'


class AppDescriptor:
    """
    Kocherga application image descriptor format v1 (little-endian byte order)::

        uint64_t signature      -- Set to the value of the SIGNATURE constant when building the image.
        uint64_t image_crc      -- Set to zero when building the image.
        uint64_t vcs_commit
        uint32_t image_size     -- Set to zero when building the image.
        uint32_t timestamp_utc
        uint32_t _reserved_a_
        uint8_t  version_major
        uint8_t  version_minor
        uint8_t  flags
        uint8_t _reserved_b_

    The structure shall be aligned at 8 bytes.

    Every field except image size and image CRC can be set either at compile time or by this script.
    The image size and image CRC shall be zeroed at compile time; the values will be assigned by this script always.
    Structures where these fields are non-zero are ignored by the script (considered invalid) in order to support
    the use case where images containing valid signatures are nested (e.g., when the application image contains a
    bootloader image).
    """

    class Flags(enum.IntEnum):
        RELEASE_BUILD = 1
        DIRTY_BUILD = 2

    SIGNATURE = b'APDesc01'
    FORMAT = struct.Struct('<8s QQ LL 4x BBB x')
    IMAGE_ALIGNMENT = 8

    def __init__(self):
        self.image_crc = 0
        self.vcs_commit = 0
        self.image_size = 0
        self.timestamp_utc = 0
        self.version = 0, 0
        self.release_build = False
        self.dirty_build = False

    def pack(self) -> bytes:
        flags = 0
        flags |= AppDescriptor.Flags.RELEASE_BUILD if self.release_build else 0
        flags |= AppDescriptor.Flags.DIRTY_BUILD if self.dirty_build else 0
        return self.FORMAT.pack(self.SIGNATURE, self.image_crc, self.vcs_commit, self.image_size, self.timestamp_utc,
                                *self.version, flags)

    @staticmethod
    def unpack_from(raw: typing.Union[bytes, bytearray, memoryview]) -> typing.Optional['AppDescriptor']:
        ob = AppDescriptor()
        try:
            (signature, ob.image_crc, ob.vcs_commit, ob.image_size, ob.timestamp_utc, v_major, v_minor, flags) \
                = AppDescriptor.FORMAT.unpack_from(bytes(raw))
        except struct.error:
            return None
        ob.version = v_major, v_minor
        ob.release_build = bool(flags & AppDescriptor.Flags.RELEASE_BUILD)
        ob.dirty_build = bool(flags & AppDescriptor.Flags.DIRTY_BUILD)
        if signature == AppDescriptor.SIGNATURE and ob.image_crc == 0 and ob.image_size == 0:
            return ob


class CRCComputer:
    MASK = 0xFFFFFFFFFFFFFFFF
    POLY = 0x42F0E1EBA9EA3693

    def __init__(self):
        self._crc = self.MASK

    def add(self, data: typing.Iterable[int]) -> 'CRCComputer':
        for b in data:
            self._crc ^= (b << 56) & self.MASK
            for _ in range(8):
                if self._crc & (1 << 63):
                    self._crc = ((self._crc << 1) & self.MASK) ^ self.POLY
                else:
                    self._crc <<= 1
        return self

    @property
    def value(self) -> int:
        return (self._crc & self.MASK) ^ self.MASK


def populate_app_descriptor(image: typing.Union[memoryview, bytes, bytearray],
                            vcs_commit: typing.Optional[int] = None,
                            timestamp_utc: typing.Optional[int] = None,
                            version: typing.Optional[typing.Tuple[int, int]] = None,
                            release_build: typing.Optional[bool] = None,
                            dirty_build: typing.Optional[bool] = None,
                            ) -> typing.Tuple[bytearray, AppDescriptor]:
    """
    :param image: A non-padded binary image to populate the app descriptor structure in.
    :param vcs_commit: If set, overrides the VCS commit field. If not set, the original value kept unchanged.
    :param timestamp_utc: If set, overrides the build timestamp field. If not set, the original value kept unchanged.
    :param version: If set to a tuple (major, minor), overrides the version fields; otherwise, the original is retained.
    :param release_build: Likewise.
    :param dirty_build: Likewise.
    :return: Updated image containing the correct descriptor plus the descriptor itself.
    """
    # Pad the image as necessary.
    image = bytearray(image)
    while len(image) % AppDescriptor.IMAGE_ALIGNMENT != 0:
        image += b'\x00'
    assert len(image) % AppDescriptor.IMAGE_ALIGNMENT == 0

    # Locate the descriptor and remember its offset.
    offset = 0
    while offset < (len(image) - AppDescriptor.FORMAT.size):
        desc = AppDescriptor.unpack_from(image[offset:])
        if desc:
            break
        offset += len(AppDescriptor.SIGNATURE)
    else:
        raise ValueError('Invalid application image: application descriptor structure not found')

    # Populate all fields of the descriptor except CRC -- it shall be zero until computed.
    desc.image_crc = 0
    desc.image_size = len(image)
    if vcs_commit is not None:
        desc.vcs_commit = int(vcs_commit)
    if timestamp_utc is not None:
        desc.timestamp_utc = int(timestamp_utc)
    if version is not None:
        desc.version = int(version[0]), int(version[1])
    if release_build is not None:
        desc.release_build = bool(release_build)
    if dirty_build is not None:
        desc.dirty_build = bool(dirty_build)

    # Populate the image CRC and replace the descriptor.
    image[offset:offset + AppDescriptor.FORMAT.size] = desc.pack()
    desc.image_crc = CRCComputer().add(image).value
    image[offset:offset + AppDescriptor.FORMAT.size] = desc.pack()
    return image, desc


def _main():
    parser = optparse.OptionParser(usage='Usage:   %prog [options] <input binary>\n'
                                         'Example: %prog firmware.bin')
    parser.add_option('--also-patch-descriptor-in',
                      default=[],
                      action='append',
                      help='file(s) where the descriptor will be updated too (e.g., an ELF executable for debugging)',
                      metavar='PATH')
    parser.add_option('--test',
                      default=False,
                      action='store_true',
                      help='Run the self test and exit')

    options, args = parser.parse_args()
    if options.test:
        _test()
        return
    if len(args) != 1:
        parser.error('Invalid usage')

    in_name = args[0]
    base_name_without_extension = os.path.basename(in_name).rsplit('.', 1)[0]

    with open(in_name, 'rb') as in_file:
        image, desc = populate_app_descriptor(in_file.read())
        # Firmware version format adopted by Kocherga (does not affect binary compatibility so can be changed easily):
        # <base name>-<major>.<minor>.<VCS commit>.<CRC>.application.bin
        out_name = '%s-%s.%s.%016x.%016x.application.bin' % (base_name_without_extension,
                                                             desc.version[0], desc.version[1],
                                                             desc.vcs_commit,
                                                             desc.image_crc)
        with open(out_name, 'wb') as out_file:
            # TODO
            out_file.write(image)
            for patchee in options.also_patch_descriptor_in:
                with open(patchee, 'rb') as im:
                    also_image = im.read()
                also_image = also_image.replace(in_image.app_descriptor.pack(), out_image.app_descriptor.pack())
                with open(patchee, 'wb') as im:
                    im.write(also_image)


def _test() -> None:
    assert 0x62ec59e3f1a4f00a == CRCComputer().add(b'123456').add(b'').add(b'789').value

    desc = AppDescriptor()
    assert b'APDesc01' + (0).to_bytes(8, 'little') + (0).to_bytes(8, 'little') + (0).to_bytes(4, 'little') + \
           (0).to_bytes(4, 'little') + b'\x00\x00\x00\x00' + bytes([0, 0, 0, 0]) == desc.pack()
    desc.image_crc = 0xdeadbeef_0ddc0ffe
    desc.vcs_commit = 0x1122334455667788
    desc.image_size = 0xaabbccdd
    desc.timestamp_utc = 1234567890
    desc.version = 42, 95
    desc.dirty_build = True
    assert b'APDesc01' + (0xdeadbeef_0ddc0ffe).to_bytes(8, 'little') + (0x1122334455667788).to_bytes(8, 'little') + \
           (0xaabbccdd).to_bytes(4, 'little') + (1234567890).to_bytes(4, 'little') + b'\x00\x00\x00\x00' + \
           bytes([42, 95, 2, 0]) == desc.pack()

    desc = AppDescriptor.unpack_from(
        b'APDesc01' + (0).to_bytes(8, 'little') + (0x1122334455667788).to_bytes(8, 'little') +
        (0).to_bytes(4, 'little') + (1234567890).to_bytes(4, 'little') + b'\xAA\xFF\xAA\xFF' +
        bytes([42, 95, 1, 23])
    )
    assert desc.image_crc == 0
    assert desc.vcs_commit == 0x1122334455667788
    assert desc.image_size == 0
    assert desc.timestamp_utc == 1234567890
    assert desc.version == (42, 95)
    assert desc.release_build
    assert not desc.dirty_build

    assert not AppDescriptor.unpack_from(
        b'APDesc00' + (0xdeadbeef_0ddc0ffe).to_bytes(8, 'little') + (0x1122334455667788).to_bytes(8, 'little') +
        (0xaabbccdd).to_bytes(4, 'little') + (1234567890).to_bytes(4, 'little') + b'\xAA\xFF\xAA\xFF' +
        bytes([42, 95, 1, 23])
    )
    assert not AppDescriptor.unpack_from(
        b'APDesc01' + (0).to_bytes(8, 'little') + (0x1122334455667788).to_bytes(8, 'little') +
        (0xaabbccdd).to_bytes(4, 'little') + (1234567890).to_bytes(4, 'little') + b'\xAA\xFF\xAA\xFF' +
        bytes([42, 95, 1, 23])
    )
    assert not AppDescriptor.unpack_from(
        b'APDesc01' + (0xdeadbeef_0ddc0ffe).to_bytes(8, 'little') + (0x1122334455667788).to_bytes(8, 'little') +
        (0).to_bytes(4, 'little') + (1234567890).to_bytes(4, 'little') + b'\xAA\xFF\xAA\xFF' +
        bytes([42, 95, 1, 23])
    )
    assert not AppDescriptor.unpack_from(b'APDesc01')

    padded_size = 56
    crc = CRCComputer().add(
        b'\x12\x34\x56\x78\x90\xab\xcd\xef' + b'APDesc01' + (0).to_bytes(8, 'little') +
        (123456).to_bytes(8, 'little') + padded_size.to_bytes(4, 'little') +
        (1234567890).to_bytes(4, 'little') + b'\x00\x00\x00\x00' + bytes([11, 22, 3, 0]) + b'abc\x00\x00\x00\x00\x00'
    ).value
    im_b, desc = populate_app_descriptor(
        b'\x12\x34\x56\x78\x90\xab\xcd\xef' + b'APDesc01' + (0).to_bytes(8, 'little') +
        (0x1122334455667788).to_bytes(8, 'little') + (0).to_bytes(4, 'little') +
        (1234567890).to_bytes(4, 'little') + b'\x00\x00\x00\x00' + bytes([42, 95, 1, 0]) + b'abc',
        vcs_commit=123456,
        version=(11, 22),
        dirty_build=True,
    )
    ref = b'\x12\x34\x56\x78\x90\xab\xcd\xef' + b'APDesc01' + crc.to_bytes(8, 'little') + \
          (123456).to_bytes(8, 'little') + padded_size.to_bytes(4, 'little') + \
          (1234567890).to_bytes(4, 'little') + b'\x00\x00\x00\x00' + bytes([11, 22, 3, 0]) + b'abc\x00\x00\x00\x00\x00'
    assert im_b == ref
    assert desc.image_crc == crc
    assert desc.vcs_commit == 123456
    assert desc.image_size == padded_size
    assert desc.timestamp_utc == 1234567890
    assert desc.version == (11, 22)
    assert desc.release_build
    assert desc.dirty_build

    try:  # Discarded as invalid because CRC and size are populated.
        populate_app_descriptor(
            b'\x12\x34\x56\x78\x90\xab\xcd\xef' + b'APDesc01' + (123456789).to_bytes(8, 'little') +
            (0x1122334455667788).to_bytes(8, 'little') + (123654987).to_bytes(4, 'little') +
            (1234567890).to_bytes(4, 'little') + b'\x00\x00\x00\x00' + bytes([42, 95, 1, 0]) + b'abc'
        )
    except ValueError:
        pass
    else:
        assert False


if __name__ == '__main__':
    _main()
