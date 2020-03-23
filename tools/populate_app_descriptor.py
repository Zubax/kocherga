#!/usr/bin/env python3
#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
#   Copyright (c) 2018-2020 Zubax Robotics <info@zubax.com>. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
#    disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Based on make_can_boot_descriptor.py originally created by Ben Dyer, David Sidrane and Pavel Kirienko for PX4.
# See https://github.com/PX4/Firmware/blob/nuttx_next/Tools/make_can_boot_descriptor.py.
# This script has no external dependencies.

"""
This script is a part of the Kocherga project. Kocherga is a compact, portable, MIT-licensed multi-transport UAVCAN
bootloader for deeply embedded systems. It is designed to be compatible with any UAVCAN transport, such as UAVCAN/CAN,
UAVCAN/serial, and so on. The UAVCAN/CAN transport in particular is designed to support the experimental version of
the protocol labeled UAVCAN v0, which is now obsolete and supported here only to ease the migration of legacy systems
to the stable version of the protocol.

Kocherga requires that the application image (i.e., firmware image -- in this documentation, the terms 'application'
and 'firmware' are used interchangeably) contains a particular data structure called "application descriptor".
The structure may be located anywhere within the image provided that its offset from the beginning is aligned at
8 bytes. It is recommended to place it near the beginning of the image (i.e., at a lower offset) to reduce the
boot-up time. The definition of the application descriptor data structure is given below in the DSDL notation.
The byte order is native for the target platform; external tools can determine it by checking the value of the
signature field.
If the image contains more than one descriptor, only that which is located at the lower offset will be used.

    uint64 signature    # Constant value used for locating the descriptor and detecting the byte order.
    uint64 image_crc    # Shall be set to zero when building the image.
    uint64 image_size   # Shall be set to zero when building the image.
    uint64 ts_utc_usec  # The UTC Unix build timestamp in microseconds.
    uint64 vcs_commit   # The version control system revision identifier, e.g., a git hash.
    void32
    bool debug_build    # Set if this is a debug image, not a released one.
    bool dirty_build    # Set if the image is built from an uncommitted revision of the sources.
    void14
    uint8[2] version_major_minor

    uint64 SIGNATURE = 0x5E44_1514_6FC0_C4C7  # A random magic number. Used for byte order detection.

Every field except image size and image CRC can be set either at compile time or by this script. By default, the
script only sets the size and the CRC, leaving all other fields at their original values; the user can specify which
other fields to overwrite by setting the appropriate command-line options when invoking the script. The timestamp
field is a special case: if the timestamp is zero, it will be automatically assigned by the script using the current
UTC timestamp sampled from the host system.

The image size and image CRC shall be zeroed at compile time (this implies that the signature shall be followed by
16 zero bytes); the values will be assigned by this script always. Structures where these fields are non-zero are
ignored by the script (considered invalid) in order to support the use case where images containing valid signatures
are nested (e.g., when the application image contains a bootloader image).

Excepting the extremely unlikely case of CRC collision, no application images may share the same image CRC.
This guarantee holds because a high-resolution build timestamp is embedded into the image.

The script does not modify the source image file; instead, it copies the data into a new file which is named using
the following naming convention:
    <basename>-<major>.<minor>.<commit>[,flag[,flag...]].app.bin
Where:
    basename     -- the name of the input file without extension.
    major, minor -- application version number pair in the output descriptor.
    commit       -- the VCS commit in the output descriptor (hexadecimal, 16 chars).
    flag         -- comma-separated flag names: 'dirty', 'debug', etc.
For example, an input file named 'com.zubax.telega-1.bin' results in:
    com.zubax.telega-1-1.2.1122334455667788,dirty,debug.app.bin

The script can be requested to copy the same descriptor into additional files. In this case, it would re-use the
same descriptor instead of computing it anew for every additional file. This feature is designed for patching ELF
files after linking to simplify debugging: after an ELF file is patched in this way, it can be simply uploaded onto
the target and it will boot successfully by virtue of having a correct application descriptor copied over from the raw
binary image.
"""

import os
import typing
import struct
import argparse
import operator
import functools
import dataclasses  # Python 3.7 or newer is required.


@dataclasses.dataclass
class Flags:
    # The flags shall be listed here from the least significant bit upwards. The serialization is automated.
    debug: bool
    dirty: bool

    def pack(self) -> int:
        return functools.reduce(operator.or_,
                                ((1 << idx) for idx, f in enumerate(dataclasses.fields(self)) if getattr(self, f.name)),
                                0)

    @staticmethod
    def unpack(value: int) -> 'Flags':
        return Flags(**{
            f.name: (value & (1 << idx)) != 0
            for idx, f in enumerate(dataclasses.fields(Flags))
        })

    def __str__(self) -> str:
        return ','.join(f.name for f in dataclasses.fields(self) if getattr(self, f.name))


@dataclasses.dataclass
class AppDescriptor:
    SIZE = 48
    ALIGNMENT = 8
    SIGNATURE = 0x5E44_1514_6FC0_C4C7

    image_crc: int
    image_size: int
    build_ts_utc_usec: int
    vcs_commit: int
    flags: Flags
    version: typing.Tuple[int, int]

    def pack(self, byte_order: str) -> bytes:
        return self._get_marshaller(byte_order).pack(
            self.SIGNATURE,
            self.image_crc,
            self.image_size,
            self.build_ts_utc_usec,
            self.vcs_commit,
            self.flags.pack(),
            *self.version,
        )

    @staticmethod
    def unpack_from(image: typing.Union[bytes, bytearray, memoryview],
                    byte_order: str,
                    offset: int = 0) -> typing.Optional['AppDescriptor']:
        try:
            signature, image_crc, image_size, build_ts_utc_usec, vcs_commit, flags, v_major, v_minor \
                = AppDescriptor._get_marshaller(byte_order).unpack_from(image, offset)
        except struct.error:
            return None
        return AppDescriptor(
            image_crc=image_crc,
            image_size=image_size,
            build_ts_utc_usec=build_ts_utc_usec,
            vcs_commit=vcs_commit,
            flags=Flags.unpack(flags),
            version=(v_major, v_minor),
        ) if signature == AppDescriptor.SIGNATURE else None

    @staticmethod
    def get_empty_search_prefix(byte_order: str) -> bytes:
        """
        An unpopulated app descriptor can be found by scanning the image for this byte pattern,
        depending on the byte order.
        """
        return AppDescriptor.SIGNATURE.to_bytes(8, byte_order) + bytes(8) * 2

    @staticmethod
    def _get_marshaller(byte_order: str) -> struct.Struct:
        byte_order = {'big': '>', 'little': '<'}[byte_order]
        out = struct.Struct(byte_order + 'Q QQ QQ 4x H BB')
        assert out.size == AppDescriptor.SIZE
        return out

    def __str__(self) -> str:
        out = f'{self.version[0]}.{self.version[1]}.{self.vcs_commit:016x}'
        flags = str(self.flags)
        if flags:
            out += ',' + flags
        return out


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


class ImageModel:
    PADDING_BYTE = b'\x00'

    def __init__(self,
                 image: bytearray,
                 app_descriptor_offset: int,
                 byte_order: str):
        """Do not construct instances manually. Use the factory method instead."""
        self._image = bytearray(image)
        self._offset = int(app_descriptor_offset)
        self._byte_order = str(byte_order)

        if AppDescriptor.unpack_from(self._image, self._byte_order, self._offset) is None:
            raise ValueError('The provided image does not contain an app descriptor at the specified offset')

        if len(self._image) % AppDescriptor.ALIGNMENT != 0 or self._offset % AppDescriptor.ALIGNMENT != 0:
            raise ValueError('Bad alignment')

        if len(self._image) <= AppDescriptor.SIZE or not (0 <= self._offset < len(self._image)):
            raise ValueError('Bad sizing')

    @property
    def byte_order(self) -> str:
        return self._byte_order

    @property
    def image(self) -> bytes:
        """The image with the specified application descriptor; the CRC and image size are assigned correctly."""
        self._sync()
        return bytes(self._image)  # Return copy to prevent mutation

    @property
    def app_descriptor(self) -> AppDescriptor:
        """The application descriptor where the CRC and image size are assigned correctly."""
        self._sync()
        return self._desc

    def set_app_descriptor(self, desc: AppDescriptor) -> None:
        """
        Update the app descriptor with the provided values.
        The image size and the image CRC fields will be ignored (overwritten by the class itself).
        """
        self._image[self._offset:self._offset + AppDescriptor.SIZE] = desc.pack(self._byte_order)
        self._sync()

    @property
    def _desc(self) -> AppDescriptor:
        desc = AppDescriptor.unpack_from(self._image, self._byte_order, self._offset)
        assert desc is not None, 'Internal protocol violation'
        return desc

    def _sync(self) -> None:
        desc = self._desc
        desc.image_crc = 0
        desc.image_size = len(self._image)
        self._image[self._offset:self._offset + AppDescriptor.SIZE] = desc.pack(self._byte_order)
        desc.image_crc = CRCComputer().add(self._image).value
        self._image[self._offset:self._offset + AppDescriptor.SIZE] = desc.pack(self._byte_order)

    @staticmethod
    def construct_from_built_image(image: typing.Union[memoryview, bytes, bytearray]) -> typing.Optional['ImageModel']:
        """The byte order will be detected automatically."""
        image = bytearray(image)
        while len(image) % AppDescriptor.ALIGNMENT != 0:
            image += ImageModel.PADDING_BYTE
        assert len(image) % AppDescriptor.ALIGNMENT == 0

        offset: int
        byte_order: str
        for bo in ['big', 'little']:
            offset = image.index(AppDescriptor.get_empty_search_prefix(bo))  # TODO Look further if not found
            if (offset >= 0) and (offset % AppDescriptor.ALIGNMENT == 0):
                byte_order = bo
                break
        else:
            return None
        assert 0 <= offset < len(image)

        if AppDescriptor.unpack_from(image, byte_order, offset) is not None:
            return ImageModel(
                image=image,
                app_descriptor_offset=offset,
                byte_order=byte_order,
            )


def _parse_version(x: str) -> typing.Tuple[int, int]:
    major, minor = (int(v) for v in x.replace(',', '.').split('.'))
    return major, minor


def _get_output_file_name(input_file_name: str, descriptor: AppDescriptor) -> str:
    base_stem = os.path.basename(input_file_name).rsplit('.', 1)[0]
    return f'{base_stem}-{descriptor}.app.bin'


def _get_utc_unix_timestamp_usec() -> int:
    import datetime
    return int((datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds() * 1e6)


def _main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=globals()['__doc__'])
    parser.add_argument('firmware_image',
                        metavar='PATH',
                        help='The name of the firmware image binary file where the descriptor should be written. '
                             'Use a special value "self-test" to run the self-test and exit immediately afterwards.')

    # Field setter options.
    parser.add_argument('--set-vcs-commit', type=int, metavar='VCS_COMMIT',
                        help='Set the VCS commit field.')
    parser.add_argument('--set-version', type=_parse_version, metavar='MAJOR.MINOR',
                        help='Set the version fields, major and minor. The values can be dot- or comma-separated.')
    parser.add_argument('--set-flag-debug-build', action='store_true',
                        help='Set the debug build flag in the app descriptor.')
    parser.add_argument('--set-flag-dirty-build', action='store_true',
                        help='Set the dirty build flag in the app descriptor.')

    # Other options.
    parser.add_argument('--also-patch', default=[], action='append', metavar='PATH',
                        help='File(s) where the descriptor will be updated too (e.g., an ELF executable).')

    args = parser.parse_args()
    if args.firmware_image == 'self-test':
        _test()
        return
    with open(args.firmware_image, 'rb') as in_file:
        image, desc = populate_app_descriptor(in_file.read())
        out_name = _get_output_file_name(args.firmware_image, desc)
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

    assert (1, 4) == _parse_version('1,4')
    assert (1, 4) == _parse_version('1.4')
    assert (1, 4) == _parse_version(' 1 , 4 ')
    try:
        _parse_version('1,4,7')
    except ValueError:
        pass
    else:
        assert False

    assert Flags(debug=False, dirty=False).pack() == 0
    assert Flags(debug=True, dirty=False).pack() == 1
    assert Flags(debug=False, dirty=True).pack() == 2
    assert Flags(debug=True, dirty=True).pack() == 3
    assert Flags.unpack(0) == Flags(debug=False, dirty=False)
    assert Flags.unpack(1) == Flags(debug=True, dirty=False)
    assert Flags.unpack(2) == Flags(debug=False, dirty=True)
    assert Flags.unpack(3) == Flags(debug=True, dirty=True)
    assert str(Flags(debug=False, dirty=False)) == ''
    assert str(Flags(debug=True, dirty=False)) == 'debug'
    assert str(Flags(debug=False, dirty=True)) == 'dirty'
    assert str(Flags(debug=True, dirty=True)) == 'debug,dirty'

    desc = AppDescriptor(
        image_crc=0,
        image_size=0,
        build_ts_utc_usec=0,
        vcs_commit=0,
        flags=Flags.unpack(0),
        version=(0, 0),
    )
    assert (AppDescriptor.SIGNATURE.to_bytes(8, 'little') +
            (0).to_bytes(8, 'little') +
            (0).to_bytes(8, 'little') +
            (0).to_bytes(8, 'little') +
            (0).to_bytes(8, 'little') +
            bytes(4) +
            (0).to_bytes(2, 'little') +
            bytes([0, 0]) == desc.pack('little'))
    desc.image_crc = 0xdeadbeef_0ddc0ffe
    desc.image_size = 0xaabbccdd
    desc.build_ts_utc_usec = 1234567890
    desc.vcs_commit = 0x1122334455667788
    desc.flags.dirty = True
    desc.version = 42, 95
    assert (AppDescriptor.SIGNATURE.to_bytes(8, 'little') +
            0xdeadbeef_0ddc0ffe.to_bytes(8, 'little') +
            0xaabbccdd.to_bytes(8, 'little') +
            (1234567890).to_bytes(8, 'little') +
            0x1122334455667788.to_bytes(8, 'little') +
            bytes(4) +
            (2).to_bytes(2, 'little') +
            bytes([42, 95]) == desc.pack('little'))
    assert (AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
            0xdeadbeef_0ddc0ffe.to_bytes(8, 'big') +
            0xaabbccdd.to_bytes(8, 'big') +
            (1234567890).to_bytes(8, 'big') +
            0x1122334455667788.to_bytes(8, 'big') +
            bytes(4) +
            (2).to_bytes(2, 'big') +
            bytes([42, 95]) == desc.pack('big'))

    desc = AppDescriptor.unpack_from(
        AppDescriptor.SIGNATURE.to_bytes(8, 'little') +
        0xdeadbeef_0ddc0ffe.to_bytes(8, 'little') +
        0xaabbccdd.to_bytes(8, 'little') +
        (1234567890).to_bytes(8, 'little') +
        0x1122334455667788.to_bytes(8, 'little') +
        bytes(4) +
        (2).to_bytes(2, 'little') +
        bytes([42, 95]),
        'little',
    )
    assert desc.image_crc == 0xdeadbeef_0ddc0ffe
    assert desc.image_size == 0xaabbccdd
    assert desc.build_ts_utc_usec == 1234567890
    assert desc.vcs_commit == 0x1122334455667788
    assert not desc.flags.debug
    assert desc.flags.dirty
    assert desc.version == (42, 95)

    desc = AppDescriptor.unpack_from(
        AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
        0xdeadbeef_0ddc0ffe.to_bytes(8, 'big') +
        0xaabbccdd.to_bytes(8, 'big') +
        (1234567890).to_bytes(8, 'big') +
        0x1122334455667788.to_bytes(8, 'big') +
        bytes(4) +
        (2).to_bytes(2, 'big') +
        bytes([42, 95]),
        'big',
    )
    assert desc.image_crc == 0xdeadbeef_0ddc0ffe
    assert desc.image_size == 0xaabbccdd
    assert desc.build_ts_utc_usec == 1234567890
    assert desc.vcs_commit == 0x1122334455667788
    assert not desc.flags.debug
    assert desc.flags.dirty
    assert desc.version == (42, 95)

    assert not AppDescriptor.unpack_from(AppDescriptor.SIGNATURE.to_bytes(8, 'big'), 'big')

    assert str(desc) == '42.95.1122334455667788,dirty'
    desc.flags.dirty = False
    assert str(desc) == '42.95.1122334455667788'

    im = ImageModel.construct_from_built_image(
        b'BEFORE MODEL    ' +
        AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
        (0).to_bytes(8, 'big') +  # CRC
        (0).to_bytes(8, 'big') +  # Size
        (1234567890).to_bytes(8, 'big') +
        0x1122334455667788.to_bytes(8, 'big') +
        bytes(4) +
        (2).to_bytes(2, 'big') +
        bytes([42, 95]) +
        b'AFTER MODEL'
    )
    assert im
    crc = CRCComputer().add(
        b'BEFORE MODEL    ' +
        AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
        (0).to_bytes(8, 'big') +  # CRC
        (80).to_bytes(8, 'big') +  # Size
        (1234567890).to_bytes(8, 'big') +
        0x1122334455667788.to_bytes(8, 'big') +
        bytes(4) +
        (2).to_bytes(2, 'big') +
        bytes([42, 95]) +
        b'AFTER MODEL' + ImageModel.PADDING_BYTE * 5
    ).value
    assert im.byte_order == 'big'
    assert im.app_descriptor == AppDescriptor(
        image_crc=crc,
        image_size=80,
        build_ts_utc_usec=1234567890,
        vcs_commit=0x1122334455667788,
        flags=Flags.unpack(2),
        version=(42, 95),
    )
    assert (b'BEFORE MODEL    ' +
            AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
            crc.to_bytes(8, 'big') +  # CRC
            (80).to_bytes(8, 'big') +  # Size
            (1234567890).to_bytes(8, 'big') +
            0x1122334455667788.to_bytes(8, 'big') +
            bytes(4) +
            (2).to_bytes(2, 'big') +
            bytes([42, 95]) +
            b'AFTER MODEL' + ImageModel.PADDING_BYTE * 5) == im.image
    desc = im.app_descriptor
    desc.version = 1, 5
    desc.flags.debug = True
    im.set_app_descriptor(desc)
    crc = CRCComputer().add(
        b'BEFORE MODEL    ' +
        AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
        (0).to_bytes(8, 'big') +  # CRC
        (80).to_bytes(8, 'big') +  # Size
        (1234567890).to_bytes(8, 'big') +
        0x1122334455667788.to_bytes(8, 'big') +
        bytes(4) +
        (3).to_bytes(2, 'big') +
        bytes([1, 5]) +
        b'AFTER MODEL' + ImageModel.PADDING_BYTE * 5
    ).value
    assert (b'BEFORE MODEL    ' +
            AppDescriptor.SIGNATURE.to_bytes(8, 'big') +
            crc.to_bytes(8, 'big') +  # CRC
            (80).to_bytes(8, 'big') +  # Size
            (1234567890).to_bytes(8, 'big') +
            0x1122334455667788.to_bytes(8, 'big') +
            bytes(4) +
            (3).to_bytes(2, 'big') +
            bytes([1, 5]) +
            b'AFTER MODEL' + ImageModel.PADDING_BYTE * 5) == im.image

    assert _get_output_file_name('firmware-1.bin', desc) == 'firmware-1-1.5.1122334455667788,debug,dirty.app.bin'

    assert 1584928309790639 < _get_utc_unix_timestamp_usec() < 32472144000000000


if __name__ == '__main__':
    _main()
