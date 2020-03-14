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
# See https://github.com/PX4/Firmware/blob/nuttx_next/Tools/make_can_boot_descriptor.py
#

import os
import sys
import struct
import optparse
import binascii
from io import BytesIO


class AppDescriptor(object):
    """
    Kocherga application image descriptor format:
        uint64_t signature (bytes [7:0] set to 'APDesc00' in source)
        uint64_t image_crc (set to 0 in source, updated by this script)
        uint32_t image_size (set to 0 in source, updated by this script)
        uint32_t vcs_commit (set in source)
        uint8_t version_major (set in source)
        uint8_t version_minor (set in source)
        uint8_t flags (1 - release build, 2 - dirty build) (set in source)
        (uint8_t reserved)
        uint32_t build_timestamp_utc (set in source)
    """

    LENGTH = 32
    SIGNATURE = b'APDesc00'
    FORMAT = '<8sQLLBBBxL'

    def __init__(self, raw=None):
        self.signature = AppDescriptor.SIGNATURE
        self.image_crc = 0
        self.image_size = 0
        self.vcs_commit = 0
        self.version_major = 0
        self.version_minor = 0
        self.release_build = False
        self.dirty_build = False
        self.build_timestamp_utc = 0

        if raw:
            try:
                self.unpack(raw)
            except Exception:
                raise ValueError("Invalid AppDescriptor: {0}".format(binascii.b2a_hex(raw)))

    def pack(self):
        flags = 0
        if self.release_build:
            flags |= 1

        if self.dirty_build:
            flags |= 2

        return struct.pack(self.FORMAT, self.signature, self.image_crc, self.image_size, self.vcs_commit,
                           self.version_major, self.version_minor, flags, self.build_timestamp_utc)

    def unpack(self, raw):
        (self.signature,
         self.image_crc,
         self.image_size,
         self.vcs_commit,
         self.version_major,
         self.version_minor,
         flags,
         self.build_timestamp_utc) = struct.unpack(self.FORMAT, raw)

        self.release_build = bool(flags & 1)
        self.dirty_build   = bool(flags & 2)

        if not self.empty and not self.valid:
            raise ValueError()

    @property
    def empty(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.image_crc == 0 and self.image_size == 0)

    @property
    def valid(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.image_crc != 0 and self.image_size > 0 and
                self.build_timestamp_utc > 0)


class FirmwareImage(object):
    # Padding to 8 bytes is required by Kocherga
    PADDING = 8

    def __init__(self, path, mode="r"):
        self._file = open(path, (mode + "b").replace("bb", "b"))
        self._padding = self.PADDING

        if "r" in mode:
            self._contents = BytesIO(self._file.read())
        else:
            self._contents = BytesIO()
        self._do_write = False

        self._length = None
        self._descriptor_offset = None
        self._descriptor_bytes = None
        self._descriptor = None

    def __enter__(self):
        return self

    def __getattr__(self, attr):
        if attr == "write":
            self._do_write = True
        return getattr(self._contents, attr)

    def __iter__(self):
        return iter(self._contents)

    def __exit__(self, *args):
        if self._do_write:
            if getattr(self._file, "seek", None):
                self._file.seek(0)
            self._file.write(self._contents.getvalue())
            if self._padding:
                self._file.write(b'\xff' * self._padding)

        self._file.close()

    def _write_descriptor_raw(self):
        # Seek to the appropriate location, write the serialized descriptor, and seek back.
        prev_offset = self._contents.tell()
        self._contents.seek(self._descriptor_offset)
        self._contents.write(self._descriptor.pack())
        self._contents.seek(prev_offset)

    def write_descriptor(self):
        # Set the descriptor's length and CRC to the values required for CRC computation
        self.app_descriptor.image_size = self.length
        self.app_descriptor.image_crc = 0

        self._write_descriptor_raw()

        # Update the descriptor's CRC based on the computed value and write it out again
        self.app_descriptor.image_crc = self.crc

        self._write_descriptor_raw()

    @property
    def crc(self):
        MASK = 0xFFFFFFFFFFFFFFFF
        POLY = 0x42F0E1EBA9EA3693

        # Calculate the image CRC with the image_crc field in the app descriptor zeroed out.
        crc_offset = self.app_descriptor_offset + len(AppDescriptor.SIGNATURE)
        content = bytearray(self._contents.getvalue())
        content[crc_offset:crc_offset + 8] = bytearray(b"\x00" * 8)
        if self._padding:
            content += bytearray(b"\xff" * self._padding)
        val = MASK
        for byte in content:
            val ^= (byte << 56) & MASK
            for bit in range(8):
                if val & (1 << 63):
                    val = ((val << 1) & MASK) ^ POLY
                else:
                    val <<= 1

        return (val & MASK) ^ MASK

    @property
    def length(self):
        if not self._length:
            # Find the length of the file by seeking to the end and getting the offset
            prev_offset = self._contents.tell()
            self._contents.seek(0, os.SEEK_END)
            self._length = self._contents.tell()
            if self._padding:
                mod = self._length % self._padding
                self._padding = self._padding - mod if mod else 0
                self._length += self._padding
            self._contents.seek(prev_offset)

        return self._length

    @property
    def app_descriptor_offset(self):
        if not self._descriptor_offset:
            # Save the current position
            prev_offset = self._contents.tell()
            # Check each byte in the file to see if a valid descriptor starts at that location.
            # Slow, but not slow enough to matter.
            offset = 0
            while offset < self.length - AppDescriptor.LENGTH:
                self._contents.seek(offset)
                try:
                    # If this throws an exception, there isn't a valid descriptor at this offset
                    AppDescriptor(self._contents.read(AppDescriptor.LENGTH))
                except Exception:
                    offset += 1
                else:
                    self._descriptor_offset = offset
                    break
            # Go back to the previous position
            self._contents.seek(prev_offset)

        return self._descriptor_offset

    @property
    def app_descriptor(self):
        if not self._descriptor:
            # Save the current position
            prev_offset = self._contents.tell()
            # Jump to the descriptor and parse it
            self._contents.seek(self.app_descriptor_offset)
            self._descriptor_bytes = self._contents.read(AppDescriptor.LENGTH)
            self._descriptor = AppDescriptor(self._descriptor_bytes)
            # Go back to the previous offset
            self._contents.seek(prev_offset)

        return self._descriptor

    @app_descriptor.setter
    def app_descriptor(self, value):
        self._descriptor = value


if __name__ == "__main__":
    parser = optparse.OptionParser(usage=
                                   "Usage:   %prog [options] <input binary>\n"
                                   "Example: %prog firmware.bin")
    parser.add_option("--also-patch-descriptor-in",
                      dest="also_patch_descriptor_in",
                      default=[],
                      action='append',
                      help="file(s) where the descriptor will be updated too (e.g., an ELF executable for debugging)",
                      metavar="PATH")

    options, args = parser.parse_args()
    if len(args) != 1:
        parser.error("Invalid usage")

    input_binary = args[0]
    base_name_without_extension = os.path.basename(input_binary).rsplit('.', 1)[0]

    with FirmwareImage(input_binary, "rb") as in_image:
        # Firmware version format adopted by this script (feel free to change):
        # <base name>-<major>.<minor>.<VCS commit>.<CRC>.application.bin
        out_file = '%s-%s.%s.%08x.%016x.application.bin' % (base_name_without_extension,
                                                            in_image.app_descriptor.version_major,
                                                            in_image.app_descriptor.version_minor,
                                                            in_image.app_descriptor.vcs_commit,
                                                            in_image.crc)
        with FirmwareImage(out_file, "wb") as out_image:
            image = in_image.read()
            out_image.write(image)
            out_image.write_descriptor()

            for patchee in options.also_patch_descriptor_in:
                with open(patchee, "rb") as im:
                    also_image = im.read()
                also_image = also_image.replace(in_image.app_descriptor.pack(), out_image.app_descriptor.pack())
                with open(patchee, "wb") as im:
                    im.write(also_image)
