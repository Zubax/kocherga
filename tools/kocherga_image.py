#!/usr/bin/env python3
#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
#   Copyright (c) 2018-2021 Zubax Robotics <info@zubax.com>. All rights reserved.
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
# Based on make_can_boot_descriptor.py originally created by Ben Dyer, David Sidrane and Pavel Kirienko for PX4:
# https://github.com/PX4/PX4-Autopilot/blob/0033c0fc510a52deabe10ba769e39f97bec86f10/src/drivers/bootloaders/make_can_boot_descriptor.py
# Redesigned for Kocherga by Pavel Kirienko <pavel.kirienko@zubax.com>.

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
The byte order is native for the target platform; external tools can determine it by checking the value of the magic.
If the image contains more than one descriptor, only that which is located at the lower offset will be used.

    uint64   magic                  # Constant value used for locating the descriptor and detecting the byte order.
    uint8[8] signature              # Set to "APDesc00"; used for compatibility with legacy deployments.
    uint64   image_crc64we          # Shall be set to zero when building the image.
    uint32   image_size             # Shall be set to zero when building the image.
    void32                          # Used to contain 32-bit vcs_revision_id, now deprecated; see replacement below.
    uint8[2] version_major_minor
    uint8    flags                  # See below.
    void8
    uint32   timestamp_utc          # UTC UNIX timestamp when the application was built.
    uint64   vcs_revision_id        # The version control system revision identifier, e.g., a git hash.
    void64
    void64

    uint8 FLAG_RELEASE = 1          # Set if this is a release build, not debug.
    uint8 FLAG_DIRTY   = 2          # Set if the image is built from an uncommitted revision of the sources.

    uint64 MAGIC = 0x5E4415146FC0C4C7

In the hex editor, the structure appears as follows:

      | 01234567 89ABCDEF
    --+
     0| MMMMMMMM AAAAAAAA
    16| CCCCCCCC SSSS----
    32| JNF-TTTT VVVVVVVV
    48| -------- --------
    64| -------- --------

Every field except image size and image CRC can be set either at compile time or by this script. By default, the
script only sets the size and the CRC, leaving all other fields at their original values; the user can specify which
other fields to overwrite by setting the appropriate command-line options when invoking the script.

The image size and image CRC shall be zeroed at compile time (this implies that the magic with the signature
shall be followed by 12 zero bytes); the values will be assigned by this script always.
Structures where these fields are non-zero are ignored by the script (considered invalid) in order to support the
use case where images containing valid magics are nested (e.g., when the application image contains a bootloader image).

The script does not modify the source image file; instead, it copies the data into a new file which is named using
the following naming convention:
    <basename>-<major>.<minor>.<revision>.<crc>.app[.flag[.flag[.flag]]].bin
Where:
    basename     -- the name of the input file without extension (may contain "-" as well).
    major, minor -- application version number pair in the output descriptor (decimal).
    revision     -- the VCS revision ID in the output descriptor (hexadecimal, 16 chars).
    crc          -- the image CRC; it is also the unique identifier of this build (hexadecimal, 16 chars).
    flag         -- list of flag names: 'dirty', etc.
For example, an input file named 'com.zubax.telega-1.bin' results in:
    com.zubax.telega-1-1.2.1122334455667788.1122334455667788.app.release.dirty.bin
This format is compatible with the automatic firmware update server implemented in the Yakut CLI tool.

The script can be requested to copy the same descriptor into additional files. In this case, it would re-use the same
descriptor instead of computing it anew for every additional file. This feature is designed for patching ELF files
after linking to simplify debugging: after an ELF file is patched in this way, it can be simply uploaded onto the
target and it will boot successfully by virtue of having the correct application descriptor copied over from the raw
binary image. This procedure is called "side-patching".

This script has no external dependencies. The script can be used as a module as well.
"""

from __future__ import annotations
import os
import sys
import typing
import struct
import logging
import argparse
import functools
import dataclasses

_logger = logging.getLogger(__name__)


@dataclasses.dataclass
class Flags:
    # The flags shall be listed here from the least significant bit upwards. The serialization is automated.
    release: bool
    dirty: bool

    def pack(self) -> int:
        return functools.reduce(
            lambda a, b: a | b, ((1 << i) for i, f in enumerate(dataclasses.fields(self)) if getattr(self, f.name)), 0
        )

    @staticmethod
    def unpack(value: int) -> Flags:
        return Flags(**{f.name: (value & (1 << idx)) != 0 for idx, f in enumerate(dataclasses.fields(Flags))})


@dataclasses.dataclass
class AppDescriptor:
    SIZE = 64
    ALIGNMENT = 8
    MAGIC = 0x5E44_1514_6FC0_C4C7
    SIGNATURE = b"APDesc00"

    image_crc: int
    image_size: int
    version: typing.Tuple[int, int]
    flags: Flags
    timestamp_utc: int
    vcs_revision_id: int

    def pack(self, byte_order: str) -> bytes:
        return self._get_marshaller(byte_order).pack(
            self.MAGIC,
            self.SIGNATURE,
            self.image_crc,
            self.image_size,
            *self.version,
            self.flags.pack(),
            self.timestamp_utc,
            self.vcs_revision_id,
        )

    @staticmethod
    def unpack_from(
        image: typing.Union[bytes, bytearray, memoryview], byte_order: str, offset: int = 0
    ) -> typing.Optional[AppDescriptor]:
        try:
            (
                magic,
                signature,
                image_crc,
                image_size,
                v_major,
                v_minor,
                flags,
                ts_utc,
                vcs_revision_id,
            ) = AppDescriptor._get_marshaller(byte_order).unpack_from(image, offset)
        except struct.error:
            return None
        return (
            AppDescriptor(
                image_crc=image_crc,
                image_size=image_size,
                version=(v_major, v_minor),
                flags=Flags.unpack(flags),
                timestamp_utc=ts_utc,
                vcs_revision_id=vcs_revision_id,
            )
            if magic == AppDescriptor.MAGIC and signature == AppDescriptor.SIGNATURE
            else None
        )

    @staticmethod
    def get_search_prefix(byte_order: str, uninitialized_only: bool = False) -> bytes:
        """
        An app descriptor can be found by scanning the image for this byte pattern, depending on the byte order.
        If the uninitialized_only flag is set, the search prefix will match only unpopulated descriptors where
        the CRC and the size are zero. Otherwise, any descriptor will match.
        """
        tail = bytes(12) if uninitialized_only else b""
        return AppDescriptor.MAGIC.to_bytes(8, byte_order) + AppDescriptor.SIGNATURE + tail

    @staticmethod
    def _get_marshaller(byte_order: str) -> struct.Struct:
        byte_order = {"big": ">", "little": "<"}[byte_order]
        out = struct.Struct(byte_order + "Q 8s Q L 4x BB B x L Q 16x")
        assert out.size == AppDescriptor.SIZE
        return out

    def __str__(self) -> str:
        out = f"{self.version[0]}.{self.version[1]}.{self.vcs_revision_id:016x}.{self.image_crc:016x}.app"
        for flag, value in dataclasses.asdict(self.flags).items():
            if value:
                out += f".{flag}"
        return out


class ImageModel:
    PADDING_BYTE = b"\x00"

    def __init__(self, image: bytearray, app_descriptor_offset: int, byte_order: str):
        """Do not construct instances manually. Use the factory method instead."""
        self._image = bytearray(image)
        self._offset = int(app_descriptor_offset)
        self._byte_order = str(byte_order)

        if AppDescriptor.unpack_from(self._image, self._byte_order, self._offset) is None:
            raise ValueError("The provided image does not contain an app descriptor at the specified offset")

        if len(self._image) % AppDescriptor.ALIGNMENT != 0 or self._offset % AppDescriptor.ALIGNMENT != 0:
            raise ValueError("Bad alignment")

        if len(self._image) <= AppDescriptor.SIZE or not (0 <= self._offset < len(self._image)):
            raise ValueError("Bad sizing")

    @property
    def byte_order(self) -> str:
        return self._byte_order

    @property
    def image(self) -> bytes:
        return bytes(self._image)  # Return copy to prevent mutation

    @property
    def app_descriptor_offset(self) -> int:
        return self._offset

    @property
    def app_descriptor(self) -> AppDescriptor:
        desc = AppDescriptor.unpack_from(self._image, self._byte_order, self._offset)
        assert desc is not None, "Internal protocol violation"
        return desc

    @app_descriptor.setter
    def app_descriptor(self, value: AppDescriptor) -> None:
        self._image[self._offset : self._offset + AppDescriptor.SIZE] = value.pack(self._byte_order)

    def update(self) -> None:
        """
        Compute and overwrite the image size and the image CRC fields in the app descriptor.
        """
        desc = self.app_descriptor
        desc.image_crc = 0
        desc.image_size = len(self._image)
        self._image[self._offset : self._offset + AppDescriptor.SIZE] = desc.pack(self._byte_order)
        desc.image_crc = CRCComputer().add(self._image).value
        self._image[self._offset : self._offset + AppDescriptor.SIZE] = desc.pack(self._byte_order)

    def validate_app_descriptor(self) -> bool:
        image = bytearray(self.image)
        desc = self.app_descriptor
        expected_crc, desc.image_crc = desc.image_crc, 0
        image[self._offset : self._offset + AppDescriptor.SIZE] = desc.pack(self._byte_order)
        return expected_crc == CRCComputer().add(image).value

    @staticmethod
    def construct_from_image(
        image: typing.Union[memoryview, bytes, bytearray], uninitialized_only: bool = False
    ) -> typing.Optional[ImageModel]:
        """
        The byte order will be detected automatically.
        :param image: The source application image.
        :param uninitialized_only: True if the image does not contain a valid descriptor yet:
            expect CRC and size to be zero. This option is designed for use with freshly built binary images
            which have not been processed yet.
        """
        image = bytearray(image)
        while len(image) % AppDescriptor.ALIGNMENT != 0:
            image += ImageModel.PADDING_BYTE
        assert len(image) % AppDescriptor.ALIGNMENT == 0

        offset: int
        byte_order: str
        for bo in ["big", "little"]:
            offset = image.find(AppDescriptor.get_search_prefix(bo, uninitialized_only=uninitialized_only))
            if (offset >= 0) and (offset % AppDescriptor.ALIGNMENT == 0):  # TODO Look further if invalid
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


class CRCComputer:
    _MASK = 0xFFFFFFFFFFFFFFFF
    # fmt: off
    _TABLE = [
        0x0000000000000000, 0x42F0E1EBA9EA3693, 0x85E1C3D753D46D26, 0xC711223CFA3E5BB5, 0x493366450E42ECDF,
        0x0BC387AEA7A8DA4C, 0xCCD2A5925D9681F9, 0x8E224479F47CB76A, 0x9266CC8A1C85D9BE, 0xD0962D61B56FEF2D,
        0x17870F5D4F51B498, 0x5577EEB6E6BB820B, 0xDB55AACF12C73561, 0x99A54B24BB2D03F2, 0x5EB4691841135847,
        0x1C4488F3E8F96ED4, 0x663D78FF90E185EF, 0x24CD9914390BB37C, 0xE3DCBB28C335E8C9, 0xA12C5AC36ADFDE5A,
        0x2F0E1EBA9EA36930, 0x6DFEFF5137495FA3, 0xAAEFDD6DCD770416, 0xE81F3C86649D3285, 0xF45BB4758C645C51,
        0xB6AB559E258E6AC2, 0x71BA77A2DFB03177, 0x334A9649765A07E4, 0xBD68D2308226B08E, 0xFF9833DB2BCC861D,
        0x388911E7D1F2DDA8, 0x7A79F00C7818EB3B, 0xCC7AF1FF21C30BDE, 0x8E8A101488293D4D, 0x499B3228721766F8,
        0x0B6BD3C3DBFD506B, 0x854997BA2F81E701, 0xC7B97651866BD192, 0x00A8546D7C558A27, 0x4258B586D5BFBCB4,
        0x5E1C3D753D46D260, 0x1CECDC9E94ACE4F3, 0xDBFDFEA26E92BF46, 0x990D1F49C77889D5, 0x172F5B3033043EBF,
        0x55DFBADB9AEE082C, 0x92CE98E760D05399, 0xD03E790CC93A650A, 0xAA478900B1228E31, 0xE8B768EB18C8B8A2,
        0x2FA64AD7E2F6E317, 0x6D56AB3C4B1CD584, 0xE374EF45BF6062EE, 0xA1840EAE168A547D, 0x66952C92ECB40FC8,
        0x2465CD79455E395B, 0x3821458AADA7578F, 0x7AD1A461044D611C, 0xBDC0865DFE733AA9, 0xFF3067B657990C3A,
        0x711223CFA3E5BB50, 0x33E2C2240A0F8DC3, 0xF4F3E018F031D676, 0xB60301F359DBE0E5, 0xDA050215EA6C212F,
        0x98F5E3FE438617BC, 0x5FE4C1C2B9B84C09, 0x1D14202910527A9A, 0x93366450E42ECDF0, 0xD1C685BB4DC4FB63,
        0x16D7A787B7FAA0D6, 0x5427466C1E109645, 0x4863CE9FF6E9F891, 0x0A932F745F03CE02, 0xCD820D48A53D95B7,
        0x8F72ECA30CD7A324, 0x0150A8DAF8AB144E, 0x43A04931514122DD, 0x84B16B0DAB7F7968, 0xC6418AE602954FFB,
        0xBC387AEA7A8DA4C0, 0xFEC89B01D3679253, 0x39D9B93D2959C9E6, 0x7B2958D680B3FF75, 0xF50B1CAF74CF481F,
        0xB7FBFD44DD257E8C, 0x70EADF78271B2539, 0x321A3E938EF113AA, 0x2E5EB66066087D7E, 0x6CAE578BCFE24BED,
        0xABBF75B735DC1058, 0xE94F945C9C3626CB, 0x676DD025684A91A1, 0x259D31CEC1A0A732, 0xE28C13F23B9EFC87,
        0xA07CF2199274CA14, 0x167FF3EACBAF2AF1, 0x548F120162451C62, 0x939E303D987B47D7, 0xD16ED1D631917144,
        0x5F4C95AFC5EDC62E, 0x1DBC74446C07F0BD, 0xDAAD56789639AB08, 0x985DB7933FD39D9B, 0x84193F60D72AF34F,
        0xC6E9DE8B7EC0C5DC, 0x01F8FCB784FE9E69, 0x43081D5C2D14A8FA, 0xCD2A5925D9681F90, 0x8FDAB8CE70822903,
        0x48CB9AF28ABC72B6, 0x0A3B7B1923564425, 0x70428B155B4EAF1E, 0x32B26AFEF2A4998D, 0xF5A348C2089AC238,
        0xB753A929A170F4AB, 0x3971ED50550C43C1, 0x7B810CBBFCE67552, 0xBC902E8706D82EE7, 0xFE60CF6CAF321874,
        0xE224479F47CB76A0, 0xA0D4A674EE214033, 0x67C58448141F1B86, 0x253565A3BDF52D15, 0xAB1721DA49899A7F,
        0xE9E7C031E063ACEC, 0x2EF6E20D1A5DF759, 0x6C0603E6B3B7C1CA, 0xF6FAE5C07D3274CD, 0xB40A042BD4D8425E,
        0x731B26172EE619EB, 0x31EBC7FC870C2F78, 0xBFC9838573709812, 0xFD39626EDA9AAE81, 0x3A28405220A4F534,
        0x78D8A1B9894EC3A7, 0x649C294A61B7AD73, 0x266CC8A1C85D9BE0, 0xE17DEA9D3263C055, 0xA38D0B769B89F6C6,
        0x2DAF4F0F6FF541AC, 0x6F5FAEE4C61F773F, 0xA84E8CD83C212C8A, 0xEABE6D3395CB1A19, 0x90C79D3FEDD3F122,
        0xD2377CD44439C7B1, 0x15265EE8BE079C04, 0x57D6BF0317EDAA97, 0xD9F4FB7AE3911DFD, 0x9B041A914A7B2B6E,
        0x5C1538ADB04570DB, 0x1EE5D94619AF4648, 0x02A151B5F156289C, 0x4051B05E58BC1E0F, 0x87409262A28245BA,
        0xC5B073890B687329, 0x4B9237F0FF14C443, 0x0962D61B56FEF2D0, 0xCE73F427ACC0A965, 0x8C8315CC052A9FF6,
        0x3A80143F5CF17F13, 0x7870F5D4F51B4980, 0xBF61D7E80F251235, 0xFD913603A6CF24A6, 0x73B3727A52B393CC,
        0x31439391FB59A55F, 0xF652B1AD0167FEEA, 0xB4A25046A88DC879, 0xA8E6D8B54074A6AD, 0xEA16395EE99E903E,
        0x2D071B6213A0CB8B, 0x6FF7FA89BA4AFD18, 0xE1D5BEF04E364A72, 0xA3255F1BE7DC7CE1, 0x64347D271DE22754,
        0x26C49CCCB40811C7, 0x5CBD6CC0CC10FAFC, 0x1E4D8D2B65FACC6F, 0xD95CAF179FC497DA, 0x9BAC4EFC362EA149,
        0x158E0A85C2521623, 0x577EEB6E6BB820B0, 0x906FC95291867B05, 0xD29F28B9386C4D96, 0xCEDBA04AD0952342,
        0x8C2B41A1797F15D1, 0x4B3A639D83414E64, 0x09CA82762AAB78F7, 0x87E8C60FDED7CF9D, 0xC51827E4773DF90E,
        0x020905D88D03A2BB, 0x40F9E43324E99428, 0x2CFFE7D5975E55E2, 0x6E0F063E3EB46371, 0xA91E2402C48A38C4,
        0xEBEEC5E96D600E57, 0x65CC8190991CB93D, 0x273C607B30F68FAE, 0xE02D4247CAC8D41B, 0xA2DDA3AC6322E288,
        0xBE992B5F8BDB8C5C, 0xFC69CAB42231BACF, 0x3B78E888D80FE17A, 0x7988096371E5D7E9, 0xF7AA4D1A85996083,
        0xB55AACF12C735610, 0x724B8ECDD64D0DA5, 0x30BB6F267FA73B36, 0x4AC29F2A07BFD00D, 0x08327EC1AE55E69E,
        0xCF235CFD546BBD2B, 0x8DD3BD16FD818BB8, 0x03F1F96F09FD3CD2, 0x41011884A0170A41, 0x86103AB85A2951F4,
        0xC4E0DB53F3C36767, 0xD8A453A01B3A09B3, 0x9A54B24BB2D03F20, 0x5D45907748EE6495, 0x1FB5719CE1045206,
        0x919735E51578E56C, 0xD367D40EBC92D3FF, 0x1476F63246AC884A, 0x568617D9EF46BED9, 0xE085162AB69D5E3C,
        0xA275F7C11F7768AF, 0x6564D5FDE549331A, 0x279434164CA30589, 0xA9B6706FB8DFB2E3, 0xEB46918411358470,
        0x2C57B3B8EB0BDFC5, 0x6EA7525342E1E956, 0x72E3DAA0AA188782, 0x30133B4B03F2B111, 0xF7021977F9CCEAA4,
        0xB5F2F89C5026DC37, 0x3BD0BCE5A45A6B5D, 0x79205D0E0DB05DCE, 0xBE317F32F78E067B, 0xFCC19ED95E6430E8,
        0x86B86ED5267CDBD3, 0xC4488F3E8F96ED40, 0x0359AD0275A8B6F5, 0x41A94CE9DC428066, 0xCF8B0890283E370C,
        0x8D7BE97B81D4019F, 0x4A6ACB477BEA5A2A, 0x089A2AACD2006CB9, 0x14DEA25F3AF9026D, 0x562E43B4931334FE,
        0x913F6188692D6F4B, 0xD3CF8063C0C759D8, 0x5DEDC41A34BBEEB2, 0x1F1D25F19D51D821, 0xD80C07CD676F8394,
        0x9AFCE626CE85B507,
    ]
    # fmt: on

    def __init__(self):
        assert len(self._TABLE) == 256
        self._crc = 0

    def add(self, data: typing.Iterable[int]) -> CRCComputer:
        mask = self._MASK
        val = self._crc ^ mask
        table = self._TABLE
        for b in data:
            val = (table[b ^ (val >> 56)] ^ (val << 8)) & mask
        self._crc = val ^ mask
        assert 0 <= self._crc < 2 ** 64
        return self

    @property
    def value(self) -> int:
        return self._crc


def _parse_version(x: str) -> typing.Tuple[int, int]:
    major, minor = (int(v) for v in x.replace(",", ".").split("."))
    return major, minor


def _get_output_file_name(input_file_name: str, descriptor: AppDescriptor) -> str:
    base_stem = os.path.basename(input_file_name).rsplit(".", 1)[0]
    return f"{base_stem}-{descriptor}.bin"


def _main() -> int:
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=globals()["__doc__"])
    parser.add_argument(
        "firmware_image",
        metavar="PATH",
        help="The name of the firmware image binary file where the descriptor should be written. "
        'Use the special value "self-test" to run the self-test and exit immediately afterwards.',
    )
    # Field setter options.
    parser.add_argument(
        "--assign-version",
        type=_parse_version,
        metavar="MAJOR.MINOR",
        help="Overwrite the version fields, major and minor. Values can be dot- or comma-separated.",
    )
    parser.add_argument(
        "--assign-flag-release",
        type=int,
        metavar="FLAG",
        help="Overwrite the release build flag in the app descriptor.",
    )
    parser.add_argument(
        "--assign-flag-dirty", type=int, metavar="FLAG", help="Overwrite the dirty build flag in the app descriptor."
    )
    parser.add_argument(
        "--assign-timestamp",
        type=lambda x: int(x, 0),
        metavar="TIMESTAMP",
        help="Overwrite the build timestamp. The value should be a UNIX timestamp in UTC.",
    )
    parser.add_argument(
        "--assign-vcs-revision-id",
        type=lambda x: int(x, 0),
        metavar="VCS_REVISION_ID",
        help='Overwrite the VCS revision ID field. Hexadecimals accepted if prefixed with "0x".',
    )
    # Other options.
    parser.add_argument(
        "--verbose", "-v", action="count", default=0, help="Enable verbose output. Twice for extra verbosity."
    )
    parser.add_argument(
        "--side-patch",
        default=[],
        action="append",
        metavar="PATH",
        help="File(s) where the same descriptor will be copied into (e.g., ELF executables). "
        "Use multiple times to patch several files at once.",
    )
    parser.add_argument(
        "--lazy",
        action="store_true",
        help="Exit silently if the image does not contain empty app descriptors (i.e., already processed). "
        "This option is helpful for integration with build systems.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        stream=sys.stderr,
        format="%(message)s",
        level={
            0: logging.WARNING,
            1: logging.INFO,
        }.get(args.verbose, logging.DEBUG),
    )

    if args.firmware_image == "self-test":
        _test()
        return 0
    _logger.debug(f"CLI arguments: {args}")

    # Read the input file. All operations done in-memory.
    with open(args.firmware_image, "rb") as in_file:
        img = in_file.read()
        model = ImageModel.construct_from_image(img, uninitialized_only=True)
        if not model:
            existing_model = ImageModel.construct_from_image(img)
            if existing_model and args.lazy:
                _logger.info(
                    f"Image {args.firmware_image!r} does not require processing because it already contains a "
                    f"valid app descriptor: {existing_model.app_descriptor!r}"
                )
                return 0
            _logger.fatal(
                f"An uninitialized app descriptor could not be found in {args.firmware_image!r}. "
                f"If this is intentional, use --lazy to squelch this error. "
                f"Existing app descriptor: {existing_model.app_descriptor if existing_model else None!r}"
            )
            return 1

    # Populate the fields as requested. The size and CRC are managed automatically by the image model class.
    desc = model.app_descriptor
    _logger.debug(f"Original app descriptor: {desc!r}")
    if args.assign_version is not None:
        assert len(args.assign_version) == 2
        assert isinstance(args.assign_version[0], int) and isinstance(args.assign_version[1], int)
        desc.version = args.assign_version
    if args.assign_flag_release is not None:
        desc.flags.release = bool(args.assign_flag_release)
    if args.assign_flag_dirty is not None:
        desc.flags.dirty = bool(args.assign_flag_dirty)
    if args.assign_timestamp is not None:
        desc.timestamp_utc = int(args.assign_timestamp)
    if args.assign_vcs_revision_id is not None:
        desc.vcs_revision_id = int(args.assign_vcs_revision_id)

    # Update the image and recompute the CRC with the new fields in the descriptor.
    model.app_descriptor = desc
    del desc
    model.update()
    assert len(model.image) == model.app_descriptor.image_size, "Internal logic error"
    assert model.validate_app_descriptor(), "Internal logic error: output image validation failed"
    _logger.info(f"Detected byte order:   {model.byte_order}")
    _logger.info(f"App descriptor offset: {model.app_descriptor_offset}")
    _logger.info(f"Final app descriptor:  {model.app_descriptor!r}")

    # Write the resulting image into the output file.
    out_name = _get_output_file_name(args.firmware_image, model.app_descriptor)
    with open(out_name, "wb") as out_file:
        assert model.validate_app_descriptor(), "Internal logic error: output image validation failed"
        out_file.write(model.image)
    _logger.info(f"Output image written into {out_name!r}")

    # Perform the side-patching.
    for path in args.side_patch:
        with open(path, "rb") as f:
            data = bytearray(f.read())
        offset = data.find(AppDescriptor.get_search_prefix(model.byte_order, uninitialized_only=True))
        if offset < 0:
            if args.lazy:
                _logger.info(
                    f"Side-patching of {path!r} is skipped because the file does not contain an "
                    f"uninitialized app descriptor and --lazy mode is enabled."
                )
                continue
            _logger.fatal(
                f"Side-patching failure: an uninitialized app descriptor could not be found in {path!r}. "
                f"If this is intentional, use --lazy to squelch this error."
            )
            return 1
        _logger.info(f"Side-patching {path!r} at offset {offset} bytes")
        data[offset : offset + AppDescriptor.SIZE] = model.app_descriptor.pack(model.byte_order)
        with open(path, "wb") as f:
            f.write(data)

    return 0


def _test() -> None:
    assert 0x62EC59E3F1A4F00A == CRCComputer().add(b"123456").add(b"").add(b"789").value

    assert (1, 4) == _parse_version("1,4")
    assert (1, 4) == _parse_version("1.4")
    assert (1, 4) == _parse_version(" 1 , 4 ")
    try:
        _parse_version("1,4,7")
    except ValueError:
        pass
    else:
        assert False

    assert Flags(release=False, dirty=False).pack() == 0
    assert Flags(release=True, dirty=False).pack() == 1
    assert Flags(release=False, dirty=True).pack() == 2
    assert Flags(release=True, dirty=True).pack() == 3
    assert Flags.unpack(0) == Flags(release=False, dirty=False)
    assert Flags.unpack(1) == Flags(release=True, dirty=False)
    assert Flags.unpack(2) == Flags(release=False, dirty=True)
    assert Flags.unpack(3) == Flags(release=True, dirty=True)

    desc = AppDescriptor(
        image_crc=0,
        image_size=0,
        version=(0, 0),
        flags=Flags.unpack(0),
        timestamp_utc=0,
        vcs_revision_id=0,
    )
    assert (
        AppDescriptor.MAGIC.to_bytes(8, "little")
        + b"APDesc00"
        + (0).to_bytes(8, "little")
        + (0).to_bytes(4, "little")
        + bytes(4)
        + bytes([0, 0])
        + bytes([0])
        + bytes(1)
        + (0).to_bytes(4, "little")
        + (0).to_bytes(8, "little")
        + bytes(16)
    ) == desc.pack("little")
    desc.image_crc = 0xDEADBEEF_0DDC0FFE
    desc.image_size = 0xAABBCCDD
    desc.version = 42, 95
    desc.flags.dirty = True
    desc.timestamp_utc = 1234567890
    desc.vcs_revision_id = 0x1122334455667788
    assert (
        AppDescriptor.MAGIC.to_bytes(8, "little")
        + b"APDesc00"
        + 0xDEADBEEF_0DDC0FFE .to_bytes(8, "little")
        + 0xAABBCCDD .to_bytes(4, "little")
        + bytes(4)
        + bytes([42, 95])
        + bytes([2])
        + bytes(1)
        + (1234567890).to_bytes(4, "little")
        + 0x1122334455667788 .to_bytes(8, "little")
        + bytes(16)
    ) == desc.pack("little")
    assert (
        AppDescriptor.MAGIC.to_bytes(8, "big")
        + b"APDesc00"
        + 0xDEADBEEF_0DDC0FFE .to_bytes(8, "big")
        + 0xAABBCCDD .to_bytes(4, "big")
        + bytes(4)
        + bytes([42, 95])
        + bytes([2])
        + bytes(1)
        + (1234567890).to_bytes(4, "big")
        + 0x1122334455667788 .to_bytes(8, "big")
        + bytes(16)
    ) == desc.pack("big")

    desc = AppDescriptor.unpack_from(
        AppDescriptor.MAGIC.to_bytes(8, "little")
        + b"APDesc00"
        + 0xDEADBEEF_0DDC0FFE .to_bytes(8, "little")
        + 0xAABBCCDD .to_bytes(4, "little")
        + bytes(4)
        + bytes([42, 95])
        + bytes([2])
        + bytes(1)
        + (1234567890).to_bytes(4, "little")
        + 0x1122334455667788 .to_bytes(8, "little")
        + bytes(16),
        "little",
    )
    assert desc.image_crc == 0xDEADBEEF_0DDC0FFE
    assert desc.image_size == 0xAABBCCDD
    assert desc.version == (42, 95)
    assert not desc.flags.release
    assert desc.flags.dirty
    assert desc.timestamp_utc == 1234567890
    assert desc.vcs_revision_id == 0x1122334455667788

    desc = AppDescriptor.unpack_from(
        AppDescriptor.MAGIC.to_bytes(8, "big")
        + b"APDesc00"
        + 0xDEADBEEF_0DDC0FFE .to_bytes(8, "big")
        + 0xAABBCCDD .to_bytes(4, "big")
        + bytes(4)
        + bytes([42, 95])
        + bytes([2])
        + bytes(1)
        + (1234567890).to_bytes(4, "big")
        + 0x1122334455667788 .to_bytes(8, "big")
        + bytes(16),
        "big",
    )
    assert desc.image_crc == 0xDEADBEEF_0DDC0FFE
    assert desc.image_size == 0xAABBCCDD
    assert desc.version == (42, 95)
    assert not desc.flags.release
    assert desc.flags.dirty
    assert desc.timestamp_utc == 1234567890
    assert desc.vcs_revision_id == 0x1122334455667788

    assert not AppDescriptor.unpack_from(AppDescriptor.MAGIC.to_bytes(8, "big"), "big")
    assert not AppDescriptor.unpack_from(AppDescriptor.MAGIC.to_bytes(8, "little") + AppDescriptor.SIGNATURE, "little")

    assert str(desc) == "42.95.1122334455667788.deadbeef0ddc0ffe.app.dirty"
    desc.flags.dirty = False
    assert str(desc) == "42.95.1122334455667788.deadbeef0ddc0ffe.app"
    desc.flags.dirty = True
    desc.flags.release = True
    assert str(desc) == "42.95.1122334455667788.deadbeef0ddc0ffe.app.release.dirty"

    im = ImageModel.construct_from_image(
        b"BEFORE MODEL    "
        + AppDescriptor.MAGIC.to_bytes(8, "big")
        + b"APDesc00"
        + (0).to_bytes(8, "big")
        + (0).to_bytes(4, "big")
        + bytes(4)
        + bytes([42, 95])
        + bytes([2])
        + bytes(1)
        + (1234567890).to_bytes(4, "big")
        + 0x1122334455667788 .to_bytes(8, "big")
        + bytes(16)
        + b"AFTER MODEL",
        uninitialized_only=True,
    )
    assert im
    crc = (
        CRCComputer()
        .add(
            b"BEFORE MODEL    "
            + AppDescriptor.MAGIC.to_bytes(8, "big")
            + b"APDesc00"
            + (0).to_bytes(8, "big")
            + (96).to_bytes(4, "big")
            + bytes(4)
            + bytes([42, 95])
            + bytes([2])
            + bytes(1)
            + (1234567890).to_bytes(4, "big")
            + 0x1122334455667788 .to_bytes(8, "big")
            + bytes(16)
            + b"AFTER MODEL"
            + ImageModel.PADDING_BYTE * 5
        )
        .value
    )
    assert im.byte_order == "big"
    assert im.app_descriptor_offset == 16
    assert im.app_descriptor == AppDescriptor(
        image_crc=0,
        image_size=0,
        version=(42, 95),
        flags=Flags.unpack(2),
        timestamp_utc=1234567890,
        vcs_revision_id=0x1122334455667788,
    )
    assert not im.validate_app_descriptor()
    im.update()
    assert im.validate_app_descriptor()
    assert im.app_descriptor == AppDescriptor(
        image_crc=crc,
        image_size=96,
        version=(42, 95),
        flags=Flags.unpack(2),
        timestamp_utc=1234567890,
        vcs_revision_id=0x1122334455667788,
    )
    assert (
        b"BEFORE MODEL    "
        + AppDescriptor.MAGIC.to_bytes(8, "big")
        + b"APDesc00"
        + crc.to_bytes(8, "big")
        + (96).to_bytes(4, "big")
        + bytes(4)
        + bytes([42, 95])
        + bytes([2])
        + bytes(1)
        + (1234567890).to_bytes(4, "big")
        + 0x1122334455667788 .to_bytes(8, "big")
        + bytes(16)
        + b"AFTER MODEL"
        + ImageModel.PADDING_BYTE * 5
    ) == im.image
    desc = im.app_descriptor
    desc.version = 1, 5
    desc.flags.release = True
    im.app_descriptor = desc
    assert not im.validate_app_descriptor()
    im.update()
    assert im.validate_app_descriptor()
    crc = (
        CRCComputer()
        .add(
            b"BEFORE MODEL    "
            + AppDescriptor.MAGIC.to_bytes(8, "big")
            + b"APDesc00"
            + (0).to_bytes(8, "big")
            + (96).to_bytes(4, "big")
            + bytes(4)
            + bytes([1, 5])
            + bytes([3])
            + bytes(1)
            + (1234567890).to_bytes(4, "big")
            + 0x1122334455667788 .to_bytes(8, "big")
            + bytes(16)
            + b"AFTER MODEL"
            + ImageModel.PADDING_BYTE * 5
        )
        .value
    )
    assert (
        b"BEFORE MODEL    "
        + AppDescriptor.MAGIC.to_bytes(8, "big")
        + b"APDesc00"
        + crc.to_bytes(8, "big")
        + (96).to_bytes(4, "big")
        + bytes(4)
        + bytes([1, 5])
        + bytes([3])
        + bytes(1)
        + (1234567890).to_bytes(4, "big")
        + 0x1122334455667788 .to_bytes(8, "big")
        + bytes(16)
        + b"AFTER MODEL"
        + ImageModel.PADDING_BYTE * 5
    ) == im.image

    assert (
        _get_output_file_name(
            "firmware-1.bin",
            desc,
        )
        == f"firmware-1-1.5.1122334455667788.{desc.image_crc:016x}.app.release.dirty.bin"
    )


if __name__ == "__main__":
    sys.exit(_main())
