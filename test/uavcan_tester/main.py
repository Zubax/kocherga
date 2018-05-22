#!/usr/bin/env python3
#
# Copyright (c) 2017-2018 Pavel Kirienko <pavel.kirienko@zubax.com>
#
# This program is distributed under the terms of the MIT software license.
#
# This is a tool used to stress test the Zubax Embedded Bootloader.
#

import os
import sys
import argparse
import logging
import binascii

try:
    # noinspection PyUnresolvedReferences
    import uavcan
except ImportError:
    print('Missing PyUAVCAN, please install it: pip3 install "uavcan>=1.0.0.dev30"', file=sys.stderr)
    exit(1)

import uavcan_tester


def _configure_file_logging():
    log_file_path = os.path.splitext(os.path.basename(__file__))[0] + '.log'

    formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-8s %(name)s: %(message)s')

    handler = logging.FileHandler(log_file_path)
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(formatter)

    logging.root.addHandler(handler)
    logging.root.setLevel(logging.DEBUG)


_configure_file_logging()


logger = logging.getLogger('main')


def _configure_stderr_logging(verbosity):
    level = {
        0: logging.INFO,
        1: logging.DEBUG
    }.get(verbosity or 0, logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s %(levelname)-.1s %(name)s: %(message)s', datefmt='%H:%M:%S')

    handler = logging.StreamHandler(sys.stderr)
    handler.setLevel(level)
    handler.setFormatter(formatter)

    logging.root.addHandler(handler)


def main() -> int:
    argparser = argparse.ArgumentParser(description='Zubax Embedded Bootloader automated test script')
    argparser.add_argument('iface', help='name of the CAN interface, e.g. "can0", "/dev/ttyACM0"')
    argparser.add_argument('uid', help='unique ID of the device under test as a hex string, e.g. '
                           '37ffdc05465430353344164300000000, or as a base64 encoded string, e.g. '
                           '"N//cBUZUMDUzRBZDAAAAAA==". The string may contain spaces, they will be removed.')
    argparser.add_argument('valid_fw_dir', help='path to the directory with valid firmware images')
    argparser.add_argument('invalid_fw_dir', help='path to the directory with invalid firmware images')
    argparser.add_argument('--verbose', '-v', action='count', help='verbosity level (-v, -vv)')
    argparser.add_argument('--nid', help='local node ID used by the script (default: 127)', type=int, default=127)
    argparser.add_argument('--no-application', action='store_true',
                           help='do not make assumption about the application; work only with the bootloader')
    argparser.add_argument('--duration-min', help='minimal testing duration, in minutes (default: infinity)',
                           type=float, default=1e9)
    args = argparser.parse_args()

    _configure_stderr_logging(args.verbose)

    iface = args.iface
    duration = float(args.duration_min) * 60
    no_application = args.no_application

    try:
        args.uid = args.uid.replace(' ', '')
        if len(args.uid) == 32:
            uid = binascii.unhexlify(args.uid)
        else:
            uid = binascii.a2b_base64(args.uid)
    except Exception as ex:
        logger.error('Could not parse UID: %r', ex, exc_info=True)
        return 1

    logger.info('Started; iface %s, UID %s', iface, binascii.hexlify(uid).decode('utf8'))

    try:
        t = uavcan_tester.Tester(iface, uid, os.getcwd(), args.valid_fw_dir, args.invalid_fw_dir, args.nid)
        t.run(duration, no_application=no_application)
    except Exception as ex:
        logger.error('Test failure: %r', ex, exc_info=True)
        return 1
    else:
        return 0


if __name__ == '__main__':
    exit(main())
