#
# Copyright (c) 2017-2018 Pavel Kirienko <pavel.kirienko@zubax.com>
#
# This program is distributed under the terms of the MIT software license.
#

import os
import time
import glob
import uavcan
import random
from enum import IntEnum
from logging import getLogger
from contextlib import closing

__all__ = ['Tester']

logger = getLogger(__name__)


CAN_BIT_RATES = [
    1000000,
    500000,
    250000,
    125000,
    100000,
]


class BootloaderError(IntEnum):
    """
    Error codes defined in the bootloader library and its dependencies.
    There are more codes than these, but they are either implementation-specific or ambiguous.
    Over UAVCAN these codes are reported via the vendor-specific status code.
    """
    OK                                      = 0

    BOOTLOADER_INVALID_STATE                = 1001
    BOOTLOADER_APP_IMAGE_TOO_LARGE          = 1002
    BOOTLOADER_ROM_WRITE_FAILURE            = 1003
    BOOTLOADER_INVALID_PARAMS               = 1004

    YMODEM_CHANNEL_WRITE_TIMED_OUT          = 2001
    YMODEM_RETRIES_EXHAUSTED                = 2002
    YMODEM_PROTOCOL_ERROR                   = 2003
    YMODEM_TRANSFER_CANCELLED_BY_REMOTE     = 2004
    YMODEM_REMOTE_REFUSED_TO_PROVIDE_FILE   = 2005
    YMODEM_PORT_ERROR                       = 2006

    UAVCAN_TIMEOUT                          = 3001
    UAVCAN_INTERRUPTED                      = 3002
    UAVCAN_FILE_READ_FAILED                 = 3003


class BootloaderState(IntEnum):
    NO_APP_TO_BOOT          = 0
    BOOT_DELAY              = 1
    BOOT_CANCELLED          = 2
    APP_UPGRADE_IN_PROGRESS = 3
    READY_TO_BOOT           = 4


NODE_STATUS_CONST = uavcan.protocol.NodeStatus()


class TestException(Exception):
    pass


def _maybe(probability=0.5):
    return random.random() <= probability


def _choose_random_path_from_directory(directory: str, glob_expression: str):
    return random.choice(glob.glob(os.path.join(directory, glob_expression)))


def _enforce(condition, fmt, *args):
    if not condition:
        raise TestException(fmt % args)


class Node:
    class TimeoutException(TestException):
        pass

    def __init__(self,
                 can_iface_name: str,
                 bitrate: int,
                 can_dump_directory: str,
                 file_server_lookup_paths: list,
                 self_node_id: int):
        os.makedirs(can_dump_directory, exist_ok=True)
        can_dump_base_name = 'can_%s.dump' % time.strftime('%Y-%m-%d-%H-%M-%S')
        can_dump_file = os.path.join(can_dump_directory, can_dump_base_name)
        self._can_dump_file = open(can_dump_file, 'w', encoding='utf8')

        self._node = uavcan.make_node(can_iface_name,
                                      bitrate=bitrate,
                                      mode=NODE_STATUS_CONST.MODE_OPERATIONAL)
        self._node.node_info.name = 'com.zubax.bootloader_tester'
        self._node.node_id = self_node_id
        self._node.can_driver.add_io_hook(self._can_io_hook)

        self.file_server = uavcan.app.file_server.FileServer(self._node, file_server_lookup_paths)

        self.monitor = uavcan.app.node_monitor.NodeMonitor(self._node)

        self.node_id_allocator = uavcan.app.dynamic_node_id.CentralizedServer(self._node, self.monitor)

        def logging_callback(event):
            logger.info('uavcan.protocol.debug.LogMessage\n%s', uavcan.to_yaml(event))

        self._node.add_handler(uavcan.protocol.debug.LogMessage, logging_callback)

    def _can_io_hook(self, direction, frame):
        # We don't flush for performance reasons
        self._can_dump_file.write('%s %s\n' % (direction.upper(), frame))

    def close(self):
        self._node.close()

    def spin_for(self, duration: float):
        self._node.spin(duration)

    def spin_until(self, termination_condition, check_interval=0.1):
        while not termination_condition():
            self.spin_for(check_interval)

    def request_async(self, payload, dest_node_id, callback=None, timeout=None, priority=None):
        callback = callback if callback is not None else lambda *_: None
        self._node.request(payload, dest_node_id, callback, timeout=timeout, priority=priority)

    def request(self, payload, dest_node_id, timeout=None, priority=None):
        """
        Synchronous request.
        Throws a TimeoutException on timeout.
        """
        result = None

        def callback(event):
            nonlocal result
            if event is not None:
                logger.debug('Response %r from %d\n%s',
                             uavcan.get_uavcan_data_type(event.response),
                             event.transfer.source_node_id,
                             uavcan.to_yaml(event.response))
                assert event.transfer.source_node_id == dest_node_id
                result = event.response
            else:
                raise Node.TimeoutException('Request to node %d with payload %r has timed out in %.3f seconds' %
                                            (dest_node_id, payload, timeout or uavcan.node.DEFAULT_SERVICE_TIMEOUT))

        logger.debug('Synchronously requesting %r from %d\n%s',
                     uavcan.get_uavcan_data_type(payload), dest_node_id, uavcan.to_yaml(payload))

        self.request_async(payload, dest_node_id, callback, timeout, priority)

        while result is None:
            self.spin_for(0.1)

        return result

    @property
    def node_id(self):
        return self._node.node_id


class Tester:
    def __init__(self,
                 can_iface_name: str,
                 device_under_test_uid: bytes,
                 can_dump_directory: str,
                 valid_firmware_dir: str,
                 invalid_firmware_dir: str,
                 node_id: int = 127):
        self._can_iface_name = can_iface_name
        self._device_uid = device_under_test_uid
        self._can_dump_directory = can_dump_directory

        self._valid_firmware_dir = valid_firmware_dir
        self._invalid_firmware_dir = invalid_firmware_dir

        self._self_node_id = int(node_id)
        if not (1 <= self._self_node_id <= 127):
            raise ValueError('Invalid local node ID')

        # Checking that the paths are valid
        _ = _choose_random_path_from_directory(valid_firmware_dir, '*.bin')
        _ = _choose_random_path_from_directory(invalid_firmware_dir, '*.bin')

    def _test_once(self, bitrate, no_application):
        logger.info('Instantiating a new node with bitrate %d bps', bitrate)
        node = Node(self._can_iface_name,
                    bitrate,
                    self._can_dump_directory,
                    [self._valid_firmware_dir, self._invalid_firmware_dir],
                    self._self_node_id)

        with closing(node):
            node.monitor.add_update_handler(lambda e: logger.debug('Node monitor event: %s', e))

            logger.info('Waiting for the device to appear online...')
            deadline = time.monotonic() + 60

            device_node_id = 0

            def is_dut_online():
                nonlocal device_node_id

                def predicate(e: uavcan.app.node_monitor.NodeMonitor.Entry):
                    return (e.info is not None) and bytes(e.info.hardware_version.unique_id) == self._device_uid

                matches = list(node.monitor.find_all(predicate))
                if len(matches) > 0:
                    device_node_id = matches[0].node_id
                    return True

                if time.monotonic() > deadline:
                    raise TestException('Node did not become online')

                return False

            node.spin_until(is_dut_online)

            logger.info('Device online, NID %d', device_node_id)

            orig_node_info = node.monitor.get(device_node_id).info

            def get_node_status():
                try:
                    e = node.monitor.get(device_node_id)
                except KeyError:
                    raise TestException('Node is offline')
                return e.status

            def match_node_health_mode(health, mode):
                try:
                    ns = get_node_status()
                except TestException:
                    return False

                if health is not None:
                    return ns.mode == mode and ns.health == health
                else:
                    return ns.mode == mode

            # Optionally stopping the allocator
            if _maybe():
                node.node_id_allocator.close()
                node.node_id_allocator = None

            # Selecting the firmware, possibly invalid
            using_valid_firmware = _maybe()
            fw_path = _choose_random_path_from_directory(self._valid_firmware_dir
                                                         if using_valid_firmware else
                                                         self._invalid_firmware_dir,
                                                         '*.bin')
            logger.info('Using firmware %r, which is %s', fw_path, 'VALID' if using_valid_firmware else 'INVALID')

            # Requesting a firmware update; about ~50% chance of providing an incorrect server node ID
            begin_fw_update_request = uavcan.protocol.file.BeginFirmwareUpdate.Request()

            begin_fw_update_request.source_node_id = random.choice(list(range(1, 128)) + [node.node_id] * 128)
            using_valid_source_node_id = begin_fw_update_request.source_node_id in (0, node.node_id)

            begin_fw_update_request.image_file_remote_path.path = fw_path

            logger.info('Sending BeginFirmwareUpdate with source NID %d, which is %s',
                        begin_fw_update_request.source_node_id, 'VALID' if using_valid_source_node_id else 'INVALID')

            response = node.request(begin_fw_update_request, device_node_id)

            logger.info('BeginFirmwareUpdate response: %s', response)

            # Spinning for a while in order to update the node status representation locally
            node.spin_for(5)

            logger.info('Latest NodeStatus\n%s', uavcan.to_yaml(get_node_status()))

            # Making sure that NodeInfo values reported by the bootloader and by the application are consistent
            def check_latest_node_info():
                curr_node_info = node.monitor.get(device_node_id).info
                logger.info('Comparing node info, original vs. current:\n%s\n---\n%s',
                            uavcan.to_yaml(orig_node_info), uavcan.to_yaml(curr_node_info))

                _enforce(orig_node_info.hardware_version.major ==
                         curr_node_info.hardware_version.major,
                         'HW version major')

                _enforce(orig_node_info.hardware_version.minor ==
                         curr_node_info.hardware_version.minor,
                         'HW version minor')

                _enforce(orig_node_info.hardware_version.unique_id ==
                         curr_node_info.hardware_version.unique_id,
                         'HW UID')

                _enforce(orig_node_info.hardware_version.certificate_of_authenticity ==
                         curr_node_info.hardware_version.certificate_of_authenticity,
                         'HW COA')

                _enforce(orig_node_info.name == curr_node_info.name,
                         'Node name')

            check_latest_node_info()

            # Checking what state the bootloader is in
            if match_node_health_mode(None, NODE_STATUS_CONST.MODE_OPERATIONAL):
                raise TestException('The node did not enter firmware update mode')

            if using_valid_source_node_id:
                _enforce(match_node_health_mode(NODE_STATUS_CONST.HEALTH_OK,
                                                NODE_STATUS_CONST.MODE_SOFTWARE_UPDATE) or
                         match_node_health_mode(NODE_STATUS_CONST.HEALTH_ERROR,
                                                NODE_STATUS_CONST.MODE_SOFTWARE_UPDATE),
                         'Expected update mode')
                # Continuing with other tests
            else:
                _enforce(match_node_health_mode(NODE_STATUS_CONST.HEALTH_WARNING,
                                                NODE_STATUS_CONST.MODE_SOFTWARE_UPDATE) or
                         match_node_health_mode(NODE_STATUS_CONST.HEALTH_ERROR,
                                                NODE_STATUS_CONST.MODE_SOFTWARE_UPDATE),
                         'Expected failure')
                _enforce(get_node_status().vendor_specific_status_code == BootloaderError.UAVCAN_TIMEOUT,
                         'Expected failure')

                # Maybe restart, maybe not...
                if _maybe():
                    logger.info('Sending RestartRequest')
                    r = uavcan.protocol.RestartNode.Request()
                    r.magic_number = r.MAGIC_NUMBER
                    response = node.request(r, device_node_id)
                    _enforce(response.ok, 'RestartNode was supposed to return ok=True')

                return  # Nothing more to test at this point

            # Waiting for the bootloader to complete downloading, then check the outcome
            logger.info('Waiting for the download to complete...')

            deadline = time.monotonic() + 3600     # An hour - a ridiculously large deadline, but whatever

            def poll():
                _enforce(time.monotonic() < deadline, 'Stuck. :(')

                if _maybe():        # Generating garbage on the bus, because why not
                    if _maybe():
                        r = uavcan.protocol.RestartNode.Request()   # Invalid magic, this is intentional
                    elif _maybe(0.01):
                        r = uavcan.protocol.GetNodeInfo.Request()
                    else:
                        # The bootloader doesn't support this service, there will be no response
                        r = uavcan.protocol.file.Read.Request()
                        r.offset = int(random.random() * 1024 * 1024 * 1024)
                        r.path.path = \
                            'Senora, pray receive with your wonted kindness Senor Don Quixote of La Mancha, '\
                            'whom you see before you, a knight-errant, and the bravest and wisest in the world.'

                    node.request_async(r, device_node_id)

                return not match_node_health_mode(NODE_STATUS_CONST.HEALTH_OK,
                                                  NODE_STATUS_CONST.MODE_SOFTWARE_UPDATE)
            node.spin_until(poll, check_interval=0.01)

            logger.info('Download finished, checking the outcome...')

            if using_valid_firmware:
                if not no_application:
                    _enforce(match_node_health_mode(NODE_STATUS_CONST.HEALTH_OK,
                                                    NODE_STATUS_CONST.MODE_INITIALIZATION),
                             'Expected successful completion')
                    logger.info('Waiting for the board to boot...')
                    node.spin_for(15)
                    _enforce(match_node_health_mode(None, NODE_STATUS_CONST.MODE_OPERATIONAL),
                             'Application did not boot')

                    check_latest_node_info()
            else:
                _enforce(match_node_health_mode(NODE_STATUS_CONST.HEALTH_ERROR,
                                                NODE_STATUS_CONST.MODE_SOFTWARE_UPDATE),
                         'Expected failure')

    def run(self, duration: float, no_application=False):
        """
        Runs the tests for at least :param duration: seconds, or until first error.
        Raises an exception on failure.
        """
        # We can't change the bit rate at run time because the device sticks to the first detected bit rate.
        bitrate = random.choice(CAN_BIT_RATES)

        deadline = time.monotonic() + float(duration)

        while time.monotonic() < deadline:
            self._test_once(bitrate, no_application)
            time.sleep(10)
