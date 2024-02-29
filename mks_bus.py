"""Convenience layer for communicating with MKS SERVO42D/57D CAN stepper drivers

This module provides a high-ish-level interface for communicating with MKS SERVO42D/57D CAN stepper
drivers. It provides a human-readable list of commands and a convenient way to send and receive
messages using plain integer values.

It is built on top of the `python-can` library.

The `Bus` class provides a context manager for opening and closing a CAN bus. It also provides
methods for sending and receiving messages to and from the bus.

The `Message` class provides a convenient way to create and parse messages to and from the bus.
It handles value conversion and checksum calculation.

"""

from __future__ import annotations

import logging
import re
import time
from copy import deepcopy

import bitstring
from serial.tools import list_ports
from typing import Optional, Union, List

import can
from can import exceptions as can_exceptions  # noqa: F401

logger = logging.getLogger(__name__)

# CAN Message Structure
#
# ID + Mode + Speed + Acceleration + Position + CRC
#
#    ID: 2 bytes - ID 01 to ID 06 (set on MKS SERVO42D_CAN and SERVO57D_CAN boards)
#    Mode: 1 byte - F5 for absolute axis
#    Speed: 2 bytes
#    Acceleration: Configured as 02 (default)
#    Position: Converted from G-code
#    CRC: Checksum (not a real CRC) calculated by the formula:
#         CRC = (ID + Mode + Speed + Acceleration + Position) & 0xFF

# Constants
COMMANDS = {  # From "MKS SERVO42&57D_CAN User Manual V1.0.3.pdf"
    # The tx/rx_struct values are format strings to pack/unpack values into/from bytes.
    # See https://docs.python.org/3/library/struct.html#format-strings.
    "encoder_carry": {  # Motor encoder value with separate carry value for whole turns
        "cmd": 0x30,
        "tx_struct": "",  # nothing to send
        "rx_struct": "intbe32, uintbe16",  # carry value, encoder value. +0x4000 = one turn counterclockwise
     },
    "encoder": {  # Motor encoder value (adds up over multiple turns)
        "cmd": 0x31,
        "tx_struct": "",  # nothing to send
        "rx_struct": "intbe48",  # encoder value (adds up). +0x4000 = one turn counterclockwise
    },
    "rpm": {  # Read current motor RPM
        "cmd": 0x32,
        "tx_struct": "",  # nothing to send
        "rx_struct": "intbe16",  # RPM (counterclockwise is positive, clockwise is negative)
    },
    # From GPT4:
    "pulses": {  # Read the number of pulses received
        "cmd": 0x33,
        "tx_struct": "",  # nothing to send
        "rx_struct": "intbe32",  # Number of pulses received
    },
    "io_status": {  # Read the status of IO ports
        "cmd": 0x34,
        "tx_struct": "",  # nothing to send
        "rx_struct": "pad4, bool, bool, bool, bool",  # Status of IO ports # OUT_2, OUT_1, IN_2, IN_1
    },
    "error_angle": {  # Read the error of the motor shaft angle
        "cmd": 0x39,
        "tx_struct": "",  # nothing to send
        "rx_struct": "intbe32",  # Motor shaft angle error (0~51200 corresponds to 0~360°)
    },
    "enabled": {  # Read the Enable pin's status
        "cmd": 0x3A,
        "tx_struct": "",  # nothing to send
        "rx_struct": "pad7,bool",  # Enable pin status (1 enabled, 0 disabled)
    },
    "zero_return_status": {  # Read the status of returning to zero point at power on
        "cmd": 0x3B,
        "tx_struct": "",  # nothing to send
        "rx_struct": "uintbe8",  # 0: going to zero, 1: success, 2: fail
    },
    "release_shaft_lock": {  # Release the motor shaft lock protection
        "cmd": 0x3D,
        "tx_struct": "",  # nothing to send
        "rx_struct": "pad7,bool",  # Status of the release operation (1: success, 0: fail)
    },
    "shaft_lock_status": {  # Read the motor shaft lock protection status
        "cmd": 0x3E,
        "tx_struct": "",  # nothing to send
        "rx_struct": "pad7,bool",  # Shaft lock protection status (1: protected, 0: not protected)
    },
    "calibrate_encoder": {  # Calibrate the motor encoder (NOTE: requires no load on motor)
        "cmd": 0x80,
        "tx_struct": "pad:8",  # 1 value byte of 0x00 should be sent according to docs
        "rx_struct": "uintbe8",  # Status: 0: calibrating, 1: success, 2: fail
    },
    "set_work_mode": {  # Same as the "Mode" option on screen
        "cmd": 0x82,
        "tx_struct": "uintbe8",  # modes 0 through 5: CR_OPEN, CR_CLOSE, CR_vFOC, SR_OPEN, SR_CLOSE, SR_vFOC
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "set_work_current": {  # For VFOC modes, set amps are max amps, otherwise fixed amps. Same as the "Ma" option on screen.
        "cmd": 0x83,
        "tx_struct": "uintbe16",  # Current in mA. SERVO042D max = 3000, SERVO57D max = 5200
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "set_subdivision": {  # Set the microsteps. Same as the "MStep" option on screen.
        "cmd": 0x84,
        "tx_struct": "uintbe8",  # Microsteps 0 .. 255
        "rx_struct": "pad7,bool",  # Status: 1: set success, 0: set fail
        # Note: The speed value is calibrated based on 16/32/64
        # subdivisions, and the speeds of other subdivisions need to be
        # calculated based on 16 subdivisions.
        # For example, setting speed=1200
        # At 8 subdivisions, the speed is 2400 (RPM)
        # At 16/32/64 subdivisions, the speed is 1200 (RPM)
        # At 128 subdivisions, the speed is 150 (RPM)
    },
    # Skipped over 0x85
    "set_direction": {  # Set the direction. Same as the "Dir" option on screen.
        "cmd": 0x86,
        "tx_struct": "pad7,bool",  # 0: clockwise, 1: counterclockwise
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    # Skipped over 0x87
    "set_shaft_lock": {  # Set the motor shaft locked-rotor protection function (Protect)
        "cmd": 0x88,
        "tx_struct": "pad7,bool",  # 1: enable, 0: disable
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "set_interpolation": {  # Set the subdivision interpolation function (Mplyer)
        "cmd": 0x89,
        "tx_struct": "pad7,bool",  # 1: enable, 0: disable
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "set_bitrate": {  # Set the CAN bitRate (CanRate)
        "cmd": 0x8A,
        "tx_struct": "uintbe8",  # 0: 125Kbps, 1: 250Kbps, 2: 500Kbps, 3: 1Mbps
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "set_response": {  # Enable or disable the CAN answer (CanRSP / Uplink frame)
        # Note: when disabled, query the motor state with 0xF1
        "cmd": 0x8C,
        "tx_struct": "pad7,bool",  # 1: enable response, 0: disable response
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "set_key_lock": {  # Lock or unlock the key
        "cmd": 0x8C,
        "tx_struct": "pad7,bool",  # 1: lock, 0: unlock
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    # Skipping a few commands
    "set_home_params": {  # Set the homing parameters
        "cmd": 0x90,
        "tx_struct": "pad7,bool, pad7,bool, uintbe16, pad7,bool",  # homeTrig 0: falling edge, 1: rising edge; homeDir: 0: CW, 1: CCW; homeSpeed: 0~3000 RPM; EndLimit: 0: disable, 1: enable
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    "home": {  # Go to the home position
        "cmd": 0x91,
        "tx_struct": "",  # nothing to send
        "rx_struct": "uintbe8",  # Status: 0: failed, 1: started, 2: success
    },
    "set_zero": {  # Zero the axis
        "cmd": 0x92,
        "tx_struct": "",  # nothing to send
        "rx_struct": "pad7,bool",  # Status: 1: success, 0: fail
    },
    # Skipping a few
    "motor_status": {  # Query the motor status
        "cmd": 0xF1,
        "tx_struct": "",  # nothing to send
        "rx_struct": "uintbe8",  # status, see below
        # status = 0 query fail.
        # status = 1 motor stop
        # status = 2 motor speed up
        # status = 3 motor speed down
        # status = 4 motor full speed
        # status = 5 motor is homing
    },
    "enable_motor": {  # Enable the motor
        "cmd": 0xF3,
        "tx_struct": "pad7,bool",  # 0: disable, 1: enable
        "rx_struct": "pad7,bool",  # Status: 1: set success, 0: set fail
    },
    "move": {  # Run the motor in speed mode (run with acceleration and speed). Stop with speed = 0
        # AVOID ACCELERATION 0 which would be a sudden start/stop!
        "cmd": 0xF6,
        "tx_struct": "bool, pad:3, uintbe12, uintbe8",  # Dir 0: CCW, 1: CW; Speed 0~3000 RPM; Accel 0~255
        "rx_struct": "uintbe8",  # Status: 0: fail, 1: started/stopping, 2: stop success
    },
    "save_clean_speed_param": {  # Save or clean the parameter in speed mode
        # Good to clean immediately at startup so that motor does not start moving when powered on
        # I would not use save, for the same reason.
        "cmd": 0xFF,
        "tx_struct": "uintbe8",  # 0xC8: save, 0xCA: clean
        "rx_struct": "pad7,bool",  # Status: 0: fail, 1: success
    },
    "move_pulses": {  # Move the motor a number of steps with speed and acceleration. Stop with speed = 0
        # AVOID ACCELERATION 0 which would be a sudden start/stop!
        "cmd": 0xFD,
        "tx_struct": "bool, pad:3, uintbe12, uintbe8, uintbe24",  # Dir 0: CCW, 1: CW; Speed 0~3000 RPM; Accel 0~255, pulses
        "rx_struct": "uintbe8",  # Status: 0: fail, 1: starting/stopping, 2: complete, 3: stopped by limit
    },
    "move_by": {  # Move the motor by a certain angle difference. Stop with speed = 0
        # AVOID ACCELERATION 0 which would be a sudden start/stop!
        "cmd": 0xF4,
        "tx_struct": "uintbe16, uintbe8, intbe24",  # Speed 0~3000 RPM; Accel 0~255, relAngle
        "rx_struct": "uintbe8",  # Status: 0: fail, 1: starting/stopping, 2: complete, 3: stopped by limit
    },
    "move_to": {  # Move the motor to a certain angle. Stop with speed = 0
        # AVOID ACCELERATION 0 which would be a sudden start/stop!
        "cmd": 0xF5,
        "tx_struct": "uintbe16, uintbe8, intbe24",  # Speed 0~3000 RPM; Accel 0~255, absAngle
        "rx_struct": "uintbe8",  # Status: 0: fail, 1: starting/stopping, 2: complete, 3: stopped by limit
    },

}
INV_COMMANDS = {v["cmd"]: k for k, v in COMMANDS.items()}  # inverse lookup table
BIG_ENDIAN_WIDTH_REGEXP = re.compile(r'([^ ,]+be:?)(\d+)', flags=re.IGNORECASE)
RECV_TIMEOUT = 0.01  # seconds
WAIT_FOR_TIMEOUT = 5  # seconds


def _unpack(fmt, bitstream):
    """Unpack a bitstream using the given format.
    Extends the bitstring unpack method to support big-endian widths that are not whole bytes.

    :param fmt: The format string
    :param bitstream: The bitstream to unpack
    :return: The unpacked values
    """
    remaining_bs = bitstream
    result = []
    current_part = []
    sp = [s.strip() for s in fmt.split(',')]
    for curr_fmt in sp:
        be_item = BIG_ENDIAN_WIDTH_REGEXP.findall(curr_fmt)
        if be_item and int(be_item[0][1]) % 8 != 0:  # split/hack unpacking at left edge of non-byte-BE-values
            dtype_prefix, var_width = be_item[0]
            # print(f"dtype_prefix: {dtype_prefix}, var_width: {var_width}")
            var_width = int(var_width)
            result_part = remaining_bs.unpack(current_part)
            # print(f"result_part: {result_part}")
            result.extend(result_part)
            bits_used = bitstring.pack(current_part, *result_part).len
            # print(f"bits_used: {bits_used}")
            required_padding = 8 - (var_width % 8)
            # print(f"required_padding for {var_width}-bit value: {required_padding}")
            remaining_bs = bitstring.BitStream(required_padding) + remaining_bs[bits_used:]
            # print(f"{remaining_bs.bin=} ({remaining_bs.len} bits)")
            curr_fmt = f"{dtype_prefix}{required_padding + var_width}"
            # print(f"{curr_fmt=}")
            current_part = []
        current_part.append(curr_fmt)
    result.extend(remaining_bs.unpack(current_part))
    return result


def _pack(fmt, *values):
    """Pack values into a bitstream using the given format.
    Extends the bitstring pack method to support big-endian widths that are not whole bytes.
    :param fmt: The format string
    :param values: The values to pack
    :return: The packed bitstream
    """
    remaining_vals = list(values).copy()
    result = bitstring.BitStream()
    formats = [s.strip() for s in fmt.split(',')]
    for curr_fmt in reversed(formats):
        # print(f"{remaining_vals=}, {curr_fmt=}")
        if curr_fmt.strip().lower().startswith('pad') or curr_fmt.strip() == '':
            curr_val = []
        else:
            curr_val = [remaining_vals.pop()]
        # print(f"at {curr_val} (remaining: {remaining_vals}) and {curr_fmt=}")

        # deal with partial-byte big-endians
        be_item = BIG_ENDIAN_WIDTH_REGEXP.findall(curr_fmt)
        if be_item and int(be_item[0][1]) % 8 != 0:
            dtype_prefix, var_width = be_item[0]
            var_width = int(var_width)
            # print(f"dtype_prefix: {dtype_prefix}, var_width: {var_width}")
            # add padding to format string to make it divisible by 8
            required_padding = 8 - (var_width % 8)
            be_fmt = f"{dtype_prefix}{required_padding + var_width}"
            bs = bitstring.pack(be_fmt, *curr_val)
            bs = bs[required_padding:]  # remove the bits added by the padding from the result
        else:
            bs = bitstring.pack(curr_fmt, *curr_val)
        # print(f"{bs.bin=}")
        result = bs + result
        # print(f"{result.bin=}")
    return result


def _test_packing():
    # test the pack function
    st = "bool, pad:3, uintbe12, uintbe:8"
    bs = bitstring.BitStream(bytes=bytearray([0b10001111, 0b11111110, 0b01111111]))
    bs_packed = _pack(st, True, 4094, 127)
    assert bs_packed == bs, f"_pack failed test. Expected {bs}, got {bs_packed}"

    # test the unpack function
    st = "bool, pad:3, uintbe12, uintbe:8"
    bs = bitstring.BitStream(bytes=bytearray([0b10001111, 0b11111110, 0b01111111]))
    bs_unpacked = _unpack(st, bs)
    assert bs_unpacked == [True, 4094, 127], f"_unpack failed test. Expected [True, 4094, 127], got {bs_unpacked}"


_test_packing()


class Message:
    """Represents an MKS-SERVO CAN message to be sent or received."""

    def __init__(self, can_id, cmd: Union[str, int], values: Optional[List[int]] = None, is_rx=False):
        """Create an MKS CAN message to be sent or received.
        :param can_id: The ID of the MKS-SERVO CAN device to communicate with.
        :param cmd: The command to send or receive. Can be a string or integer.
        :param values: The value(s) to send or receive. Optional list of integers.
        :param is_rx: Whether the message is to be received or sent (default: False).
        """
        self.can_id: int = can_id
        self.cmd: int = cmd if isinstance(cmd, int) else COMMANDS[cmd]["cmd"]
        self.cmd_str: str = cmd if isinstance(cmd, str) else INV_COMMANDS[cmd]
        self.values = values if values is not None else []
        rx_or_tx_struct = "rx_struct" if is_rx else "tx_struct"
        val_struct = COMMANDS[self.cmd_str][rx_or_tx_struct]
        struct_vals = "" if val_struct.strip() == "" else val_struct.split(",")
        struct_vals = [v for v in struct_vals if not v.lower().strip().startswith("pad")]
        num_struct_vals = len(struct_vals)
        if len(self.values) == num_struct_vals:
            # self.value_bytes = bitstring.pack(val_struct, *self.values).bytes
            # The expression above does not support partial-byte big-endian values, using our own wrapper:
            self.value_bytes = _pack(val_struct, *self.values).bytes
            # print(f"My bytes: {self.value_bytes}")
            # print(f"self.value_bytes: {self.value_bytes} -- should fit {val_struct=} {val_lengths=} {num_struct_vals=} {self.values=}")
        else:
            # if len(self.values) != len(val_lengths):
            swapped_rx_or_tx = "tx_struct" if is_rx else "rx_struct"
            other_val_structs = COMMANDS[self.cmd_str][swapped_rx_or_tx].split(",")
            other_val_structs = [v for v in other_val_structs if not v.lower().strip().startswith("pad")]
            # print(f"other_val_structs: {other_val_structs}")
            hint = ""
            if len(other_val_structs) == len(self.values):
                hint = f" Maybe you meant to specify is_rx={not is_rx}, " \
                       f"as there are {len(other_val_structs)} {swapped_rx_or_tx} values?"
            raise ValueError(
                f"Got {len(self.values)} instead of {num_struct_vals} {rx_or_tx_struct} values for "
                f"command '{self.cmd_str}' (0x{self.cmd:02X}).{hint}")
        # for val_idx, value in enumerate(self.values):
        #     self.value_bytes += value.to_bytes(val_lengths[val_idx], "big")
        self.crc = sum([self.can_id, self.cmd, *self.value_bytes]) & 0xFF
        self.is_rx = is_rx  # tx = send, rx = receive

    def to_can_msg(self) -> can.Message:
        """Convert the MKS CAN message to a python-can message.
        :return: A python-can message object.
        """
        data = [self.cmd, *self.value_bytes, self.crc]
        return can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False, is_rx=self.is_rx)

    @classmethod
    def from_can_msg(cls, msg: can.Message) -> Message:
        """Create an MKS CAN message from a python-can message.
        :param msg: A python-can message object.
        :return: An MKS CAN message object.
        """
        can_id = msg.arbitration_id
        cmd = msg.data[0]
        cmd_str = INV_COMMANDS[cmd]
        _ = msg.data[-1]  # crc will be ignored
        value_bytes = msg.data[1:-1]
        rx_or_tx_struct = "rx_struct" if msg.is_rx else "tx_struct"
        val_struct = COMMANDS[cmd_str][rx_or_tx_struct]

        b = bitstring.BitStream(bytes=value_bytes)
        # values = list(b.unpack(val_struct))
        # The expression above does not support partial-byte big-endian values, using our own wrapper:
        values = list(_unpack(val_struct, b))

        return cls(can_id, cmd, values, msg.is_rx)

    #    def send(self, bus: can.interface.Bus):
    #        bus.send(self.to_can_msg())
    #
    #    @classmethod
    #    def receive(cls, bus: can.interface.Bus, timeout=RECV_TIMEOUT):
    #        can_msg = bus.recv(timeout=timeout)
    #        msg = cls.from_can_msg(can_msg)
    #        return msg
    #
    #    @classmethod
    #    def recv(cls, bus: can.interface.Bus, timeout=RECV_TIMEOUT):
    #        return cls.receive(bus, timeout=timeout)

    def __str__(self):
        return f"{'RX' if self.is_rx else 'TX'} {self.can_id} {self.cmd_str} (0x{self.cmd:02X}): {self.values}"

    def __repr__(self):
        return f"Message({self.can_id}, {self.cmd_str}, values={self.values}, is_rx={self.is_rx})"

    def __eq__(self, other: Message):
        return (
            self.can_id == other.can_id and
            self.cmd == other.cmd and
            self.cmd_str == other.cmd_str and
            self.values == other.values and
            self.value_bytes == other.value_bytes and
            self.is_rx == other.is_rx and
            self.crc == other.crc
        )

    def __hash__(self):
        return hash((self.can_id, self.cmd, self.cmd_str, self.values, self.value_bytes, self.is_rx, self.crc))

    def __len__(self):
        return len(self.values)

    def __getitem__(self, item):
        return self.values[item]

    def __iter__(self):
        return iter(self.values)

    def __contains__(self, item):
        return item in self.values

    @classmethod
    def test(cls):
        """Test the Message class."""
        print(cls(1, 0x31))
        print(cls(2, "encoder"))
        try:
            _bla = cls(3, "encoder", [14])  # should raise ValueError
            raise Exception(
                f"Message(3, 'encoder', [14]) should have raised an error, but was successfully constructed as {_bla}")
        except ValueError as e:
            print(f"Successfully triggered ValueError: {e}")
        print(cls(4, "encoder", [14], is_rx=True))
        print(cls(5, "encoder_carry", [1, 2], is_rx=True))
        print(cls(7, "encoder_carry", []), "-->")
        print(cls(7, "encoder_carry", []).to_can_msg())
        print(cls(6, "encoder_carry", [-1, 2], is_rx=True), "-->")
        msg1 = cls(6, "encoder_carry", [-1, 2], is_rx=True)
        can_msg1 = msg1.to_can_msg()
        print(can_msg1)
        msg2 = cls.from_can_msg(can_msg1)
        print(msg1, "=?=", msg2)
        assert msg1 == msg2, f"{msg1} and {msg2} should be equal"
        assert msg1 == cls(6, "encoder_carry", [-1, 2], is_rx=True), \
            f"{msg1} and {cls(6, 'encoder_carry', [-1, 2], is_rx=True)} should be equal"
        assert cls(6, "encoder_carry", []) != cls(7, "encoder_carry", []), f"should not be equal"


class Bus:
    """A class and contextmanager for managing a CAN bus of MKS SERVO devices."""

    def __init__(self, bustype='slcan', device: Optional[str] = None, bitrate=500000, device_searchstr="canable",
                 **kwargs):
        """Initializes a CAN bus of MKS SERVO devices.

        Example usage:
        ```python
        with Bus() as bus:
            msg = Message(1, "encoder")
            bus.send(msg)
            msg = bus.recv()
            print(msg)
        ```

        :param bustype: The type of CAN bus to use. Default is 'slcan'.
        :param device: The device name of the CAN bus. Default is None, which will search for a serial device with
            `device_searchstr` in its name.
        :param bitrate: The bitrate of the CAN bus. Default is 500000.
        :param device_searchstr: The string to search for in the device name (case-insensitive). Default is "canable".
        :param kwargs: Additional keyword arguments to pass to the `can.interface.Bus` constructor.
        """
        if device is None:
            device = self._find_device(substr=device_searchstr)
            if device is None:
                raise RuntimeError(f"No serial CAN device with {device_searchstr} in its name. Is it plugged in?")
        self.device = device
        self.bitrate = bitrate
        self.bustype = bustype
        self._kwargs = kwargs
        self._msg_buffer = []  # For functions like wait_for, we have to keep non-matching messages in a buffer
        self.bus = can.interface.Bus(bustype=self.bustype, channel=self.device, bitrate=self.bitrate, **self._kwargs)

    def close(self):
        self.bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, value, traceback):
        self.close()

    def send(self, can_id_or_msg: Union[Message, can.Message, int], cmd: Optional[Union[str, int]] = None,
             *values: Union[list[Union[int, bool]], int, bool]):
        """Send a message on the CAN bus.

        Examples:
        ```python
        # using direct arguments:
        bus.send(1, "encoder")
        bus.send(2, 0x31)
        bus.send(4, "move", [14], is_rx=True)
        # using a message object:
        msg = Message(5, "encoder_carry", [1, 2], is_rx=True)
        bus.send(msg)
        ```

        :param can_id_or_msg: Either a CAN ID (int) or a complete Message object.
        :param cmd: The command string or integer.
        :param values: The values to send with the command.
        :return: The sent message.
        """
        # For convenience, support both a list of values and values as *args
        values = list(values)
        if len(values) == 1:
            if values[0] is None:
                values = None
            elif isinstance(values[0], (list, tuple)):
                values = values[0]

        if isinstance(can_id_or_msg, Message):
            msg = can_id_or_msg
        elif isinstance(can_id_or_msg, can.Message):
            msg = Message.from_can_msg(can_id_or_msg)
        else:
            can_id = can_id_or_msg
            msg = Message(can_id, cmd, values)
        can_msg = msg.to_can_msg()
        self.bus.send(can_msg)
        return msg

    def ask(self, can_id_or_msg: Union[Message, can.Message, int], cmd: Optional[Union[str, int]] = None,
            *values: Union[list[Union[int, bool]], int, bool], answer_pattern: Optional[list] = None, timeout=WAIT_FOR_TIMEOUT):
        """Send a message and wait for a response. Combines `send` and `wait_for`.
        See `send` and `wait_for` for more details.
        """
        sent_msg = self.send(can_id_or_msg, cmd, *values)
        return self.wait_for(sent_msg, value_pattern=answer_pattern, timeout=timeout)

    def receive(self, timeout=RECV_TIMEOUT):
        """Receive a message from the CAN bus.
        Blocking with timeout. If no message is received, returns None.

        :param timeout: The timeout in seconds. Default is 0.1. Use None for blocking indefinitely.
        :return: The received message, or None if no message was received within the timeout period.
        """
        if len(self._msg_buffer) > 0:
            return self._msg_buffer.pop(0)
        if self.bus.state != can.BusState.ACTIVE:
            state = "ERROR" if self.bus.state == can.BusState.ERROR else "PASSIVE"
            raise RuntimeError(f"Bus state is {state}. Is the device connected and powered?")
        can_msg = self.bus.recv(timeout=timeout)
        msg = can_msg and Message.from_can_msg(can_msg)
        # if msg:
        #     print(f"RAW RECEIVE: {msg}")
        return msg

    def recv(self, timeout=RECV_TIMEOUT):
        """Alias for `receive`."""
        return self.receive(timeout=timeout)

    def receive_all(self, timeout=RECV_TIMEOUT):
        """Receive all messages from the CAN bus.
        Blocking with timeout. If no message is received, returns an empty list.

        :param timeout: The timeout in seconds. Default is 0.1. Use None for blocking indefinitely.
        :return: A list of all received messages, or an empty list if no message was received within the timeout period.
        """
        msgs = []
        while True:
            msg = self.receive(timeout=timeout)
            if msg is None:
                break
            msgs.append(msg)
        return msgs

    def flush(self):
        """Flush the receive buffer, discarding all messages."""
        while self.receive(timeout=0.01):
            pass

    def wait_for(self, can_id_or_msg: Union[Message, can.Message, int], cmd: Optional[Union[str, int]] = None, value_pattern: Optional[list] = None, timeout=WAIT_FOR_TIMEOUT):
        """Wait until a message from a CAN ID with a specific command is received (such as "encoder" or 0x31) or
        until the timeout is reached (default: 5 seconds).

        You can optionally specify a `value_pattern` list to match the message's values. None-entries in the pattern are
        ignored. Note that this will examine and discard any preceding non-matching messages of the same CAN ID and
        command. If you want more control over this, don't specify a pattern and check the received message yourself.

        To easily wait for an answer to a message you just sent, you can pass the sent message object as
        `can_id_or_msg` (it is returned by the `send` method), and it will match the CAN ID and command.

        :param can_id_or_msg: Either a CAN ID (int) or a complete Message object.
        :param cmd: The command to wait for for this CAN ID (not needed when a message object is provided).
        :param value_pattern: Optional: A list or tuple of values to match in the message data.
                                        If not None, the message data must match all values in the tuple (logical AND).
                                        None-entries in the pattern are ignored.
                                        If a list of value patterns is provided, the any match of the value patterns counts as success (logical OR).
        :param timeout: The maximum time to wait for a matching message.
        :return: The first matching message that was received.

        :raises TimeoutError: If no matching message was received within the timeout period.
        """
        if isinstance(can_id_or_msg, can.Message):
            can_id_or_msg = Message.from_can_msg(can_id_or_msg)
        if isinstance(can_id_or_msg, Message):
            can_id = can_id_or_msg.can_id
            cmd = can_id_or_msg.cmd_str or can_id_or_msg.cmd
            # values != value_pattern, so this needs to be provided separately by the user if needed
        else:
            can_id = can_id_or_msg

        # For convenience, if `value_pattern` is a single value, wrap it in a list
        if value_pattern is not None:
            value_pattern = list(value_pattern)

        start_time = time.time()
        got_command = False
        values = None
        while time.time() - start_time < timeout:
            # print("pre-receive")
            msgs = self.receive_all()
            # print(f"post-receive: {msgs}")
            for msg_idx, msg in enumerate(msgs):
                if msg.can_id != can_id:
                    # Ignoring non-matching CAN ID
                    self._msg_buffer.append(msg)
                    continue
                if cmd == msg.cmd or str(cmd).lower() == msg.cmd_str.lower():
                    # Consuming all messages with matching command
                    got_command = True
                    values = msg.values
                    if value_pattern is None:
                        # None matches anything
                        self._msg_buffer.extend(msgs[msg_idx+1:])  # Regression: don't throw out remaining messages
                        return msg
                    else:
                        if len(value_pattern) > 0 and not isinstance(value_pattern[0], (list, tuple)):
                            value_patterns = [value_pattern]
                        else:
                            value_patterns = value_pattern
                        if len(value_patterns[0]) != len(values):
                            raise ValueError(f"Value pattern length {len(value_patterns[0])} does not match message length {len(values)}")
                        # print(f"Checking message {msg} for values {value_pattern}")
                        # Work with list of value patterns, check if any of them match
                        for pattern in value_patterns:
                            # print(f"Checking pattern {pattern} against {values}")
                            if all([values[i] == v for i, v in enumerate(pattern) if v is not None]):
                                # Got a match.
                                self._msg_buffer.extend(msgs[msg_idx + 1:])  # Regression: don't throw out remaining messages
                                return msg
                    print(f"Discarded message {msg} while waiting for values {value_pattern}")
                    # print("(the current buffer is \n" + '\n'.join([str(m) for m in self._msg_buffer]))
                else:
                    # Ignoring non-matching command
                    self._msg_buffer.append(msg)
            time.sleep(RECV_TIMEOUT)
        hint = f" (got the command, but with values {values})" if got_command else ""
        raise TimeoutError(f"Timeout after {timeout}s waiting for msg {can_id} '{cmd}' "
                           f"and vals '{value_pattern}'{hint}")

    @property
    def commands(self):
        """A copy of the dict of all available command name (keys) and their details."""
        return deepcopy(COMMANDS)

    @staticmethod
    def _find_device(substr="canable"):
        """Find a serial device with a given substring in its name."""
        for com_device in list_ports.comports():
            readable_name = str(com_device)
            if substr in str(readable_name):
                print(f"Found '{substr}' device: {readable_name}")
                return com_device.device
        return None
