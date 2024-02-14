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

from copy import deepcopy

from serial.tools import list_ports
from typing import Optional, Union, List

import can
from can import exceptions as can_exceptions  # noqa: F401

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
    "encoder_carry": {
        "cmd": 0x30,
        "tx_vals": [],  # byte lengths of value(s) to send
        "rx_vals": [4, 2],  # byte lengths of value(s) to receive # todo: change to struct format string
     },
    "encoder": {
        "cmd": 0x31,
        "tx_vals": [],  # byte lengths of value(s) to send
        "rx_vals": [6],  # byte lengths of value(s) to receive # todo: change to struct format string
    },
}
INV_COMMANDS = {v["cmd"]: k for k, v in COMMANDS.items()}  # inverse lookup table


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
        self.cmd_str: int = cmd if isinstance(cmd, str) else INV_COMMANDS[cmd]
        self.value_bytes: bytearray = bytearray()
        self.values = values if values is not None else []
        rx_or_tx_vals = "rx_vals" if is_rx else "tx_vals"
        val_lengths = COMMANDS[self.cmd_str][rx_or_tx_vals]
        if len(self.values) != len(val_lengths):
            swapped_rx_or_tx = "tx_vals" if is_rx else "rx_vals"
            other_val_lengths = COMMANDS[self.cmd_str][swapped_rx_or_tx]
            hint = ""
            if len(other_val_lengths) == len(self.values):
                hint = f" Maybe you meant to specify is_rx={not is_rx}, " \
                       f"as there are {len(other_val_lengths)} {swapped_rx_or_tx}?"
            raise ValueError(
                f"Got {len(self.values)} instead of {len(val_lengths)} {rx_or_tx_vals} for "
                f"command '{self.cmd_str}' (0x{self.cmd:02X}).{hint}")
        for val_idx, value in enumerate(self.values):
            self.value_bytes += value.to_bytes(val_lengths[val_idx], "big")
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
        rx_or_tx_vals = "rx_vals" if msg.is_rx else "tx_vals"
        val_lengths = COMMANDS[cmd_str][rx_or_tx_vals]
        values = []
        offset = 0
        if len(value_bytes) != sum(val_lengths):
            raise ValueError(
                f"Length of value data '{''.join([format(b, '02X') for b in value_bytes])}' "
                f"({len(msg.data)}-1(cmd)-1(crc)={len(value_bytes)}) does not match "
                f"expected {'RX' if msg.is_rx else 'TX'} value byte lengths {val_lengths}")
        # todo change to struct.unpack
        for val_len in val_lengths:
            val_bytes = value_bytes[offset:offset + val_len]
            value = int.from_bytes(val_bytes, 'big')
            values.append(value)
            offset += val_len
        assert offset == sum(val_lengths), f"byte count mismatch: {offset=} != sum({val_lengths})"
        return cls(can_id, cmd, values, msg.is_rx)

    #    def send(self, bus: can.interface.Bus):
    #        bus.send(self.to_can_msg())
    #
    #    @classmethod
    #    def receive(cls, bus: can.interface.Bus, timeout=0.1):
    #        can_msg = bus.recv(timeout=timeout)
    #        msg = cls.from_can_msg(can_msg)
    #        return msg
    #
    #    @classmethod
    #    def recv(cls, bus: can.interface.Bus, timeout=0.1):
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
        msg1 = cls(6, "encoder_carry", [1, 2], is_rx=True)
        can_msg1 = msg1.to_can_msg()
        print(can_msg1)
        msg2 = cls.from_can_msg(can_msg1)
        print(msg1, "=?=", msg2)
        assert msg1 == msg2, f"{msg1} and {msg2} should be equal"
        assert msg1 == cls(6, "encoder_carry", [1, 2],
                           is_rx=True), f"{msg1} and {cls(6, 'encoder_carry', [1, 2], is_rx=True)} should be equal"
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
        self.bus = can.interface.Bus(bustype=self.bustype, channel=self.device, bitrate=self.bitrate, **self._kwargs)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, value, traceback):
        self.bus.shutdown()

    def send(self, msg_or_can_id: Union[Message, int], cmd: Optional[Union[str, int]],
             values: Optional[List[int]] = None):
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

        :param msg_or_can_id: Either a Message object or a CAN ID (int).
        :param cmd: The command string or integer.
        :param values: The values to send with the command.
        :return: The sent message.
        """
        if isinstance(msg_or_can_id, Message):
            msg = msg_or_can_id
        else:
            can_id = msg_or_can_id
            msg = Message(can_id, cmd, values)
        can_msg = msg.to_can_msg()
        self.bus.send(can_msg)
        return msg

    def receive(self, timeout=0.1):
        """Receive a message from the CAN bus.
        Blocking with timeout. If no message is received, returns None.

        :param timeout: The timeout in seconds. Default is 0.1. Use None for blocking indefinitely.
        :return: The received message, or None if no message was received within the timeout period.
        """
        if self.bus.state != can.BusState.ACTIVE:
            state = "ERROR" if self.bus.state == can.BusState.ERROR else "PASSIVE"
            raise RuntimeError(f"Bus state is {state}. Is the device connected and powered?")
        can_msg = self.bus.recv(timeout=timeout)
        msg = can_msg and Message.from_can_msg(can_msg)
        return msg

    def recv(self, timeout=0.1):
        """Alias for `receive`."""
        return self.receive(timeout=timeout)

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
