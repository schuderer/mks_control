import os
import time

from serial.tools import list_ports
import can

# Constants
NUM_AXES = 6
AXES = range(1, NUM_AXES + 1)

# globals
quit_app = False

# Initialize zero positions and last positions
initial_positions = [0] * NUM_AXES
last_positions = [0] * NUM_AXES


def find_device(substr="canable"):
    for com_device in list_ports.comports():
        readable_name = str(com_device)
        if substr in str(readable_name):
            print(f"Found '{substr}' device: {readable_name}")
            return com_device.device
    return None


def calc_checksum(axis, data):
    return sum([axis, *data]) & 0xFF


def get_move_message_ARCTOS(axis_id, speed, position, gear_ratio=1.0):
    can_id = format(axis_id, '02X')
    speed_hex = format(speed, '04X')
    mode = 'F5'  # absolute
    accel = '02'  # default

    # Calculate relative position based on the initial position
    rel_position = int((position * gear_ratio - initial_positions[axis_id - 1]) * 100)

    # Handle signed 24-bit integer using two's complement representation
    rel_position_hex = format(rel_position & 0xFFFFFF, '06X')

    # Update last_position for the axis
    last_positions[axis_id - 1] = position * gear_ratio

    return can_id + mode + speed_hex + accel + rel_position_hex


def message(can_id, raw_data):
    checksum = calc_checksum(can_id, raw_data)
    data = [*raw_data, checksum]
    return can.Message(arbitration_id=can_id, data=data, is_extended_id=False)


def send_msg(can_id, raw_data):
    msg = message(can_id, raw_data)
    try:  # todo proper error handling
        bus.send(msg)
        print(f"Sent <{msg}>")
    except can.CanError as e:
        print("Could not send <{msg}>: {e}")


def request_encoder(can_id):
    send_msg(can_id, [0x31])


def recv_msg(timeout=0.1):
    rmsg = bus.recv(timeout=0.1)
    can_id = rmsg.arbitration_id
    message = rmsg.data
    cmd = message[0]
    data = int.from_bytes(message[1:-1], 'little')
    crc = message[-1]
    return can_id, data


if __name__ == "__main__":
    device = find_device()
    if device is None:
        print("Could not detect serial CAN device. Is it plugged in?")
        exit(1)

    with can.interface.Bus(
            bustype='slcan',
            channel=device,
            bitrate=500000
    ) as bus:
        responses = [None] * 6
        tick = time.time()
        # start_keyboard_listener()
        while not quit_app:
            try:
                rmsg = bus.recv(timeout=0.1)
                # print(f"Received {rmsg}")
                if rmsg is not None:
                    can_id = rmsg.arbitration_id
                    data = rmsg.data
                    responses[can_id - 1] = data
                if time.time() - tick > 0.5:
                    os.system('clear')
                    # print(key_listener)
                    # print(key_listener.running)
                    print("Known devices")
                    for axis, data in enumerate(responses):
                        print(f"{axis + 1}: {data}")
                    tick = time.time()
                    for axis in AXES:
                        request_encoder(axis)
            except can.exceptions.CanOperationError as e:
                print("Error: {e}")
                exit()
