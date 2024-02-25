import os
import time
import sys
import termios
from datetime import datetime

from pynput import keyboard

from mks_bus import Bus

# Constants
NUM_AXES = 6
AXES = list(range(NUM_AXES))
ERROR = "error"
INIT = "init"
STOPPED = "stopped"
MOVING = "moving"
STOPPING = "stopping"
HOMING = "homing"
AXES_HOMING_DIRECTION = [None, 1, 0, None, 1, 1]
AXES_ANGLE_RANGE = [4700, -32200, 22700, 1820, 2120, 2120]  # between 0 and this angle in degrees (may be negative)
AXES_SAFE_HOMING_ANGLE = [AXES_ANGLE_RANGE[i] * factor for i, factor in enumerate([0, 0.25, 0.75, 0, 0.5, 0.5])]
AXES_CURRENT_LIMIT = [2000, 1600, 1000, 600, 600, 600]  # in mA
AXES_HOMING_TIMEOUT = [15, 25, 15, 15, 10, 10]

# State machine
ACTIONS = {
    INIT: {
        # No commands allowed, wait for stop
    },
    STOPPED: {
        "left":       (("move", [0, 600, 50]), MOVING),
        "right":      (("move", [1, 600, 50]), MOVING),
        "home_axis":  (lambda axis: sensorless_home(axis), HOMING),
        "home_all":   (lambda _: home_all(), HOMING),
    },
    MOVING: {
        "stop":       (("move", [0, 0, 50]), STOPPING),
    },
    STOPPING: {
        # No commands allowed, wait for stop
    },
    HOMING: {
        # No commands allowed, wait for homing
    },
    ERROR: {
        # No commands allowed, wait for reset
    },
}

# Globals
quit_app = False
state = [STOPPED] * NUM_AXES
next_command = ["init" for _ in range(NUM_AXES)]
active_axis = 0
axis_data = [{
    "can_id": None,
    "state": None,
    "angle": None,
    "timestamp": None
} for _ in range(NUM_AXES)]


# Exceptions
class HomingError(Exception):
    pass


# Helpers
def axis2canid(axis):
    """Convert axis index to can id. Axes are zero-based, while device CAN IDs start at 1."""
    return axis + 1


def canid2axis(can_id):
    """Convert CAN ID to axis index. Axes are zero-based, while device CAN IDs start at 1."""
    return can_id - 1


def encoder_to_angle(encoder_value):
    """Convert encoder value to angle in degrees."""
    return encoder_value / 0x4000 * 360


def angle_to_encoder(angle):
    """Convert angle in degrees to encoder value."""
    return int(angle / 360 * 0x4000)


def send_axes(axes_or_messages, cmd=None, values=None):
    for axis in axes_or_messages:
        if isinstance(axis, int):
            bus.send(axis2canid(axis), cmd, values)
        else:
            bus.send(axis, cmd, values)


# def wait_for(can_id: int, cmd: Union[str, int], value_pattern=None, timeout=5):
#     """Wait until a message with a specific command name or number is received (such as "encoder" or 0x31).
#     Warning: this discards all other messages in the buffer! (for now)
#
#     :param cmd_substring: The substring to look for in the message command.
#     :param value_pattern: A tuple of values to match in the message data. If not None, the message data must match all values in the tuple.
#     :param timeout: The maximum time to wait for the message.
#     :return: The message that was received.
#     """
#     start_time = time.time()
#     got_command = False
#     values = None
#     while time.time() - start_time < timeout:
#         for msg in bus.receive_all():
#             print(f"Received message {msg} while waiting for {can_id} {cmd} with values {value_pattern}")
#             if msg.can_id != can_id:
#                 continue
#             if cmd == msg.cmd or str(cmd).lower() in msg.cmd_str.lower():
#                 got_command = True
#                 values = msg.values
#                 if not value_pattern or all([values[i] == v for i, v in enumerate(value_pattern) if v is not None]):
#                     return msg
#         time.sleep(0.01)
#     raise TimeoutError(f"Timeout after {timeout}s waiting for {can_id} cmd '{cmd}' and vals '{value_pattern}'"
#                        f"{' (got the command, but with values ' + str(values) + ')' if got_command else ''}")


def on_press(key):
    # print('{0} pressed'.format(key))
    if key == keyboard.Key.esc:
        print("Stopping movement")
        stop_all()
    global next_command
    if key == keyboard.Key.left:
        next_command[active_axis] = "left"
    if key == keyboard.Key.right:
        next_command[active_axis] = "right"
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)


def on_release(key):
    # print('{0} release'.format(key))
    if key == keyboard.Key.esc:
        print("Stopping application")
        stop_all()
        global quit_app
        quit_app = True
        return False
    global active_axis
    global next_command
    if key == keyboard.Key.up:
        next_command[active_axis] = "stop"
        active_axis += 1
    if key == keyboard.Key.down:
        next_command[active_axis] = "stop"
        active_axis -= 1
    active_axis = active_axis % NUM_AXES
    if key == keyboard.Key.left or key == keyboard.Key.right:
        next_command[active_axis] = "stop"
    if key == keyboard.KeyCode.from_char('0'):
        stop_all()
        for axis in AXES:
            bus.ask(axis2canid(axis), "set_zero")
    if key == keyboard.KeyCode.from_char('h'):
        next_command[active_axis] = "home_axis"
    if key == keyboard.KeyCode.from_char('H'):
        next_command[active_axis] = "home_all"
    if key == keyboard.KeyCode.from_char('l'):
        bus.send(axis2canid(active_axis), "release_shaft_lock")
    if key == keyboard.KeyCode.from_char('L'):
        for axis in AXES:
            bus.send(axis2canid(axis), "release_shaft_lock")
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)


def start_keyboard_listener():
    print("Keybard input active")
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()


def print_axes():
    print("Control robot arm axes using the arrow keys:")
    for axis in reversed(AXES):
        print(f"{axis2canid(axis)}: {'<- @ ->' if axis == active_axis else '   @   '}")


def print_devices():
    print("Known devices")
    for axis, data in enumerate(axis_data):
        pretty_data = ", ".join(
            [f"{str(k)}: {(format(v, ' >6.1f') if isinstance(v, float) else str(v))}" for k, v in data.items()])
        print(f"{axis + 1}: {pretty_data}")


def stop_all():
    for axis in AXES:
        bus.send(axis2canid(axis), "move", [0, 0, 100])


def update_state_from_devices():
    if any([state[axis] in [INIT, HOMING] for axis in AXES]):
        return
    time.sleep(0.05)
    for axis in AXES:
        (encoder,) = bus.ask(axis2canid(axis), "encoder")
        angle = encoder_to_angle(encoder)
        axis_data[axis].update(angle=angle, timestamp=timestamp())

        (motor_state,) = bus.ask(axis2canid(axis), "motor_status")
        # 0: error, 1: stopped, 2: accelerate, 3: decelerate, 4: full speed, 5: homing
        state[axis] = [ERROR, STOPPED, MOVING, STOPPING, MOVING, MOVING][motor_state]
        pretty_state = format(f"{state[axis]} ({motor_state})", "<12")
        axis_data[axis].update(state=pretty_state, timestamp=timestamp())

        (lock_state,) = bus.ask(axis2canid(axis), "shaft_lock_status")
        axis_data[axis].update(shaft_lock=lock_state, timestamp=timestamp())

    # for msg in bus.receive_all():
    #     can_id = msg.can_id
    #     axis = canid2axis(can_id)
    #     if msg.cmd_str == "encoder":
    #         angle = encoder_to_angle(msg[0])
    #         axis_data[axis].update(angle=angle, timestamp=timestamp())
    #     # elif msg.cmd_str == "move":
    #     #     # print(f"Move response: {msg}")
    #     #     move_state = msg[0]
    #     #     if move_state == 0:
    #     #         state[axis] = ERROR
    #     #     elif move_state == 2:
    #     #         state[axis] = STOPPED
    #     elif msg.cmd_str == "motor_status":
    #         # print(f"Motor status: {msg}")
    #         motor_state = msg[0]
    #         if motor_state == 0:
    #             state[axis] = ERROR
    #         elif motor_state == 1:
    #             state[axis] = STOPPED
    #         elif motor_state == 2 or motor_state == 4:
    #             state[axis] = MOVING
    #         elif motor_state == 3:
    #             state[axis] = STOPPING
    #         pretty_state = format(f"{state[axis]} ({motor_state})", "<12")
    #         axis_data[axis].update(state=pretty_state, timestamp=timestamp())
    #     elif msg.cmd_str == "shaft_lock_status":
    #         lock_state = msg[0]
    #         axis_data[axis].update(lock=lock_state, timestamp=timestamp())
    # for axis in AXES:
    #     bus.send(axis2canid(axis), "encoder")
    #     bus.send(axis2canid(axis), "motor_status")
    #     bus.send(axis2canid(axis), "shaft_lock_status")


def timestamp():
    return datetime.utcnow().strftime("%H:%M:%S.%f")[:-3]


def execute_transitions():
    print("ID Curr-state command  action  state-after-action")
    curr_cmd = next_command.copy()
    for axis in AXES:
        can_id = axis2canid(axis)
        action, new_state = ACTIONS[state[axis]].get(curr_cmd[axis], (None, state[axis]))
        print(f"{can_id}: {state[axis]} -> {curr_cmd[axis]} -> {action},  {new_state}")
        if action is not None:
            state[axis] = new_state
            if callable(action):
                action(axis)
            else:
                bus.ask(can_id, *action)
        if next_command[axis] == curr_cmd[axis]:
            next_command[axis] = None  # Clear action if not changed by other thread


def sensorless_home(axis):
    global state
    if AXES_HOMING_DIRECTION[axis] is None:
        print(f"Axis {axis} currently does not support homing")
        if state[axis] == HOMING:
            state[axis] = STOPPED
        return
    try:
        can_id = axis2canid(axis)
        bus.send(can_id, "set_work_current", [550])
        # bus.send(can_id, "set_shaft_lock", [True])
        # bus.wait_for(can_id, "set_shaft_lock", [True])
        bus.ask(can_id, "set_shaft_lock", [True], answer_pattern=[True])
        # bus.send(can_id, "release_shaft_lock")
        bus.ask(can_id, "release_shaft_lock")  # may also be False if already released, so no answer_pattern
        bus.send(can_id, "move", [AXES_HOMING_DIRECTION[axis], 250, 50])

        start_time = time.time()
        locked = False
        print(f"Homing axis {axis}")
        while not locked:
            if time.time() - start_time > AXES_HOMING_TIMEOUT[axis]:
                bus.send(can_id, "move", [0, 0, 100])  # stop
                raise HomingError(f"Homing movement of axis {axis} timed out. Consider increasing AXES_HOMING_TIMEOUT for this axis.")
            time.sleep(0.5)
            # bus.send(can_id, "shaft_lock_status")
            # responses = bus.receive_all()
            # for msg in responses:
            #     print(msg)
            #     if msg.cmd_str == "shaft_lock_status" and msg[0]:
            #         print(f"Axis {axis} locked")
            #         locked = True
            locked_resp = bus.ask(can_id, "shaft_lock_status", timeout=0.2)
            if locked_resp[0]:
                # print(f"Axis {axis} reached home position.")
                locked = True

        print(f"Homed axis {axis}. Performing cleanup.")
        # bus.send(can_id, "release_shaft_lock")
        # bus.wait_for(can_id, "release_shaft_lock", [True])
        bus.ask(can_id, "release_shaft_lock", answer_pattern=[True])
        # bus.send(can_id, "set_zero")
        # bus.wait_for(can_id, "set_zero", [True])
        bus.ask(can_id, "set_zero", answer_pattern=[True])
        # bus.send(can_id, "set_work_current", [AXES_CURRENT_LIMIT[axis]])
        # bus.wait_for(can_id, "set_work_current", [True])
        bus.ask(can_id, "set_work_current", [AXES_CURRENT_LIMIT[axis]], answer_pattern=[True])
        time.sleep(0.25)
        print(f"Homed axis {axis}")
    except TimeoutError as e:
        raise HomingError(f"Error homing axis {axis}: {e}")
    if state[axis] == HOMING:
        state[axis] = STOPPED


def home_all():
    for axis in reversed(AXES):
        if AXES_HOMING_DIRECTION[axis] is None:
            continue
        can_id = axis2canid(axis)
        sensorless_home(axis)
        # bus.send(can_id, "encoder")
        # response = bus.wait_for(can_id, "encoder")
        (encoder_val,) = bus.ask(can_id, "encoder")
        print(f"Axis {axis} homed and zeroed. Current angle: {encoder_to_angle(encoder_val)}")
        bus.send(can_id, "move_to", [500, 50, angle_to_encoder(AXES_SAFE_HOMING_ANGLE[axis])])
        start_time = time.time()
        while True:
            response = bus.wait_for(can_id, "move_to", timeout=AXES_HOMING_TIMEOUT[axis])
            # print(f"Motor status: {response}")
            if response[0] in [2, 3]:
                break
            elif response[0] == 0 or time.time() - start_time > AXES_HOMING_TIMEOUT[axis]:
                raise HomingError(f"Homing of axis {axis} failed. Could not move to neutral position.")
            time.sleep(0.1)
    print(f"All axes homed")
    # for axis in AXES:
    #     bus.send(axis2canid(axis), "move_to", [500, 50, angle_to_encoder(AXES_SAFE_HOMING_ANGLE[axis])])
    global state
    if any([state[axis] == HOMING for axis in AXES]):
        state = [STOPPED] * len(AXES)


def init():
    # Initialize
    print("Initializing")
    send_axes(AXES, "set_response", [True])
    send_axes(AXES, "release_shaft_lock")
    global state
    state = [STOPPED] * len(AXES)
    # # for_all(lambda i: bus.send(i, "set_key_lock", [False]))  # what does this do?


# Main
if __name__ == "__main__":
    with Bus() as bus:
        tick = time.time()
        start_keyboard_listener()
        while not quit_app:
            try:
                if time.time() - tick > 0.2:
                    os.system('clear')
                    print_devices()
                    print("\n\n")
                    print(f"Press up/down arrow keys to choose axis, left/right arrow keys to move, esc to quit.")
                    print(f"'0' to set zero, 'l' to release lock, 'L' to release all locks.")
                    print_axes()
                    print("\n\n")
                    execute_transitions()
                    update_state_from_devices()
                    tick = time.time()
            except Exception as e:
                stop_all()
                raise e

