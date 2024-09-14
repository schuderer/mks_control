import os
import sys
import termios
import time
import traceback
from copy import deepcopy
from datetime import datetime
from typing import Optional, Any

from pynput import keyboard

import arctos_arm as arm
from kinematics import KinematicChain
from mks_bus import encoder_to_motor_angle, motor_angle_to_encoder
from motion import AXES, axis2canid, joint_angles_abs, motor_angles_abs, stop_all, move_to_motor_position, \
    BusProxy, current_bus_proxy, ERROR, STOPPED, STOPPING, MOVING


# Constants
# ERROR = "error"  # from motion.py
# STOPPED = "stopped"
# MOVING = "moving"
# STOPPING = "stopping"
INIT = "init"
HOMING = "homing"
PLAYBACK = "playback"

# State machine
ACTIONS = {
    INIT: {
        # No commands allowed, wait for stop
    },
    STOPPED: {
        "init":       (lambda axis: init(axis), INIT),
        "left":       (("move", [0, 200, 150]), MOVING),
        "right":      (("move", [1, 200, 150]), MOVING),
        "home_axis":  (lambda axis: home(axis), HOMING),
        "home_all":   (lambda _: home_all(zero_pose=arm.MOVE_ZERO_AFTER_HOME), HOMING),
        "play_pause": (lambda _: play_pause(), PLAYBACK),
        "input":      (lambda axis: input_angle(axis), STOPPED),
        "go_to_saved_position":      (lambda axis: go_to_saved_position(), STOPPED),
        "move_all_to_zero": (lambda _: move_to_motor_position(motor_angles_abs([0] * arm.NUM_AXES), bus=current_bus_proxy.get()), STOPPED),
        "release_lock": (("release_shaft_lock", []), STOPPED),
        "set_zero":   (("set_zero", []), STOPPED),
    },
    MOVING: {
        "stop":       (("move", [0, 0, 150]), STOPPING),
    },
    STOPPING: {
        # Underscore prefix = default action, auto-fires when no other action is specified
        # Target state None = don't change state (action is responsible for state transition)
        "_check_stopped": (lambda axis: check_stopped(axis), None),
    },
    HOMING: {
        # No commands allowed, wait for homing
    },
    ERROR: {
        # No commands allowed, wait for reset
    },
    PLAYBACK: {
        "play_pause":  (("move", [0, 0, 50]), STOPPING),
    }
}

# Globals
keyboard_listener: Optional[keyboard.Listener] = None
quit_app = False
state = [STOPPED] * arm.NUM_AXES
next_command: list[Optional[str]] = ["init" for _ in range(arm.NUM_AXES)]
active_axis = 0
axis_data: list[dict[str, Optional[Any]]] = [{
    # "can_id": axis2canid(axis),
    "state": STOPPED,
    "angle": None,
    "motor_angle": None,
    "timestamp": None
} for axis in AXES]
sequence = []  # Lists of lists of saved positions

# Exceptions
class HomingError(Exception):
    pass


# Helpers
def on_press(key):
    # print('{0} pressed'.format(key))
    global next_command
    if key == keyboard.Key.left:
        next_command[active_axis] = "left"
    if key == keyboard.Key.right:
        next_command[active_axis] = "right"
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)


def on_release(key):
    # print('{0} release'.format(key))
    # bus = current_bus_proxy.get()
    if key == keyboard.Key.esc:
        print("Stopping application")
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
    active_axis = active_axis % arm.NUM_AXES
    if key == keyboard.Key.left or key == keyboard.Key.right:
        print(f"RELEASE key {key} (axis {active_axis + 1})")
        next_command[active_axis] = "stop"
    if key == keyboard.KeyCode.from_char('z'):  # todo: happens rather often by accident
        next_command[active_axis] = "set_zero"
    if key == keyboard.KeyCode.from_char('Z'):  # todo: happens rather often by accident
        for axis in AXES:
            next_command[axis] = "set_zero"
    if key == keyboard.KeyCode.from_char('h'):
        next_command[active_axis] = "home_axis"
    if key == keyboard.KeyCode.from_char('H'):
        next_command[active_axis] = "home_all"
    if key == keyboard.KeyCode.from_char('l'):
        next_command[active_axis] = "release_lock"
    if key == keyboard.KeyCode.from_char('L'):
        for axis in AXES:
            next_command[axis] = "release_lock"
    if key == keyboard.KeyCode.from_char('s'):
        save_position()
    if key == keyboard.KeyCode.from_char('p'):
        next_command[active_axis] = "play_pause"
    if key == keyboard.KeyCode.from_char('c'):
        print("Clearing position sequence")
        time.sleep(0.5)
        sequence.clear()
    if key == keyboard.Key.enter:
        next_command[active_axis] = "input"
    if key == keyboard.KeyCode.from_char('g'):
        next_command[active_axis] = "go_to_saved_position"
    if key == keyboard.KeyCode.from_char('0'):
        next_command[active_axis] = "move_all_to_zero"
    if key == keyboard.KeyCode.from_char('0'):
        next_command[active_axis] = "move_all_to_zero"
    if hasattr(key, 'char') and key.char in [str(i) for i in range(1, arm.NUM_AXES + 1)]:
        active_axis = int(key.char) - 1
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)


def start_keyboard_listener():
    print("Keybard input active")
    global keyboard_listener
    keyboard_listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    keyboard_listener.start()


def print_axes():
    print("Control robot arm axes using the arrow keys:")
    for axis in reversed(AXES):
        print(f"{axis2canid(axis)}: {'<- @ ->' if axis == active_axis else '   @   '}")


def print_devices():
    print("Known devices")
    for axis, data in reversed(list(enumerate(axis_data))):
        pretty_data = ", ".join(
            [f"{str(k)}: {(format(v, ' >6.1f') if isinstance(v, float) else str(v))}" for k, v in data.items()])
        print(f"{axis + 1}: {pretty_data}")


def input_angle(axis, bus=None):
    bus = bus or current_bus_proxy.get()
    keyboard_listener.stop()
    time.sleep(1)
    try:
        # new_angle = float(input(f"Enter MOTOR angle for axis {active_axis}: "))
        new_angle = float(input(f"Enter joint angle for axis {active_axis}: "))
        joint_angles = [axis_data[axis]["angle"] for axis in AXES]
        joint_angles[axis] = new_angle
        motor_angles = motor_angles_abs(joint_angles)
    except ValueError as e:
        print(f"Invalid angle: {e}")
        time.sleep(1)
        start_keyboard_listener()
        return
    move_to_motor_position(motor_angles, override=True, bus=bus)
    # bus.ask(axis2canid(active_axis), "move_to", [600, 50, motor_angle_to_encoder(new_motor_angle)], answer_pattern=[[0], [2], [3]],
    #         timeout=arm.AXES_MOVE_TIMEOUT[active_axis])
    start_keyboard_listener()



def save_position():
    joint_pos = [axis_data[axis]["angle"] for axis in AXES]
    if None in joint_pos:
        print(f"Cannot save current position as it is not known completely: {joint_pos}")
        time.sleep(1)
        return
    sequence.append(joint_pos)
    print(f"Saved position {len(sequence)}:\n{joint_pos}")


def go_to_saved_position(bus=None):
    bus = bus or current_bus_proxy.get()
    keyboard_listener.stop()
    time.sleep(1)
    pos_idx = None
    try:
        global sequence
        if len(sequence) == 0:
            sequence = arm.ARM_DEMO_SEQUENCE
        pos_idx = input(f"Enter position number to go to (0 to {len(sequence)-1}): ")
        joint_pos = sequence[int(pos_idx)]
        motor_pos = motor_angles_abs(joint_pos)
        move_to_motor_position(motor_pos, bus=bus)

        print("Waiting for end of trajectory...")
        while (s := bus.get_state()["mode"]) != "idle":
            print(f"Waiting for end of trajectory. Current motion controller state: {s}")
            time.sleep(1)
    except ValueError as e:
        print(f"Invalid number: {pos_idx} ({e})")
    finally:
        time.sleep(1)
        start_keyboard_listener()


def check_stopped(axis, bus=None):
    bus = bus or current_bus_proxy.get()
    (motor_status_code,) = bus.ask(axis2canid(axis), "motor_status", timeout=5.0)
    if motor_status_code == 1:
        state[axis] = STOPPED

def play_pause(bus=None):
    bus = bus or current_bus_proxy.get()
    global sequence
    if len(sequence) == 0:
        sequence = arm.ARM_DEMO_SEQUENCE
    print(f"Playing/pausing sequence {sequence}")
    bus.flush()
    for axis in AXES:
        if state[axis] == PLAYBACK:
            stop_all()
            state[axis] = STOPPED
        else:
            state[axis] = PLAYBACK
    start_pos = None  # Initially start from current position
    for j in range(3):  # TODO: remove this
        for i, position in enumerate(sequence):
            motor_pos = motor_angles_abs(position)
            print(f"Planning position {i + 1}")
            # Important to update the start_pos with the previous end position
            start_pos = move_to_motor_position(motor_pos, start_pos=start_pos, bus=bus)
            print(f"Position {i + 1} planned")

    print("Waiting for end of trajectory...")
    while (s := bus.get_state()["mode"]) != "idle":
        print(f"Waiting for end of trajectory. Current motion controller state: {s}")
        time.sleep(1)

    # return to stopped status
    for axis in AXES:
        if state[axis] == PLAYBACK:
            state[axis] = STOPPED

    print("Playback completed")


def update_state_from_devices(bus=None):
    bus = bus or current_bus_proxy.get()
    global state, axis_data

    retrieved_state = bus.get_state()["motors"]
    low_level_state = [retrieved_state[axis]["state"] for axis in AXES]
    for axis in AXES:
        ll_state = low_level_state[axis]
        if state[axis] in [INIT, HOMING, PLAYBACK]:
            # Don't meddle with high-level states
            continue
        if state[axis] in [STOPPED]:
            state[axis] = ll_state

    motor_angles = [retrieved_state[axis]["angle"] for axis in AXES]
    joint_angles = joint_angles_abs(motor_angles)
    for axis in AXES:
        motor_lock_state = retrieved_state[axis]["locked"]
        pretty_state = format(f"{state[axis]}", "<12")
        timestamp = retrieved_state[axis]["timestamp"]
        pretty_timestamp = timestamp.strftime("%H:%M:%S.%f")[:-3] if timestamp is not None else None
        axis_data[axis].update(
            motor_angle=motor_angles[axis],
            angle=joint_angles[axis],
            state=pretty_state,
            shaft_lock=motor_lock_state,
            timestamp=pretty_timestamp
        )

# def timestamp():
#     return datetime.utcnow().strftime("%H:%M:%S.%f")[:-3]


def home(axis, bus=None):
    bus = bus or current_bus_proxy.get()
    can_id = axis2canid(axis)
    if arm.AXES_HOMING_DIRECTION[axis] is None:  # Homing disabled
        print(f"Axis {axis} currently does not support homing")
        if state[axis] == HOMING:
            state[axis] = STOPPED
        return
    elif arm.AXES_HOMING_DIRECTION[axis] == -1:  # Native homing specified
        try:
            # If currently resting on endstop, move off the endstop for better accuracy
            (_, _, _, active_low_endstop) = bus.ask(can_id, "io_status")
            # if not active_low_endstop:  # TODO
            #     bus.ask(can_id, "move_by", [60, 50, -3000], answer_pattern=[2])
            # Home to endstop
            bus.ask(can_id, "home", answer_pattern=[1])  # 1 = has started
            bus.wait_for(can_id, "home", timeout=arm.AXES_MOVE_TIMEOUT[axis], value_pattern=[2])
            bus.ask(can_id, "set_zero", answer_pattern=[True])  # this should not be necessary, but it is :(
        except TimeoutError as e:
            raise HomingError(f"Endstop homing of axis {axis} timed out. {e}"
                              f"Consider increasing AXES_HOMING_TIMEOUT for axis {axis}.")
    else:
        # Sensorless homing
        sensorless_home(axis, bus=bus)

    # Move to neutral position
    (encoder_val,) = bus.ask(can_id, "encoder")
    print(f"Axis {axis} homed and zeroed. Current angle: {encoder_to_motor_angle(encoder_val):.1f}°")
    bus.ask(can_id, "move_to",
            [500, 50, motor_angle_to_encoder(arm.AXES_SAFE_HOMING_ANGLE[axis])],
            answer_pattern=[2], timeout=arm.AXES_MOVE_TIMEOUT[axis])
    if state[axis] == HOMING:
        state[axis] = STOPPED


def sensorless_home(axis, bus=None):
    bus = bus or current_bus_proxy.get()
    global state
    if arm.AXES_HOMING_DIRECTION[axis] is None:
        print(f"Axis {axis} currently does not support homing")
        if state[axis] == HOMING:
            state[axis] = STOPPED
        return
    try:
        can_id = axis2canid(axis)
        bus.send(can_id, "set_work_current", [550])
        bus.ask(can_id, "set_shaft_lock", [True], answer_pattern=[True])
        bus.ask(can_id, "release_shaft_lock")  # may also be False if already released, so no answer_pattern
        bus.send(can_id, "move", [arm.AXES_HOMING_DIRECTION[axis], 250, 50])

        start_time = time.time()
        locked = False
        print(f"Sensorless homing of axis {axis}")
        while not locked:
            if time.time() - start_time > arm.AXES_MOVE_TIMEOUT[axis]:
                bus.send(can_id, "move", [0, 0, 100])  # stop
                raise HomingError(f"Homing movement of axis {axis} timed out. Consider increasing AXES_HOMING_TIMEOUT for this axis.")
            time.sleep(0.5)
            locked_resp = bus.ask(can_id, "shaft_lock_status", timeout=0.2)
            if locked_resp[0]:
                # print(f"Axis {axis} reached home position.")
                locked = True

        print(f"Homed axis {axis}. Performing cleanup.")
        bus.ask(can_id, "release_shaft_lock", answer_pattern=[True])
        bus.ask(can_id, "set_zero", answer_pattern=[True])
        bus.ask(can_id, "set_work_current", [arm.AXES_CURRENT_LIMIT[axis]], answer_pattern=[True])
        time.sleep(0.25)
        print(f"Homed axis {axis}")
    except TimeoutError as e:
        raise HomingError(f"Error on sensorless homing of axis {axis}: {e}")


def home_all(zero_pose=True, bus=None):
    """
    Home all axes according to configuration in arm.py.
    :param zero_pose: Move to zero pose after homing
    :type zero_pose: bool
    """
    bus = bus or current_bus_proxy.get()
    global state
    for axis in arm.HOMING_ORDER:
        home(axis, bus=bus)
    print(f"All axes homed")
    if zero_pose:
        zero_motor_angles = motor_angles_abs([0] * arm.NUM_AXES)
        for axis in arm.HOMING_ORDER:
            zero_angle = zero_motor_angles[axis]
            bus.send(axis2canid(axis), "move_to", [500, 50, motor_angle_to_encoder(zero_angle)])
            state[axis] = MOVING
        for axis in arm.HOMING_ORDER:
            (err_code, ) = bus.wait_for(axis2canid(axis), "move_to", value_pattern=[(2,), (0,)], timeout=arm.AXES_MOVE_TIMEOUT[axis])
            state[axis] = STOPPED
            if err_code == 0:
                print(f"Ignoring error while moving {axis} to zero position.")
    for axis, curr_state in enumerate(state):
        if state[axis] == HOMING:
            state[axis] = STOPPED


def execute_transitions(bus=None):
    bus = bus or current_bus_proxy.get()
    print("ID Curr-state command  action  state-after-action")
    curr_cmd = next_command.copy()
    for axis in reversed(AXES):
        curr_state = state[axis]
        can_id = axis2canid(axis)
        if curr_cmd[axis] not in ACTIONS[curr_state]:
            try:
                # Default action for current state
                curr_cmd[axis] = [k for k in ACTIONS[curr_state].keys() if k.startswith("_")][0]
            except IndexError:
                pass
                # print(f"No valid actions for state {curr_state}, command {curr_cmd[axis]} (axis {axis})")
                # continue
        action, new_state = ACTIONS[curr_state].get(curr_cmd[axis], (None, curr_state))
        print(f"{can_id}: {curr_state} -> {curr_cmd[axis]} -> {action},  {new_state}")
        if action is not None:
            state[axis] = new_state if new_state is not None else curr_state  # None state = action responsible for changing it
            if callable(action):
                action(axis)
            else:
                print(bus.ask(can_id, *action))
        if next_command[axis] == curr_cmd[axis]:
            next_command[axis] = None  # Clear action if not changed by other thread


def init(axis, bus=None):
    bus = bus or current_bus_proxy.get()
    print(f"Initializing axis {axis}")
    can_id = axis2canid(axis)
    bus.ask(can_id, "set_response", [True], answer_pattern=[None])
    bus.ask(can_id, "set_shaft_lock", [True], answer_pattern=[True])
    bus.ask(can_id, "release_shaft_lock", answer_pattern=[None])
    bus.ask(can_id, "set_work_current", [arm.AXES_CURRENT_LIMIT[axis]], answer_pattern=[True])
    # bus.ask(can_id, "set_zero", answer_pattern=[1])
    global state
    state[axis] = STOPPED
    # # for_all(lambda i: bus.send(i, "set_key_lock", [False]))  # what does this do?



def main():
    with BusProxy() as bus:
        chain = KinematicChain.from_configuration(arm)
        # chain.visualize_link_angles([0] * len(chain), interactive=True)
        tick = time.time()
        start_keyboard_listener()
        try:
            # start_control(bus=bus)
            while not quit_app:
                if time.time() - tick > 0.1:
                    os.system('clear')
                    print(f"{' '*85}{(time.time() - tick)*1000:.1f}ms tick")
                    print_devices()
                    print("\n\n")
                    print(f"Press up/down arrow keys to choose axis, left/right arrow keys to move, esc to quit.")
                    print(f"'l' to release lock, 'L' to release all locks, 'h' to home axis, 'H' to home all axes.")
                    print(f"'0' to move to zero pose, 'z' to zero axis, 'enter' to input angle for current axis.")
                    print(f"'s' to save position in sequence, 'p' to play/pause sequence, 'c' to clear sequence,")
                    print(f"'g' to go to saved position.")
                    print_axes()
                    print("\n\n")
                    execute_transitions(bus=bus)
                    update_state_from_devices(bus=bus)
                    joint_angles = [axis_data[axis]["angle"] for axis in AXES]
                    print("Current position:", joint_angles)
                    # if not None in joint_angles:
                    #     chain.update_visualization(joint_angles, use_degrees=True)
                    tick = time.time()
                    chain.sleep(0.05)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"An error occurred: {e}")
            print(traceback.format_exc())
        finally:
            # stop_control()
            ...

        # stop_all()
        # chain.close_visualization()


if __name__ == "__main__":
    main()