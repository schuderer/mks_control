import os
import time
import sys
import termios
from enum import Enum

from pynput import keyboard

from mks_bus import Bus, can_exceptions

# Constants
NUM_AXES = 6
AXES = list(range(NUM_AXES))
ERROR = "error"
STOPPED = "stopped"
MOVING = "moving"
STOPPING = "stopping"

# State machine
ACTIONS = {
    STOPPED: {
        "left":  (("move", [0, 600, 50]), MOVING),
        "right": (("move", [1, 600, 50]), MOVING),
    },
    MOVING: {
        "stop":  (("move", [0, 0, 50]), STOPPING),
    },
    STOPPING: {
        # No commands allowed, wait for stop
    },
    ERROR: {
        # No commands allowed, wait for reset
    },
}

# Globals
quit_app = False
state = [STOPPED] * NUM_AXES
next_command = ["stop" for _ in range(NUM_AXES)]
active_axis = 0
positions = [0] * NUM_AXES


# Helpers
def axis2canid(axis):
    return axis + 1


def canid2axis(can_id):
    return can_id - 1


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


def stop_all():
    for axis in AXES:
        bus.send(axis2canid(axis), "move", [0, 0, 100])


def update_state_from_devices():
    time.sleep(0.05)
    for msg in bus.receive_all():
        can_id = msg.can_id
        axis = canid2axis(can_id)
        if msg.cmd_str == "encoder":
            positions[axis] = msg[0] / 0x4000 * 360
        elif msg.cmd_str == "move":
            # print(f"Move response: {msg}")
            move_state = msg[0]
            if move_state == 0:
                state[axis] = ERROR
            elif move_state == 2:
                state[axis] = STOPPED
        elif msg.cmd_str == "motor_status":
            # print(f"Motor status: {msg}")
            motor_state = msg[0]
            if motor_state == 0:
                state[axis] = ERROR
            elif motor_state == 1:
                state[axis] = STOPPED
            elif motor_state == 2 or motor_state == 4:
                state[axis] = MOVING
            elif motor_state == 3:
                state[axis] = STOPPING
    for axis in AXES:
        bus.send(axis2canid(axis), "encoder")
        bus.send(axis2canid(axis), "motor_status")


def execute_transitions():
    print("State transitions")
    for axis in AXES:
        todo = ACTIONS[state[axis]].get(next_command[axis])
        can_id = axis2canid(axis)
        print(f"{can_id}: {state[axis]} -> {next_command[axis]} -> {todo}")
        if todo is not None:
            cmd_params, new_state = todo
            bus.send(can_id, *cmd_params)
            state[axis] = new_state


# Main
if __name__ == "__main__":
    with Bus() as bus:
        tick = time.time()
        start_keyboard_listener()
        while not quit_app:
            try:
                if time.time() - tick > 0.1:
                    os.system('clear')
                    print("Known devices")
                    for axis, angle in enumerate(positions):
                        print(f"{axis + 1}: {angle:.1f}°")
                    print("\n\n")
                    print(f"Press up/down arrow keys to choose axis, left/right arrow keys to move, esc to quit.")
                    print_axes()
                    print("\n\n")
                    execute_transitions()
                    update_state_from_devices()
                    tick = time.time()
            except Exception as e:
                stop_all()
                raise e
