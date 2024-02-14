import os
import time

from pynput import keyboard

from mks_bus import Bus, can_exceptions

# Constants
NUM_AXES = 6
AXES = list(range(1, NUM_AXES + 1))

# Globals
quit_app = False
last_key = None


# Helpers
def on_press(key):
    # print('{0} pressed'.format(key))
    ...


def on_release(key):
    print('{0} release'.format(key))
    if key == keyboard.Key.esc:
        print("Stopping application")
        global quit_app
        quit_app = True
        return False


def start_keyboard_listener():
    print("Keybard input active")
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()


# Main
if __name__ == "__main__":
    with Bus() as bus:
        responses = [None] * NUM_AXES
        tick = time.time()
        # start_keyboard_listener()
        while not quit_app:
            try:
                msg = bus.receive()
                if msg is not None:
                    can_id = msg.can_id
                    responses[can_id - 1] = msg
                if time.time() - tick > 0.5:
                    os.system('clear')
                    # print(key_listener)
                    # print(key_listener.running)
                    print("Known devices")
                    for axis, message in enumerate(responses):
                        angle = message and message[0]
                        print(f"{axis + 1}: {message} -- {angle}°")
                    tick = time.time()
                    for axis in AXES:
                        bus.send(axis, "encoder_carry")
            except can_exceptions.CanOperationError as e:
                print("Error: {e}")
                exit()
