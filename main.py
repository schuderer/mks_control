import os
import time
import sys
import termios
from datetime import datetime
from math import sqrt
from typing import Optional, Any, Union

from pynput import keyboard

import arctos_arm as arm
from kinematics import KinematicChain
from mks_bus import Bus

# Constants
AXES = list(range(arm.NUM_AXES))
ERROR = "error"
INIT = "init"
STOPPED = "stopped"
MOVING = "moving"
STOPPING = "stopping"
HOMING = "homing"
PLAYBACK = "playback"

# State machine
ACTIONS = {
    INIT: {
        # No commands allowed, wait for stop
    },
    STOPPED: {
        "init":       (lambda axis: init(axis), INIT),
        "left":       (("move", [0, 200, 50]), MOVING),
        "right":      (("move", [1, 200, 50]), MOVING),
        "home_axis":  (lambda axis: home(axis), HOMING),
        "home_all":   (lambda _: home_all(), HOMING),
        "play_pause": (lambda _: play_pause(), PLAYBACK),
        "input":      (lambda axis: input_angle(axis), STOPPED),
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
    "can_id": None,
    "state": None,
    "angle": None,
    "timestamp": None
} for _ in range(arm.NUM_AXES)]
sequence = []  # Lists of lists of saved positions

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


def encoder_to_motor_angle(encoder_value):
    """Convert encoder value to angle in degrees."""
    return encoder_value / 0x4000 * 360


def motor_angle_to_encoder(angle):
    """Convert angle in degrees to encoder value."""
    return int(angle / 360 * 0x4000)


def joint_angles_rel(motor_angle: Union[float, list[float]], axis: Optional[int] = None):
    """Convert motor angle change in degrees to joint angle changes in degrees.
    Provide either a list of motor angles for all axes, or an axis index and the angle change for that axis.
    :param motor_angle: angle change in degrees or a list of angle changes in degrees for all motor axes
    :param axis: index of the axis (if one single motor angle is specified)
    """
    if not isinstance(motor_angle, list):
        motor_angle = [0 if a != axis else motor_angle for a in AXES]
    joint_angles = [0] * arm.NUM_AXES  # we're lucky in that num_motors = num_joints = NUM_AXES
    for motor in AXES:
        # print(f"{motor=}")
        for joint in AXES:
            # print(f"{joint=}")
            motor_joint_ratio = arm.AXES_ANGLE_RATIO[motor][joint]
            if motor_joint_ratio != 0:
                # print(f"{joint_angles[joint]=} += {motor_angle[motor]=} / {motor_joint_ratio=}")
                joint_angles[joint] += motor_angle[motor] / motor_joint_ratio
                # print(f"{joint_angles[joint]=}")
    # print(f"translated relative {motor_angle=} to {joint_angles=}")
    return joint_angles


def joint_angles_abs(motor_angle: Union[float, list[float]], axis: Optional[int] = None):
    """Convert absolute motor angle in degrees to absolute joint angle in degrees.
    Provide either a list of motor angles for all axes, or an axis index and the angle for that axis.
    :param motor_angle: angle in degrees or a list of angles in degrees for all motor axes
    :param axis: index of the axis (if one single motor angle is specified)
    """
    rel_angles = joint_angles_rel(motor_angle, axis=axis)
    abs_angles = [a + offset for a, offset in zip(rel_angles, arm.JOINT_ZERO_OFFSET)]
    # print(f"translated absolute {motor_angle=} to {abs_angles=}")
    return abs_angles


def send_axes(axes_or_messages, cmd=None, values=None, answer_pattern=None):
    for axis_msg in axes_or_messages:
        if isinstance(axis_msg, int):
            axis = axis_msg
            if answer_pattern:
                bus.ask(axis2canid(axis), cmd, values, answer_pattern=answer_pattern)
            else:
                bus.send(axis2canid(axis), cmd, values)
        else:
            msg = axis_msg
            bus.send(msg)


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
    active_axis = active_axis % arm.NUM_AXES
    if key == keyboard.Key.left or key == keyboard.Key.right:
        next_command[active_axis] = "stop"
    # if key == keyboard.KeyCode.from_char('0'):  # happens too often by accident
    #     stop_all()
    #     for axis in AXES:
    #         bus.ask(axis2canid(axis), "set_zero")
    if key == keyboard.KeyCode.from_char('h'):
        next_command[active_axis] = "home_axis"
    if key == keyboard.KeyCode.from_char('H'):
        next_command[active_axis] = "home_all"
    if key == keyboard.KeyCode.from_char('l'):
        bus.send(axis2canid(active_axis), "release_shaft_lock")
    if key == keyboard.KeyCode.from_char('L'):
        for axis in AXES:
            bus.send(axis2canid(axis), "release_shaft_lock")
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


def stop_all():
    for axis in AXES:
        bus.send(axis2canid(axis), "move", [0, 0, 100])


def input_angle(axis):
    keyboard_listener.stop()
    time.sleep(1)
    try:
        new_angle = float(input(f"Enter MOTOR angle for axis {active_axis}: "))  # todo accept joint angles
    except ValueError:
        print("Invalid angle")
        time.sleep(1)
        start_keyboard_listener()
        return
    bus.ask(axis2canid(active_axis), "move_to", [600, 50, motor_angle_to_encoder(new_angle)], answer_pattern=[[0], [2], [3]],
            timeout=arm.AXES_MOVE_TIMEOUT[active_axis])
    start_keyboard_listener()



def save_position():
    curr_pos = [axis_data[axis]["angle"] for axis in AXES]
    if None in curr_pos:
        print(f"Cannot save current position as it is not known completely: {curr_pos}")
        time.sleep(1)
        return
    sequence.append(curr_pos)
    print(f"Saved position {len(sequence)}")


def mks_accel_to_rpm_accel(mks_accel_value: int, fix_bounds=False) -> float:
    """Convert the MKS board's acceleration value to physical acceleration
    that is consistent with the speed parameters (RPM/min) and suitable for calculations.
    """
    # So the acceleration value 'acc' determines the TIME before RPM is incremented/decremented by 1
    # Example: 50s/1000000 * (256-236) = 1ms  (where acc = 236)
    # td = (256-acc) * 0.00005s
    # a = dv / td = 1 / ((256-acc) * 0.00005s)  # 50us
    # 50us in minutes (has to match RPM):
    acc_tick = 0.00005/60.0
    above_upper_bound_accel = 1.0 / ((256.0 - 255.5) * acc_tick)
    if mks_accel_value < 1 or mks_accel_value > 255:  # instant-speed mode = above-bounds acceleration
        if fix_bounds:
            return above_upper_bound_accel
        else:
            raise ValueError(f"Acceleration value {mks_accel_value} is out of bounds (1-255).")
    return 1.0 / ((256.0 - mks_accel_value) * acc_tick)


def constrain_rpm_accel(rpm_accel_value: float) -> float:
    """Constrain the acceleration value to the valid range for the MKS board"""
    upper_bound = 1.0 / ((256.0 - 255.0) * 0.00005/60.0)
    lower_bound = 1.0 / ((256.0 - 1.0) * 0.00005/60.0)
    if rpm_accel_value > upper_bound:
        return upper_bound
    elif rpm_accel_value < lower_bound:
        return lower_bound
    return rpm_accel_value


def rpm_accel_to_mks_accel(rpm_accel_value, fix_bounds=False) -> int:
    """Convert a physical acceleration parameter (RPM/min) to the MKS board's acceleration parameter"""
    acc_tick = 0.00005/60.0
    # rpm_accel = 1.0 / ((256.0 - mks_accel_value) * acc_tick)
    # rpm_accel * ((256.0 - mks_accel_value) * acc_tick) = 1.0
    # rpm_accel * (256.0 * acc_tick - mks_accel_value * acc_tick) = 1.0
    # rpm_accel * 256.0 * acc_tick - rpm_accel * mks_accel_value * acc_tick = 1.0
    # rpm_accel * 256.0 * acc_tick - 1.0 = rpm_accel * mks_accel_value * acc_tick
    # (rpm_accel * 256.0 * acc_tick - 1.0) / (rpm_accel * acc_tick) = mks_accel_value
    # mks_accel_value = (rpm_accel * 256.0 * acc_tick - 1.0) / (rpm_accel * acc_tick)
    # mks_accel_value = (rpm_accel * 256.0 * acc_tick) / (rpm_accel * acc_tick) - 1.0 / (rpm_accel * acc_tick)
    # mks_accel_value = 256.0 - 1.0 / (rpm_accel * acc_tick)
    upper_bound = 1.0 / ((256.0 - 255.0) * acc_tick)
    lower_bound = 1.0 / ((256.0 - 1.0) * acc_tick)
    result_candidate = int(256.0 - 1.0 / (rpm_accel_value * acc_tick)) if rpm_accel_value > 0 else 0
    if result_candidate > 255:  #rpm_accel_value > upper_bound:
        if fix_bounds:
            return 0  # above-threshold acceleration = instant-speed mode
        else:
            raise ValueError(f"Acceleration value {rpm_accel_value} is above or too close to the upper bound of {upper_bound}.")
    elif result_candidate < 1:  # rpm_accel_value < lower_bound:
        if fix_bounds:
            return 1
        else:
            raise ValueError(f"Acceleration value {rpm_accel_value} is below or too close to the lower bound of {lower_bound}.")
    return int(256.0 - 1.0 / (rpm_accel_value * acc_tick))


def move_to_position(position: list[int]):
    curr_pos = [0] * arm.NUM_AXES
    if None in position:
        raise ValueError(f"Cannot move to {position}. It contains incomplete information.")
    for axis in AXES:
        (encoder_value,) = bus.ask(axis2canid(axis), "encoder")
        curr_pos[axis] = encoder_to_motor_angle(encoder_value)
    # if None in curr_pos:
    #     raise ValueError(f"Cannot plan movement as the current position is not known completely: {curr_pos}")

    # Plan movement
    # NOTE: distances are in Turns and time is in Minutes (because of RPM!)
    # Calculate time and travel distance numbers for all axes
    distances = [abs(p - c)/360 for c, p in zip(curr_pos, position)]  # TODO check if needed: absolute because speed is separate from direction
    print(f"{distances=}")
    # Convert MKS so-called acceleration value that is unit-compatible with RPM (the MKS speed parameter)
    max_accelerations = [mks_accel_to_rpm_accel(a) for a in arm.AXES_ACCEL_LIMIT]
    print(f"{max_accelerations=}")
    # a = v / t => t = v / a
    vmax_ramp_times_theoretical = [v / a for v, a in zip(arm.AXES_SPEED_LIMIT, max_accelerations)]  # may exceed distance
    print(f"{vmax_ramp_times_theoretical=}")
    # # d = a * t^2 / 2 => t = sqrt(2 * d / a)
    # ramp_times = [sqrt(2 * d / 2) for d, a in zip(distances, AXES_ACCEL_LIMIT)]
    vmax_ramp_distances_theoretical = [a * t**2 / 2 for t, a in zip(vmax_ramp_times_theoretical, max_accelerations)]
    print(f"{vmax_ramp_distances_theoretical=}")
    ramp_distances = [min(2*rd, d)/2 for rd, d in zip(vmax_ramp_distances_theoretical, distances)]
    print(f"{ramp_distances=}")
    vmax_distances = [d - (2*rd) for d, rd in zip(distances, ramp_distances)]  # note: vmax_dist of 0 = vmax not reached
    print(f"{vmax_distances=}")
    # d = a * t^2 / 2 => t = sqrt(2 * d / a)
    ramp_times = [sqrt(2 * rd / a) for rd, a in zip(ramp_distances, max_accelerations)]
    print(f"{ramp_times=}")
    # v = d / t => t = d / v
    vmax_times = [vd / v for vd, v in zip(vmax_distances, arm.AXES_SPEED_LIMIT)]
    print(f"{vmax_times=}")
    total_times = [2 * rt + vt for rt, vt in zip(ramp_times, vmax_times)]
    print(f"{total_times=}")

    # Determine the slowest axis. We will adapt all others to that one.
    slowest_axis_idx = max(enumerate(total_times), key=lambda i_x_tuple: i_x_tuple[1])[0]
    print(f"{slowest_axis_idx=}")

    # Now that we know the slowest axis, scale all accelerations and speeds so all accelerations, vmax travels, and
    # decelerations start/end at the same time (that is, the one of the slowest axis).
    slowest_ramp_time = ramp_times[slowest_axis_idx]
    print(f"{slowest_ramp_time=}")
    slowest_vmax_time = vmax_times[slowest_axis_idx]
    print(f"{slowest_vmax_time=}")

    # For all other axes, the ramping-up, vmax and ramping-down times have to match the one of the slowest axis.
    # Solve the system of equations for a and vmax (tt, tr, tvmax and d are known, the rest are unknown):
    # I   tt = tr + tvmax + tr
    # II  d = dr + dvmax + dr
    # III tr = sqrt(2 * dr / a)
    # IV  tvmax = dvmax / vmax
    # V   vmax = a * tr
    #
    # V in IV: tvmax = dvmax / (a * tr)
    # VI  dvmax = tvmax * (a * tr)
    #
    # VI in II: d = dr + tvmax * (a * tr) + dr = 2*dr + tvmax * a * tr
    # 2*dr = d - tvmax * a * tr
    # VII dr = (d - tvmax * a * tr) / 2
    #
    # VII in III: tr = sqrt(2 * (d - tvmax * a * tr) / 2 / a) = sqrt((d - tvmax * a * tr) / a)
    # tr^2 = (d - tvmax * a * tr) / a  | * a
    # a * tr^2 = d - tvmax * a * tr  | +tvmax * a * tr
    # d = a * tr^2 + tvmax * a * tr = a (tr^2 + tvmax * tr)
    # VIII a = d / (tr^2 + tvmax * tr) = d / (tr * (tr + tvmax))  <-- new acceleration
    #
    # VIII in V:
    # vmax = tr * d / (tr^2 + tvmax * tr) = d / (tr + tvmax)      <-- new vmax
    new_accelerations_to_check = [min(d / (slowest_ramp_time * (slowest_ramp_time + slowest_vmax_time)), amax) for d, amax in zip(distances, max_accelerations)]
    print(f"old accelerations: {max_accelerations}")
    print(f"{new_accelerations_to_check=}")

    complex_speed_adaption = False
    if complex_speed_adaption:
        # Check accelerations for validity and fix them to bounds if needed (they must be usable by the MKS board)
        constrained_accelerations = [constrain_rpm_accel(a) for a in new_accelerations_to_check]
        print(f"{constrained_accelerations=}")

        # Now the acceleration values are potentially so fast that we would arrive early. We release the limitation
        # to match ramp times and need to adjust the ramp time tr and speed vmax so that the total time is correct.
        # I   tt = tr + tvmax + tr
        # II  d = dr + dvmax + dr
        # III tr = sqrt(2 * dr / a)   and   dr = a * tr^2 / 2
        # IV  tvmax = dvmax / vmax    and  dvmax = tvmax * vmax
        # V   vmax = a * tr
        # III, IV in I: tt = 2 * sqrt(2 * dr / a) + dvmax / vmax
        # II:  dvmax = d - 2dr
        #
        # Geometric intuition  _________ (time/velocity chart where width is tt and area is d)
        #                     /         \  ramp-up/down triangles together form a rectangle, vmax is also a rectangle
        # d = tr * vmax/2 + tvmax * vmax + tr * vmax/2 = tr*vmax + tvmax*vmax
        # tr = vmax / a --> substitute tr in d equation:
        # d = vmax / a * vmax + tvmax*vmax = vmax^2/a + tvmax*vmax
        # tvmax = tt - 2*tr  # Derived from tt = 2*tr + tvmax
        # Substitute tr in tvmax equation:
        # tvmax = tt - 2*vmax / a  --> Substitute tvmax in d equation:
        # d = vmax^2/a + tvmax*vmax = vmax^2/a + (tt - 2*vmax / a) * vmax
        # Try to solve for vmax:
        # d = vmax^2/a + (tt - 2*vmax / a) * vmax
        # vmax^2/a + (tt - 2*vmax / a) * vmax - d = 0
        # 1/a * vmax^2 + tt * vmax - 2/a * vmax^2 - d = 0
        # -1/a * vmax^2 + tt * vmax - d = 0   | * -1 to make it cleaner
        #  1/a * vmax^2 - tt * vmax + d = 0
        # Solve the quadratic equation:
        #          + tt +/- sqrt(tt^2 - 4 * 1/a * d)
        #  vmax =  ---------------------------------
        #                       2 * 1/a
        #
        # = (tt +/- sqrt(tt^2 -4/a*d))*a / 2
        vmax_solution = []
        for tt, d, a, v_limit in zip(total_times, distances, constrained_accelerations, arm.AXES_SPEED_LIMIT):
            discriminant = tt**2 - 4/a*d
            if discriminant < 0:
                print(f"Could not find a solution for planning movement speed: {tt**2 - 4/a*d=} "
                                 f"(tt={tt}, d={d}, a={a}). Assuming max speed.")
                solution = v_limit
            else:
                solution_1 = (tt - sqrt(discriminant))*a / 2
                solution_2 = (tt + sqrt(discriminant))*a / 2
                if 0 < solution_1 <= v_limit:
                    solution = solution_1
                elif 0 < solution_2 <= v_limit:
                    solution = solution_2
                else:
                    # At this point, we accept non-perfect syncing of speeds if v_limit would be exceeded
                    solution = min(max(solution_1, solution_2, 1), v_limit)
                if solution <= 0:
                    raise ValueError(f"The solutions found for movement speed were negative.")
            vmax_solution.append(solution)
        new_speeds = vmax_solution
        print(f"old speeds: {arm.AXES_SPEED_LIMIT}")
        # intermediary_speeds = [min(a * slowest_ramp_time, vmax) for a, vmax in zip(intermediate_accelerations, AXES_SPEED_LIMIT)]
        # print(f"potential speeds: {intermediary_speeds}")
        print(f"{new_speeds=}")

        new_accels = [rpm_accel_to_mks_accel(accel) for accel in constrained_accelerations]
    else:
        new_accels = []
        new_speeds = []
        for a, vmax, d in zip(new_accelerations_to_check, arm.AXES_SPEED_LIMIT, distances):
            try:
                new_accels.append(rpm_accel_to_mks_accel(a, fix_bounds=False))
                new_speeds.append(min(a * slowest_ramp_time, vmax))
                print(f"calculation worked: new_accel={new_accels[-1]}, new_speed={new_speeds[-1]}")
            except ValueError:
                new_accels.append(0)  # instant-speed mode
                new_speeds.append(min(d / total_times[slowest_axis_idx], vmax))  # linear scaling
                print(f"fallback: new_accel={new_accels[-1]}, new_speed={new_speeds[-1]} (min({d}/{total_times[slowest_axis_idx]}, {vmax}))")

    print(f"{new_accels=}")
    print(f"{new_speeds=}")

    actually_moving = []
    for axis, angle in enumerate(position):
        speed = int(new_speeds[axis])
        accel = int(new_accels[axis])
        if speed > 0.05:
            bus.send(axis2canid(axis), "move_to", [speed, accel, motor_angle_to_encoder(angle)])
            actually_moving.append(axis)
    for axis in actually_moving:
        print(f"Waiting for axis {axis} to reach position")
        # 0 and 2 are both valid results -- 0 = fail means we're already at position, 2 = moved successfully
        resp = bus.wait_for(axis2canid(axis), "move_to", value_pattern=[[0], [2]], timeout=arm.AXES_MOVE_TIMEOUT[axis])
        print(f"Axis {axis} reached position: {resp}")


def play_pause():
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
    for i, position in enumerate(sequence):
        print(f"Moving to position {i + 1}")
        move_to_position(position)
        print(f"Position {i + 1} reached")
        time.sleep(0.5)
    time.sleep(2)
    # return to stopped status
    for axis in AXES:
        if state[axis] == PLAYBACK:
            state[axis] = STOPPED


def update_state_from_devices():
    if any([state[axis] in [INIT, HOMING, PLAYBACK] for axis in AXES]):
        return
    time.sleep(0.05)
    motor_angles = [0] * arm.NUM_AXES
    for axis in AXES:
        try:
            (encoder,) = bus.ask(axis2canid(axis), "encoder", timeout=0.1)
            angle = encoder_to_motor_angle(encoder)
            motor_angles[axis] = angle

            (motor_state,) = bus.ask(axis2canid(axis), "motor_status", timeout=0.1)
            # 0: error, 1: stopped, 2: accelerate, 3: decelerate, 4: full speed, 5: homing
            state[axis] = [ERROR, STOPPED, MOVING, STOPPING, MOVING, MOVING][motor_state]
            pretty_state = format(f"{state[axis]} ({motor_state})", "<12")
            axis_data[axis].update(state=pretty_state, timestamp=timestamp())

            (lock_state,) = bus.ask(axis2canid(axis), "shaft_lock_status", timeout=0.1)
            axis_data[axis].update(shaft_lock=lock_state, timestamp=timestamp())
        except TimeoutError as e:
            print(f"No answer from CAN ID {axis2canid(axis)}: {e}")

    joint_angles = joint_angles_abs(motor_angles)
    for axis in AXES:
        axis_data[axis].update(angle=joint_angles[axis], timestamp=timestamp())


def timestamp():
    return datetime.utcnow().strftime("%H:%M:%S.%f")[:-3]


def home(axis):
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
            if not active_low_endstop:
                bus.ask(can_id, "move_by", [60, 50, -3000], answer_pattern=[2])
            # Home to endstop
            bus.ask(can_id, "home", answer_pattern=[1])  # 1 = has started
            bus.wait_for(can_id, "home", timeout=arm.AXES_MOVE_TIMEOUT[axis], value_pattern=[2])
            bus.ask(can_id, "set_zero", answer_pattern=[True])  # this should not be necessary, but it is :(
        except TimeoutError as e:
            raise HomingError(f"Endstop homing of axis {axis} timed out. "
                              f"Consider increasing AXES_HOMING_TIMEOUT for this axis: {e}")
    else:
        # Sensorless homing
        sensorless_home(axis)

    # Move to neutral position
    (encoder_val,) = bus.ask(can_id, "encoder")
    print(f"Axis {axis} homed and zeroed. Current angle: {encoder_to_motor_angle(encoder_val):.1f}°")
    bus.ask(can_id, "move_to",
            [500, 50, motor_angle_to_encoder(arm.AXES_SAFE_HOMING_ANGLE[axis])],
            answer_pattern=[2], timeout=arm.AXES_MOVE_TIMEOUT[axis])
    if state[axis] == HOMING:
        state[axis] = STOPPED


def sensorless_home(axis):
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


def home_all():
    for axis in arm.HOMING_ORDER:
        home(axis)
    print(f"All axes homed")
    # for axis in AXES:
    #     bus.send(axis2canid(axis), "move_to", [500, 50, motor_angle_to_encoder(AXES_SAFE_HOMING_ANGLE[axis])])
    global state
    if any([state[axis] == HOMING for axis in AXES]):
        state = [STOPPED] * len(AXES)


def execute_transitions():
    print("ID Curr-state command  action  state-after-action")
    curr_cmd = next_command.copy()
    for axis in reversed(AXES):
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


def init(axis):
    # Initialize
    print(f"Initializing axis {axis}")
    can_id = axis2canid(axis)
    bus.send(can_id, "set_response", [True])
    bus.ask(can_id, "set_shaft_lock", [True], answer_pattern=[True])
    bus.ask(can_id, "release_shaft_lock", answer_pattern=[None])
    bus.ask(can_id, "set_work_current", [arm.AXES_CURRENT_LIMIT[axis]], answer_pattern=[True])
    global state
    state[axis] = [STOPPED]
    # # for_all(lambda i: bus.send(i, "set_key_lock", [False]))  # what does this do?

# Main
if __name__ == "__main__":
    with Bus() as bus:
        chain = KinematicChain.from_configuration(arm)
        chain.visualize_link_angles([0] * len(chain), interactive=True)
        tick = time.time()
        start_keyboard_listener()
        while not quit_app:
            try:
                if time.time() - tick > 0.2:
                    os.system('clear')
                    print_devices()
                    print("\n\n")
                    print(f"Press up/down arrow keys to choose axis, left/right arrow keys to move, esc to quit.")
                    print(f"'0' to set zero, 'l' to release lock, 'L' to release all locks, 'h' to home axis, 'H' to home all axes.")
                    print(f"'s' to save position in sequence, 'p' to play/pause sequence, 'c' to clear sequence.")
                    print_axes()
                    print("\n\n")
                    execute_transitions()
                    update_state_from_devices()
                    joint_angles = [axis_data[axis]["angle"] for axis in AXES]
                    if not None in joint_angles:
                        chain.update_visualization(joint_angles, use_degrees=True)
                    tick = time.time()
                chain.sleep(0.05)
            except Exception as e:
                stop_all()
                raise e
        chain.close_visualization()
