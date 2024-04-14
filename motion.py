import time
from contextvars import ContextVar
from functools import cache
from math import sqrt
from typing import Union, Optional

import arctos_arm as arm
from matrix import Matrix
from mks_bus import encoder_to_motor_angle, motor_angle_to_encoder, current_bus

AXES = list(range(arm.NUM_AXES))
MOTOR_TO_JOINT_TRANSFORM_MAT_REL = 1.0 / Matrix(arm.AXES_ANGLE_RATIO).T
MOTOR_TO_JOINT_TRANSFORM_MAT_ABS = MOTOR_TO_JOINT_TRANSFORM_MAT_REL.extend_bottom(Matrix([0, 0, 0, 0, 0, 0]).T)
MOTOR_TO_JOINT_TRANSFORM_MAT_ABS = MOTOR_TO_JOINT_TRANSFORM_MAT_ABS.extend_right(Matrix(arm.JOINT_ZERO_OFFSET + [1]))
JOINT_TO_MOTOR_TRANSFORM_MAT_REL = MOTOR_TO_JOINT_TRANSFORM_MAT_REL.inverse()
JOINT_TO_MOTOR_TRANSFORM_MAT_ABS = MOTOR_TO_JOINT_TRANSFORM_MAT_ABS.inverse()

def axis2canid(axis):
    """Convert axis index to can id. Axes are zero-based, while device CAN IDs start at 1."""
    return axis + 1


def canid2axis(can_id):
    """Convert CAN ID to axis index. Axes are zero-based, while device CAN IDs start at 1."""
    return can_id - 1


def joint_angles_rel(motor_angle: Union[float, list[float]], axis: Optional[int] = None):
    """Convert motor angle change in degrees to joint angle changes in degrees.
    Provide either a list of motor angles for all axes, or an axis index and the angle change for that axis.
    :param motor_angle: angle change in degrees or a list of angle changes in degrees for all motor axes
    :param axis: index of the axis (if one single motor angle is specified)
    """
    if not isinstance(motor_angle, list):
        motor_angle = [0 if a != axis else motor_angle for a in AXES]
    mot_vec = Matrix(motor_angle)
    joint_vec = MOTOR_TO_JOINT_TRANSFORM_MAT_REL * mot_vec
    return list(joint_vec)


def joint_angles_abs(motor_angle: Union[float, list[float]], axis: Optional[int] = None):
    """Convert absolute motor angle in degrees to absolute joint angle in degrees.
    Provide either a list of motor angles for all axes, or an axis index and the angle for that axis.
    :param motor_angle: angle in degrees or a list of angles in degrees for all motor axes
    :param axis: index of the axis (if one single motor angle is specified)
    """
    if not isinstance(motor_angle, list):
        motor_angle = [0 if a != axis else motor_angle for a in AXES]
    mot_vec = Matrix(motor_angle + [1])  # fill with 1 to make it a 1x7 vector
    joint_vec = MOTOR_TO_JOINT_TRANSFORM_MAT_ABS * mot_vec
    return list(joint_vec)[:6]


def motor_angles_rel(joint_angle: Union[float, list[float]], axis: Optional[int] = None):
    """Convert joint angle change in degrees to motor angle changes in degrees.
    Provide either a list of joint angles for all axes, or an axis index and the angle change for that axis.
    :param joint_angle: angle change in degrees or a list of angle changes in degrees for all joint axes
    :param axis: index of the axis (if one single joint angle is specified)
    """
    if not isinstance(joint_angle, list):
        joint_angle = [0 if a != axis else joint_angle for a in AXES]
    joint_vec = Matrix(joint_angle)
    mot_vec = JOINT_TO_MOTOR_TRANSFORM_MAT_REL * joint_vec
    return list(mot_vec)


def motor_angles_abs(joint_angle: Union[float, list[float]], axis: Optional[int] = None):
    """Convert absolute joint angle in degrees to absolute motor angle in degrees.
    Provide either a list of joint angles for all axes, or an axis index and the angle for that axis.
    :param joint_angle: angle in degrees or a list of angles in degrees for all joint axes
    :param axis: index of the axis (if one single joint angle is specified)
    """
    if not isinstance(joint_angle, list):
        joint_angle = [0 if a != axis else joint_angle for a in AXES]
    joint_vec = Matrix(joint_angle + [1])  # fill with 1 to make it a 1x7 vector
    mot_vec = JOINT_TO_MOTOR_TRANSFORM_MAT_ABS * joint_vec
    return list(mot_vec)[:6]


def stop_all(bus=None):
    """Stop all axes of the robot arm.
    :param bus: the bus object to use for communication (default: current_bus)
    """
    if bus is None:
        bus = current_bus.get()
    for axis in AXES:
        bus.send(axis2canid(axis), "move", [0, 0, 200])
    for axis in AXES:
        bus.wait_for(axis2canid(axis), "move", value_pattern=[[0], [2]], timeout=max(arm.AXES_MOVE_TIMEOUT))


def constrain(value, min_val, max_val):
    """Constrain a value to a given range.
    :param value: the value to constrain
    :param min_val: the minimum value
    :param max_val: the maximum value
    :return: the constrained value
    """
    return max(min_val, min(max_val, value))


class PIDController:
    """A simple PID controller for control of a single value (e.g. position of an axis.
    For those unfamiliar with PID controllers, the controller calculates a control signal based on the error between
    the target value (setpoint) and the current value. The control signal is a weighted sum of the proportional error,
    the integral of the error, and the differential of the error. The proportional gain (kP) determines the influence
    of the current error, the integral gain (kI) determines the influence of past errors, and the differential gain (kD)
    determines the influence of the rate of change of the error. The integral error is the sum of all past errors, and
    the differential error is the difference between the current error and the previous error.

    Values > 0 mean that the value needs to be increased, values < 0 mean that the value needs to be decreased.
    :param kP: proportional gain
    :param kI: integral gain
    :param kD: differential gain
    :param max_I: maximum integral error value
    :param max_error: maximum error value
    """
    def __init__(self, kP=0.7, kI=0.35, kD=0.16, max_I=300.0, max_error=300.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        if max_I < 0 or max_error < 0:
            raise ValueError("max_I and max_error must be positive.")
        self.max_I = max_I
        self.max_error = max_error
        self.integral_error = 0.0
        self.prev_error = None

    def reset(self):
        """Reset the integral and previous error values."""
        self.integral_error = 0.0
        self.prev_error = None

    def __call__(self, setpoint, curr, dt):
        """Calculate the PID control signal for the given setpoint and current value.
        Results > 0 suggests increasing the value proportionally, < 0 suggests decreasing the value proportionally.
        :param setpoint: the target value
        :param curr: the current value
        :param dt: the time difference since the last call
        :return: the PID control signal
        """
        if dt == 0:
            return 0.0
        # P: Calculate the proportional error between the target and current position
        error = setpoint - curr
        error = constrain(error, -self.max_error, self.max_error)  # Saturation to prevent extremes

        # I: Calculate the integral of the position error
        self.integral_error += error * dt
        self.integral_error = constrain(self.integral_error, -self.max_I, self.max_I)  # Saturation to prevent windup

        # D: Calculate the differential of the position error
        if self.prev_error is None:
            self.prev_error = error  # Initializing with 0 would cause a derivative spike in the first call
        diff_error = (error - self.prev_error) / dt
        self.prev_error = error

        # Calculate the PID control signal
        pid_error = self.kP * error + self.kI * self.integral_error + self.kD * diff_error
        # print(f"{self.kP * error=} + {self.kI * self.integral_error=} + {self.kD=} * {diff_error=} = {pid_error=}")
        return pid_error


def move_to_joint_position(joint_angles: Union[list[float], float], axis: Optional[int] = None, force_end_pos=True, bus=None):
    if not isinstance(joint_angles, list):
        if axis is None:
            raise ValueError("Either provide a list of joint_angles for all axes or specify an single joint angle and an axis index.")
        angle = joint_angles
        end_pos: list = [None] * arm.NUM_AXES
        end_pos[axis] = angle
    else:
        end_pos: list = joint_angles
    motor_angles = joint_angles_abs(end_pos)
    move_to_motor_position(motor_angles, force_end_pos=force_end_pos, bus=bus)

def move_to_motor_position(motor_angles: Union[list[float], float], axis: Optional[int] = None, force_end_pos=True, bus=None):
    """Move the robot arm to a specific position given in motor angles in degrees.
    The function will calculate the necessary acceleration and speed values to reach the target position in a
    coordinated manner and then send the movement commands to the individual axes.
    :param motor_angles: list of target motor angles in degrees for all axes
    :param axis: index of the axis to move (if only one motor angle is specified)
    :param bus: the bus object to use for communication (default: current_bus)
    """
    if bus is None:
        bus = current_bus.get()
        if bus is None:
            print("*** move_to_motor_position: No bus object available: running in TEST mode ***")
    if not isinstance(motor_angles, list):
        if axis is None:
            raise ValueError("Either provide a list of motor angles for all axes or specify an single motor_angle and an axis index.")
        angle = motor_angles
        end_pos: list = [None] * arm.NUM_AXES
        end_pos[axis] = angle
    else:
        end_pos: list = motor_angles

    start_pos = [0] * arm.NUM_AXES if bus is None else get_curr_motor_positions(bus)

    # Fill in missing ending positions with the current positions (no movement)
    for i, angle in enumerate(end_pos):
        if angle is None:
            end_pos[i] = start_pos[i]
    print(f"{start_pos=}, {end_pos=}")

    # NOTE: distances are in Turns and time is in Minutes (because of RPM!)

    # Calculate time and travel distance numbers for all axes (in turns because of RPM)
    rel_distances = [(e - s) / 360 for s, e in zip(start_pos, end_pos)]
    print(f"{rel_distances=}")
    abs_distances = [abs(d) for d in rel_distances]
    print(f"{abs_distances=}")

    total_times, ramp_times, vmax_times = calc_joint_movement_times(abs_distances)

    # Determine the slowest axis. We will adapt all others to that one.
    slowest_axis_idx = max(enumerate(total_times), key=lambda i_x_tuple: i_x_tuple[1])[0]
    print(f"{slowest_axis_idx=}")
    slowest_ramp_time = ramp_times[slowest_axis_idx]
    print(f"{slowest_ramp_time=}")
    slowest_vmax_time = vmax_times[slowest_axis_idx]
    print(f"{slowest_vmax_time=}")
    slowest_total_time = total_times[slowest_axis_idx]

    # Now that we know the slowest axis, scale all accelerations and speeds so all accelerations, vmax travels, and
    # decelerations start/end at the same time (that is, the one of the slowest axis).

    (new_accelerations,
     new_speeds,
     speed_correction) = calc_adapted_accelerations_and_vmax(slowest_ramp_time, slowest_vmax_time, abs_distances)

    if speed_correction != 1.0:
        raise NotImplementedError("Speed correction is not yet implemented.")

    if bus is None:
        ...
        # Test mode: print the calculated values
        test_times_equal(abs_distances, new_accelerations, new_speeds, slowest_axis_idx, slowest_ramp_time,
                         slowest_vmax_time)
        plot_move_speeds_and_positions(abs_distances, new_accelerations, new_speeds, ramp_times, vmax_times,
                                       slowest_ramp_time, slowest_vmax_time)

    # Run loop to move all axes to the target position
    tick_len = 0.1 / 60.0  # in minutes (due to RPM units)
    last_tick = time.time()
    dt = tick_len  # in minutes (due to RPM units)
    elapsed_time = 0  # in minutes (due to RPM units)
    # pid = [PIDController(kP=1.0, kI=0.0, kD=0.01, max_I=arm.AXES_SPEED_LIMIT[axis]/2, max_error=arm.AXES_SPEED_LIMIT[axis]/2) for axis in AXES]
    # pid = [PIDController(kP=0.8, kI=0.01, kD=0.1, max_I=arm.AXES_SPEED_LIMIT[axis]/2, max_error=arm.AXES_SPEED_LIMIT[axis]/2) for axis in AXES]
    # pid = [PIDController(kP=0.25, kI=0.05, kD=0.025, max_I=arm.AXES_SPEED_LIMIT[axis], max_error=arm.AXES_SPEED_LIMIT[axis]) for axis in AXES]
    pid = [PIDController(kP=0.6, kI=0.005, kD=0.075, max_I=arm.AXES_SPEED_LIMIT[axis]/2, max_error=arm.AXES_SPEED_LIMIT[axis]/2) for axis in AXES]
    planned_speed_mixin = 0.5  # default: average = 0.5 as in https://source-robotics.github.io/PAROL-docs/page4/#list-of-active-commands
    speed_threshold = 1.5  # RPM
    speed_low_pass_factor = 0.5
    commanded_speeds = [2.0 * speed_threshold] * arm.NUM_AXES  # give some extra settling time
    while (elapsed_time <= slowest_total_time or any(abs(s) > speed_threshold for s in commanded_speeds)) and elapsed_time <= 2* slowest_total_time:
        for axis in AXES:
            # Calculate the planned position (in motor degrees. Note the slight lookahead, calculations were in turns)
            planned_position = 360 * position_at_time(elapsed_time, new_accelerations[axis], new_speeds[axis],
                                                     rel_distances[axis], slowest_ramp_time, slowest_vmax_time)
            planned_position += start_pos[axis]
            # print(f"Axis {axis} calc'd end pos: {360*rel_distances[axis]=} + {start_pos[axis]=} = {360*rel_distances[axis] + start_pos[axis]}")
            # print(f"Axis {axis} {start_pos[axis]=}, {end_pos[axis]=}, {planned_position=}")

            # Get the current position
            if bus is None:
                # Test mode: set current position to target position
                curr_pos = planned_position
            else:
                (encoder_value,) = bus.ask(axis2canid(axis), "encoder")
                curr_pos = encoder_to_motor_angle(encoder_value)

            # Calculate the PID error
            pid_error = pid[axis](planned_position, curr_pos, dt * 60)
            # print(f"{elapsed_time=}, {axis=}: {curr_pos=}, {planned_position=}, {dt=}, {planned_position-curr_pos=}, {pid_error=}")

            target_velocity = speed_at_time(elapsed_time, new_accelerations[axis], new_speeds[axis],
                                            rel_distances[axis], slowest_ramp_time, slowest_vmax_time)
            adjusted_velocity = planned_speed_mixin * target_velocity + (1 - planned_speed_mixin) * pid_error
            adjusted_velocity = constrain(adjusted_velocity, -arm.AXES_SPEED_LIMIT[axis], arm.AXES_SPEED_LIMIT[axis])
            if abs(adjusted_velocity) < speed_threshold:
                adjusted_velocity = 0
            direction = 1 if adjusted_velocity > 0 else 0
            direction = 1 - direction if arm.AXES_RAW_DIRECTION[axis] else direction
            accel = arm.AXES_ACCEL_LIMIT[axis]
            # print(f"Axis {axis}: {direction=}, {adjusted_velocity=}, {accel=}")
            if bus is not None:
                bus.ask(axis2canid(axis), "move", [direction, abs(adjusted_velocity), accel], answer_pattern=[None])

            commanded_speeds[axis] += speed_low_pass_factor * (adjusted_velocity - commanded_speeds[axis])
            if abs(commanded_speeds[axis]) < speed_threshold:
                commanded_speeds[axis] = 0

        # Wait for the next tick
        dt = 0
        while dt < tick_len:
            dt = (time.time() - last_tick) / 60.0
            time.sleep(0.01)
        elapsed_time += dt
        last_tick = time.time()

    print(f"{commanded_speeds=}")

    if bus is not None:
        stop_all(bus=bus)

        if force_end_pos:
            # time.sleep(2 * tick_len)  # wait for vibrations to settle
            # Move to target position just to be safe
            for axis in AXES:
                bus.send(axis2canid(axis), "move_to",
                         [arm.AXES_SPEED_LIMIT[axis], arm.AXES_ACCEL_LIMIT[axis], motor_angle_to_encoder(end_pos[axis])])
            for axis in AXES:
                bus.wait_for(axis2canid(axis), "move_to", value_pattern=[[0], [2], [3]], timeout=max(arm.AXES_MOVE_TIMEOUT))


def mks_accel_to_rpm_accel(mks_accel_value: int, fix_bounds=False) -> float:
    """Convert the MKS board's acceleration value to physical acceleration
    that is consistent with the speed parameters (RPM/min) and suitable for calculations.
    :param mks_accel_value: the acceleration value from the MKS board (1-255)
    :param fix_bounds: whether to fix the value if it is out of bounds (default: False)
    :return: the acceleration value in RPM/min/min
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
    """Constrain the acceleration value to the valid range for the MKS board
    :param rpm_accel_value: the acceleration value in RPM/min/min
    :return: the constrained acceleration value in RPM/min/min
    """
    upper_bound = 1.0 / ((256.0 - 255.0) * 0.00005/60.0)
    lower_bound = 1.0 / ((256.0 - 1.0) * 0.00005/60.0)
    if rpm_accel_value > upper_bound:
        return upper_bound
    elif rpm_accel_value < lower_bound:
        return lower_bound
    return rpm_accel_value


def rpm_accel_to_mks_accel(rpm_accel_value, fix_bounds=False) -> int:
    """Convert a physical acceleration parameter (RPM/min) to the MKS board's acceleration parameter.
    :param rpm_accel_value: the acceleration value in RPM/min
    :param fix_bounds: whether to fix the value if it is out of bounds (default: False)
    :return: the MKS board's acceleration value
    """
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


def get_curr_motor_positions(bus=None):
    """Get the current motor positions of all axes in degrees
    :param bus: the bus object to use for communication (default: current_bus)
    :return: list of motor positions in degrees for all axes
    """
    if bus is None:
        bus = current_bus.get()
    curr_pos = [0] * arm.NUM_AXES
    for axis in AXES:
        (encoder_value,) = bus.ask(axis2canid(axis), "encoder")
        curr_pos[axis] = None if encoder_value is None else encoder_to_motor_angle(encoder_value)
    return curr_pos


@cache
def max_accels_in_rpmpm():
    """Return the maximum acceleration values in degrees/min/min for all axes"""
    return [mks_accel_to_rpm_accel(arm.AXES_ACCEL_LIMIT[axis]) for axis in AXES]


def speed_at_time(t, accel, vmax, distance, ramp_time, vmax_time):
    """Calculate the speed and position of an axis at a given time.
    All units are in RPM, minutes, and turns.
    :param t: time in minutes
    :param accel: acceleration in RPM/min/min
    :param vmax: maximum speed in RPM
    :param distance: distance to travel in turns
    :param ramp_time: time to reach maximum speed in minutes
    :param vmax_time: time to travel at maximum speed in minutes
    :return: speed in RPM
    """
    if distance < 0:
        accel = -accel
        vmax = -vmax
    if t < ramp_time:
        return accel * t
    elif t < ramp_time + vmax_time:
        return vmax
    elif t < 2 * ramp_time + vmax_time:
        t_ramp_down = t - ramp_time - vmax_time
        return accel * (ramp_time - t_ramp_down)
    else:
        return 0


def position_at_time(t, accel, vmax, distance, ramp_time, vmax_time):
    """Calculate the speed and position of an axis at a given time.
    All units are in RPM, minutes, and turns.
    :param t: time in minutes
    :param accel: acceleration in RPM/min/min
    :param vmax: maximum speed in RPM
    :param distance: distance to travel in turns
    :param ramp_time: time to reach maximum speed in minutes
    :param vmax_time: time to travel at maximum speed in minutes
    :return: position in turns
    """
    if distance < 0:
        accel = -accel
        vmax = -vmax
    ramp_up_position = 0.5 * accel * min(t, ramp_time) ** 2
    if t < ramp_time:
        return ramp_up_position
    elif t < ramp_time + vmax_time:
        return ramp_up_position + vmax * (t - ramp_time)
    elif t < 2 * ramp_time + vmax_time:
        t_ramp_down = t - ramp_time - vmax_time
        return distance - 0.5 * accel * (ramp_time - t_ramp_down) ** 2
    else:
        return distance


def plot_move_speeds_and_positions(abs_distances, new_accelerations, new_speeds, ramp_times, vmax_times,
                                   target_ramp_time, target_vmax_time):
    """Create line plots of the speeds and positions of all axes over time. The original speeds and positions at the
    # left side, and the adapted speeds and positions (using speed_and_positions_at_time) at the right side.
    """
    import matplotlib.pyplot as plt
    import numpy as np
    fig, axs = plt.subplots(2, 2)
    for axis in AXES:
        t = np.linspace(0, 2 * target_ramp_time + target_vmax_time, 100)
        speeds = []
        positions = []
        for time_point in t:
            speed = speed_at_time(time_point, max_accels_in_rpmpm()[axis], arm.AXES_SPEED_LIMIT[axis],
                                  abs_distances[axis], ramp_times[axis], vmax_times[axis])
            position = position_at_time(time_point, max_accels_in_rpmpm()[axis], arm.AXES_SPEED_LIMIT[axis],
                                        abs_distances[axis], ramp_times[axis], vmax_times[axis])
            speeds.append(speed)
            positions.append(position)
        axs[0, 0].plot(t, speeds, label=f"Axis {axis}")
        axs[1, 0].plot(t, positions, label=f"Axis {axis}")
        speeds = []
        positions = []
        for time_point in t:
            speed = speed_at_time(time_point, new_accelerations[axis], new_speeds[axis], abs_distances[axis],
                                  target_ramp_time, target_vmax_time)
            position = position_at_time(time_point, new_accelerations[axis], new_speeds[axis], abs_distances[axis],
                                        target_ramp_time, target_vmax_time)
            speeds.append(speed)
            positions.append(position)
        axs[0, 1].plot(t, speeds, label=f"Axis {axis}")
        axs[1, 1].plot(t, positions, label=f"Axis {axis}")
    axs[0, 0].set_title("Original")
    axs[0, 0].set_ylabel("Speed (RPM)")
    axs[0, 0].legend()
    axs[1, 0].set_xlabel("Time (min)")
    axs[1, 0].set_ylabel("Position (Turns)")
    axs[1, 0].legend()
    axs[0, 1].set_title("Adapted")
    axs[0, 1].set_ylabel("Speed (RPM)")
    axs[0, 1].legend()
    axs[1, 1].set_xlabel("Time (min)")
    axs[1, 1].legend()
    plt.show()
    plt.pause(0.1)
    input("Press Enter to continue...")


def test_times_equal(abs_distances, new_accelerations, new_speeds, slowest_axis_idx, slowest_ramp_time,
                     slowest_vmax_time):
    # Check new total times with changed accel and vmax, should equal slowest axis' times:
    new_total_times, new_ramp_times, new_vmax_times = calc_joint_movement_times(abs_distances, new_accelerations,
                                                                                new_speeds)
    print(f"{new_ramp_times=}")
    print(f"{new_vmax_times=}")
    print(f"{new_total_times=}")
    # Fix zeros (for zero distances we got zero as time to avoid division by zero). They should all be the same, so just use any non-zero value of the list.
    new_total_times = [new_total_times[slowest_axis_idx] if t == 0 else t for t in new_total_times]
    new_ramp_times = [new_ramp_times[slowest_axis_idx] if t == 0 else t for t in new_ramp_times]
    new_vmax_times = [new_vmax_times[slowest_axis_idx] if t == 0 else t for t in new_vmax_times]
    assert all(abs(rt - slowest_ramp_time) < 0.0001 for rt in new_ramp_times), "Ramp times do not match."
    assert all(abs(vt - slowest_vmax_time) < 0.0001 for vt in new_vmax_times), "Vmax times do not match."
    assert all(abs(tt - (2 * slowest_ramp_time + slowest_vmax_time)) < 0.0001 for tt in
               new_total_times), "Total times do not match."
    print("Adapted times match the slowest axis.")


def safer_div(a, b, allow_div0=False, default=0):
    if allow_div0:
        return a / b if b != 0 else default
    else:
        return a / b if a != 0 else 0


def calc_joint_movement_times(abs_distances_in_rounds, accels=None, speeds=None):
    """Calculate the time it takes to move each axis to a new position according to the arm speed and acceleration
    values (defaults to defined max values). Returns the ramp-up, total, and vmax times for each axis.
    :param abs_distances_in_rounds: list of distances in rounds for each axis
    :param speeds: list of speeds in RPM/min for each axis (default: AXES_SPEED_LIMIT)
    :param accels: list of accelerations in RPM/min^2 for each axis (default: max_accels_in_rpmpm())
    :return: lists of ramp-up, total, and vmax times for each axis
    """
    if accels is None:
        # Convert MKS so-called acceleration value into something that is unit-compatible with RPM (the MKS speed parameter)
        accels = max_accels_in_rpmpm()
        print(f"{accels=}")
    if speeds is None:
        speeds = arm.AXES_SPEED_LIMIT

    # a = v / t => t = v / a
    vmax_ramp_times_theoretical = [safer_div(v,  a) for v, a in
                                   zip(speeds, accels)]  # may exceed distance
    print(f"{vmax_ramp_times_theoretical=}")
    vmax_ramp_distances_theoretical = [a * t ** 2 / 2 for t, a in zip(vmax_ramp_times_theoretical, accels)]
    print(f"{vmax_ramp_distances_theoretical=}")
    ramp_distances = [min(2 * rd, d) / 2 for rd, d in zip(vmax_ramp_distances_theoretical, abs_distances_in_rounds)]
    print(f"{ramp_distances=}")
    vmax_distances = [d - (2 * rd) for d, rd in
                      zip(abs_distances_in_rounds, ramp_distances)]  # note: vmax_dist of 0 = vmax not reached
    print(f"{vmax_distances=}")
    # d = a * t^2 / 2 => t = sqrt(2 * d / a)
    ramp_times = [sqrt(safer_div(2 * rd, a)) for rd, a in zip(ramp_distances, accels)]
    print(f"{ramp_times=}")
    # v = d / t => t = d / v
    vmax_times = [safer_div(vd, v) for vd, v in zip(vmax_distances, speeds)]
    print(f"{vmax_times=}")
    total_times = [2 * rt + vt for rt, vt in zip(ramp_times, vmax_times)]
    print(f"{total_times=}")
    return total_times, ramp_times, vmax_times


def calc_adapted_accelerations_and_vmax(slowest_ramp_time, slowest_vmax_time, abs_distances):
    """Calculate the new acceleration values for all axes so that they all reach their target position at the same time.
    Returns the new acceleration and speed values for each axis.
    If the new acceleration or speed values exceed the maximum values, they will be scaled down linearly so that the
    maximum values are not exceeded. In this case, the speed/acceleration scaling factor is not equal to 1.0.
    :param slowest_ramp_time: the ramp-up time of the slowest axis
    :param slowest_vmax_time: the time the slowest axis stays at vmax speed
    :param abs_distances: list of distances in motor rounds for each axis
    :return: lists of new acceleration and speed values for each axis, and the speed/acceleration scaling factor
    """
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

    max_accelerations = max_accels_in_rpmpm()
    new_accelerations = [safer_div(d, (slowest_ramp_time * (slowest_ramp_time + slowest_vmax_time))) for d in abs_distances]
    print(f"old accelerations: {max_accelerations}")
    print(f"{new_accelerations=}")
    new_speeds = [safer_div(d, (slowest_ramp_time + slowest_vmax_time)) for d in abs_distances]
    print(f"{new_speeds=}")

    # TODO: Adapted accelerations and speeds may be higher than max values for some axes. We need to check,
    #  and if necessary, scale *everything* down linearly so that their max values are not exceeded.
    #  This will make the slowest axis slower, but that is acceptable.
    speed_scale = 1.0
    # accels_over_max = [a > m for a, m in zip(new_accelerations, max_accelerations)]
    # if any(accels_over_max):
    #     speed_scale = min(m / a for m, a in zip(max_accelerations, new_accelerations) if a != 0)
    #     print(f"Warning: Some new accelerations exceed the maximum values. Scaling down by factor {speed_scale}.")
    #     new_accelerations = [a * speed_scale for a in new_accelerations]
    #     new_speeds = [s * speed_scale for s in new_speeds]
    # speeds_over_max = [s > arm.AXES_SPEED_LIMIT[i] for i, s in enumerate(new_speeds)]
    # if any(speeds_over_max):
    #     speed_scale = min(arm.AXES_SPEED_LIMIT[i] / s for i, s in enumerate(new_speeds) if s != 0)
    #     print(f"Warning: Some new speeds exceed the maximum values. Scaling down by factor {speed_scale}.")
    #     new_accelerations = [a * speed_scale for a in new_accelerations]
    #     new_speeds = [s * speed_scale for s in new_speeds]
    return new_accelerations, new_speeds, speed_scale


def send_axes(axes_or_messages=None, cmd=None, values=None, answer_pattern=None, bus=None):
    """Send a command to one or more axes.
    If an answer pattern is provided, waits for the answer and returns it. Use [None] to accept any answer.
    Otherwise, the command is sent without waiting for an answer.
    :param axes_or_messages: list of axis indices or messages to send (None for all axes)
    :param cmd: command to send
    :param values: values to send
    :param answer_pattern: pattern to match the answer against
    :param bus: the bus object to use for communication (default: current_bus)
    :return: the answer message if an answer pattern is provided
    """
    if axes_or_messages is None:
        axes_or_messages = AXES
    result = [None] * len(AXES)
    for axis_msg in axes_or_messages:
        if isinstance(axis_msg, int):
            axis = axis_msg
            if answer_pattern:
                result[axis] = bus.ask(axis2canid(axis), cmd, values, answer_pattern=answer_pattern)
            else:
                bus.send(axis2canid(axis), cmd, values)
        else:
            msg = axis_msg
            bus.send(msg)
    return result

if __name__ == "__main__":
    from mks_bus import Bus
    bus = Bus()
    print(get_curr_motor_positions(bus))
    send_axes(cmd="release_shaft_lock", bus=bus, answer_pattern=[None])
    send_axes(cmd="set_zero", bus=bus, answer_pattern=[None])
    # send_axes(cmd="zero_mode", values=[2, 1, 0, 0], bus=bus, answer_pattern=[1])
    print(get_curr_motor_positions(bus))
    axis = 3
    force_end_pos = True
    pos = [None, None, None, None, None, None]
    pos[axis] = 3000
    move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
    time.sleep(1)
    # print("correcting...")
    # move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
    time.sleep(2)
    pos = [None, None, None, None, None, None]
    pos[axis] = -2000
    move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
    time.sleep(1)
    # print("correcting...")
    # move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
    time.sleep(2)
    send_axes([axis], cmd="move_to", values=[500, 100, 0], bus=bus, answer_pattern=[[0], [2], [3]])
    time.sleep(2)
    send_axes([axis], cmd="move_to", values=[500, 100, 0], bus=bus, answer_pattern=[[0], [2], [3]])

    never = False
    if never:
        from mks_bus import Bus
        bus = Bus()
        print(get_curr_motor_positions(bus))
        send_axes(cmd="release_shaft_lock", bus=bus, answer_pattern=[None])
        send_axes(cmd="set_zero", bus=bus, answer_pattern=[None])
        # send_axes(cmd="zero_mode", values=[2, 1, 0, 0], bus=bus, answer_pattern=[1])
        print(get_curr_motor_positions(bus))
        axes = [1]
        send_axes(axes, cmd="set_direction", values=[1], bus=bus, answer_pattern=[1])
        print("before move_to", get_curr_motor_positions(bus))
        send_axes(axes, cmd="move_to", values=[500, 100, 0x4000], bus=bus, answer_pattern=[[0], [2], [3]])
        print("after move_to", get_curr_motor_positions(bus))
        send_axes(axes, cmd="move_to", values=[500, 100, 0], bus=bus, answer_pattern=[[0], [2], [3]])

        print("before move", get_curr_motor_positions(bus))
        send_axes(axes, cmd="move", values=[0, 100, 50], bus=bus)
        time.sleep(1.5)
        send_axes(axes, cmd="move", values=[0, 0, 50], bus=bus, answer_pattern=[[0], [2], [3]])
        print("after move", get_curr_motor_positions(bus))
        send_axes(axes, cmd="move_to", values=[500, 100, 0], bus=bus, answer_pattern=[[0], [2], [3]])

        force_end_pos=True
        pos = [0, -2000, 0, 0, 0, 0]
        move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
        move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
        pos = [None, 4000, None, None, None, None]
        move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
        move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
