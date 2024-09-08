import time
from contextvars import ContextVar
from functools import cache
from math import sqrt
import multiprocessing
from multiprocessing import Process
from multiprocessing.connection import Connection
from queue import SimpleQueue
from typing import Union, Optional, Any

import arctos_arm as arm
from matrix import Matrix
from mks_bus import Bus, WAIT_FOR_TIMEOUT, encoder_to_motor_angle, motor_angle_to_encoder, current_bus

current_bus_proxy = ContextVar('current_bus_proxy')


# Constants
ERROR = "error"
STOPPING = "stopping"
STOPPED = "stopped"
MOVING = "moving"

AXES = list(range(arm.NUM_AXES))
MOTOR_TO_JOINT_TRANSFORM_MAT_REL = 1.0 / Matrix(arm.AXES_ANGLE_RATIO).T
MOTOR_TO_JOINT_TRANSFORM_MAT_ABS = MOTOR_TO_JOINT_TRANSFORM_MAT_REL.extend_bottom(Matrix([0, 0, 0, 0, 0, 0]).T)
MOTOR_TO_JOINT_TRANSFORM_MAT_ABS = MOTOR_TO_JOINT_TRANSFORM_MAT_ABS.extend_right(Matrix(arm.JOINT_ZERO_OFFSET + [1]))
JOINT_TO_MOTOR_TRANSFORM_MAT_REL = MOTOR_TO_JOINT_TRANSFORM_MAT_REL.inverse()
JOINT_TO_MOTOR_TRANSFORM_MAT_ABS = MOTOR_TO_JOINT_TRANSFORM_MAT_ABS.inverse()

control_process: Optional[multiprocessing.Process] = None
motion_conn: Optional[Connection] = None

BUS_ARGS = {k:v for k, v in {
    "bustype_or_bus": getattr(arm, "CAN_BUS_TYPE", None),
    "device": getattr(arm, "CAN_DEVICE", None),
    "bitrate": getattr(arm, "CAN_BITRATE", None),
    "device_searchstr": getattr(arm, "CAN_AUTODETECT_STRING", None),
}.items() if v is not None}

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


class BusProxy:
    """A proxy class for the bus object to send messages to the controller subprocess which owns the CAN connection."""
    def __init__(self, bus_args=None) -> None:
        if hasattr(self, 'connection') and getattr(self, "connection") is not None:
            raise RuntimeError(f"There is already an active bus connection for this BusProxy.")
        self.connection: Connection = start_control(bus_args=bus_args)
        self._buffer = []
        self._id_seq = 0

    def _new_id(self):
        self._id_seq += 1
        return self._id_seq

    def _slurp_messages(self, timeout=0):
        if self.connection.poll(timeout):
            while self.connection.poll(0):
                self._buffer.append(self.connection.recv())
        # else:
        #     print(3)
        #     import traceback
        #     traceback.print_stack()
        #     print(4)
        #     raise TimeoutError(f"BusProxy: Timeout while waiting for CAN responses.")

    def _get_message(self, msg_id):
        for i, msg in enumerate(self._buffer):
            if msg.get("id") == msg_id:
                return self._buffer.pop(i)["result"]
        raise RuntimeError(f"BusProxy did not receive response with id {msg_id}. Messages: {self._buffer}")

    def _wait_for(self, msg_id, timeout):
        self._slurp_messages(timeout)
        return self._get_message(msg_id)

    # Low-level CAN commands
    def send(self, can_id: int, cmd: Optional[Union[str, int]] = None, *values: Union[list[Union[int, bool]], int, bool]):
        # print(f"BusProxy send: {can_id=}, {cmd=}, {values=}")
        my_id = self._new_id()
        message = {"id": my_id, "protocol": "can_bus", "data": ("send", (can_id, cmd, *values), {})}
        # print(f"BusProxy send:\n{message=}\n-->\n{None}")
        self.connection.send(message)

    def wait_for(self, can_id: int, cmd: Optional[Union[str, int]] = None, value_pattern: Optional[list] = None, timeout=WAIT_FOR_TIMEOUT):
        # print(f"BusProxy wait_for: {can_id=}, {cmd=}, {value_pattern=}")
        my_id = self._new_id()
        message = {"id": my_id, "protocol": "can_bus", "data": ("wait_for", (can_id, cmd, value_pattern), {"timeout": timeout})}
        # print(f"BusProxy wait_for:\n{message=}")
        self.connection.send(message)
        answer = self._wait_for(my_id, timeout)
        # print(f"BusProxy wait_for answer:\n{answer=}")
        return answer

    def ask(self, can_id: int, cmd: Optional[Union[str, int]] = None,
            *values: Union[list[Union[int, bool]], int, bool], answer_pattern: Optional[list] = None, timeout=WAIT_FOR_TIMEOUT):
        # print(f"BusProxy ask: {can_id=}, {cmd=}, {values=}, {answer_pattern=}")
        my_id = self._new_id()
        message = {"id": my_id, "protocol": "can_bus", "data": ("ask", (can_id, cmd, *values), {"answer_pattern": answer_pattern, "timeout": timeout})}
        # print(f"BusProxy ask: {message=}")
        # print(f"BusProxy {self._buffer=}")
        self.connection.send(message)
        answer = self._wait_for(my_id, timeout)
        # print(f"BusProxy ask answer:\n{answer=}")
        return answer

    def flush(self, ):
        """Flush the receive buffer, discarding all messages."""
        my_id = self._new_id()
        message = {"id": my_id, "protocol": "can_bus", "data": ("flush", (), {})}
        self.connection.send(message)

    # Higher-level commands
    def get_state(self):
        message = {"protocol": "system", "data": "state"}
        self.connection.send(message)
        return self.connection.recv()
        # if self.connection.poll(timeout):
        #     return self.connection.recv()
        # return None

    def submit_trajectory(self, trajectory):
        message = {"protocol": "trajectory", "data": trajectory}
        self.connection.send(message)

    def clear_trajectory_queue(self):
        message = {"protocol": "system", "data": "clear_trajectory_queue"}
        self.connection.send(message)

    def stop(self):
        message = {"protocol": "system", "data": "stop"}
        self.connection.send(message)

    def close(self):
        if self.connection is not None:
            stop_control()  # todo pull function in here?
            # message = {"protocol": "can_bus", "data": ("close", (), {})}
            # self.connection.send(message)
            # time.sleep(0.2)  # wait for the message to be sent

    # Context manager housekeeping
    def __enter__(self):
        self._curr_bus_token = current_bus_proxy.set(self)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        current_bus_proxy.reset(self._curr_bus_token)
        self.close()
        return False


class MockBus:
    """A drop-in mock replacementfor the bus object to send messages to the controller subprocess which owns the CAN connection."""
    def __init__(self, *args, **kwargs) -> None:
        print(f"MockBus with args={args}, kwargs={kwargs}")

    def send(self, can_id: int, cmd: Optional[Union[str, int]] = None, *values: Union[list[Union[int, bool]], int, bool]):
        print(f"MockBus send: {can_id=}, {cmd=}, {values=}")

    def wait_for(self, can_id: int, cmd: Optional[Union[str, int]] = None, value_pattern: Optional[list] = None, timeout=WAIT_FOR_TIMEOUT):
        print(f"MockBus wait_for: {can_id=}, {cmd=}, {value_pattern=}")
        return (42,)

    def ask(self, can_id: int, cmd: Optional[Union[str, int]] = None,
            *values: Union[list[Union[int, bool]], int, bool], answer_pattern: Optional[list] = None, timeout=WAIT_FOR_TIMEOUT):
        print(f"MockBus ask: {can_id=}, {cmd=}, {values=}, {answer_pattern=}")
        return (42,)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return False


def stop_all(bus=None):
    """Stop all axes of the robot arm.
    :param bus: the bus object to use for communication (default: current_bus)
    """
    bus = bus or current_bus_proxy.get()
    for axis in AXES:
        bus.send(axis2canid(axis), "move", [0, 0, 200])
    # for axis in AXES:
    #     bus.wait_for(axis2canid(axis), "move", value_pattern=[[0], [2]], timeout=max(arm.AXES_MOVE_TIMEOUT))


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


def move_to_joint_position(joint_angles: Union[list[float], float], axis: Optional[int] = None, bus=None):
    if not isinstance(joint_angles, list):
        if axis is None:
            raise ValueError("Either provide a list of joint_angles for all axes or specify an single joint angle and an axis index.")
        angle = joint_angles
        end_pos: list = [None] * arm.NUM_AXES
        end_pos[axis] = angle
    else:
        end_pos: list = joint_angles
    motor_angles = joint_angles_abs(end_pos)
    move_to_motor_position(motor_angles, bus=bus)

def move_to_motor_position(motor_angles: Union[list[float], float], axis: Optional[int] = None, start_pos=None, override=False, bus=None) -> list[float]:
    """Move the robot arm to a specific position given in motor angles in degrees.
    The function will calculate the necessary acceleration and speed values to reach the target position in a
    coordinated manner and then send the movement commands to the individual axes.
    :param motor_angles: list of target motor angles in degrees for all axes
    :param axis: index of the axis to move (if only one motor angle is specified)
    :param start_pos: list of current motor angles in degrees for all axes (default: None = get current positions)
    :param override: whether to override the current trajectory conn (default: False)
    :param bus: the bus object to use for communication (default: current_bus)
    :return: the target motor angles in degrees
    """
    bus = bus or current_bus.get()
    # print(f"move_to_motor_position got bus: {bus}")
    if isinstance(motor_angles, list):
        end_pos: list = motor_angles
    else:
        if axis is None:
            raise ValueError("Either provide a list of motor angles for all axes or specify an single motor_angle and an axis index.")
        angle = motor_angles
        end_pos: list = [None] * arm.NUM_AXES
        end_pos[axis] = angle

    if start_pos is None:
        start_pos = [0] * arm.NUM_AXES if bus is None else get_curr_motor_positions(bus)  # TODO: reuse busproxy abstraction

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

    # Provide the trajectory planning for each axis
    # Format:
    # planned_trajectory = [
    #    (t1, [p1_ax0, p1_ax1, ...], [s1_ax0, s1_ax1, ...]),  # waypoint 1 at time t1: positions and speeds for all 6 axes
    #    (t2, [p1_ax0, p1_ax1, ...], [s1_ax0, s1_ax1, ...]),  # waypoint 2 at time t2
    #    ...
    # ]
    tick_len: float = 0.1 / 60.0  # in minutes (due to RPM units)
    planned_trajectory: list[tuple[float, list[float], list[float]]] = []  # tuples of times, positions and speeds for each axis
    t: float = 0.0
    while t <= slowest_total_time + tick_len:  # + tick_len to be sure to include the last point
        waypoint: tuple[float, list[float], list[float]] = (t, [], [])
        for axis in AXES:
            # Position in motor degrees (calculations were in turns)
            position: float = start_pos[axis] + 360 * position_at_time(
                t, new_accelerations[axis], new_speeds[axis], rel_distances[axis], slowest_ramp_time, slowest_vmax_time)
            # Speed in RPM
            speed: float = speed_at_time(t, new_accelerations[axis], new_speeds[axis],
                                  rel_distances[axis], slowest_ramp_time, slowest_vmax_time)
            waypoint[1].append(position)
            waypoint[2].append(speed)
        planned_trajectory.append(waypoint)
        t += tick_len
    print(f"{slowest_total_time=}")
    print(f"{planned_trajectory=}")

    # Add the trajectory to the control conn
    if override:
        # bus.clear_trajectory_queue()  # TODO: reuse busproxy abstraction
        motion_conn.send({"protocol": "system", "data": "clear_trajectory_queue"})
    # bus.submit_trajectory(planned_trajectory)  # TODO: reuse abstraction
    motion_conn.send({"protocol": "trajectory", "data": planned_trajectory})

    return end_pos

    # control_trajectory(bus)

    # stop_all(bus=bus)
    #
    # if force_end_pos:
    #     # time.sleep(2 * tick_len)  # wait for vibrations to settle
    #     # Move to target position just to be safe
    #     for axis in AXES:
    #         bus.send(axis2canid(axis), "move_to",
    #                  [arm.AXES_SPEED_LIMIT[axis], arm.AXES_ACCEL_LIMIT[axis], motor_angle_to_encoder(end_pos[axis])])
    #     for axis in AXES:
    #         bus.wait_for(axis2canid(axis), "move_to", value_pattern=[[0], [2], [3]], timeout=max(arm.AXES_MOVE_TIMEOUT))


def start_control(bus_args=None):
    """Start the control loop for the robot arm."""
    if bus_args is None:
        bus_args = BUS_ARGS
    global control_process
    if control_process is not None:
        raise ValueError("Control loop is already running.")
    # Create a multiprocessing connection
    global motion_conn
    motion_conn, controller_conn = multiprocessing.Pipe(duplex=True)
    # Start the consumer process
    control_process = Process(target=control_trajectory, args=(controller_conn, bus_args))
    control_process.start()
    return motion_conn


def stop_control():
    """Stop the control loop for the robot arm."""
    global control_process
    if control_process is None:
        raise ValueError("Control loop is not running.")
    motion_conn.send({"protocol": "system", "data": "quit"})
    control_process.join()
    motion_conn.close()
    control_process = None
    print("Control loop stopped.")


def control_trajectory(controller_conn: Connection, bus_args: dict):
    print("Motion controller: Initializing")
    tick_len = 0.1 / 60.0  # in minutes (due to RPM units)
    # Run loop to move all axes to the target position
    last_tick = time.time()
    dt = tick_len  # in minutes (due to RPM units)
    elapsed_time = 0  # in minutes (due to RPM units)
    last_stop_time = 0  # when the last trajectory has ended (used for sleep timeout)
    last_info_update = 0
    last_heartbeat = 0
    # pid = [PIDController(kP=1.0, kI=0.0, kD=0.01, max_I=arm.AXES_SPEED_LIMIT[axis]/2, max_error=arm.AXES_SPEED_LIMIT[axis]/2) for axis in AXES]
    # pid = [PIDController(kP=0.8, kI=0.01, kD=0.1, max_I=arm.AXES_SPEED_LIMIT[axis]/2, max_error=arm.AXES_SPEED_LIMIT[axis]/2) for axis in AXES]
    # pid = [PIDController(kP=0.25, kI=0.05, kD=0.025, max_I=arm.AXES_SPEED_LIMIT[axis], max_error=arm.AXES_SPEED_LIMIT[axis]) for axis in AXES]
    pid = [PIDController(kP=0.6, kI=0.005, kD=0.075, max_I=arm.AXES_SPEED_LIMIT[axis] / 2,
                         max_error=arm.AXES_SPEED_LIMIT[axis] / 2) for axis in AXES]
    # pid = [PIDController(kP=0.6, kI=0.08, kD=0.25, max_I=arm.AXES_SPEED_LIMIT[axis]/2, max_error=arm.AXES_SPEED_LIMIT[axis]/2) for axis in AXES]
    planned_speed_mixin = 0.5  # default: average = 0.5 as in https://source-robotics.github.io/PAROL-docs/page4/#list-of-active-commands
    speed_threshold = 1.5  # RPM
    planned_positions: list[Optional[float]] = [None] * arm.NUM_AXES
    planned_speeds = [0.0] * arm.NUM_AXES
    current_trajectory = None
    # Format:
    # current_trajectory = [
    #    (t1, [p1_ax0, p1_ax1, ...], [s1_ax0, s1_ax1, ...]),  # waypoint 1 at time t1: positions and speeds for all axes
    #    (t2, [p1_ax0, p1_ax1, ...], [s1_ax0, s1_ax1, ...]),  # waypoint 2 at time t2
    #    ...
    # ]
    trajectory_queue = SimpleQueue()
    motor_state: list[dict[str, Optional[Any]]] = [{
        "can_id": axis2canid(axis),
        "state": STOPPED,
        "angle": None,
        "locked": None,
    } for axis in range(arm.NUM_AXES)]
    message_waiting = False  # flag to make sure that we only poll controller_conn once per loop

    control_loop_active = True

    def update_motor_angles(bus, axis=None):
        nonlocal motor_state
        axes_to_handle = [axis] if axis is not None else AXES
        for axis in axes_to_handle:
            try:
                (encoder,) = bus.ask(axis2canid(axis), "encoder", timeout=0.1)
                angle = encoder_to_motor_angle(encoder)
                motor_state[axis]["angle"] = angle
            except TimeoutError as e:
                print(f"WARNING: Motion controller got no answer from CAN ID {axis2canid(axis)} when asking for encoder: {e}")
                return
        if axis is not None:
            return motor_state[axis]["angle"]

    def update_motor_states(bus, axis=None):
        nonlocal motor_state
        axes_to_handle = [axis] if axis is not None else AXES
        for axis in axes_to_handle:
            try:
                (motor_status_code,) = bus.ask(axis2canid(axis), "motor_status", timeout=0.1)
                # 0: error, 1: stopped, 2: accelerate, 3: decelerate, 4: full speed, 5: homing
                # print(f"Motor state: {motor_state}")
                motor_state[axis]["state"] = [ERROR, STOPPED, MOVING, STOPPING, MOVING, MOVING][motor_status_code]
            except TimeoutError as e:
                print(f"WARNING: Motion controller got no answer from CAN ID {axis2canid(axis)} when asking for motor_status: {e}")
                return
        if axis is not None:
            return motor_state[axis]["state"]

    def update_motor_locks(bus, axis=None):
        nonlocal motor_state
        axes_to_handle = [axis] if axis is not None else AXES
        for axis in axes_to_handle:
            try:
                (lock_state,) = bus.ask(axis2canid(axis), "shaft_lock_status", timeout=0.1)
                motor_state[axis]["locked"] = lock_state
            except TimeoutError as e:
                print(f"WARNING: Motion controller got no answer from CAN ID {axis2canid(axis)} when asking for shaft_lock_status: {e}")
                return
        if axis is not None:
            return motor_state[axis]["locked"]

    def update_all_motor_info(bus):
        update_motor_angles(bus)
        update_motor_states(bus)
        update_motor_locks(bus)

    def clear_queue():
        while not trajectory_queue.empty():
            trajectory_queue.get_nowait()
        print("Motion controller: Trajectory queue cleared")

    def end_trajectory():
        nonlocal current_trajectory, planned_positions, planned_speeds, last_stop_time, elapsed_time
        if current_trajectory is not None:
            print("Motion controller: Enable idling at end position")
            planned_positions = current_trajectory[-1][1]
            planned_speeds = [0.0] * arm.NUM_AXES
            last_stop_time = elapsed_time
            current_trajectory = None
        else:
            print("Motion controller: No trajectory to end.")

    def check_next_trajectory():
        nonlocal current_trajectory, elapsed_time, control_loop_active
        if current_trajectory is None:
            if not trajectory_queue.empty():
                current_trajectory = trajectory_queue.get_nowait()
                elapsed_time = 0
                control_loop_active = True
                print(f"Motion controller: Starting new trajectory: {current_trajectory}")

    print("Motion controller: Starting control loop.")
    # with MockBus(**bus_args) as bus:
    with Bus(**bus_args) as bus:
        # TODO: catch and handle fatal errors on this level
        update_all_motor_info(bus)
        while True:
            # TODO: catch and handle non-fatal errors on this level
            if (elapsed_time - last_heartbeat) * 60 >= 1.0:  # every second (should be every 10 seconds or so eventually)
                print(f"Motion controller: Heartbeat {60*dt=}, {elapsed_time=}, {planned_positions=}, {planned_speeds=}, {current_trajectory=}, {motion_conn=}")
                last_heartbeat = elapsed_time

            if (elapsed_time - last_info_update) * 60 >= 200.0:  # TODO: change to every second, but make sure that main program maintains its own always-up-to-date motor state
                if not control_loop_active:
                    update_motor_angles(bus)  # Gets updated by the loop itself, saves one transaction per device
                update_motor_states(bus)
                update_motor_locks(bus)
                last_info_update = elapsed_time

            # Check if the current trajectory has ended
            if current_trajectory is not None and elapsed_time > current_trajectory[-1][0]:
                # current trajectory ends
                print("Motion controller: Trajectory ended")
                end_trajectory()

            check_next_trajectory()

            # Get the next trajectory if there is none or the previous one has ended
            # if current_trajectory is None:  # TODO: should this also work if a trajectory is running?
            if message_waiting or controller_conn.poll():  # got a message via the control connection
                item = controller_conn.recv()  # Todo: change to movement (including speeds) (how to interpolate then?)
                protocol: str = item and str(item.get('protocol')).lower()
                data: Union[list, str] = protocol and item.get('data')
                if protocol == "system":
                    if data.lower() == "quit":
                        print("Motion controller: Received quit signal")
                        stop_all(bus=bus)
                        break
                    if data.lower() == "stop":
                        print("Motion controller: Received stop signal")
                        stop_all(bus=bus)
                        clear_queue()
                        end_trajectory()
                        last_stop_time = -1
                    elif data.lower() == "clear_trajectory_queue":
                        print("Motion controller: Clearing trajectory queue")
                        clear_queue()
                    elif data.lower() == "state":
                        if not control_loop_active:
                            mode = "can_mode"
                        elif current_trajectory is not None:
                            mode = "trajectory"
                        else:
                            mode = "idle"
                        state = {
                            "mode": mode,
                            "motors": motor_state,
                        }
                        # print(f"Getting mode {state}")
                        controller_conn.send(state)
                elif protocol == "trajectory":
                    # Add path to trajectory queue
                    trajectory_queue.put_nowait(data)
                    print(f"Motion controller: New trajectory queued: {data}")
                elif protocol == "can_bus":
                    # Low-level CAN commands
                    # print(f"Received CAN command: {data}")
                    control_loop_active = False
                    msg_id = item["id"]  # TODO: should we also enforce ID matching for trajectories?
                    method, args, kwargs = data
                    if method == "send":
                        # print(f"Sending CAN message: {args=}, {kwargs=} to {bus}")
                        bus.send(*args, **kwargs)
                    elif method == "wait_for":
                        # print(f"Waiting for CAN message: {args=}, {kwargs=} from {bus}")
                        result = bus.wait_for(*args, **kwargs)
                        controller_conn.send({"id": msg_id, "result": result})
                    elif method == "ask":
                        start_time = time.time()
                        # print(f"Control dispatcher at 0s: Asking for CAN message: {args=}, {kwargs=} from {bus}")
                        # print(f"{bus._msg_buffer=}")
                        try:
                            result = bus.ask(*args, **kwargs)
                            # print(f"Control dispatcher at {time.time() - start_time}s: Received CAN message: {result}")
                            controller_conn.send({"id": msg_id, "result": result})
                        except Exception:
                            # print(f"Control dispatcher at {time.time() - start_time}s: Exception while asking for CAN message")
                            raise
                    elif method == "flush":
                        # print(f"Asking for CAN message: {args=}, {kwargs=} from {bus}")
                        bus.flush()
                    elif method == "close":
                        # print(f"Asking for CAN message: {args=}, {kwargs=} from {bus}")
                        bus.close()
                    else:
                        raise TypeError(f"Motion controller: Received unknown CAN method: {method}")
                else:
                    raise TypeError(f"Motion controller: Received unknown protocol: {protocol}")

            if control_loop_active:
                # Get the positions and speeds for the current time
                if  current_trajectory is not None:
                    planned_positions, planned_speeds = get_positions_and_speeds(current_trajectory, elapsed_time)

                # Control the arm with planned positions and speeds for the axes
                for axis in AXES:
                    # Get the current position
                    curr_pos = update_motor_angles(bus, axis=axis)

                    if planned_positions[axis] is None:  # No planned position: keep current position
                        planned_positions[axis] = curr_pos

                    planned_speed = planned_speeds[axis]
                    planned_position = planned_positions[axis]

                    # Calculate the PID error
                    pid_error = pid[axis](planned_position, curr_pos,
                                          dt * 60)  # *60 is just arbitrary scaling to have sensible PID values
                    # print(f"{elapsed_time=}, {axis=}: {curr_pos=}, {planned_position=}, {dt=}, {planned_position-curr_pos=}, {pid_error=}")

                    adjusted_speed = planned_speed_mixin * planned_speed + (1 - planned_speed_mixin) * pid_error
                    adjusted_speed = constrain(adjusted_speed, -arm.AXES_SPEED_LIMIT[axis], arm.AXES_SPEED_LIMIT[axis])
                    if abs(adjusted_speed) < speed_threshold:
                        adjusted_speed = 0
                    direction = 1 if adjusted_speed > 0 else 0
                    direction = 1 - direction if arm.AXES_RAW_DIRECTION[axis] else direction
                    accel = arm.AXES_ACCEL_LIMIT[axis]
                    # print(f"Axis {axis}: {direction=}, {adjusted_velocity=}, {accel=}")
                    bus.ask(axis2canid(axis), "move", [direction, abs(adjusted_speed), accel], answer_pattern=[None])
                    planned_speeds[axis] = adjusted_speed
                    planned_positions[axis] = curr_pos

                # TODO: re-enable idling after debugging
                # # Stop all axes after a short while if no movement is planned
                # if current_trajectory is None and (last_stop_time - elapsed_time)*60 >= 1.0:
                #     print("Trajectory control entering sleep mode.")
                #     stop_all(bus=bus)
                #     last_stop_time = -1
                #     control_loop_active = False

            # Wait for the next tick (incoming control message overrules waiting)
            dt = 0
            message_waiting = False  # flag to make sure that we only poll once per loop
            while not(dt > tick_len or (message_waiting := controller_conn.poll())):
                dt = (time.time() - last_tick) / 60.0
                time.sleep(0.01)
            elapsed_time += dt
            last_tick = time.time()



# def get_control_state():
#     """Get the current state of the control loop."""
#     if control_process is None:
#         return "stopped"
#     motion_conn.send({"protocol": "system", "data": "state"})
#     if motion_conn.poll(timeout=1):
#         return motion_conn.recv()
#     return None

def get_positions_and_speeds(trajectory: list[tuple[float, list[float], list[float]]], t: float, t0: float = 0.0) -> tuple[list[float], list[float]]:
    """Get the positions and speeds of all axes at a given time from a trajectory.
    Interpolates between the given time points.
    :param trajectory: the trajectory list of tuples of times, positions, and speeds for each axis
    :param t: the time to get the positions and speeds for
    :param t0: the start time offset of the trajectory (default: 0.0 = no offset)
    :return: a tuple of lists of axis positions and speeds
    """
    if not trajectory:
        raise ValueError("Trajectory is empty.")
    if t <= 0:
        return trajectory[0][1], trajectory[0][2]
    if t >= trajectory[-1][0]:
        return trajectory[-1][1], trajectory[-1][2]
    for i in range(len(trajectory) - 1):
        if trajectory[i][0] <= t < trajectory[i + 1][0]:
            # Trajectory waypoints are lists of axis values, each a tuple of time, position and speed
            t1 = trajectory[i][0]
            t2 = trajectory[i + 1][0]
            positions: list[float] = []
            speeds: list[float] = []
            for p1, s1, p2, s2 in zip(*trajectory[i][1:], *trajectory[i + 1][1:]):
                # For all axes, interpolate between the two waypoints
                if t1 == t2:
                    positions.append(p1)
                    speeds.append(s1)
                elif t1 <= t < t2:
                    positions.append(p1 + (p2 - p1) * (t - t1) / (t2 - t1))
                    speeds.append(s1 + (s2 - s1) * (t - t1) / (t2 - t1))
            return positions, speeds
    return trajectory[-1][1], trajectory[-1][2]  # In case something goes wrong with the time comparison

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
    bus = bus or current_bus.get()
    # print(f"get_curr_motor_positions got bus: {bus}")
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
    from mks_bus import encoder_to_motor_angle, motor_angle_to_encoder
    from motion import BusProxy
    bus = BusProxy()
    motor_pos = [encoder_to_motor_angle(bus.ask(axis, "encoder")[0]) for axis in range(1, 7)]
    print(f"Motor positions: {motor_pos}")

    # with Bus() as bus:
    #     bus.send(1, "move", [0, 200, 150])
    #     # bus.send(1, "move_by", [0, 150, 50])

    # with BusProxy() as bus:
    #     print(f"Control process: {control_process}")
    #     time.sleep(5)
    #     bus.send(1, "move", [0, 200, 150])
    #     time.sleep(5)
    #     bus.send(1, "move", [0, 0, 150])
    #     time.sleep(5)


#     from mks_bus import Bus
#     bus = Bus()
#     print(get_curr_motor_positions(bus))
#     send_axes(cmd="release_shaft_lock", bus=bus, answer_pattern=[None])
#     send_axes(cmd="set_zero", bus=bus, answer_pattern=[None])
#     # send_axes(cmd="zero_mode", values=[2, 1, 0, 0], bus=bus, answer_pattern=[1])
#     print(get_curr_motor_positions(bus))
#     axis = 0
#     force_end_pos = True
#     pos = [None, None, None, None, None, None]
#     pos[axis] = 3000
#     move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
#     time.sleep(1)
#     # print("correcting...")
#     # move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
#     # time.sleep(2)
#     pos = [None, None, None, None, None, None]
#     pos[axis] = 0
#     move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
#     time.sleep(1)
#     bus.close()
#
#     never = False
#     if never:
#         from mks_bus import Bus
#         bus = Bus()
#         print(get_curr_motor_positions(bus))
#         send_axes(cmd="release_shaft_lock", bus=bus, answer_pattern=[None])
#         send_axes(cmd="set_zero", bus=bus, answer_pattern=[None])
#         # send_axes(cmd="zero_mode", values=[2, 1, 0, 0], bus=bus, answer_pattern=[1])
#         print(get_curr_motor_positions(bus))
#         axes = [1]
#         send_axes(axes, cmd="set_direction", values=[1], bus=bus, answer_pattern=[1])
#         print("before move_to", get_curr_motor_positions(bus))
#         send_axes(axes, cmd="move_to", values=[500, 100, 0x4000], bus=bus, answer_pattern=[[0], [2], [3]])
#         print("after move_to", get_curr_motor_positions(bus))
#         send_axes(axes, cmd="move_to", values=[500, 100, 0], bus=bus, answer_pattern=[[0], [2], [3]])
#
#         print("before move", get_curr_motor_positions(bus))
#         send_axes(axes, cmd="move", values=[0, 100, 50], bus=bus)
#         time.sleep(1.5)
#         send_axes(axes, cmd="move", values=[0, 0, 50], bus=bus, answer_pattern=[[0], [2], [3]])
#         print("after move", get_curr_motor_positions(bus))
#         send_axes(axes, cmd="move_to", values=[500, 100, 0], bus=bus, answer_pattern=[[0], [2], [3]])
#
#         force_end_pos=True
#         pos = [0, -2000, 0, 0, 0, 0]
#         move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
#         move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
#         pos = [None, 4000, None, None, None, None]
#         move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
#         move_to_motor_position(pos, force_end_pos=force_end_pos, bus=bus)
