import time

import numpy as np

from hand_hardware_interface import ACTUATOR_STOP_VELOCITY, NUM_SENSORS, HandHardwareInterface
import viser

from hand_layout import JOINT_POSITION_SENSOR_INDEX, Joint
from utils import print_sensor_bars


class HandController:

    def __init__(self, p_gain: float):
        self.p_gain = p_gain

    def update(self, state: np.ndarray):
        self.state = state

    def get_input(self, goal_state: np.ndarray):
        delta_x = goal_state - self.state
        return self.p_gain * delta_x


class SensorCalibration:
    """Linear mapping per sensor: calibrated = scale * raw + offset."""

    def __init__(self):
        self.scales = np.ones(NUM_SENSORS)
        self.offsets = np.zeros(NUM_SENSORS)

    def set_mapping(self, index, scale, offset):
        self.scales[index] = scale
        self.offsets[index] = offset

    def set_from_two_points(self, index, raw_a, value_a, raw_b, value_b):
        """Fit the mapping of one sensor through two (raw, calibrated value) points."""
        scale = (value_b - value_a) / (raw_b - raw_a)
        self.set_mapping(index, scale, value_a - scale * raw_a)

    def apply(self, raw_values):
        return self.scales * np.asarray(raw_values) + self.offsets


def server(hand: HandHardwareInterface, controller: HandController, calibration: SensorCalibration):

    server = viser.ViserServer()

    button_groups = []

    # option -> (velocity, duration)

    SERVO_VELOCITY_SMALL = 20
    SERVO_VELOCITY_LARGE = 50
    SERVO_SMALL_MOVE_SECONDS = 0.05
    SERVO_LARGE_MOVE_SECONDS = 0.1
    manual_moves = {
        "--": (ACTUATOR_STOP_VELOCITY - SERVO_VELOCITY_LARGE, SERVO_LARGE_MOVE_SECONDS),
        "-": (ACTUATOR_STOP_VELOCITY - SERVO_VELOCITY_SMALL, SERVO_SMALL_MOVE_SECONDS),
        "+": (ACTUATOR_STOP_VELOCITY + SERVO_VELOCITY_SMALL, SERVO_SMALL_MOVE_SECONDS),
        "++": (ACTUATOR_STOP_VELOCITY + SERVO_VELOCITY_LARGE, SERVO_LARGE_MOVE_SECONDS),
    }

    with server.gui.add_folder("Manual controls"):

        for i in range(16):
            button_group = server.gui.add_button_group(
                label=f"Channel {i + 1}",
                options=["--", "-", "+", "++"],
            )
            button_groups.append(button_group)

            @button_group.on_click
            def _(event, index=i):
                velocity, duration = manual_moves[event.target.value]
                try:
                    hand.manually_move_actuator(index, velocity, duration)
                except RuntimeError as e:
                    print(e)

    with server.gui.add_folder("Controller"):
        p_gain_slider = server.gui.add_slider("P gain", min=0, max=200, initial_value=0, step=1)

    goal_state_sliders = []
    with server.gui.add_folder("Goal State"):
        for i in range(16):
            goal_state_slider = server.gui.add_slider(f"Goal {i+1}", min=5, max=80, initial_value=45, step=1)
            goal_state_sliders.append(goal_state_slider)

    last_received_count = 0
    while True:

        # goal_position_slider.value
        joint_states = calibration.apply(hand.sensor_values)

        current_state = np.zeros(16)
        # fill state manually
        THUMB_A_STATE_INDEX = 13
        THUMB_B_STATE_INDEX = 15
        current_state[THUMB_A_STATE_INDEX] = joint_states[JOINT_POSITION_SENSOR_INDEX[Joint.THUMB_A]]
        current_state[THUMB_B_STATE_INDEX] = joint_states[JOINT_POSITION_SENSOR_INDEX[Joint.THUMB_B]]

        goal_state = np.zeros(16)
        for i, slider in enumerate(goal_state_sliders):
            goal_state[i] = np.deg2rad(slider.value)

        actuator_inputs = p_gain_slider.value * (goal_state - current_state)
        actuator_inputs *= -1  # fix flipped actuator axis

        for actuator_index in [THUMB_A_STATE_INDEX, THUMB_B_STATE_INDEX]:
            # only set the ones that work
            actuator_value = int(ACTUATOR_STOP_VELOCITY + actuator_inputs[actuator_index])
            hand.set_actuator_velocity(actuator_index, actuator_value)

        # redraw whenever new sensor values arrived
        received_count = hand.received_count
        if received_count != last_received_count:
            last_received_count = received_count
            print_sensor_bars(hand.sensor_values, max_value=1023)

        time.sleep(0.05)


if __name__ == "__main__":

    controller = HandController(
        p_gain=0.1,
    )

    calibration = SensorCalibration()

    # by manual testing
    calibration.set_from_two_points(JOINT_POSITION_SENSOR_INDEX[Joint.THUMB_A], 515, np.deg2rad(-5), 217, np.deg2rad(90))
    calibration.set_from_two_points(JOINT_POSITION_SENSOR_INDEX[Joint.THUMB_B], 525, np.deg2rad(0), 770, np.deg2rad(85))

    with HandHardwareInterface(initial_actuator_velocity=ACTUATOR_STOP_VELOCITY) as hand:
        server(hand, controller, calibration)


"""
 0
 1
 2
 3
 4
 5
 6
 7
 8 2/B
 9 3/B
10 2/A
11 3/A
12
13
14
15
16 4/A
17 5/A
18 1/A
19 wrist_swivel (or wave idk)
20 1/B
21 4/B
22 5/B
23 wrist_flex (forward and backward)
24
25
26
27
28
29
30
31
"""
