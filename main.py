import time

import numpy as np

from hand_controller import HandController
import viser

from utils import print_sensor_bars

# class HandController:

#     def __init__(self, p_gain: float):
#         self.p_gain = p_gain

#     def update(self, state):
#         self.state = state

#     def get_input(self, goal_state):
#         delta_x = goal_state - self.state
#         return self.p_gain * delta_x


SERVO_OFF = 335


def server(hand):

    server = viser.ViserServer()

    button_groups = []

    # option -> (level, duration)

    SERVO_MOVE_SMALL = 20
    SERVO_MOVE_LARGE = 50
    SERVO_SMALL_MOVE_SECONDS = 0.05
    SERVO_LARGE_MOVE_SECONDS = 0.1
    manual_moves = {
        "--": (SERVO_OFF - SERVO_MOVE_LARGE, SERVO_LARGE_MOVE_SECONDS),
        "-": (SERVO_OFF - SERVO_MOVE_SMALL, SERVO_SMALL_MOVE_SECONDS),
        "+": (SERVO_OFF + SERVO_MOVE_SMALL, SERVO_SMALL_MOVE_SECONDS),
        "++": (SERVO_OFF + SERVO_MOVE_LARGE, SERVO_LARGE_MOVE_SECONDS),
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
                level, duration = manual_moves[event.target.value]
                try:
                    hand.manually_move_actuator(index, level, duration)
                except RuntimeError as e:
                    print(e)

    last_received_count = 0
    while True:
        # redraw whenever new sensor values arrived
        received_count = hand.received_count
        if received_count != last_received_count:
            last_received_count = received_count
            print_sensor_bars(hand.sensors, max_value=1023)
        time.sleep(0.05)


if __name__ == "__main__":

    with HandController(initial_actuator_value=SERVO_OFF) as hand:
        server(hand)


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
17 5/B
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
