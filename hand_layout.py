from enum import Enum, IntEnum


class Finger(IntEnum):
    THUMB = 1
    INDEX = 2
    MIDDLE = 3
    RING = 4
    PINKY = 5


class Joint(Enum):
    """Values are the labels used on the hand, e.g. "3/B" is the second joint of the middle finger."""

    THUMB_A = "1/A"
    THUMB_B = "1/B"
    INDEX_A = "2/A"
    INDEX_B = "2/B"
    MIDDLE_A = "3/A"
    MIDDLE_B = "3/B"
    RING_A = "4/A"
    RING_B = "4/B"
    PINKY_A = "5/A"
    PINKY_B = "5/B"
    WRIST_SWIVEL = "wrist_swivel"  # (or wave)
    WRIST_FLEX = "wrist_flex"  # forward and backward


FINGER_JOINTS = {
    Finger.THUMB: (Joint.THUMB_A, Joint.THUMB_B),
    Finger.INDEX: (Joint.INDEX_A, Joint.INDEX_B),
    Finger.MIDDLE: (Joint.MIDDLE_A, Joint.MIDDLE_B),
    Finger.RING: (Joint.RING_A, Joint.RING_B),
    Finger.PINKY: (Joint.PINKY_A, Joint.PINKY_B),
}

# sensor index measuring the absolute position of each joint, None if unknown
JOINT_POSITION_SENSOR_INDEX = {
    Joint.THUMB_A: 18,
    Joint.THUMB_B: 20,
    Joint.INDEX_A: 10,
    Joint.INDEX_B: 8,
    Joint.MIDDLE_A: 11,
    Joint.MIDDLE_B: 9,
    Joint.RING_A: 16,
    Joint.RING_B: 21,
    Joint.PINKY_A: 17,
    Joint.PINKY_B: 22,
    Joint.WRIST_SWIVEL: 19,
    Joint.WRIST_FLEX: 23,
}

# TODO: actuator index -> sensor index measuring the torque exerted on that servo,
# once the actuator order is known


def lookup_joint_position(sensor_values, joint: Joint):
    index = JOINT_POSITION_SENSOR_INDEX[joint]
    if index is None:
        raise KeyError(f"No position sensor known for {joint.name}")
    return sensor_values[index]
