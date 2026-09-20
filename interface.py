import ctypes


class HostToHandCommand(ctypes.Structure):
    _fields_ = [
        ("actuators", ctypes.c_int16 * 16),
    ]


class HandToHostCommand(ctypes.Structure):
    _fields_ = [
        ("sensors", ctypes.c_int16 * 32),
    ]
