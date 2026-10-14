import threading
import time

import serial

from hand_layout import Joint, lookup_joint_position
from interface import HostToHandCommand, HandToHostCommand
from message_protocol_wrapper import add_received_bytes, encapsulate_message, extract_message

NUM_ACTUATORS = 16
NUM_SENSORS = 32

# actuator velocity command at which the servo stands still
ACTUATOR_STOP_VELOCITY = 335


class HandHardwareInterface:
    """Talks to the hand over serial in a background thread.

    The thread keeps (re)connecting to the device, feeds received bytes into the
    protocol parser, stores the latest sensor values and sends the current
    actuator velocities every `send_interval` seconds.

    Actuator values are angular velocity commands for the servos. Sensor values
    are either the torque exerted on a servo or the absolute position of a
    joint, see hand_layout.
    """

    def __init__(
        self,
        port="/dev/ttyACM0",
        send_interval=0.01,
        reconnect_interval=5.0,
        read_timeout=0.005,
        initial_actuator_velocity=ACTUATOR_STOP_VELOCITY,
    ):
        self.port = port
        self.send_interval = send_interval
        self.reconnect_interval = reconnect_interval
        self.read_timeout = read_timeout

        self._serial = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # serial access is shared between the communication thread and manual moves.
        # while _pause_event is set the communication thread neither reads nor sends.
        self._serial_lock = threading.Lock()
        self._pause_event = threading.Event()

        # state shared with the communication thread, guarded by _lock
        self._actuator_velocities = [initial_actuator_velocity] * NUM_ACTUATORS
        self._sensor_values = [0] * NUM_SENSORS
        self._last_received_time = None
        self._received_count = 0

        self._thread = threading.Thread(target=self._run, name="HandHardwareInterface", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    @property
    def sensor_values(self):
        """Copy of the last received raw sensor values (torques and joint positions)."""
        with self._lock:
            return list(self._sensor_values)

    def joint_position(self, joint: Joint):
        """Last received absolute position of `joint`."""
        return lookup_joint_position(self.sensor_values, joint)

    @property
    def last_received_time(self):
        """time.monotonic() timestamp of the last received message, or None."""
        with self._lock:
            return self._last_received_time

    @property
    def received_count(self):
        with self._lock:
            return self._received_count

    @property
    def actuator_velocities(self):
        """Copy of the actuator velocities that are being sent."""
        with self._lock:
            return list(self._actuator_velocities)

    def set_actuator_velocity(self, index, velocity):
        with self._lock:
            self._actuator_velocities[index] = int(velocity)

    def set_actuator_velocities(self, velocities):
        velocities = [int(v) for v in velocities]
        if len(velocities) != NUM_ACTUATORS:
            raise ValueError(f"Expected {NUM_ACTUATORS} actuator velocities, got {len(velocities)}")
        with self._lock:
            self._actuator_velocities = velocities

    def manually_move_actuator(self, index, velocity, duration, end_velocity=None):
        """Drive one actuator at `velocity` for exactly `duration` seconds, then stop it.

        Blocks for `duration`. While moving, the communication thread is paused:
        received messages are discarded and the other actuators keep the velocities
        they had when the move started. Afterwards the actuator is set to
        `end_velocity` (default: the velocity it had before the move).
        """
        if not 0 <= index < NUM_ACTUATORS:
            raise IndexError(f"Actuator index {index} out of range")

        self._pause_event.set()
        try:
            with self._serial_lock:
                if not self.is_connected:
                    raise RuntimeError("Hand is not connected")

                with self._lock:
                    velocities = list(self._actuator_velocities)
                if end_velocity is None:
                    end_velocity = velocities[index]

                velocities[index] = int(velocity)
                self._write_actuator_velocities(velocities)
                start = time.perf_counter()
                with self._lock:
                    self._actuator_velocities[index] = int(velocity)

                self._sleep_until(start + duration)

                velocities[index] = int(end_velocity)
                self._write_actuator_velocities(velocities)
                with self._lock:
                    self._actuator_velocities[index] = int(end_velocity)

                # ignore everything received during the move
                self._serial.reset_input_buffer()
        finally:
            self._pause_event.clear()

    def close(self, timeout=None):
        """Stop the communication thread and close the serial port."""
        self._stop_event.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout)
        self._disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    # ------------------------------------------------------------------
    # Serial management
    # ------------------------------------------------------------------

    def _connect(self):
        """Try to open the serial port once. Returns True on success."""
        try:
            self._serial = serial.Serial(self.port, timeout=self.read_timeout)
            print("Successfully connected to serial!")
            return True
        except serial.SerialException as e:
            print(e)
            self._serial = None
            return False

    def _disconnect(self):
        ser = self._serial
        self._serial = None
        if ser is not None:
            try:
                ser.close()
            except serial.SerialException as e:
                print(e)

    def _wait_for_connection(self):
        """Retry connecting until it succeeds or the interface is closed."""
        while not self._stop_event.is_set():
            if self._connect():
                return True
            print("Waiting to reconnect...")
            self._stop_event.wait(self.reconnect_interval)
        return False

    # ------------------------------------------------------------------
    # Communication thread
    # ------------------------------------------------------------------

    def _run(self):
        try:
            while not self._stop_event.is_set():
                if not self.is_connected and not self._wait_for_connection():
                    break

                try:
                    self._receive_until(time.monotonic() + self.send_interval)
                    self._send()
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    self._disconnect()
        finally:
            self._disconnect()

    def _receive_until(self, deadline):
        while time.monotonic() < deadline and not self._stop_event.is_set():
            if self._pause_event.is_set():
                # a manual move owns the serial port
                time.sleep(self.read_timeout)
                continue

            # ADD ALL RECEIVED BYTES
            # blocks for at most read_timeout when nothing is waiting
            with self._serial_lock:
                data = self._serial.read(max(self._serial.in_waiting, 1))
            add_received_bytes(data)

            # TRY TO EXTRACT STRUCT
            hand_to_host_command = HandToHostCommand()
            while extract_message(hand_to_host_command):
                with self._lock:
                    self._sensor_values = hand_to_host_command.sensors[:]
                    self._last_received_time = time.monotonic()
                    self._received_count += 1

    def _send(self):
        if self._pause_event.is_set():
            return
        with self._lock:
            velocities = list(self._actuator_velocities)
        with self._serial_lock:
            self._write_actuator_velocities(velocities)

    def _write_actuator_velocities(self, velocities):
        """Send one actuator velocity command. Caller must hold _serial_lock."""
        command_to_send = HostToHandCommand()
        for i, velocity in enumerate(velocities):
            command_to_send.actuators[i] = velocity
        message = encapsulate_message(bytes(command_to_send))
        self._serial.write(message)
        self._serial.flush()

    @staticmethod
    def _sleep_until(deadline, spin_time=0.002):
        """Sleep until time.perf_counter() reaches deadline, spinning for the last bit."""
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return
            if remaining > spin_time:
                time.sleep(remaining - spin_time)
