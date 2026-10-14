import threading
import time

import serial

from interface import HostToHandCommand, HandToHostCommand
from message_protocol_wrapper import add_received_bytes, encapsulate_message, extract_message

NUM_ACTUATORS = 16
NUM_SENSORS = 32


class HandController:
    """Talks to the hand over serial in a background thread.

    The thread keeps (re)connecting to the device, feeds received bytes into the
    protocol parser, stores the latest sensor state and sends the current
    actuator values every `send_interval` seconds.
    """

    def __init__(
        self,
        port="/dev/ttyACM0",
        send_interval=0.01,
        reconnect_interval=5.0,
        read_timeout=0.005,
        initial_actuator_value=335,
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
        self._actuators = [initial_actuator_value] * NUM_ACTUATORS
        self._sensors = [0] * NUM_SENSORS
        self._last_received_time = None
        self._received_count = 0

        self._thread = threading.Thread(target=self._run, name="HandController", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    @property
    def sensors(self):
        """Copy of the last received sensor values."""
        with self._lock:
            return list(self._sensors)

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
    def actuators(self):
        """Copy of the actuator values that are being sent."""
        with self._lock:
            return list(self._actuators)

    def set_actuator(self, index, value):
        with self._lock:
            self._actuators[index] = int(value)

    def set_actuators(self, values):
        values = [int(v) for v in values]
        if len(values) != NUM_ACTUATORS:
            raise ValueError(f"Expected {NUM_ACTUATORS} actuator values, got {len(values)}")
        with self._lock:
            self._actuators = values

    def manually_move_actuator(self, index, level, duration, end_value=None):
        """Drive one actuator at `level` for exactly `duration` seconds, then stop it.

        Blocks for `duration`. While moving, the communication thread is paused:
        received messages are discarded and the other actuators keep the values
        they had when the move started. Afterwards the actuator is set to
        `end_value` (default: the value it had before the move).
        """
        if not 0 <= index < NUM_ACTUATORS:
            raise IndexError(f"Actuator index {index} out of range")

        self._pause_event.set()
        try:
            with self._serial_lock:
                if not self.is_connected:
                    raise RuntimeError("Hand is not connected")

                with self._lock:
                    values = list(self._actuators)
                if end_value is None:
                    end_value = values[index]

                values[index] = int(level)
                self._write_actuators(values)
                start = time.perf_counter()
                with self._lock:
                    self._actuators[index] = int(level)

                self._sleep_until(start + duration)

                values[index] = int(end_value)
                self._write_actuators(values)
                with self._lock:
                    self._actuators[index] = int(end_value)

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
        """Retry connecting until it succeeds or the controller is closed."""
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
                    self._sensors = hand_to_host_command.sensors[:]
                    self._last_received_time = time.monotonic()
                    self._received_count += 1

    def _send(self):
        if self._pause_event.is_set():
            return
        with self._lock:
            values = list(self._actuators)
        with self._serial_lock:
            self._write_actuators(values)

    def _write_actuators(self, values):
        """Send one actuator command. Caller must hold _serial_lock."""
        command_to_send = HostToHandCommand()
        for i, value in enumerate(values):
            command_to_send.actuators[i] = value
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
