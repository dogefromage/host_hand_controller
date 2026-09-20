import time

from interface import HostToHandCommand, HandToHostCommand
from message_protocol_wrapper import add_received_bytes, encapsulate_message, extract_message
import viser

import serial


def server(ser):

    server = viser.ViserServer()

    sliders = []

    for i in range(16):
        slider = server.gui.add_slider(
            label=f"Slider {i + 1}",
            min=200,
            max=410,
            step=1,
            initial_value=330,
        )
        sliders.append(slider)

    while True:

        SEND_INTERVAL = 0.5  # 20 ms

        start_time = time.monotonic()
        while start_time + SEND_INTERVAL > time.monotonic():
            # do other things in the meantime

            # ADD ALL RECEIVED BYTES
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                add_received_bytes(data)

            # TRY TO EXTRACT STRUCT
            hand_to_host_command = HandToHostCommand()
            while extract_message(hand_to_host_command):
                print("HandToHostCommand:")
                print(hand_to_host_command.sensors[:])

        # SEND VALUES
        command_to_send = HostToHandCommand()
        for i in range(16):
            command_to_send.actuators[i] = sliders[i].value
        message = encapsulate_message(bytes(command_to_send))
        ser.write(message)

        # print("Payload:")
        # print(bytes(command_to_send).hex(sep=" "))
        # print("Message:")
        # print(message.hex(sep=" "))


if __name__ == "__main__":

    while True:
        try:
            ser = serial.Serial(
                "/dev/ttyACM0",
                timeout=3,
            )
            print("Successfully connected to serial!")
            break
        except Exception as e:
            print(e)
        print("Waiting to reconnect...")
        time.sleep(5)

    server(ser)
