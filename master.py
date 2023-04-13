import serial
import time
from serial.tools import list_ports

MAX_SEQUENCE_LENGTH = 1

def find_arduino_port(allowed_description_substrings=["Arduino", "mEDBG", "USB Serial Device"]):
    print("Looking for Arduino...")
    print([(p.device, p.description, p.manufacturer, p.name, p.product) for p in list_ports.comports()])

    def is_arduino(p):
        return any(substring in p.description for substring in allowed_description_substrings)

    arduino_ports = [
        p.device
        for p in list_ports.comports()
        if is_arduino(p)
    ]

    if not arduino_ports:
        raise IOError("No Arduino found")
    if len(arduino_ports) > 1:
        print(f'Multiple Arduinos found: {", ".join(arduino_ports)}')
        print('Using the first one')

    return arduino_ports[0]

def send_connect(ser):
    ser.write(b"CONNECT\n")
    print("Awaiting config from Arduino...")
    await_config(ser)
    print("Waiting for Arduino to be ready...")
    await_state(ser, "STATE:READY_FOR_NEW_DRAWING")
    print("Connected to Arduino\n")

def send_initial_position(ser, x, y):
    cmd = f"INIT:{x},{y}\n"
    print(f"Setting initial position to {x}, {y}...")
    ser.write(cmd.encode())
    await_state(ser, "STATE:DRAWING")
    print("Initial position set")
    await_state(ser, "STATE:AWAITING_COMMAND_SEQUENCE")
    print("Ready to receive binary data\n")

def send_binary_data(ser, data):
    sequence_length = len(data) // 4  # There are 4 bytes per control point
    if sequence_length > MAX_SEQUENCE_LENGTH:
        raise Exception(f"Sequence length ({sequence_length}) exceeds maximum ({MAX_SEQUENCE_LENGTH})")
    cmd = f"DRAW:{len(data)}\n"
    ser.write(cmd.encode())
    await_state(ser, "STATE:READING_COMMAND_SEQUENCE")
    ser.write(data)

def draw(ser, data):
    print("Sending draw command...")
    chunks = [data[i:i + MAX_SEQUENCE_LENGTH * 4] for i in range(0, len(data), MAX_SEQUENCE_LENGTH * 4)]
    print(f"Sending {len(chunks)} chunks of data...")
    
    for chunk in chunks:
        send_binary_data(ser, chunk)
        await_state(ser, "STATE:DRAWING")
        print("Arduino drawing")
        await_state(ser, "STATE:AWAITING_COMMAND_SEQUENCE")
        print("Ready to receive binary data\n")

def send_end_drawing(ser):
    print("Sending end drawing command...")
    ser.write(b"END_DRAWING\n")
    await_state(ser, "STATE:READY_FOR_NEW_DRAWING")
    print("Arduino ready for new drawing\n")

def await_config(ser):
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Received Config: {response}")
            if not response.startswith("CONFIG"):
                raise Exception(f"Unexpected response from Arduino: {response}")
            config_str = response.split(":")[1]
            config = [config_val.split("=") for config_val in config_str.split(",")]
            # This config is now a list of key value pairs
            # For now we only handle MAX_SEQUENCE_LENGTH as the key sequence_length
            for key, value in config:
                if key == "sequence_length":
                    global MAX_SEQUENCE_LENGTH
                    MAX_SEQUENCE_LENGTH = int(value)
                    print(f"Setting MAX_SEQUENCE_LENGTH to {MAX_SEQUENCE_LENGTH}")
            break

def await_state(ser, expected_state):
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Received: {response}")
            if response == expected_state:
                break
            elif response.startswith("ERROR"):
                # print(f"Error received from Arduino: {response}")
                raise Exception(f"Error received from Arduino: {response}")
                break
            elif response.startswith("INFO"):
                # print(f"Info received from Arduino: {response}")
                pass

if __name__ == "__main__":
    arduino_port = find_arduino_port()
    ser = serial.Serial(arduino_port, 9600)
    time.sleep(2)  # Give the Arduino some time to start up

    # Connect and wait for the Arduino to be ready for a new drawing
    send_connect(ser)

    # Set initial position (x, y)
    send_initial_position(ser, 10, 20)

    # Send binary data
    binary_data = b'\x00\x01\x10\x01\x00\x00\x00\x00'  # Example binary data
    draw(ser, binary_data)

    # End drawing
    send_end_drawing(ser)

    ser.close()