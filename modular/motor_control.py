# motor_control.py

import serial
import time
import re
import serial.tools.list_ports
from threading import Lock
from tkinter import messagebox

# ----------------------------------------------------------------------
# GLOBALS for motor
# ----------------------------------------------------------------------
motor_ser = None
serial_lock = Lock()
current_speed = 75  # Default speed (range: 0-150)

# Command cooldown
last_command_time = 0
command_cooldown = 0.002  # seconds

# Hardcoded origin for each axis (in the same units as move_linear_stage)
axis_origins = {
    'X': 15340,
    'Y': 8722.5,
    'Z': 4991.25,
    'r': 3254.91,
    't': 712850,  # We'll skip 't' in go_to_all_origins, but you may use it elsewhere
    'T': 14102.5
}

base_displacement = 75
r_displacement = 0.25

# ----------------------------------------------------------------------
# Connection / Setup
# ----------------------------------------------------------------------

def find_port(device_description):
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if device_description in port.description:
            print(f"{device_description} detected on port {port.device}")
            return port.device
    print(f"No device with description '{device_description}' found.")
    return None

def connect_to_device(port, device_name):
    """Connects to the specified device on a COM port."""
    global motor_ser
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        print(f"Connected successfully to {device_name} on {port}")
        if device_name == "Motor Controller":
            motor_ser = ser  # store globally
            initialize_motor_controller(ser)
        return ser
    except serial.SerialException as e:
        print(f"Connection error for {device_name} on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to {device_name} on {port}")
        return None

def initialize_motor_controller(ser):
    """Check that the motor controller responds properly."""
    print("Initializing motor controller...")
    response = send_command(ser, "?R\r", "Motor Controller")
    if response == "?R":
        print("Motor controller initialized successfully.")
    else:
        print(f"Motor controller initialization may have failed. Response: {response}")

def auto_connect_motor():
    """Try auto-detection for the motor (USB-SERIAL CH340)."""
    motor_port = find_port("USB-SERIAL CH340")
    if motor_port:
        connect_to_device(motor_port, "Motor Controller")
    else:
        messagebox.showerror("Error", "Motor control device not found.")

# ----------------------------------------------------------------------
# Core Serial / Commands
# ----------------------------------------------------------------------

def send_command(ser, command, device_name, retries=3, delay=0.125):
    """Send a command and attempt to read response, with a cooldown."""
    import time  # local import to avoid confusion
    global last_command_time

    current_time = time.time()
    if current_time - last_command_time < command_cooldown:
        print(f"{device_name} command cooldown active. Skipping '{command.strip()}'.")
        return None

    with serial_lock:
        try:
            ser.write(command.encode())
            print(f"Sent command to {device_name}: {command.strip()}")
            last_command_time = current_time

            time.sleep(0.2)  # small pause before reading

            for attempt in range(retries):
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting).decode().strip()
                    print(f"{device_name} response: {response}")
                    return response
                print(f"Retrying read ({attempt + 1}/{retries})...")
                time.sleep(1.5)

            print(f"Warning: No response from {device_name} after {retries} attempts.")
            return None

        except serial.SerialException as e:
            print(f"Serial exception while sending '{command.strip()}' to {device_name}: {e}")
            return None

def handle_motor_response(response):
    if response is not None and "ERR" in response:
        print("Error from Motor Controller. Check configuration or command syntax.")
    else:
        print(f"Motor Controller response: {response}")

# ----------------------------------------------------------------------
# Speed / Positioning
# ----------------------------------------------------------------------

def retrieve_motor_speed():
    global current_speed, motor_ser
    if motor_ser:
        motor_ser.reset_input_buffer()
        response = send_command(motor_ser, "?V\r", "Motor Controller")
        if response:
            match = re.search(r'V\s*(\d+)', response)
            if match:
                speed_value = int(match.group(1))
                if 0 <= speed_value <= 255:
                    current_speed = speed_value
                    print(f"Motor speed retrieved: {current_speed}")
                else:
                    print(f"Retrieved speed out of range: {speed_value}")
            else:
                print("No valid speed found in response.")
        else:
            print("No response received from speed inquiry.")
    else:
        print("motor_ser is None; cannot retrieve speed.")

def update_speed(new_speed):
    global current_speed, motor_ser
    if motor_ser:
        command = f"V{new_speed}\r"
        response = send_command(motor_ser, command, "Motor Controller")
        if response:
            current_speed = new_speed
            print(f"Speed updated to {new_speed}.")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")

# ----------------------------------------------------------------------
# Conversions
# ----------------------------------------------------------------------

def µm_to_steps(µm, axis, screw_pitch=2.0, steps_per_rev=200):
    microstepping = 1 if axis == 't' else 8
    steps_per_µm = (steps_per_rev * microstepping) / (screw_pitch * 1000)
    return round(µm * steps_per_µm)

def steps_to_µm(steps, axis, screw_pitch=2.0, steps_per_rev=200):
    microstepping = 1 if axis == 't' else 8
    µm_per_step = (screw_pitch * 1000) / (steps_per_rev * microstepping)
    return steps * µm_per_step

def convert_degrees_to_pulses(deg):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rot_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    return round(deg / rot_pulse_equivalent)

def pulses_to_degrees(pulses):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rot_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    return pulses * rot_pulse_equivalent

# ----------------------------------------------------------------------
# Movement & Polling
# ----------------------------------------------------------------------

def get_current_position(axis):
    global motor_ser
    if not motor_ser:
        print("Motor device not connected.")
        return None
    motor_ser.reset_input_buffer()

    resp = send_command(motor_ser, f"?{axis}\r", "Motor Controller")
    if not resp:
        print(f"No response for position inquiry on axis {axis}.")
        return None

    match = re.search(rf'{axis}\s*=?\s*([+-]?\d+)', resp)
    if not match:
        print(f"Could not parse position from response: {resp}")
        return None

    steps = int(match.group(1))
    if axis == 'r':
        return pulses_to_degrees(steps)
    else:
        return steps_to_µm(steps, axis)

def wait_for_axis_stop(axis, max_wait=10.0):
    start_time = time.time()
    old_pos = get_current_position(axis)
    if old_pos is None:
        return False

    time.sleep(0.01)

    while time.time() - start_time < max_wait:
        new_pos = get_current_position(axis)
        if new_pos is None:
            return False
        if abs(new_pos - old_pos) < 0.5:  # tolerance
            return True
        old_pos = new_pos
        time.sleep(0.01)
    return False

def move_linear_stage(axis, direction, displacement_µm, wait_for_stop=False, max_wait=15.0):
    global motor_ser
    if not motor_ser:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return

    valid_axes = ['X','Y','Z','r','t','T']
    if axis not in valid_axes or direction not in ['+','-']:
        messagebox.showerror("Error", f"Invalid axis '{axis}' or direction '{direction}'.")
        return

    try:
        if axis == 'r':
            steps = convert_degrees_to_pulses(displacement_µm)
        else:
            steps = µm_to_steps(displacement_µm, axis)
        cmd = f"{axis}{direction}{int(steps)}\r"
        resp = send_command(motor_ser, cmd, "Motor Controller")
        if not resp:
            print(f"Warning: No response for movement on axis {axis}.")

        if wait_for_stop:
            stopped = wait_for_axis_stop(axis, max_wait)
            if not stopped:
                print(f"Warning: axis {axis} may still be moving or no feedback after {max_wait}s.")
            else:
                print(f"Axis {axis} move complete.")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {e}")
        print(f"Exception: {e}")

def stop_motor_control():
    global motor_ser
    if motor_ser:
        send_command(motor_ser, "STOP", "Motor Controller")
        print("Motor control stopped.")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")

def query_all_axes_positions():
    axes_list = ['X','Y','Z','r','t','T']
    print("\n--- Querying All Axes ---")
    for ax in axes_list:
        pos = get_current_position(ax)
        if pos is None:
            print(f"Axis {ax}: position unknown")
        else:
            print(f"Axis {ax}: {pos:.3f}")
    print("--- End of Query ---\n")

def go_to_all_origins():
    """Moves X,Y,Z,r,T to their stored origin in `axis_origins`."""
    axes_to_move = ['X','Y','Z','r','T']
    print("\n--- Moving All Axes to Stored Origins ---")
    for ax in axes_to_move:
        current_pos = get_current_position(ax)
        if current_pos is None:
            print(f"Cannot move axis {ax} to origin: position unknown.")
            continue
        origin_pos = axis_origins[ax]
        diff = origin_pos - current_pos
        direction = '+' if diff >= 0 else '-'
        displacement = abs(diff)
        print(f"Moving axis {ax} from {current_pos:.3f} to {origin_pos:.3f}...")
        move_linear_stage(ax, direction, displacement, wait_for_stop=True, max_wait=30.0)
    print("--- Finished Moving All Axes ---\n")

def run_full_manual_loop():
        """Example sequence of moves."""
        print("Starting Full Manual Loop...")

        move_linear_stage("Z", "+", 950, wait_for_stop=True, max_wait=30.0)
        time.sleep(1)

        laser_relay_on()
        time.sleep(0.25)

        move_linear_stage("T", "+", 20000, wait_for_stop=True, max_wait=30.0)
        time.sleep(3)

        laser_relay_off()
        time.sleep(1)

        move_linear_stage("T", "-", 20000, wait_for_stop=True, max_wait=30.0)
        time.sleep(3.5)

        move_linear_stage("Z", "-", 950, wait_for_stop=True, max_wait=30.0)
        time.sleep(1)

        print("Full Manual Loop completed.")
