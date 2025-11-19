# motor_control.py

import serial
import time
import re
import serial.tools.list_ports
from threading import Lock
from tkinter import messagebox

# Globals for motor
motor_ser = None
serial_lock = Lock()

#STOP BUTTON FLAG AND FIX  edit:11/05/2025
request_stop_flag = False
#---------------------------

# Speed-related
current_speed = 75  # default speed
last_command_time = 0
command_cooldown = 0.002

# Hardcoded origin for each axis
axis_origins = { ##gustavo edit:11/05/2025
    'X': 98572.500, # old 102032.500
    'Y': 16880.000, #old 15837.500
    'Z': 19027.500, # old 17557.500
    'r': 4300.840,  #old 4001.740
    't': 3010020.000, #old 2647220.000
    'T': 2611552.500
}

# For manual control
base_displacement = 75     # degrees for linear axes
r_displacement = 0.25      # degrees for rotary axis (r)

def find_port(device_description: str):
    """Search through serial ports for one matching 'device_description' in port.description."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if device_description in port.description:
            print(f"{device_description} detected on port {port.device}")
            return port.device
    print(f"No device with description '{device_description}' found.")
    return None

def connect_to_device(port, device_name):
    """Create and return a serial connection for the motor."""
    global motor_ser
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        print(f"Connected successfully to {device_name} on {port}")
        if device_name == "Motor Controller":
            motor_ser = ser
            initialize_motor_controller(ser)
        return ser
    except serial.SerialException as e:
        print(f"Connection error for {device_name} on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to {device_name} on {port}")
        return None

def initialize_motor_controller(ser):
    """Check the motor controller is responding."""
    print("Initializing motor controller...")
    response = send_command(ser, "?R\r", "Motor Controller")
    if response == "?R":
        print("Motor controller initialized successfully.")
    else:
        print(f"Motor controller initialization may have failed. Response: {response}")

def auto_connect_motor():
    """Auto-detect and connect to the motor device."""
    motor_port = find_port("USB-SERIAL CH340")
    if motor_port:
        connect_to_device(motor_port, "Motor Controller")
    else:
        messagebox.showerror("Error", "Motor control device not found.")

def send_command(ser, command, device_name, axis=None, pos_tolerance=0.5, retries=100, delay=0.125):
    """
    Sends a command over serial and attempts to read a response.
    Includes:
      - A cooldown to avoid spamming commands.
      - Up to 'retries' attempts to read data.
      - If 'axis' is given, checks position each retry. If the axis 
        hasn't moved more than 'pos_tolerance' between retries, 
        we assume no response is coming and break early.

    Returns the response if received, or None if no response after 
    'retries' attempts or if the axis is no longer changing.
    """

    global last_command_time
    current_time = time.time()

    # Cooldown check
    if current_time - last_command_time < command_cooldown:
        print(f"{device_name} command cooldown active. Skipping command '{command.strip()}'.")
        return None

    # Send the command
    with serial_lock:
        ser.write(command.encode())
        print(f"Sent command to {device_name}: {command.strip()}")
        last_command_time = current_time

    # Small initial pause before reading
    time.sleep(0.2)

    # If we want to track position changes, capture the old position
    old_pos = None
    if axis:
        old_pos = get_current_position(axis)  # or your function for reading axis pos
        if old_pos is None:
            print(f"Warning: Cannot read initial position for axis {axis}. Position check skipped.")

    # Retry logic
    for attempt in range(retries):
        # Check if there's incoming data
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode().strip()
            print(f"{device_name} response: {response}")
            return response
        
        # No incoming data, so let's see if the axis is done moving
        if axis and old_pos is not None:
            new_pos = get_current_position(axis)  # read position again
            if new_pos is not None:
                # If it hasn't changed much, break out (no real response is coming)
                if abs(new_pos - old_pos) < pos_tolerance:
                    print(f"{device_name}: position stable on axis {axis}, no serial response.")
                    return None
                # Otherwise, update old_pos for next check
                old_pos = new_pos
            else:
                print(f"Warning: lost position feedback for axis {axis}.")

        # If we got here, no data yet, axis not stable => wait then retry
        print(f"Retrying read ({attempt + 1}/{retries})... No response yet.")
        time.sleep(1)

    # If we finish all retries, return None
    print(f"Warning: No response from {device_name} after {retries} attempts.")
    return None

def retrieve_motor_speed():
    """Query the current speed from the motor controller and store it in current_speed."""
    global motor_ser, current_speed
    if motor_ser:
        motor_ser.reset_input_buffer()
        resp = send_command(motor_ser, "?V\r", "Motor Controller", delay=0.5)
        if resp:
            m = re.search(r'V\s*(\d+)', resp)
            if m:
                speed_val = int(m.group(1))
                if 0 <= speed_val <= 255:
                    current_speed = speed_val
                    print(f"Motor speed retrieved: {current_speed}")
                else:
                    print(f"Speed value out of range: {speed_val}")
            else:
                print("No valid speed found in response.")
        else:
            print("No response to speed inquiry.")
    else:
        print("motor_ser is None; cannot retrieve speed.")

def update_speed(new_speed):
    """Set a new speed in the motor controller."""
    global motor_ser, current_speed
    if motor_ser:
        cmd = f"V{new_speed}\r"
        resp = send_command(motor_ser, cmd, "Motor Controller")
        if resp:
            current_speed = new_speed
            print(f"Speed updated to {new_speed}.")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")

def get_current_speed():
    """Accessor so the GUI can read the global current_speed."""
    return current_speed

# ---------------------------
# Conversions
# ---------------------------
def µm_to_steps(µm, axis, screw_pitch=2.0, steps_per_rev=200):
    microstepping = 1 if axis == 't' else 8
    steps_per_µm = (steps_per_rev * microstepping) / (screw_pitch * 2000)
    return round(µm * steps_per_µm)

def steps_to_µm(steps, axis, screw_pitch=2.0, steps_per_rev=200):
    microstepping = 1 if axis == 't' else 8
    µm_per_step = (screw_pitch * 2000) / (steps_per_rev * microstepping)
    return steps * µm_per_step

def convert_degrees_to_pulses(deg):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rot_pulse_equiv = stepper_angle_deg / (transmission_ratio * subdivision)
    return round(deg / rot_pulse_equiv)

def pulses_to_degrees(pulses):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rot_pulse_equiv = stepper_angle_deg / (transmission_ratio * subdivision)
    return pulses * rot_pulse_equiv

# ---------------------------
# Movement & Polling
# ---------------------------
def get_current_position(axis):
    global motor_ser
    if not motor_ser:
        print("Motor device not connected.")
        return None

    motor_ser.reset_input_buffer()
    resp = send_command(motor_ser, f"?{axis}\r", "Motor Controller", delay=0.5)
    if not resp:
        print(f"No response for position inquiry on axis {axis}.")
        return None

    m = re.search(rf'{axis}\s*=?\s*([+-]?\d+)', resp)
    if not m:
        print(f"Could not parse position from response: {resp}")
        return None

    steps = int(m.group(1))
    if axis == 'r':
        return pulses_to_degrees(steps)
    else:
        return steps_to_µm(steps, axis)

def wait_for_axis_stop(axis, max_wait=None):
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

def move_linear_stage(axis, direction, displacement_µm,
                      wait_for_stop=False, max_wait=15.0):
    global motor_ser
    if not motor_ser:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return

    if axis not in ['X','Y','Z','r','t','T'] or direction not in ['+','-']:
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
        #IF stop is resset  edit:11/05/2025
        global request_stop_flag
        if request_stop_flag:
            print("Movement interrupted by stop request.")
            request_stop_flag = False  # reset flag
            return
        #---------------------------

        if wait_for_stop:
            stopped = wait_for_axis_stop(axis, max_wait)
            if not stopped:
                print(f"Warning: axis {axis} may still be moving after {max_wait}s.")
            else:
                print(f"Axis {axis} move complete.")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {e}")
        print(f"Exception: {e}")

def stop_motor_control():
 #ADDING STOP REQUEST FLAG SETTING  edit:11/05/2025
    global motor_ser, request_stop_flag
    request_stop_flag = True
    if motor_ser:
        send_command(motor_ser, "STOP", "Motor Controller")
        print("Motor STOP command.")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")
    #---------------------------
    #ORIGINAL
    # global motor_ser
    # if motor_ser:
    #     send_command(motor_ser, "STOP", "Motor Controller")
    #     print("Motor control stopped.")
    # else:
    #     messagebox.showerror("Error", "Not connected to motor control device.")

#ADDING STOP REQUEST FUNCTION  edit:11/05/2025 
def request_stop():
    global request_stop_flag
    request_stop_flag = True
    print("Stop requested.")
#---------------------------

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

def set_origin_to_current():
    """
    Sets the origin values in axis_origins dictionary to the current position of each axis.
    """
    global axis_origins
    axes_list = ['X','Y','Z','r','t','T']
    print("\n--- Setting New Origin Values ---")
    for ax in axes_list:
        pos = get_current_position(ax)
        if pos is None:
            print(f"Axis {ax}: position unknown, origin not updated")
        else:
            axis_origins[ax] = pos
            print(f"Axis {ax}: origin set to {pos:.3f}")
    print("--- Origin Values Updated ---\n")

def return_to_origin():
    axes_to_move = ['X','Y','Z','r','T']  # skip 't'
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
