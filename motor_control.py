# motor_control.py

import serial
import time
import re
import serial.tools.list_ports
from threading import Lock, Event
from tkinter import messagebox

# Globals for motor
motor_ser = None
serial_lock = Lock()
keyboard_pause = Event()  # set while a blocking GUI command is running; keyboard control suspends
emergency_stop_event = Event()  # latched when GUI emergency stop is requested

# Speed-related
current_speed = 30  # default speed
last_command_time = {}   # per-axis key -> timestamp, avoids one axis blocking another
command_cooldown = 0.015  # seconds — allows one full command to transmit at 9600 baud before the next
last_any_write_time = 0.0  # timestamp of the most recent write to any axis
last_written_axis = None   # which axis was written last
inter_axis_delay = 0.025   # seconds to wait when switching between different axes

# Hardcoded origin for each axis
axis_origins = { 
    'X': 94127.500, 
    'Y': 2564577.500, 
    'Z': 2611352.500, 
    'r': 4383.490, 
    't': 3020520.000, 
    'T': 2611552.500 
}

# For manual control
base_displacement = 5     # degrees for linear axes
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

def send_command(ser, command, device_name, axis=None, pos_tolerance=0.5, retries=100, delay=1.0, blocking=True):
    """
    Sends a command over serial and attempts to read a response.
    Includes:
      - A cooldown to avoid spamming commands.
      - reset_input_buffer() before each write to clear stale bytes.
      - If blocking=False, writes the command and returns immediately —
        used by keyboard control so X and Y pulses can interleave without
        one axis blocking the thread while waiting for the other's response.
      - If blocking=True, polls up to 'retries' times (delay seconds apart,
        default 100 s total) to accommodate very slow moves.
      - If 'axis' is given, polls position each retry and exits as soon as the
        axis stops moving, so fast moves release the port early.

    The lock covers only the write so that concurrent threads can still
    send their own commands without waiting for a read to complete.

    Returns the response if received, or None otherwise.
    """

    global last_command_time, last_any_write_time, last_written_axis

    # Derive a per-axis cooldown key from the first letter of the command.
    # e.g. 'X+12\r' -> 'X', 'Y-8\r' -> 'Y', '?X\r' -> '?', 'V150\r' -> 'V'
    # This lets X and Y each have their own independent cooldown so both
    # axes can fire in the same keyboard loop iteration without blocking each other.
    cmd_key = command[0] if command else 'general'

    # Relay controller Arduino uses readStringUntil('\n') — each command must end
    # with '\n' so the Arduino processes it immediately instead of waiting for a
    # 1-second timeout.  Without this, rapid commands (e.g. stop turning off nordson
    # right after a routine turned it on) arrive inside the same readStringUntil
    # window and get concatenated into a garbage string that matches nothing.
    if device_name == "Relay Controller" and not command.endswith('\n'):
        command = command + '\n'

    # During emergency stop, block all motor movement commands.
    # Relay commands (laser, nordson, solenoid) are never blocked — routines abort
    # via _abort_if_emergency_stop() and clean up in finally blocks; blocking relay
    # commands here would break GUI toggles because the GUI runs in a non-main thread.
    if emergency_stop_event.is_set() and device_name == "Motor Controller" and command.strip() != "S":
        print(f"Emergency stop active. Skipping motor command: {command.strip()}")
        return None

    with serial_lock:
        now = time.time()
        if now - last_command_time.get(cmd_key, 0) < command_cooldown:
            print(f"{device_name} cooldown active ({cmd_key}). Skipping '{command.strip()}'.")
            return None

        # If switching to a different axis, wait for the inter-axis gap so the
        # controller has time to finish processing the previous axis command.
        if last_written_axis is not None and cmd_key != last_written_axis:
            elapsed = time.time() - last_any_write_time
            if elapsed < inter_axis_delay:
                time.sleep(inter_axis_delay - elapsed)

        # Only clear the input buffer for blocking commands (queries/slow moves).
        # Non-blocking keyboard writes must NOT clear it — a concurrent blocking
        # read may be waiting for a response that would be wiped.
        if blocking:
            ser.reset_input_buffer()

        ser.write(command.encode())
        print(f"Sent command to {device_name}: {command.strip()}")
        now = time.time()
        last_command_time[cmd_key] = now
        last_any_write_time = now
        last_written_axis = cmd_key

    # Non-blocking: fire-and-forget — used by keyboard control so rapid
    # X/Y pulses can interleave without one axis stalling the thread.
    if not blocking:
        return None

    # Short initial wait — at 9600 baud a typical response arrives in ~15 ms,
    # so we check quickly first before falling into the slower polling loop.
    time.sleep(0.05)
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting).decode('utf-8', errors='replace').strip()
        print(f"{device_name} response: {response}")
        return response

    # Capture initial position for early-exit detection on slow moves
    old_pos = None
    if axis:
        old_pos = get_current_position(axis)
        if old_pos is None:
            print(f"Warning: Cannot read initial position for axis {axis}. Position check skipped.")

    # Polling loop — 1 s between checks, up to 100 s total
    for attempt in range(retries):
        if emergency_stop_event.is_set() and command.strip() != "S":
            print(f"Emergency stop active. Aborting wait for '{command.strip()}'.")
            return None

        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='replace').strip()
            print(f"{device_name} response: {response}")
            return response

        # Exit early once the axis has stopped moving
        if axis and old_pos is not None:
            new_pos = get_current_position(axis)
            if new_pos is not None:
                if abs(new_pos - old_pos) < pos_tolerance:
                    print(f"{device_name}: position stable on axis {axis}, no serial response.")
                    return None
                old_pos = new_pos
            else:
                print(f"Warning: lost position feedback for axis {axis}.")

        print(f"Retrying read ({attempt + 1}/{retries})... No response yet.")
        time.sleep(delay)

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

def mm_to_um(mm_value):
    
    # Validate input type
    if not isinstance(mm_value, (int, float)):
        raise TypeError("Input must be an integer or float representing millimeters.")

    # Check for invalid numeric values
    if mm_value != mm_value or mm_value in (float("inf"), float("-inf")):
        raise ValueError("Input must be a finite number.")

    # Conversion: 1 mm = 1000 µm
    return mm_value * 1000

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
    if max_wait is None:
        max_wait = 30.0

    start_time = time.time()
    old_pos = get_current_position(axis)
    if old_pos is None:
        return False

    time.sleep(0.01)
    while time.time() - start_time < max_wait:
        if emergency_stop_event.is_set():
            print(f"Emergency stop active while waiting for axis {axis} to stop.")
            return False

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

    if emergency_stop_event.is_set():
        print(f"Emergency stop active. Ignoring move request {axis}{direction}{displacement_µm}.")
        return False

    if not motor_ser:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return False

    if axis not in ['X','Y','Z','r','t','T'] or direction not in ['+','-']:
        messagebox.showerror("Error", f"Invalid axis '{axis}' or direction '{direction}'.")
        return False

    try:
        if axis == 'r':
            steps = convert_degrees_to_pulses(displacement_µm)
        else:
            steps = µm_to_steps(displacement_µm, axis)

        cmd = f"{axis}{direction}{int(steps)}\r"

        if wait_for_stop:
            # Suspend keyboard control for the full duration of this blocking
            # command so keyboard pulses don't corrupt the serial read loop.
            keyboard_pause.set()

        resp = send_command(motor_ser, cmd, "Motor Controller", blocking=wait_for_stop)
        if not resp and wait_for_stop:
            print(f"Warning: No response for movement on axis {axis}.")

        if wait_for_stop:
            stopped = wait_for_axis_stop(axis, max_wait)
            if not stopped:
                print(f"Warning: axis {axis} may still be moving after {max_wait}s.")
            else:
                print(f"Axis {axis} move complete.")
        return True
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {e}")
        print(f"Exception: {e}")
        return False
    finally:
        # Always clear the pause flag, even if an exception occurred.
        if wait_for_stop:
            keyboard_pause.clear()

def flush_serial():
    """Clear both serial buffers. Call when switching between keyboard and GUI modes."""
    global motor_ser
    if motor_ser:
        with serial_lock:
            motor_ser.reset_input_buffer()
            motor_ser.reset_output_buffer()
        print("Serial buffers flushed.")

def stop_motor_control():
    global motor_ser
    if motor_ser:
        # Controller stop command is "S\r" (not "STOP") per the manufacturer example.
        # Use blocking=False so it fires immediately without waiting for a response.
        send_command(motor_ser, "S\r", "Motor Controller", blocking=False)
        # Clear any pause state and flush buffers so the port is clean after stop.
        keyboard_pause.clear()
        flush_serial()
        print("Motor control stopped.")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")

def emergency_stop_motors():
    """Latch emergency stop state and send immediate stop to motor controller."""
    emergency_stop_event.set()
    stop_motor_control()
    print("Emergency stop latched.")

def clear_emergency_stop():
    """Clear latched emergency stop state so routines can run again."""
    if emergency_stop_event.is_set():
        emergency_stop_event.clear()
        print("Emergency stop cleared.")

def is_emergency_stop_requested():
    return emergency_stop_event.is_set()

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
