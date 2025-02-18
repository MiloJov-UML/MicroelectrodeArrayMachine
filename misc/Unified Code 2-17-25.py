import serial
import threading
import tkinter as tk
import cv2
import time
import serial.tools.list_ports
import keyboard
from tkinter import messagebox
import re

# Global serial variables for motor and relay
motor_ser = None
relay_ser = None
serial_lock = threading.Lock()
camera_running = False
current_speed = 75  # Default speed (range: 0-150)
speed_display_label = None  # To dynamically update speed in GUI

# Global variable to track the first connection to the relay
first_relay_connection = True

# Command cooldown time
last_command_time = 0
command_cooldown = 0.002  # seconds

# Control settings
keyboard_control_enabled = False  # State of keyboard control mode
base_displacement = 75  # Default step size in degrees for all axes except 'r'
r_displacement = 0.25    # Default step size in degrees for 'r' axis

# Axis controls for keyboard movement
axis_controls = {
    'w': ('X', '-'),     # X axis positive
    's': ('X', '+'),     # X axis negative
    'a': ('Y', '+'),     # Y axis positive
    'd': ('Y', '-'),     # Y axis negative
    'shift': ('Z', '+'), # Z axis positive
    'ctrl': ('Z', '-'),  # Z axis negative
    'e': ('r', '+'),     # r axis positive (rotary)
    'q': ('r', '-'),     # r axis negative (rotary)
    'z': ('t', '-'),     # T1 axis negative
    'x': ('t', '+'),     # T1 axis positive
    'r': ('T', '-'),     # T2 axis negative
    'f': ('T', '+')      # T2 axis positive
}

# ----------------------------------------------------------------------
# Hardcoded origin for each axis (in the same units as move_linear_stage):
# For linear axes (X, Y, Z, T, t): micrometers
# For rotary axis (r): degrees
# Update these with your desired "home" positions.
# ----------------------------------------------------------------------
axis_origins = {
    'X': 15340,
    'Y': 8722.5,
    'Z': 4991.25,
    'r': 3254.91,
    't': 712850,  # We'll skip 't' in go_to_all_origins, but you may use it elsewhere
    'T': 14102.5
}


# ----------------------------------------------------------------------
# Auto-detect devices
# ----------------------------------------------------------------------
def find_port(device_description):
    """Automatically detect a port based on the device description."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if device_description in port.description:
            print(f"{device_description} detected on port {port.device}")
            return port.device
    print(f"No device with description '{device_description}' found.")
    return None

def auto_connect():
    """Automatically detects and connects to the motor control and Arduino devices."""
    global motor_ser, relay_ser

    motor_port = find_port("USB-SERIAL CH340")
    relay_port = find_port("Arduino Uno")

    if motor_port:
        motor_ser = connect_to_device(motor_port, "Motor Controller")
    else:
        messagebox.showerror("Error", "Motor control device not found.")

    if relay_port:
        relay_ser = connect_to_device(relay_port, "Arduino Controller")
    else:
        messagebox.showerror("Error", "Relay device not found.")

    if motor_ser:
        retrieve_motor_speed()


def connect_to_device(port, device_name):
    """Connects to the specified device on a COM port and initializes the motor controller if needed."""
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        print(f"Connected successfully to {device_name} on {port}")
        if device_name == "Motor Controller":
            initialize_motor_controller(ser)  # Initialize motor controller on connection
        return ser
    except serial.SerialException as e:
        print(f"Connection error for {device_name} on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to {device_name} on {port}")
        return None


# ----------------------------------------------------------------------
# Motor controller initialization and queries
# ----------------------------------------------------------------------
def initialize_motor_controller(ser):
    """Connects to the motor controller and initializes it."""
    try:
        print("Initializing motor controller...")
        response = send_command(ser, "?R\r", "Motor Controller")  # Initialization check command
        if response == "?R":
            print("Motor controller initialized successfully.")
        else:
            print(f"Motor controller initialization may have failed. Response: {response}")
    except Exception as e:
        print(f"Error during motor controller initialization: {e}")


def retrieve_motor_speed():
    global current_speed

    if motor_ser:
        motor_ser.reset_input_buffer()  # Clear any previous data
        response = send_command(motor_ser, "?V\r", "Motor Controller", delay=0.5)

        if response:
            print(f"Raw speed response: {response}")
            match = re.search(r'V\s*(\d+)', response)
            if match:
                speed_value = int(match.group(1))
                if 0 <= speed_value <= 255:
                    current_speed = speed_value
                    print(f"Motor speed successfully retrieved: {current_speed}")
                    if speed_display_label:
                        speed_display_label.config(text=f"Current Speed: {current_speed}")
                else:
                    print(f"Retrieved speed value out of range: {speed_value}")
            else:
                print("No valid speed value found in the response.")
        else:
            print("No response received from speed inquiry.")


# ----------------------------------------------------------------------
# Command send / read utilities
# ----------------------------------------------------------------------
last_command_time = 0  # Already declared, re-stated for clarity

def send_command(ser, command, device_name, retries=3, delay=0.125):
    """
    Send a command over serial and attempt to read a response.
    Includes a simple cooldown and optional retries.
    """
    global last_command_time
    current_time = time.time()

    if current_time - last_command_time < command_cooldown:
        print(f"{device_name} command cooldown active. Skipping command '{command.strip()}'.")
        return None

    try:
        with serial_lock:
            ser.write(command.encode())
            print(f"Sent command to {device_name}: {command.strip()}")
            last_command_time = current_time

            time.sleep(0.2)  # brief pause before reading

            for attempt in range(retries):
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting).decode().strip()
                    print(f"{device_name} response: {response}")
                    return response  # Return response if we got something

                print(f"Retrying read ({attempt + 1}/{retries})... No response yet.")
                time.sleep(1.5)

            print(f"Warning: No response from {device_name} after {retries} attempts.")
            return None

    except serial.SerialException as e:
        print(f"Serial exception while sending command '{command.strip()}' to {device_name}: {e}")
        return None


def handle_motor_response(response):
    if "ERR" in response:
        print("Error received from Motor Controller. Check motor configuration or command syntax.")
    else:
        print(f"Motor Controller response: {response}")


# ----------------------------------------------------------------------
# Conversion helpers (match your hardware)
# ----------------------------------------------------------------------
def µm_to_steps(µm, axis, screw_pitch=2.0, steps_per_rev=200):
    """Converts micrometers to steps for linear axes."""
    microstepping = 1 if axis == 't' else 8
    steps_per_µm = (steps_per_rev * microstepping) / (screw_pitch * 1000)
    steps = round(µm * steps_per_µm)
    print(f"Converting {µm} µm on axis {axis} to {steps} steps. (pitch={screw_pitch}, micro={microstepping})")
    return steps

def steps_to_µm(steps, axis, screw_pitch=2.0, steps_per_rev=200):
    """Inverse of µm_to_steps: steps -> micrometers."""
    microstepping = 1 if axis == 't' else 8
    µm_per_step = (screw_pitch * 1000) / (steps_per_rev * microstepping)
    return steps * µm_per_step

def convert_degrees_to_pulses(degrees):
    """Convert degrees to step pulses for the rotary axis 'r'."""
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def pulses_to_degrees(pulses):
    """Inverse of convert_degrees_to_pulses: pulses -> degrees."""
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    degrees = pulses * rotation_pulse_equivalent
    return degrees


# ----------------------------------------------------------------------
# Polling to detect axis stop
# ----------------------------------------------------------------------
def wait_for_axis_stop(axis, max_wait=10.0):
    """
    Polls the current position every 0.2s for up to 'max_wait' seconds.
    If the position does not change between polls, we assume motion is complete.

    Returns True if motion stopped, or False if no detection within 'max_wait'.
    """
    start_time = time.time()
    old_pos = get_current_position(axis)
    if old_pos is None:
        return False

    time.sleep(0.01)

    while time.time() - start_time < max_wait:
        new_pos = get_current_position(axis)
        if new_pos is None:
            return False

        # If positions are effectively the same, motion is done
        if abs(new_pos - old_pos) < 0.5:  # Tolerance in µm or degrees
            return True

        old_pos = new_pos
        time.sleep(0.01)

    return False


# ----------------------------------------------------------------------
# Movement with optional wait
# ----------------------------------------------------------------------
def move_linear_stage(axis, direction, displacement_µm,
                      wait_for_stop=False,  # NEW: block until motion completes?
                      max_wait=15.0):       # NEW: how long to wait before giving up
    """
    Moves the specified axis by 'displacement_µm' in the given direction ('+' or '-').
    If axis == 'r', interpret 'displacement_µm' as degrees.
    If wait_for_stop=True, this function will poll the axis until it stops or times out.
    """

    if motor_ser is None:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return

    valid_axes = ['X', 'Y', 'Z', 'r', 't', 'T']
    if axis not in valid_axes or direction not in ['+', '-']:
        messagebox.showerror("Error", f"Invalid axis '{axis}' or direction '{direction}'.")
        return

    try:
        # Convert user displacement to steps (or pulses if axis == 'r')
        if axis == 'r':
            displacement_steps = convert_degrees_to_pulses(displacement_µm)
        else:
            displacement_steps = µm_to_steps(displacement_µm, axis)

        command = f"{axis}{direction}{int(displacement_steps)}\r"
        response = send_command(motor_ser, command, "Motor Controller")
        if response is None:
            print(f"Warning: No response for motor movement on axis {axis}.")

        # If requested, poll axis until it stops or times out
        if wait_for_stop:
            stopped = wait_for_axis_stop(axis, max_wait)
            if not stopped:
                print(f"Warning: axis {axis} may still be moving or no position feedback after {max_wait}s.")
            else:
                print(f"Axis {axis} move complete.")

    except Exception as e:
        messagebox.showerror("Error", f"An error occurred while moving the stage: {e}")
        print(f"Exception in move_linear_stage: {e}")


def update_speed(new_speed):
    """Update the motor speed."""
    global current_speed
    if motor_ser:
        command = f"V{new_speed}\r"
        response = send_command(motor_ser, command, "Motor Controller")
        if response:
            current_speed = new_speed
            print(f"Speed updated to {new_speed}.")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")


def stop_motor_control():
    """Stop the motor control."""
    if motor_ser:
        send_command(motor_ser, "STOP", "Motor Controller")  # Adjust if your firmware uses a different stop command
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")
    print("Motor control stopped.")


# ----------------------------------------------------------------------
# Position Inquiry
# ----------------------------------------------------------------------
def get_current_position(axis):
    """
    Ask the motor controller for the current absolute position of `axis`.
    Returns the position in µm (for X, Y, Z, t, T) or degrees (for r).
    """
    if motor_ser is None:
        print("Motor device not connected.")
        return None

    motor_ser.reset_input_buffer()  # clear any stale data

    command = f"?{axis}\r"
    response = send_command(motor_ser, command, "Motor Controller", delay=0.5)
    if not response:
        print(f"No response for position inquiry on axis {axis}.")
        return None

    match = re.search(rf'{axis}\s*=?\s*([+-]?\d+)', response)
    if not match:
        print(f"Could not parse position from response: {response}")
        return None

    steps = int(match.group(1))
    if axis == 'r':
        return pulses_to_degrees(steps)
    else:
        return steps_to_µm(steps, axis)


# ----------------------------------------------------------------------
# Query All Axes
# ----------------------------------------------------------------------
def query_all_axes_positions():
    """
    Queries the current position for all axes and prints them to the console.
    """
    axes_list = ['X', 'Y', 'Z', 'r', 't', 'T']
    print("\n--- Querying All Axes ---")
    for ax in axes_list:
        pos = get_current_position(ax)
        if pos is None:
            print(f"Axis {ax}: position unknown")
        else:
            print(f"Axis {ax}: {pos:.3f} (µm or deg)")
    print("--- End of Query ---\n")


# ----------------------------------------------------------------------
# Go to All Origins (Now uses wait_for_stop inside move_linear_stage)
# ----------------------------------------------------------------------
def go_to_all_origins():
    """
    Moves each axis (X, Y, Z, r, T) from current position to axis_origins[ax].
    Waits for each axis to finish before proceeding to the next.
    """
    print("\n--- Moving All Axes to Stored Origins ---")
    axes_to_move = ['X', 'Y', 'Z', 'r', 'T']  # Exclude 't' if you don't want to move it

    for ax in axes_to_move:
        current_pos = get_current_position(ax)
        if current_pos is None:
            print(f"Cannot move axis {ax} to origin: position unknown.")
            continue

        origin_pos = axis_origins[ax]
        difference = origin_pos - current_pos
        direction = '+' if difference >= 0 else '-'
        displacement = abs(difference)

        print(f"Moving axis {ax} from {current_pos:.3f} to origin {origin_pos:.3f}...")
        # Use move_linear_stage with wait_for_stop=True
        move_linear_stage(ax, direction, displacement, wait_for_stop=True, max_wait=30.0)

    print("--- Finished Moving All Axes ---\n")


# ----------------------------------------------------------------------
# Keyboard control loop
# ----------------------------------------------------------------------
def continuous_motor_control():
    global keyboard_control_enabled
    while True:
        if keyboard_control_enabled:
            try:
                for key, (axis, direction) in axis_controls.items():
                    if keyboard.is_pressed(key):
                        step_size = r_displacement if axis == 'r' else base_displacement
                        if keyboard.is_pressed("space"):
                            step_size /= 2
                        # For keyboard, let's do quick moves without waiting
                        move_linear_stage(axis, direction, step_size, wait_for_stop=False)
            except serial.SerialException as e:
                print(f"Serial exception during keyboard control: {e}")
                messagebox.showerror("Error", "Serial communication interrupted during keyboard control.")
                break
            except Exception as e:
                print(f"Unexpected error during keyboard control: {e}")
        time.sleep(0.01)


# ----------------------------------------------------------------------
# Relay control functions
# ----------------------------------------------------------------------
def laser_relay_on():
    """Turn the laser relay on."""
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_On", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def laser_relay_off():
    """Turn the laser relay off."""
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_Off", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")


# ----------------------------------------------------------------------
# Keyboard control toggling
# ----------------------------------------------------------------------
def toggle_keyboard_control():
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")


# ----------------------------------------------------------------------
# Camera
# ----------------------------------------------------------------------
def open_camera(camera_index):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from camera {camera_index}")
            break

        cv2.imshow(f"Camera {camera_index}", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ----------------------------------------------------------------------
# Main GUI
# ----------------------------------------------------------------------
def launch_gui():
    global speed_display_label

    root = tk.Tk()
    root.title("Motor Control and Camera Feed")

    # Attempt auto-connect on startup
    auto_connect()
    # Retrieve & display current speed
    retrieve_motor_speed()

    # Display the current motor speed
    speed_display_label = tk.Label(root, text=f"Current Speed: {current_speed}")
    speed_display_label.pack(pady=5)

    # Frame for speed UI
    speed_frame = tk.Frame(root)
    speed_frame.pack(pady=10)

    # Entry for setting speed
    tk.Label(speed_frame, text="Set Speed (0-150): ").pack(side='left')
    speed_entry = tk.Entry(speed_frame, width=5)
    speed_entry.insert(0, str(current_speed))
    speed_entry.pack(side='left', padx=5)

    def update_speed_gui():
        """Update the speed based on user input."""
        try:
            new_speed = int(speed_entry.get().strip())
            if 0 <= new_speed <= 150:
                update_speed(new_speed)
                speed_display_label.config(text=f"Current Speed: {current_speed}")
            else:
                messagebox.showerror("Error", "Speed must be between 0 and 150.")
        except ValueError:
            messagebox.showerror("Error", "Invalid speed value.")

    # Button to update speed
    tk.Button(root, text="Update Speed", command=update_speed_gui).pack(pady=5)

    # Axis Input (still available if you want single-axis moves)
    tk.Label(root, text="Axis (X, Y, Z, r, t, T):").pack()
    axis_entry = tk.Entry(root)
    axis_entry.pack()

    # Direction Input
    direction_var = tk.StringVar(value='+')
    tk.Radiobutton(root, text='Positive (+)', variable=direction_var, value='+').pack()
    tk.Radiobutton(root, text='Negative (-)', variable=direction_var, value='-').pack()

    # Displacement Input
    tk.Label(root, text="Displacement (µm for linear, degrees for 'r')").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    def move_stage_gui():
        """Move the motor stage with user inputs, waiting until it completes."""
        try:
            axis = axis_entry.get().strip()
            direction = direction_var.get()
            displacement = float(displacement_entry.get().strip())
            # We'll do wait_for_stop=True so the GUI blocks until the move is done
            move_linear_stage(axis, direction, displacement, wait_for_stop=True, max_wait=30.0)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    # Buttons for Motor Control
    tk.Button(root, text="Move Stage", command=move_stage_gui).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    # Keyboard Control Toggle
    keyboard_control_var = tk.IntVar(value=0)
    keyboard_control_checkbox = tk.Checkbutton(root, text="Keyboard Movement Mode",
                                               variable=keyboard_control_var,
                                               command=toggle_keyboard_control)
    keyboard_control_checkbox.pack(pady=5)

    # Buttons for Laser Relay Control
    tk.Button(root, text="Laser Relay On", command=laser_relay_on).pack(pady=5)
    tk.Button(root, text="Laser Relay Off", command=laser_relay_off).pack(pady=5)

    # Full manual loop (with blocking moves)
    tk.Button(root, text="Run Full Manual Loop", command=run_full_manual_loop).pack(pady=10)

    # Query all axes
    tk.Button(root, text="Query All Axes", command=query_all_axes_positions).pack(pady=5)

    # Go to all origins (Approach B with polling in move_linear_stage)
    tk.Button(root, text="Go to All Origins", command=go_to_all_origins).pack(pady=5)

    root.mainloop()


def run_full_manual_loop():
    """
    Execute a manual loop of motor commands, each time waiting for the move to finish.
    Adjust to your hardware commands and sequence as needed.
    """
    try:
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
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during the manual loop: {e}")


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------
if __name__ == "__main__":
    # Start the GUI in a separate thread
    threading.Thread(target=launch_gui).start()

    # Start keyboard control thread
    threading.Thread(target=continuous_motor_control).start()

    # Start camera threads
    camera_0_thread = threading.Thread(target=open_camera, args=(0,))
    camera_1_thread = threading.Thread(target=open_camera, args=(1,))
    camera_0_thread.start()
    camera_1_thread.start()

    camera_0_thread.join()
    camera_1_thread.join()
