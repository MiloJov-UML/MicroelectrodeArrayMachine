import serial
import threading
import tkinter as tk
import cv2
import time
import serial.tools.list_ports
import keyboard
from tkinter import messagebox

# ========================
# === Motor/Relay Setup ===
# ========================
motor_ser = None
relay_ser = None
serial_lock = threading.Lock()
camera_running = False
current_speed = 75  # Default speed (range: 0-150)
first_relay_connection = True

# Command cooldown
last_command_time = 0
command_cooldown = 0.002  # seconds

# Control settings
keyboard_control_enabled = False
base_displacement = 75  # Default step size (µm or degrees) for all axes except 'r'
r_displacement = 0.25   # Default step size (degrees) for rotary axis 'r'

# Axis controls for keyboard input
axis_controls = {
    'w': ('X', '-'),  # X axis positive
    's': ('X', '+'),  # X axis negative
    'a': ('Y', '+'),  # Y axis positive
    'd': ('Y', '-'),  # Y axis negative
    'shift': ('Z', '+'),  # Z axis positive
    'ctrl': ('Z', '-'),   # Z axis negative
    'e': ('r', '+'),      # r axis positive (rotary)
    'q': ('r', '-'),      # r axis negative (rotary)
    'z': ('t', '-'),      # T1 axis negative
    'x': ('t', '+'),      # T1 axis positive
    'r': ('T', '-'),      # T2 axis negative
    'f': ('T', '+')       # T2 axis positive
}

# =========================
# === Camera Recording  ===
# =========================
def record_and_display_camera(camera_index, output_filename, display_window_name):
    """
    Opens camera `camera_index`, displays the feed in a window,
    and records it to an AVI file (`output_filename`).
    Press 'q' in the window to stop recording and close.
    """
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return

    # Query camera's default resolution
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define codec and create VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    fps = 30.0
    out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    print(f"Recording camera {camera_index} to {output_filename}. Press 'q' to stop.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from camera {camera_index}")
            break

        # Write the frame to the video file
        out.write(frame)

        # Optional: display the live feed
        cv2.imshow(display_window_name, frame)

        # Stop if 'q' is pressed in that camera's window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"Stopping recording for camera {camera_index}.")
            break

    # Release resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()

# =========================
# === Serial Port Setup ===
# =========================
def find_port(device_description):
    """Automatically detect a port based on the device description."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if device_description in port.description:
            print(f"{device_description} detected on port {port.device}")
            return port.device
    print(f"No device with description '{device_description}' found.")
    return None

def auto_detect_ports():
    """Automatically detect and populate the COM ports for motor and relay devices."""
    global motor_port_entry, relay_port_entry
    motor_port = find_port("USB-SERIAL CH340")
    relay_port = find_port("Arduino Uno")

    if motor_port:
        motor_port_entry.delete(0, tk.END)
        motor_port_entry.insert(0, motor_port)
    else:
        messagebox.showerror("Error", "Motor control device not found.")

    if relay_port:
        relay_port_entry.delete(0, tk.END)
        relay_port_entry.insert(0, relay_port)
    else:
        messagebox.showerror("Error", "Relay device not found.")

def connect_to_device(port, device_name):
    """Connects to the specified device on a COM port and initializes if needed."""
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        print(f"Connected successfully to {device_name} on {port}")
        if device_name == "Motor Controller":
            initialize_motor_controller(ser)
        return ser
    except serial.SerialException as e:
        print(f"Connection error for {device_name} on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to {device_name} on {port}")
        return None

def initialize_motor_controller(ser):
    """Sends an initialization command to the motor controller."""
    try:
        print("Initializing motor controller...")
        response = send_command(ser, "?R\r", "Motor Controller")
        if response == "?R":
            print("Motor controller initialized successfully.")
        else:
            print(f"Motor controller initialization failed. Response: {response}")
    except Exception as e:
        print(f"Error during motor controller initialization: {e}")

def send_command(ser, command, device_name):
    """Sends a command to a serial device, respecting the global cooldown."""
    global last_command_time
    current_time = time.time()

    if current_time - last_command_time < command_cooldown:
        print(f"{device_name} command cooldown active. Skipping command.")
        return  # Skip command if in cooldown

    try:
        with serial_lock:
            ser.write(command.encode())
            print(f"Sent command to {device_name}: {command.strip()}")
            last_command_time = current_time
            time.sleep(0.01)  # Short wait for a response

            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode().strip()
                print(f"{device_name} response: {response}")
                return response
            else:
                print(f"No response from {device_name}.")
                return None
    except serial.SerialException as e:
        print(f"Failed to send command '{command.strip()}' to {device_name}: {e}")
        return None

# =========================
# === Motor Control API ===
# =========================
def convert_degrees_to_pulses(degrees):
    """Convert degrees to pulses for a rotary stepper system."""
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def µm_to_steps(µm, axis, screw_pitch=2.0, steps_per_rev=200):
    """
    Convert micrometers to step counts.
    For axis 't', microstepping=1; for others, microstepping=8.
    """
    microstepping = 1 if axis == 't' else 8
    steps_per_µm = (steps_per_rev * microstepping) / (screw_pitch * 1000)
    steps = round(µm * steps_per_µm)
    print(f"Converting {µm} µm on axis {axis} to {steps} steps "
          f"(screw pitch={screw_pitch}, microstepping={microstepping}).")
    return steps

def move_linear_stage(axis, direction, displacement_µm):
    """Sends a move command to the motor controller."""
    if motor_ser is None:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return

    valid_axes = ['X', 'Y', 'Z', 'r', 't', 'T']
    if axis not in valid_axes or direction not in ['+', '-']:
        messagebox.showerror("Error", "Invalid axis or direction.")
        return

    if axis == 'r':
        # For rotary axis, interpret displacement_µm as degrees (or unify naming if needed)
        steps = convert_degrees_to_pulses(displacement_µm)
    else:
        # For linear axes, interpret displacement_µm as micrometers
        steps = µm_to_steps(displacement_µm, axis)

    command = f"{axis}{direction}{int(steps)}\r"
    send_command(motor_ser, command, "Motor Controller")

def update_speed(new_speed):
    """Send a speed update command to the motor controller."""
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
    """Sends a stop command to the motor controller."""
    if motor_ser:
        send_command(motor_ser, "STOP", "Motor Controller")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")
    print("Motor control stopped.")

def continuous_motor_control():
    """
    Continuously listens for keyboard presses and, if keyboard_control_enabled is True,
    moves the motor accordingly.
    """
    global keyboard_control_enabled
    while True:
        if keyboard_control_enabled:
            for key, (axis, direction) in axis_controls.items():
                if keyboard.is_pressed(key):
                    step_size = r_displacement if axis == 'r' else base_displacement
                    # If space is held, halve the step size
                    if keyboard.is_pressed("space"):
                        step_size /= 2
                    move_linear_stage(axis, direction, step_size)
        time.sleep(0.1)

# ===================
# === Relay API   ===
# ===================
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

def toggle_keyboard_control():
    """Toggle whether keyboard input drives the motor."""
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")

# ==================
# === GUI Logic  ===
# ==================
def run_full_manual_loop():
    """
    Execute a manual loop of motor commands:
      - For each angle in [0, 10, 20, -10, -20]:
        1) Move r from 0 to 'angle' (relative).
        2) For each axis in [X, Y, Z]:
           - Move +1000 from 0, wait, move -1000 to return to 0
           - Move -1000 from 0, wait, move +1000 to return to 0
        3) Move r back to 0 if angle != 0.
      - Finally, do a safety check to ensure all axes are at zero.

    All moves are relative, with 1 second pauses so you can observe each step.
    """

    try:
        print("Starting Full Manual Loop: ±1000 µm moves in X, Y, Z at various r angles...")

        angles = [0, 10, 20, -10, -20]  # The r angles to test
        axes = ['X', 'Y', 'Z']         # Which linear axes we'll move

        for angle in angles:
            # Move r from 0 to angle (relative)
            if angle != 0:
                print(f"\n--- Moving r to {angle} degrees ---")
                if angle > 0:
                    move_linear_stage("r", "+", angle)
                else:
                    move_linear_stage("r", "-", abs(angle))
                time.sleep(1)

            # For each of X, Y, Z, move +1000 and back, then -1000 and back
            for axis in axes:
                # +1000, then return
                print(f"Moving {axis} +1000 at r={angle}")
                move_linear_stage(axis, "+", 1000)
                time.sleep(1)
                print(f"Returning {axis} to 0 from +1000")
                move_linear_stage(axis, "-", 1000)
                time.sleep(1)

                # -1000, then return
                print(f"Moving {axis} -1000 at r={angle}")
                move_linear_stage(axis, "-", 1000)
                time.sleep(1)
                print(f"Returning {axis} to 0 from -1000")
                move_linear_stage(axis, "+", 1000)
                time.sleep(1)

            # Return r to 0 if needed
            if angle != 0:
                print(f"Returning r to 0 from {angle} degrees")
                if angle > 0:
                    move_linear_stage("r", "-", angle)
                else:
                    move_linear_stage("r", "+", abs(angle))
                time.sleep(1)

        # === Final Safety Check ===
        # Since everything is relative, theoretically we should be at net zero now.
        # But let's explicitly ensure r=0, X=0, Y=0, Z=0 just in case.
        print("\nFinal safety check: ensuring all axes are at 0.")
        
        # Rotary axis to 0 (if not already)
        move_linear_stage("r", "+", 0)  # This won't move if we're already at 0,
                                        # but let's keep the call symmetrical
        time.sleep(0.5)

        # For each of X, Y, Z, do zero net move
        # (Again, won't move if we truly are at 0, but ensures we send a command if needed)
        for axis in axes:
            move_linear_stage(axis, "+", 0)
            time.sleep(0.5)

        print("\nFull Manual Loop completed, and all axes should be at the original starting position.")

    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during the manual loop: {e}")

def launch_gui():
    """Creates and launches the main Tkinter GUI."""
    global motor_port_entry, relay_port_entry, speed_entry

    # ----------------
    # Inner functions
    # ----------------
    def connect():
        """Connect to motor and relay based on user-entered ports."""
        global motor_ser, relay_ser
        motor_port = motor_port_entry.get().strip()
        relay_port = relay_port_entry.get().strip()

        motor_ser = connect_to_device(motor_port, "Motor Controller")
        relay_ser = connect_to_device(relay_port, "Arduino Controller")

        # You could start camera threads here if you prefer to only capture
        # once the devices are connected. 
        # For simplicity, we start cameras in __main__ below.

    def move_stage():
        """Move the motor stage based on GUI user input."""
        try:
            axis = axis_entry.get().strip()
            direction = direction_var.get()
            displacement = int(displacement_entry.get().strip())
            move_linear_stage(axis, direction, displacement)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def update_speed_gui():
        """Update speed from user input."""
        try:
            new_speed = int(speed_entry.get().strip())
            if 0 <= new_speed <= 150:
                update_speed(new_speed)
            else:
                messagebox.showerror("Error", "Speed must be between 0 and 150.")
        except ValueError:
            messagebox.showerror("Error", "Invalid speed value.")

    # ----------------
    # GUI Layout
    # ----------------
    root = tk.Tk()
    root.title("Motor Control and Camera Feed")

    # Auto-detect button
    tk.Button(root, text="Auto-Detect Ports", command=auto_detect_ports).pack(pady=5)

    # Motor port input
    tk.Label(root, text="Motor Port:").pack()
    motor_port_entry = tk.Entry(root)
    motor_port_entry.pack()

    # Relay port input
    tk.Label(root, text="Arduino Port:").pack()
    relay_port_entry = tk.Entry(root)
    relay_port_entry.pack()

    # Connect button
    tk.Button(root, text="Connect", command=connect).pack(pady=5)

    # Speed control
    tk.Label(root, text="Current Speed (0-150):").pack()
    speed_entry = tk.Entry(root)
    speed_entry.insert(0, str(current_speed))
    speed_entry.pack()
    tk.Button(root, text="Update Speed", command=update_speed_gui).pack(pady=5)

    # Axis Input
    tk.Label(root, text="Axis (X, Y, Z, r, t, T):").pack()
    axis_entry = tk.Entry(root)
    axis_entry.pack()

    # Direction Input
    direction_var = tk.StringVar(value='+')
    tk.Radiobutton(root, text='Positive (+)', variable=direction_var, value='+').pack()
    tk.Radiobutton(root, text='Negative (-)', variable=direction_var, value='-').pack()

    # Displacement Input
    tk.Label(root, text="Displacement (µm or degrees for 'r'):").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    # Motor Control Buttons
    tk.Button(root, text="Move Stage", command=move_stage).pack(pady=5)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=5)

    # Keyboard Control Toggle
    tk.Checkbutton(root, text="Keyboard Movement Mode",
                   command=toggle_keyboard_control).pack(pady=5)

    # Relay Control
    tk.Button(root, text="Laser Relay On", command=laser_relay_on).pack(pady=5)
    tk.Button(root, text="Laser Relay Off", command=laser_relay_off).pack(pady=5)

    # Example "Full Manual Loop"
    tk.Button(root, text="Run Full Manual Loop", command=run_full_manual_loop).pack(pady=10)

    # Start the main event loop
    root.mainloop()

# ===================
# === Entry Point ===
# ===================
if __name__ == "__main__":
    # 1. Start the GUI in a separate thread
    gui_thread = threading.Thread(target=launch_gui)
    gui_thread.start()

    # 2. Start the keyboard control thread
    motor_control_thread = threading.Thread(target=continuous_motor_control)
    motor_control_thread.start()

    # 3. Start camera recording threads
    camera_0_thread = threading.Thread(
        target=record_and_display_camera,
        args=(0, "camera0.avi", "Camera 0 Feed")
    )
    camera_1_thread = threading.Thread(
        target=record_and_display_camera,
        args=(1, "camera1.avi", "Camera 1 Feed")
    )
    camera_0_thread.start()
    camera_1_thread.start()

    # 4. Wait for the camera threads to finish (when 'q' is pressed in each window)
    camera_0_thread.join()
    camera_1_thread.join()

    print("Both camera recordings finished.")

    # Optionally, join the GUI thread so script doesn't exit until GUI closes
    gui_thread.join()
    print("GUI closed. All done.")
