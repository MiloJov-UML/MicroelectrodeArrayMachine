import serial, threading, tkinter as tk, cv2, time, serial.tools.list_ports, keyboard
from tkinter import messagebox

# Global serial variables for motor and relay
motor_ser = None
relay_ser = None
serial_lock = threading.Lock()
camera_running = False
current_speed = 75  # Default speed (range: 0-150)

# Global variable to track the first connection to the relay
first_relay_connection = True

# Command cooldown time
last_command_time = 0
command_cooldown = 0.002  # seconds

# Control settings
keyboard_control_enabled = False  # State of keyboard control mode
base_displacement = 75  # Default step size in degrees for all axes except 'r'
r_displacement = 0.25    # Default step size in degrees for 'r' axis

# Axis controls
axis_controls = {
    'w': ('X', '-'),          # X axis positive
    's': ('X', '+'),          # X axis negative
    'a': ('Y', '+'),          # Y axis positive
    'd': ('Y', '-'),          # Y axis negative
    'shift': ('Z', '+'),      # Z axis positive
    'ctrl': ('Z', '-'),       # Z axis negative (changed from 'caps lock' to 'ctrl')
    'e': ('r', '+'),          # r axis positive (rotary)
    'q': ('r', '-'),          # r axis negative (rotary)
    'z': ('t', '-'),          # T1 axis negative
    'x': ('t', '+'),          # T1 axis positive
    'r': ('T', '-'),          # T2 axis negative
    'f': ('T', '+')           # T2 axis positive
}

# Auto-detect serial port
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
    
def initialize_motor_controller(ser):
    """Connects to the motor controller and initializes it."""
    try:
        # Send the initialization command and check the response
        print("Initializing motor controller...")
        response = send_command(ser, "?R\r", "Motor Controller")  # Add the device_name argument
        
        if response == "?R":
            print("Motor controller initialized successfully.")
        else:
            print(f"Motor controller initialization failed. Response: {response}")
    except Exception as e:
        print(f"Error during motor controller initialization: {e}")

def send_command(ser, command, device_name):
    global last_command_time
    current_time = time.time()

    if current_time - last_command_time < command_cooldown:
        print(f"{device_name} command cooldown active. Skipping command.")
        return  # Skip sending command if in cooldown period

    try:
        with serial_lock:
            ser.write((command).encode())
            print(f"Sent command to {device_name}: {command.strip()}")
            last_command_time = current_time  # Update last command time
            time.sleep(0.01)  # Give the device time to respond

            # Read the device's response
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode().strip()
                print(f"{device_name} response: {response}")
                return response  # Return the response for further handling 
            else:
                print(f"No response from {device_name}.")
                return None
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()} to {device_name}: {e}")
        return None

def convert_degrees_to_pulses(degrees):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def µm_to_steps(µm, axis, screw_pitch=2.0, steps_per_rev=200):
    # Determine microstepping settings based on the axis
    microstepping = 1 if axis == 't' else 8  # t axis uses 1/1, others use 1/8

    # Calculate steps per micrometer
    steps_per_µm = (steps_per_rev * microstepping) / (screw_pitch * 1000)
    steps = round(µm * steps_per_µm)
    print(f"Converting {µm} µm on axis {axis} to {steps} steps (screw pitch={screw_pitch}, microstepping={microstepping}).")
    return steps

def move_linear_stage(axis, direction, displacement_µm):
    if motor_ser is None:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return

    print(f"Received axis: '{axis}', direction: '{direction}', displacement: {displacement_µm} µm")

    valid_axes = ['X', 'Y', 'Z', 'r', 't', 'T']
    if axis not in valid_axes or direction not in ['+', '-']:
        messagebox.showerror("Error", "Invalid axis or direction.")
        return

    # Convert displacement from µm to steps
    displacement_steps = µm_to_steps(displacement_µm, axis)

    # For rotary stage ('r'), ensure conversion to pulses if applicable
    if axis == 'r':
        displacement_steps = convert_degrees_to_pulses(displacement_µm)  # Optional if rotary uses angles

    command = f"{axis}{direction}{int(displacement_steps)}\r"
    send_command(motor_ser, command, "Motor Controller")

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
        send_command(motor_ser, "STOP", "Motor Controller")  # Assuming 'STOP' is a valid command
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")
    print("Motor control stopped.")

def continuous_motor_control():
    """Continuously check for key presses and control motor based on mapped keys if enabled."""
    global keyboard_control_enabled
    while True:
        if keyboard_control_enabled:
            for key, (axis, direction) in axis_controls.items():
                if keyboard.is_pressed(key):
                    # Determine step size, halve if space bar is pressed
                    step_size = r_displacement if axis == 'r' else base_displacement
                    if keyboard.is_pressed("space"):
                        step_size /= 2
                    move_linear_stage(axis, direction, step_size)
        time.sleep(0.1)  # Add a small delay to avoid excessive CPU usage

# Relay control functions
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
    """Toggle keyboard control for motor movement."""
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")

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

def launch_gui():
    """Create and launch the main GUI."""
    global motor_port_entry, relay_port_entry, speed_entry

    def connect():
        """Connect to the motor and relay devices."""
        global motor_ser, relay_ser

        motor_port = motor_port_entry.get().strip()
        relay_port = relay_port_entry.get().strip()

        motor_ser = connect_to_device(motor_port, "Motor Controller")
        relay_ser = connect_to_device(relay_port, "Arduino Controller")

        # Confirm connections
        if motor_ser and relay_ser:
            global camera_running
            camera_running = True  # Start the cameras when devices are connected

    def move_stage():
        """Move the motor stage with user inputs."""
        try:
            axis = axis_entry.get().strip()
            direction = direction_var.get()
            displacement = int(displacement_entry.get().strip())
            move_linear_stage(axis, direction, displacement)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def update_speed_gui():
        """Update the speed based on user input."""
        try:
            new_speed = int(speed_entry.get().strip())
            if 0 <= new_speed <= 150:
                update_speed(new_speed)
            else:
                messagebox.showerror("Error", "Speed must be between 0 and 150.")
        except ValueError:
            messagebox.showerror("Error", "Invalid speed value.")

    # Create main GUI window
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
    tk.Label(root, text="Displacement (in µm or degrees for 'r'):").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    # Buttons for Motor Control
    tk.Button(root, text="Move Stage", command=move_stage).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    # Keyboard Control Toggle
    keyboard_control_var = tk.IntVar(value=0)
    keyboard_control_checkbox = tk.Checkbutton(root, text="Keyboard Movement Mode", variable=keyboard_control_var, command=toggle_keyboard_control)
    keyboard_control_checkbox.pack(pady=5)

    # Buttons for Laser Relay Control
    tk.Button(root, text="Laser Relay On", command=laser_relay_on).pack(pady=5)
    tk.Button(root, text="Laser Relay Off", command=laser_relay_off).pack(pady=5)

     # Add "Run Full Manual Loop" button
    tk.Button(root, text="Run Full Manual Loop", command=run_full_manual_loop).pack(pady=10)

    root.mainloop()

def run_full_manual_loop():
    """
    Execute a manual loop of motor commands.
    """
    try:
        print("Starting Full Manual Loop...")
        move_linear_stage("Z", "+", 950)  
        time.sleep(5)
        laser_relay_on()
        time.sleep(5)
        move_linear_stage("T", "+", 20000)
        time.sleep(5)
        laser_relay_off()
        time.sleep(5)
        move_linear_stage("T", "-", 20000)
        time.sleep(5)
        move_linear_stage("Z", "-", 950)
        time.sleep(5)
        print("Full Manual Loop completed.")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during the manual loop: {e}")

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