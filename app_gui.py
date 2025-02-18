# app_gui.py

import threading
import tkinter as tk
import keyboard
import time
from tkinter import messagebox

# Imports from our modules
from motor_control import (
    auto_connect_motor,
    retrieve_motor_speed,
    get_current_speed,
    update_speed,
    query_all_axes_positions,
    go_to_all_origins,
    stop_motor_control,
    move_linear_stage,
    base_displacement,
    r_displacement
)
from relay_control import (
    auto_connect_relay,
    laser_relay_on,
    laser_relay_off
)
from image_recognition import open_camera  # Renamed file

speed_display_label = None
keyboard_control_enabled = False

# Axis controls for keyboard movement
axis_controls = {
    'w': ('X', '-'),
    's': ('X', '+'),
    'a': ('Y', '+'),
    'd': ('Y', '-'),
    'shift': ('Z', '+'),
    'ctrl': ('Z', '-'),
    'e': ('r', '+'),
    'q': ('r', '-'),
    'z': ('t', '-'),
    'x': ('t', '+'),
    'r': ('T', '-'),
    'f': ('T', '+')
}

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
                        # Quick moves
                        move_linear_stage(axis, direction, step_size, wait_for_stop=False)
            except Exception as e:
                print(f"Exception in keyboard control: {e}")
        time.sleep(0.01)

def toggle_keyboard_control():
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")

def run_full_manual_loop():
    """
    Demonstration: Turn laser on/off, move some axes, etc.
    """
    try:
        print("Starting Full Manual Loop...")

        # Move Z up
        move_linear_stage("Z", "+", 950, wait_for_stop=True, max_wait=30.0)
        time.sleep(1)

        # Turn laser ON
        laser_relay_on()
        time.sleep(0.25)

        # Move T forward
        move_linear_stage("T", "+", 20000, wait_for_stop=True, max_wait=30.0)
        time.sleep(3)

        # Turn laser OFF
        laser_relay_off()
        time.sleep(1)

        # Move T back
        move_linear_stage("T", "-", 20000, wait_for_stop=True, max_wait=30.0)
        time.sleep(3.5)

        # Move Z down
        move_linear_stage("Z", "-", 950, wait_for_stop=True, max_wait=30.0)
        time.sleep(1)

        print("Full Manual Loop completed.")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during the manual loop: {e}")
        print(f"Exception in run_full_manual_loop: {e}")

def launch_gui():
    global speed_display_label

    root = tk.Tk()
    root.title("Motor Control and Camera Feed")

    # 1) Auto-connect motor
    auto_connect_motor()
    # 2) Auto-connect relay
    auto_connect_relay()
    # 3) Retrieve motor speed
    retrieve_motor_speed()

    # Display the current motor speed
    speed_display_label = tk.Label(root, text=f"Current Speed: {get_current_speed()}")
    speed_display_label.pack(pady=5)

    # Frame for speed UI
    speed_frame = tk.Frame(root)
    speed_frame.pack(pady=10)

    tk.Label(speed_frame, text="Set Speed (0-150): ").pack(side='left')
    speed_entry = tk.Entry(speed_frame, width=5)
    speed_entry.insert(0, str(get_current_speed()))
    speed_entry.pack(side='left', padx=5)

    def update_speed_gui():
        """Update the speed based on user input, then refresh label."""
        try:
            new_speed = int(speed_entry.get().strip())
            if 0 <= new_speed <= 150:
                update_speed(new_speed)
                # Refresh the label
                speed_display_label.config(text=f"Current Speed: {get_current_speed()}")
            else:
                messagebox.showerror("Error", "Speed must be between 0 and 150.")
        except ValueError:
            messagebox.showerror("Error", "Invalid speed value.")

    tk.Button(root, text="Update Speed", command=update_speed_gui).pack(pady=5)

    # Axis and direction input
    tk.Label(root, text="Axis (X, Y, Z, r, t, T):").pack()
    axis_entry = tk.Entry(root)
    axis_entry.pack()

    direction_var = tk.StringVar(value='+')
    tk.Radiobutton(root, text='Positive (+)', variable=direction_var, value='+').pack()
    tk.Radiobutton(root, text='Negative (-)', variable=direction_var, value='-').pack()

    tk.Label(root, text="Displacement (µm for linear, degrees for 'r')").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    def move_stage_gui():
        """Single-axis move from the GUI, blocking until done."""
        try:
            axis = axis_entry.get().strip()
            direction = direction_var.get()
            displacement = float(displacement_entry.get().strip())
            move_linear_stage(axis, direction, displacement, wait_for_stop=True, max_wait=30.0)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    tk.Button(root, text="Move Stage", command=move_stage_gui).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    # Keyboard control toggle
    kb_var = tk.IntVar(value=0)
    tk.Checkbutton(root, text="Keyboard Movement Mode",
                   variable=kb_var, command=toggle_keyboard_control).pack(pady=5)

    # --- Laser section with side-by-side radio buttons ---
    laser_state = tk.StringVar(value='Off')

    def set_laser():
        if laser_state.get() == 'On':
            laser_relay_on()
        else:
            laser_relay_off()

    laser_frame = tk.Frame(root)
    laser_frame.pack(pady=5)

    tk.Label(laser_frame, text="Laser: ").pack(side='left')
    tk.Radiobutton(laser_frame, text="On",  variable=laser_state, value='On',  command=set_laser).pack(side='left')
    tk.Radiobutton(laser_frame, text="Off", variable=laser_state, value='Off', command=set_laser).pack(side='left')

    # Query & origin
    tk.Button(root, text="Query All Axes", command=query_all_axes_positions).pack(pady=5)
    tk.Button(root, text="Return to Origin", command=go_to_all_origins).pack(pady=5)

    # Place the "Run Full Manual Loop" button at the very bottom
    tk.Button(root, text="Run Full Manual Loop", command=run_full_manual_loop).pack(side='bottom', pady=15)

    root.mainloop()

def start_camera_threads():
    cam0_thread = threading.Thread(target=open_camera, args=(0,))
    cam1_thread = threading.Thread(target=open_camera, args=(1,))
    cam0_thread.start()
    cam1_thread.start()
    return cam0_thread, cam1_thread
