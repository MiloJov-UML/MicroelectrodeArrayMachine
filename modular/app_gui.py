# app_gui.py

import threading
import tkinter as tk
import keyboard
import time
from tkinter import messagebox

# Import your motor and relay functions
from motor_control import (
    auto_connect_motor,
    retrieve_motor_speed,
    current_speed,
    update_speed,
    move_linear_stage,
    stop_motor_control,
    query_all_axes_positions,
    go_to_all_origins,
    run_full_manual_loop
)
from relay_control import laser_relay_on, laser_relay_off
from image_recognition import open_camera

# Global for GUI label
speed_display_label = None

# Keyboard control state
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
    from motor_control import base_displacement, r_displacement
    while True:
        if keyboard_control_enabled:
            try:
                for key, (axis, direction) in axis_controls.items():
                    if keyboard.is_pressed(key):
                        step_size = r_displacement if axis == 'r' else base_displacement
                        if keyboard.is_pressed("space"):
                            step_size /= 2
                        move_linear_stage(axis, direction, step_size, wait_for_stop=False)
            except Exception as e:
                print(f"Exception in keyboard control: {e}")
        time.sleep(0.01)

def toggle_keyboard_control():
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    print(f"Keyboard motor control: {'enabled' if keyboard_control_enabled else 'disabled'}")

def launch_gui():
    global speed_display_label

    root = tk.Tk()
    root.title("Motor Control and Camera Feed")

    # Auto-connect
    auto_connect_motor()
    # Retrieve motor speed
    retrieve_motor_speed()

    # Speed label
    speed_display_label = tk.Label(root, text=f"Current Speed: {current_speed}")
    speed_display_label.pack(pady=5)

    speed_frame = tk.Frame(root)
    speed_frame.pack(pady=10)

    tk.Label(speed_frame, text="Set Speed (0-150): ").pack(side='left')
    speed_entry = tk.Entry(speed_frame, width=5)
    speed_entry.insert(0, str(current_speed))
    speed_entry.pack(side='left', padx=5)

    def update_speed_gui():
        try:
            new_speed = int(speed_entry.get().strip())
            if 0 <= new_speed <= 150:
                update_speed(new_speed)
                speed_display_label.config(text=f"Current Speed: {current_speed}")
            else:
                messagebox.showerror("Error", "Speed must be between 0 and 150.")
        except ValueError:
            messagebox.showerror("Error", "Invalid speed value.")

    tk.Button(root, text="Update Speed", command=update_speed_gui).pack(pady=5)

    # Axis & displacement
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
    kb_checkbox = tk.Checkbutton(root, text="Keyboard Movement Mode", 
                                 variable=kb_var, command=toggle_keyboard_control)
    kb_checkbox.pack(pady=5)

    # Relay buttons
    tk.Button(root, text="Laser Relay On", command=laser_relay_on).pack(pady=5)
    tk.Button(root, text="Laser Relay Off", command=laser_relay_off).pack(pady=5)

    # Full manual loop
    tk.Button(root, text="Run Full Manual Loop", command=run_full_manual_loop).pack(pady=10)

    # Query & go to origin
    tk.Button(root, text="Query All Axes", command=query_all_axes_positions).pack(pady=5)
    tk.Button(root, text="Go to All Origins", command=go_to_all_origins).pack(pady=5)

    root.mainloop()

def start_camera_threads():
    camera_0_thread = threading.Thread(target=open_camera, args=(0,))
    camera_1_thread = threading.Thread(target=open_camera, args=(1,))
    camera_0_thread.start()
    camera_1_thread.start()
    return camera_0_thread, camera_1_thread
