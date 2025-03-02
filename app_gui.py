# app_gui.py

import threading
import tkinter as tk
import keyboard
import time
from tkinter import messagebox
import json
import os

# Imports from our modules
from motor_control import (
    auto_connect_motor,
    retrieve_motor_speed,
    get_current_speed,
    update_speed,
    query_all_axes_positions,
    go_to_all_origins,
    stop_motor_control,
    move_linear_stage
)
from relay_control import (
    auto_connect_relay,
    laser_relay_on,
    laser_relay_off
)
from image_recognition import open_camera

# Path to JSON settings file
SETTINGS_FILE = "pcb_settings.json"

# Globals to store user inputs
PAD_COUNT = 0
PAD_SPACING = 0.0
FIRST_PAD_OFFSET = 0.0   # sum of user input + fixture_offset from JSON

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

def load_last_settings():
    """
    Load pcb settings from SETTINGS_FILE if exists, else return defaults.
    Return dict with:
      pad_count (int),
      pad_spacing (float),
      offset (float),
      fixture_offset (float)  # Now we store fixture offset in JSON too
    """
    defaults = {
        "pad_count": 8,
        "pad_spacing": 500.0,
        "offset": 550.0,
        "fixture_offset": 700.0  # default fixture offset
    }
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
            # Merge with defaults, so if fixture_offset is missing, we use default 700
            for key in defaults:
                data.setdefault(key, defaults[key])
            return data
        except Exception as e:
            print(f"Warning: Could not read {SETTINGS_FILE}: {e}")
    return defaults

def save_settings(pad_count, pad_spacing, offset, fixture_offset):
    """
    Save the user's pad settings to SETTINGS_FILE in JSON format.
    'offset' is the user-specified offset (NOT including the fixture offset).
    'fixture_offset' is stored so advanced users can tweak it too.
    """
    data = {
        "pad_count": pad_count,
        "pad_spacing": pad_spacing,
        "offset": offset,            # raw user offset
        "fixture_offset": fixture_offset
    }
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved PCB settings to {SETTINGS_FILE}")
    except Exception as e:
        print(f"Warning: Could not write to {SETTINGS_FILE}: {e}")

def continuous_motor_control():
    global keyboard_control_enabled
    while True:
        if keyboard_control_enabled:
            try:
                for key, (axis, direction) in axis_controls.items():
                    if keyboard.is_pressed(key):
                        step_size = 1
                        move_linear_stage(axis, direction, step_size, wait_for_stop=False)
            except Exception as e:
                print(f"Exception in keyboard control: {e}")
        time.sleep(0.01)

def toggle_keyboard_control():
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")

def laser_cut():
    """
    Internal function to handle a laser cutting sequence.
    """
    try:
        print("--- Starting Laser Cutting Sequence ---")
        move_linear_stage("Z", "+", 1900, wait_for_stop=True, max_wait=30.0)
        laser_relay_on()
        move_linear_stage("T", "+", 40000, wait_for_stop=True, max_wait=30.0)
        laser_relay_off()
        move_linear_stage("T", "-", 40000, wait_for_stop=True, max_wait=30.0)
        move_linear_stage("Z", "-", 1900, wait_for_stop=True, max_wait=30.0)
        print("Laser cutting sequence completed.")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during laser_cut: {e}")
        print(f"Exception in laser_cut: {e}")

def run_full_manual_loop():
    """
    1) Go to all origins
    2) Move X negatively by FIRST_PAD_OFFSET => laser_cut => pad #1
    3) For pads #2..PAD_COUNT => X- PAD_SPACING => laser_cut
    4) Go to all origins
    """
    global PAD_COUNT, PAD_SPACING, FIRST_PAD_OFFSET
    try:
        print("--- Starting Full Manual Loop ---")
        go_to_all_origins()

        if PAD_COUNT <= 0:
            print("No pads specified. Exiting loop.")
            return

        # Pad #1: Move X negative by FIRST_PAD_OFFSET
        print(f"Moving to Pad #1 offset: {FIRST_PAD_OFFSET} µm (neg direction)")
        move_linear_stage("X", "-", FIRST_PAD_OFFSET, wait_for_stop=True, max_wait=30.0)
        print("Laser cutting on Pad #1")
        laser_cut()

        # Pad #2..PAD_COUNT
        for pad_index in range(2, PAD_COUNT + 1):
            print(f"Moving to Pad #{pad_index} offset: {PAD_SPACING} µm (neg direction)")
            move_linear_stage("X", "-", PAD_SPACING, wait_for_stop=True, max_wait=30.0)
            print(f"Laser cutting on Pad #{pad_index}")
            laser_cut()

        print("--- Full Manual Loop completed ---")
        go_to_all_origins()
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during run_full_manual_loop: {e}")
        print(f"Exception in run_full_manual_loop: {e}")

def ask_pcb_info_popup(root, defaults):
    """
    Single Toplevel to get:
      - pad_count
      - pad_spacing
      - user_offset (the distance from right bottom corner, WITHOUT fixture offset).
    We'll later add fixture_offset from the JSON to get the final FIRST_PAD_OFFSET.
    'defaults' is a dict with keys: 'pad_count', 'pad_spacing', 'offset', 'fixture_offset'
    but we only show user the first three.
    """
    popup = tk.Toplevel(root)
    popup.title("PCB Setup")

    pad_count_var = tk.StringVar(value=str(defaults.get("pad_count", 8)))
    pad_spacing_var = tk.StringVar(value=str(defaults.get("pad_spacing", 500.0)))
    user_offset_var = tk.StringVar(value=str(defaults.get("offset", 550.0)))

    tk.Label(popup, text="How many pads?").pack(pady=5)
    e1 = tk.Entry(popup, textvariable=pad_count_var)
    e1.pack()

    tk.Label(popup, text="Distance between pad centers (µm):").pack(pady=5)
    e2 = tk.Entry(popup, textvariable=pad_spacing_var)
    e2.pack()

    tk.Label(popup, text="Distance from bottom-right corner to 1st pad (µm):").pack(pady=5)
    e3 = tk.Entry(popup, textvariable=user_offset_var)
    e3.pack()

    # We won't show fixture_offset in GUI, but we'll read it from defaults if we need it.

    result = {"pc": 0, "ps": 0.0, "user_off": 0.0, "submitted": False}

    def on_ok():
        try:
            pc = int(pad_count_var.get())
            ps = float(pad_spacing_var.get())
            off = float(user_offset_var.get())
            result["pc"] = pc
            result["ps"] = ps
            result["user_off"] = off
            result["submitted"] = True
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values.")
            return
        popup.destroy()

    def on_cancel():
        popup.destroy()

    tk.Button(popup, text="OK", command=on_ok).pack(side='left', padx=20, pady=10)
    tk.Button(popup, text="Cancel", command=on_cancel).pack(side='right', padx=20, pady=10)

    popup.grab_set()
    root.wait_window(popup)

    if result["submitted"]:
        return (result["pc"], result["ps"], result["user_off"])
    else:
        return (0, 0.0, 0.0)

def launch_gui():
    global PAD_COUNT, PAD_SPACING, FIRST_PAD_OFFSET

    root = tk.Tk()
    root.title("Motor Control and Camera Feed")

    # Hide main window initially
    root.withdraw()

    # 1) Load last-known settings (including fixture_offset)
    last_vals = load_last_settings()
    # last_vals keys: 'pad_count', 'pad_spacing', 'offset', 'fixture_offset'

    # 2) Ask user for the three main values
    pc, ps, user_off = ask_pcb_info_popup(root, last_vals)

    # Merge user offset with fixture offset from JSON
    fixture_off = last_vals["fixture_offset"]
    final_offset = user_off + fixture_off

    PAD_COUNT = pc
    PAD_SPACING = ps
    FIRST_PAD_OFFSET = final_offset

    # 3) Save the user input so next time it’s remembered
    # We store user’s raw offset (NOT final_offset), and also keep fixture_offset as is
    if pc > 0:
        save_settings(pc, ps, user_off, fixture_off)

    # Show main window now
    root.deiconify()

    # Connect motor/relay
    auto_connect_motor()
    auto_connect_relay()
    retrieve_motor_speed()

    info_label = tk.Label(
        root,
        text=(
            f"Pads: {PAD_COUNT}, Pad Spacing: {PAD_SPACING}µm, "
            f"1st Pad Offset: {FIRST_PAD_OFFSET}µm"
        )
    )
    info_label.pack(pady=5)

    global speed_display_label
    speed_display_label = tk.Label(root, text=f"Current Speed: {get_current_speed()}")
    speed_display_label.pack(pady=5)

    speed_frame = tk.Frame(root)
    speed_frame.pack(pady=10)

    tk.Label(speed_frame, text="Set Speed (0-150): ").pack(side='left')
    speed_entry = tk.Entry(speed_frame, width=5)
    speed_entry.insert(0, str(get_current_speed()))
    speed_entry.pack(side='left', padx=5)

    def update_speed_gui():
        try:
            new_speed = int(speed_entry.get().strip())
            if 0 <= new_speed <= 150:
                update_speed(new_speed)
                speed_display_label.config(text=f"Current Speed: {get_current_speed()}")
            else:
                messagebox.showerror("Error", "Speed must be between 0 and 150.")
        except ValueError:
            messagebox.showerror("Error", "Invalid speed value.")

    tk.Button(root, text="Update Speed", command=update_speed_gui).pack(pady=5)

    # Axis + Displacement
    tk.Label(root, text="Axis (X, Y, Z, r, t, T):").pack()
    axis_entry = tk.Entry(root)
    axis_entry.pack()

    tk.Label(root, text="Signed Displacement (e.g., 1000 or -1000):").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    def move_stage_gui():
        try:
            axis = axis_entry.get().strip()
            displacement_value = float(displacement_entry.get().strip())
            direction = '+' if displacement_value >= 0 else '-'
            magnitude = abs(displacement_value)
            move_linear_stage(axis, direction, magnitude, wait_for_stop=True, max_wait=30.0)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid displacement: {e}")

    tk.Button(root, text="Move Stage", command=move_stage_gui).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    # Keyboard control toggle
    kb_var = tk.IntVar(value=0)
    tk.Checkbutton(
        root, text="Keyboard Movement Mode",
        variable=kb_var, command=toggle_keyboard_control
    ).pack(pady=5)

    # Laser radio
    laser_state = tk.StringVar(value='Off')

    def set_laser():
        if laser_state.get() == 'On':
            laser_relay_on()
        else:
            laser_relay_off()

    laser_frame = tk.Frame(root)
    laser_frame.pack(pady=5)

    tk.Label(laser_frame, text="Laser: ").pack(side='left')
    tk.Radiobutton(laser_frame, text="On", variable=laser_state, value='On', command=set_laser).pack(side='left')
    tk.Radiobutton(laser_frame, text="Off", variable=laser_state, value='Off', command=set_laser).pack(side='left')

    # Query & origin
    tk.Button(root, text="Query All Axes", command=query_all_axes_positions).pack(pady=5)
    tk.Button(root, text="Return to Origin", command=go_to_all_origins).pack(pady=5)

    # We do NOT show a Laser Cut button, but keep the function in code.

    tk.Button(root, text="Run Full Manual Loop", command=run_full_manual_loop).pack(side='bottom', pady=15)

    root.mainloop()


def start_camera_threads():
    cam0_thread = threading.Thread(target=open_camera, args=(0,))
    cam1_thread = threading.Thread(target=open_camera, args=(1,))
    cam0_thread.start()
    cam1_thread.start()
    return cam0_thread, cam1_thread
