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
    move_linear_stage
)
from relay_control import (
    auto_connect_relay,
    laser_relay_on,
    laser_relay_off
)
from image_recognition import open_camera

# Globals to store user inputs
PAD_COUNT = 0            # how many pads
PAD_SPACING = 0.0        # distance between pad centers
FIRST_PAD_OFFSET = 0.0   # edge-of-pcb offset plus 700 µm fixture offset

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
                        # Example step size
                        step_size = 50
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
    Not shown as a button; called from run_full_manual_loop().
    """
    try:
        print("--- Starting Laser Cutting Sequence ---")
        move_linear_stage("Z", "+", 950, wait_for_stop=True, max_wait=30.0)
        laser_relay_on()
        move_linear_stage("T", "+", 20000, wait_for_stop=True, max_wait=30.0)
        laser_relay_off()
        move_linear_stage("T", "-", 20000, wait_for_stop=True, max_wait=30.0)
        move_linear_stage("Z", "-", 950, wait_for_stop=True, max_wait=30.0)
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

def ask_pcb_info_popup(root):
    """
    Single Toplevel to get:
      - PAD_COUNT
      - PAD_SPACING
      - OFFSET from right bottom corner (without fixture)
    We'll add 700 µm after user input.
    """
    popup = tk.Toplevel(root)
    popup.title("PCB Setup")

    pad_count_var = tk.StringVar(value="8")     # default to 8 if you like
    pad_spacing_var = tk.StringVar(value="1000")   # default
    offset_var = tk.StringVar(value="5000")    # default

    tk.Label(popup, text="How many pads?").pack(pady=5)
    e1 = tk.Entry(popup, textvariable=pad_count_var)
    e1.pack()

    tk.Label(popup, text="Distance between pad centers (µm):").pack(pady=5)
    e2 = tk.Entry(popup, textvariable=pad_spacing_var)
    e2.pack()

    tk.Label(popup, text="Distance from bottom-right corner to 1st pad (µm):").pack(pady=5)
    e3 = tk.Entry(popup, textvariable=offset_var)
    e3.pack()

    result = {"pc": 0, "ps": 0.0, "off": 0.0, "submitted": False}

    def on_ok():
        try:
            pc = int(pad_count_var.get())
            ps = float(pad_spacing_var.get())
            off = float(offset_var.get())
            result["pc"] = pc
            result["ps"] = ps
            result["off"] = off
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
        return (result["pc"], result["ps"], result["off"])
    else:
        return (0, 0.0, 0.0)

def launch_gui():
    global PAD_COUNT, PAD_SPACING, FIRST_PAD_OFFSET

    root = tk.Tk()
    root.title("Motor Control and Camera Feed")

    # Hide main window initially
    root.withdraw()

    # Ask user for pad info
    pc, ps, off = ask_pcb_info_popup(root)

    # Add 700µm fixture offset
    off += 700.0

    PAD_COUNT = pc
    PAD_SPACING = ps
    FIRST_PAD_OFFSET = off

    root.deiconify()  # now show main window

    # Connect motor/relay
    auto_connect_motor()
    auto_connect_relay()
    retrieve_motor_speed()

    info_label = tk.Label(
        root, 
        text=f"Pads: {PAD_COUNT}, Spacing: {PAD_SPACING}µm, Offset: {FIRST_PAD_OFFSET}µm (+700 fixture)"
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
