# app_gui.py

import threading
import tkinter as tk
import keyboard
import time
from tkinter import messagebox, Toplevel
import json
import os

from motor_control import (
    auto_connect_motor,
    retrieve_motor_speed,
    get_current_speed,
    update_speed,
    query_all_axes_positions,
    return_to_origin,
    stop_motor_control,
    move_linear_stage,
    set_origin_to_current,
    
)

from relay_control import (
    auto_connect_relay,
    motor_forward, 
    motor_backward,
    motor_release,
    laser_relay_on,
    laser_relay_off,
    solenoid_relay_on,
    solenoid_relay_off,
    nordson_on,
    nordson_off
)

import image_recognition
from image_recognition import (
    open_camera, 
    extrude, 
    x_align, 
    r_align)

# ################################
# # GLOBAL VARIABLES & SETTINGS edit:11/05/2025
# def launch_gui():
#     global axis_entry, displacement_entry
# ################################

SETTINGS_FILE = "pcb_settings.json"

PAD_COUNT = 0
PAD_SPACING = 0.0
FIRST_PAD_OFFSET = 0.0
speed_display_label = None
keyboard_control_enabled = False

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
    defaults = {
        "pad_count": 8,
        "pad_spacing": 1000.0,
        "offset": 1100.0,
        "fixture_offset": 2000.0
    }
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
            for key in defaults:
                data.setdefault(key, defaults[key])
            return data
        except Exception as e:
            print(f"Warning: Could not read {SETTINGS_FILE}: {e}")
    return defaults

def save_settings(pad_count, pad_spacing, offset, fixture_offset):
    data = {
        "pad_count": pad_count,
        "pad_spacing": pad_spacing,
        "offset": offset,
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

def wait_for_extrude_done(poll_interval=0.1):
    """
    Blocks (sleeps) until image_recognition.extrude_done == True.
    poll_interval is how often (in seconds) we check the flag.
    """
    while not image_recognition.extrude_done:
        time.sleep(poll_interval)

def wait_for_r_align_done(poll_interval=0.1):
    """
    Blocks until image_recognition.r_align_done == True.
    """
    while not image_recognition.r_align_done:
        time.sleep(poll_interval)

def wait_for_x_align_done(poll_interval=0.1):
    """
    Blocks until image_recognition.x_align_done == True.
    """
    while not image_recognition.x_align_done:
        time.sleep(poll_interval)

def laser_cut():
    try:
        print("--- Starting Laser Cutting Sequence ---")
        update_speed(1)
        move_linear_stage("Z", "+", 2400, wait_for_stop=True, max_wait=30.0)
        update_speed(30)
        move_linear_stage("T", "+", 22700, wait_for_stop=True, max_wait=30.0)
        laser_relay_on()
        update_speed(1)
        move_linear_stage("T", "+", 1800, wait_for_stop=True, max_wait=30.0)
        laser_relay_off()
        update_speed(30)
        move_linear_stage("T", "-", 40000, wait_for_stop=True, max_wait=30.0)
        move_linear_stage("Z", "-", 2400, wait_for_stop=True, max_wait=30.0)
        print("Laser cutting sequence completed.")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during laser_cut: {e}")
        print(f"Exception in laser_cut: {e}")

def run_full_manual_loop():
    global PAD_COUNT, PAD_SPACING, FIRST_PAD_OFFSET

    try:
        print("--- Starting Automated Routine ---")

        motor_forward(20, timeout=30)
        time.sleep(2)  # Wait for motor to move
        motor_backward(20, timeout=30)
        time.sleep(2)  # Wait for motor to move
        motor_release()
        
        # Move everything to origin before we begin
        """return_to_origin()

        for pad_num in range(1, PAD_COUNT+1):
            print(f"Automated Alignment on Pad #{pad_num}")
            set_origin_to_current
            update_speed(30)
            move_linear_stage("Z", "-", 1220, wait_for_stop=True, max_wait=30.0)
            extrude(pad_num)
            wait_for_extrude_done()
            r_align()
            wait_for_r_align_done()
            x_align(pad_num)
            wait_for_x_align_done()
            update_speed(1)
            move_linear_stage("Z", "+", 1220, wait_for_stop=True, max_wait=30.0)
            print(f"Laser cutting on Pad #{pad_num}")
            laser_cut()
            
            # Return to origin after finishing this pad
            return_to_origin()"""

        print("--- Automated Routine Completed ---")

    except Exception as e:
        messagebox.showerror("Error", f"An error occurred during run_full_manual_loop: {e}")
        print(f"Exception in run_full_manual_loop: {e}")

def ask_pcb_info_popup(root, defaults):
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

###############################
# BOUNDING BOX & RECORDING
###############################
def toggle_bounding_boxes():
    val = box_var.get()  # 'On' or 'Off'
    if val == 'On':
        image_recognition.draw_bounding_boxes = True
        print("[GUI] Bounding Boxes => ON")
    else:
        image_recognition.draw_bounding_boxes = False
        print("[GUI] Bounding Boxes => OFF")

def toggle_recording():
    val = record_var.get()  # 'On' or 'Off'
    if val == 'On':
        image_recognition.record_camera0 = True
        image_recognition.record_camera1 = True
        image_recognition.record_camera2 = True  
        print("[GUI] Recording => ON for all cameras")
    else:
        image_recognition.record_camera0 = False
        image_recognition.record_camera1 = False
        image_recognition.record_camera2 = False  
        print("[GUI] Recording => OFF for all cameras")

###############################
# IMAGE ADJUSTMENT SLIDER POPUP
###############################
def open_image_adjustment_window():
    adj_win = Toplevel()
    adj_win.title("Image Adjustments")

    tk.Label(adj_win, text="Contrast (ALPHA)").pack()
    alpha_scale = tk.Scale(
        adj_win, from_=0.1, to=3.0, resolution=0.1,
        orient='horizontal', command=lambda val: set_alpha(val)
    )
    alpha_scale.set(image_recognition.ALPHA)
    alpha_scale.pack()

    tk.Label(adj_win, text="Brightness (BETA)").pack()
    beta_scale = tk.Scale(
        adj_win, from_=-100, to=100, resolution=1,
        orient='horizontal', command=lambda val: set_beta(val)
    )
    beta_scale.set(image_recognition.BETA)
    beta_scale.pack()

    tk.Label(adj_win, text="Saturation Factor").pack()
    sat_scale = tk.Scale(
        adj_win, from_=0.0, to=2.0, resolution=0.1,
        orient='horizontal', command=lambda val: set_saturation(val)
    )
    sat_scale.set(image_recognition.SAT_FACTOR)
    sat_scale.pack()

    tk.Label(adj_win, text="Gamma").pack()
    gamma_scale = tk.Scale(
        adj_win, from_=0.1, to=2.5, resolution=0.1,
        orient='horizontal', command=lambda val: set_gamma(val)
    )
    gamma_scale.set(image_recognition.GAMMA)
    gamma_scale.pack()

    tk.Label(adj_win, text="Sharpness").pack()
    sharp_scale = tk.Scale(
        adj_win, from_=0.0, to=2.0, resolution=0.1,
        orient='horizontal', command=lambda val: set_sharpness(val)
    )
    sharp_scale.set(image_recognition.SHARP_STRENGTH)
    sharp_scale.pack()

    tk.Button(adj_win, text="Close", command=adj_win.destroy).pack(pady=5)

def set_alpha(val):
    image_recognition.ALPHA = float(val)

def set_beta(val):
    image_recognition.BETA = float(val)

def set_saturation(val):
    image_recognition.SAT_FACTOR = float(val)

def set_gamma(val):
    image_recognition.GAMMA = float(val)

def set_sharpness(val):
    image_recognition.SHARP_STRENGTH = float(val)

###############################
# MAIN GUI LAUNCH
###############################
def launch_gui():
    global PAD_COUNT, PAD_SPACING, FIRST_PAD_OFFSET

    root = tk.Tk()
    root.title("Motor & Camera Feed Control")

    # Hide main window, ask user for pad info
    root.withdraw()
    last_vals = load_last_settings()
    pc, ps, user_off = ask_pcb_info_popup(root, last_vals)

    fixture_off = last_vals["fixture_offset"]
    final_offset = user_off + fixture_off

    PAD_COUNT = pc
    PAD_SPACING = ps
    FIRST_PAD_OFFSET = final_offset

    if pc > 0:
        save_settings(pc, ps, user_off, fixture_off)

    # Show main window
    root.deiconify()

    # Connect motor & relay
    auto_connect_motor()
    auto_connect_relay()
    
    retrieve_motor_speed()

    info_label = tk.Label(
        root,
        text=(f"Pads: {PAD_COUNT}, Spacing: {PAD_SPACING} µm, Offset: {FIRST_PAD_OFFSET} µm")
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
        #New code to fix the stop issue  edit:11/05/2025
        import threading
        try:
            axis = axis_entry.get().strip()
            
            raw = displacement_entry.get().strip()
            value = float(raw)

            direction = '+' if value >= 0 else '-'
            displacement = abs(value)
        except ValueError:
            messagebox.showerror("Error","Invalid displacement entry")
            return
        def move_thread():

            try:
                move_linear_stage(axis, direction, displacement, wait_for_stop=True, max_wait=30.0)
            except Exception as e:
                messagebox.showerror("Error", f"Movement error: {e}")
            finally:
                axis_entry.config(state='normal')
                displacement_entry.config(state='normal')
        axis_entry.config(state='disabled')
        displacement_entry.config(state='disabled')
        threading.Thread(target=move_thread).start()
      
        #---------------------------
        # ORIGINAL CODE COMMENTED OUT FOR TESTING PURPOSES  edit:11/05/2025
        # try:
        #     axis = axis_entry.get().strip()
        #     displacement_value = float(displacement_entry.get().strip())
        #     direction = '+' if displacement_value >= 0 else '-'
        #     magnitude = abs(displacement_value)
        #     move_linear_stage(axis, direction, magnitude, wait_for_stop=True, max_wait=30.0)
        # except ValueError as e:
        #     messagebox.showerror("Error", f"Invalid displacement: {e}")

    tk.Button(root, text="Move Stage", command=move_stage_gui).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    # Keyboard control
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

    # Solenoid radio Phill's edit
    solenoid_state = tk.StringVar(value='Off')
    def set_solenoid():
        if solenoid_state.get() == 'On':
            solenoid_relay_on()
        else:
            solenoid_relay_off()

    solenoid_frame = tk.Frame(root)
    solenoid_frame.pack(pady=8)
    tk.Label(solenoid_frame, text="Solenoid: ").pack(side='left')
    tk.Radiobutton(solenoid_frame, text="On", variable=solenoid_state, value='On', command=set_solenoid).pack(side='left')
    tk.Radiobutton(solenoid_frame, text="Off", variable=solenoid_state, value='Off', command=set_solenoid).pack(side='left')

    # Nordson radio
    nord_state = tk.StringVar(value='Off')
    def set_nord():
        if nord_state.get() == 'On':
            nordson_on()
        else:
            nordson_off()

    nord_frame = tk.Frame(root)
    nord_frame.pack(pady=10)
    tk.Label(nord_frame, text="Nordson: ").pack(side='left')
    tk.Radiobutton(nord_frame, text="On", variable=nord_state, value='On', command=set_nord).pack(side='left')
    tk.Radiobutton(nord_frame, text="Off", variable=nord_state, value='Off', command=set_nord).pack(side='left')

    # Query & origin
    tk.Button(root, text="Set Origin", command=set_origin_to_current).pack(pady=5)
    tk.Button(root, text="Return to Origin", command=return_to_origin).pack(pady=5)

    # BOUNDING BOX radio
    global box_var
    box_var = tk.StringVar(value='On')  # default => bounding boxes off
    box_frame = tk.Frame(root)
    box_frame.pack(pady=5)
    tk.Label(box_frame, text="Bounding Boxes: ").pack(side='left')
    tk.Radiobutton(box_frame, text="On", variable=box_var, value='On', command=toggle_bounding_boxes).pack(side='left')
    tk.Radiobutton(box_frame, text="Off", variable=box_var, value='Off', command=toggle_bounding_boxes).pack(side='left')

    # RECORDING radio
    global record_var
    record_var = tk.StringVar(value='Off')  # default => not recording
    record_frame = tk.Frame(root)
    record_frame.pack(pady=5)
    tk.Label(record_frame, text="Recording: ").pack(side='left')
    tk.Radiobutton(record_frame, text="On", variable=record_var, value='On', command=toggle_recording).pack(side='left')
    tk.Radiobutton(record_frame, text="Off", variable=record_var, value='Off', command=toggle_recording).pack(side='left')

    # Add a button to manually launch the Image Adjustments
    tk.Button(root, text="Open Image Adjustments", command=open_image_adjustment_window).pack(pady=5)

    # Full manual loop
    tk.Button(root, text="Start Automation Routine", command=run_full_manual_loop).pack(side='bottom', pady=15)

    from pcb_mapping import print_trace_pattern
    from pcb_mapping import test_diagonal
    # Button: Print Trace Pattern
    tk.Button(root, text="Print Trace Pattern", command=print_trace_pattern).pack(pady=10)

    # Button: Test Diagonal Move
    tk.Button(root, text="Test Diagonal Move", command=test_diagonal).pack(pady=10)

    root.mainloop()

def start_camera_threads():
    """
    Launch camera0, camera1, and camera2 in separate threads. 
    They run open_camera(...) from image_recognition.
    """
    cam0_thread = threading.Thread(target=open_camera, args=(0,))
    cam1_thread = threading.Thread(target=open_camera, args=(1,))
    cam2_thread = threading.Thread(target=open_camera, args=(2,))  
    cam0_thread.start()
    cam1_thread.start()
    cam2_thread.start()  
    return cam0_thread, cam1_thread, cam2_thread
