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
    base_displacement,
    r_displacement,
    keyboard_pause,
    flush_serial,
    emergency_stop_motors,
    clear_emergency_stop,
    is_emergency_stop_requested,
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
    nordson_off,
    pnp_forward,
    pnp_backward,
    pnp_release
)

from print import (
    glue_sequence,
    print_tester,
    calibrate,
    r_limit,
    r_corrector,
    Z_probe,
    get_coord,
    confirm_origin_set,
    notify_fine_tune_choice,
    register_origin_prompt_callback,
    register_origin_ask_callback,
    extruder_origin_setup,
)

from assembly import run_full_assembly

import image_recognition
from image_recognition import (
    open_camera,
    extrude,
    x_align,
    r_align
)

SETTINGS_FILE = "pcb_settings.json"

_camera_threads = {0: None, 1: None, 2: None}

PAD_COUNT = 0
PAD_SPACING = 0.0
FIRST_PAD_OFFSET = 0.0
speed_display_label = None
keyboard_control_enabled = False
routine_thread = None
laser_state_var = None
nord_state_var = None

axis_controls = {
    'w': ('Y', '-'),
    's': ('Y', '+'),
    'a': ('X', '+'),
    'd': ('X', '-'),
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
        "fixture_offset": 2000.0,
        "camera_ports": {"0": 0, "1": 1, "2": 2}
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
    # Read existing data so camera_ports (and other keys) are not lost
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing.update({
        "pad_count": pad_count,
        "pad_spacing": pad_spacing,
        "offset": offset,
        "fixture_offset": fixture_offset,
    })
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Saved PCB settings to {SETTINGS_FILE}")
    except Exception as e:
        print(f"Warning: Could not write to {SETTINGS_FILE}: {e}")

def save_camera_ports(ports_dict):
    """Persist camera port assignments to pcb_settings.json."""
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing["camera_ports"] = {str(k): v for k, v in ports_dict.items()}
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Saved camera ports to {SETTINGS_FILE}: {ports_dict}")
    except Exception as e:
        print(f"Warning: Could not save camera ports: {e}")

def continuous_motor_control():
    global keyboard_control_enabled
    while True:
        if keyboard_control_enabled and not keyboard_pause.is_set():
            try:
                for key, (axis, direction) in axis_controls.items():
                    if keyboard.is_pressed(key):
                        step_size = r_displacement if axis == 'r' else base_displacement
                        if keyboard.is_pressed("space"):
                            step_size /= 2
                        move_linear_stage(axis, direction, step_size, wait_for_stop=False)
            except Exception as e:
                print(f"Exception in keyboard control: {e}")
        time.sleep(0.02)

def toggle_keyboard_control():
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")
    if keyboard_control_enabled:
        update_speed(150)
        if speed_display_label:
            speed_display_label.config(text=f"Current Speed: {get_current_speed()}")
    else:
        # Flush stale keyboard-pulse bytes so the next GUI command gets a clean port.
        flush_serial()

def wait_for_extrude_done(poll_interval=0.1):
    """
    Blocks (sleeps) until image_recognition.extrude_done == True.
    poll_interval is how often (in seconds) we check the flag.
    """
    while not image_recognition.extrude_done:
        if is_emergency_stop_requested():
            print("Emergency stop requested during extrude wait.")
            return False
        time.sleep(poll_interval)
    return True

def wait_for_r_align_done(poll_interval=0.1):
    """
    Blocks until image_recognition.r_align_done == True.
    """
    while not image_recognition.r_align_done:
        if is_emergency_stop_requested():
            print("Emergency stop requested during rotational alignment wait.")
            return False
        time.sleep(poll_interval)
    return True

def wait_for_x_align_done(poll_interval=0.1):
    """
    Blocks until image_recognition.x_align_done == True.
    """
    while not image_recognition.x_align_done:
        if is_emergency_stop_requested():
            print("Emergency stop requested during X alignment wait.")
            return False
        time.sleep(poll_interval)
    return True

def _abort_if_emergency_stop():
    if is_emergency_stop_requested():
        raise RuntimeError("Emergency stop requested.")

def _sleep_with_abort(seconds, step=0.05):
    elapsed = 0.0
    while elapsed < seconds:
        _abort_if_emergency_stop()
        dt = min(step, seconds - elapsed)
        time.sleep(dt)
        elapsed += dt

def on_stop_motors():
    global laser_state_var, nord_state_var

    # Latch emergency immediately to abort active routines and stop motion.
    emergency_stop_motors()

    # Force outputs off (allowed during emergency by command whitelist).

    try:
        laser_relay_off()
        if laser_state_var is not None:
            laser_state_var.set('Off')
    except Exception as e:
        print(f"Warning: could not turn laser off during stop: {e}")

    try:
        nordson_off()
        if nord_state_var is not None:
            nord_state_var.set('Off')
    except Exception as e:
        print(f"Warning: could not turn Nordson off during stop: {e}")

def start_routine_thread(target, routine_name):
    global routine_thread

    if routine_thread and routine_thread.is_alive():
        print("Another routine is already running. Stop it before starting a new one.")
        return

    def runner():
        try:
            target()
        except Exception as e:
            print(f"Exception in {routine_name}: {e}")

    routine_thread = threading.Thread(target=runner, daemon=True)
    routine_thread.start()

# Test extrude and alignment for one pad only — no laser cut
def test_extrude_align():
    threading.Thread(target=_test_extrude_align_thread).start()

def _test_extrude_align_thread():
    clear_emergency_stop()
    extrude(1)
    if not wait_for_extrude_done():
        return
    r_align()
    if not wait_for_r_align_done():
        return
    print("Extrude and align test complete.")

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
        clear_emergency_stop()
        laser_relay_off()
        nordson_off()
        if laser_state_var is not None:
            laser_state_var.set('Off')
        if nord_state_var is not None:
            nord_state_var.set('Off')
        print("--- Starting Automated Routine ---")

        _abort_if_emergency_stop()
        motor_forward(20, timeout=30)
        _sleep_with_abort(2.0)
        _abort_if_emergency_stop()
        motor_backward(20, timeout=30)
        _sleep_with_abort(2.0)
        motor_release()

        # Navigate to (and fine-tune) the extruder/laser-alignment origin.
        # The confirmed position is saved to pcb_settings.json and reused next run.
        # set_origin_to_current() then locks it in so return_to_origin() throughout
        # the pad loop returns here every time.
        _abort_if_emergency_stop()
        extruder_origin_setup()
        set_origin_to_current()

        # Move everything to origin before we begin
        _abort_if_emergency_stop()
        return_to_origin()

        for pad_num in range(1, PAD_COUNT+1):
            _abort_if_emergency_stop()
            print(f"Automated Alignment on Pad #{pad_num}")
            set_origin_to_current
            update_speed(30)
            move_linear_stage("Z", "-", 1220, wait_for_stop=True, max_wait=30.0)
            _abort_if_emergency_stop()
            extrude(pad_num)
            if not wait_for_extrude_done():
                raise RuntimeError("Emergency stop requested.")
            _abort_if_emergency_stop()
            r_align()
            if not wait_for_r_align_done():
                raise RuntimeError("Emergency stop requested.")
            _abort_if_emergency_stop()
            x_align(pad_num)
            if not wait_for_x_align_done():
                raise RuntimeError("Emergency stop requested.")
            _abort_if_emergency_stop()
            update_speed(1)
            move_linear_stage("Z", "+", 1220, wait_for_stop=True, max_wait=30.0)
            print(f"Laser cutting on Pad #{pad_num}")
            _abort_if_emergency_stop()
            laser_cut()
            
            # Return to origin after finishing this pad
            _abort_if_emergency_stop()
            return_to_origin()

        print("--- Automated Routine Completed ---")

    except Exception as e:
        if str(e) == "Emergency stop requested.":
            print("Wire/laser automation stopped by emergency stop.")
        else:
            messagebox.showerror("Error", f"An error occurred during run_full_manual_loop: {e}")
            print(f"Exception in run_full_manual_loop: {e}")
    finally:
        try:
            laser_relay_off()
            nordson_off()
            if laser_state_var is not None:
                laser_state_var.set('Off')
            if nord_state_var is not None:
                nord_state_var.set('Off')
        except Exception as e:
            print(f"Warning: failed to force outputs off after run_full_manual_loop: {e}")

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
def open_camera_port_settings_window(root):
    """Popup to reassign which OS camera device port each logical camera role uses."""
    win = Toplevel(root)
    win.title("Camera Port Assignments")
    win.resizable(False, False)

    role_labels = {
        0: "Camera 0 — Main PCB View",
        1: "Camera 1 — Wire Tip View",
        2: "Camera 2 — Clog Detection",
    }

    port_vars = {}
    for role, label in role_labels.items():
        current_port = image_recognition.camera_ports.get(role, role)
        tk.Label(win, text=f"{label}:").grid(row=role, column=0, padx=10, pady=6, sticky='w')
        var = tk.StringVar(value=str(current_port))
        port_vars[role] = var
        spinbox = tk.Spinbox(win, from_=0, to=9, width=4, textvariable=var)
        spinbox.grid(row=role, column=1, padx=10, pady=6)

    tk.Label(
        win,
        text="Changes take effect on next program start.",
        fg="gray"
    ).grid(row=3, column=0, columnspan=2, padx=10, pady=(2, 8))

    def on_apply():
        try:
            new_ports = {role: int(port_vars[role].get()) for role in role_labels}
        except ValueError:
            messagebox.showerror("Error", "Port numbers must be integers.", parent=win)
            return
        image_recognition.camera_ports.update(new_ports)
        save_camera_ports(new_ports)
        win.destroy()
        if messagebox.askyesno(
            "Restart Cameras",
            "Port assignments saved.\nRestart cameras now to apply the new ports?"
        ):
            threading.Thread(target=restart_cameras, daemon=True).start()

    btn_frame = tk.Frame(win)
    btn_frame.grid(row=4, column=0, columnspan=2, pady=8)
    tk.Button(btn_frame, text="Apply & Save", command=on_apply).pack(side='left', padx=10)
    tk.Button(btn_frame, text="Cancel", command=win.destroy).pack(side='left', padx=10)

    win.grab_set()

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
            clear_emergency_stop()
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
                root.after(0, lambda: axis_entry.config(state='normal'))
                root.after(0, lambda: displacement_entry.config(state='normal'))
                root.after(0, lambda: move_stage_btn.config(state='normal'))
        axis_entry.config(state='disabled')
        displacement_entry.config(state='disabled')
        move_stage_btn.config(state='disabled')
        threading.Thread(target=move_thread).start()

    move_stage_btn = tk.Button(root, text="Move Stage", command=move_stage_gui)
    move_stage_btn.pack(pady=10)
    tk.Button(root, text="Stop Motors", command=on_stop_motors, bg="red", fg="black", font=(15)).pack(pady=10)

    # Keyboard control
    kb_var = tk.IntVar(value=0)
    tk.Checkbutton(
        root, text="Keyboard Movement Mode",
        variable=kb_var, command=toggle_keyboard_control
    ).pack(pady=5)

    # Laser radio
    global laser_state_var
    laser_state_var = tk.StringVar(value='Off')
    def set_laser():
        if laser_state_var.get() == 'On':
            laser_relay_on()
        else:
            laser_relay_off()

    laser_frame = tk.Frame(root)
    laser_frame.pack(pady=5)
    tk.Label(laser_frame, text="Laser: ").pack(side='left')
    tk.Radiobutton(laser_frame, text="On", variable=laser_state_var, value='On', command=set_laser).pack(side='left')
    tk.Radiobutton(laser_frame, text="Off", variable=laser_state_var, value='Off', command=set_laser).pack(side='left')

    # Solenoid radio Phill's edit
    global solenoid_state_var
    solenoid_state_var = tk.StringVar(value='Off')
    def set_solenoid():
        if solenoid_state_var.get() == 'On':
            solenoid_relay_on()
        else:
            solenoid_relay_off()

    solenoid_frame = tk.Frame(root)
    solenoid_frame.pack(pady=8)
    tk.Label(solenoid_frame, text="PNP Vacuum: ").pack(side='left')
    tk.Radiobutton(solenoid_frame, text="On", variable=solenoid_state_var, value='On', command=set_solenoid).pack(side='left')
    tk.Radiobutton(solenoid_frame, text="Off", variable=solenoid_state_var, value='Off', command=set_solenoid).pack(side='left')

    # Nordson radio
    global nord_state_var
    nord_state_var = tk.StringVar(value='Off')
    def set_nord():
        if nord_state_var.get() == 'On':
            nordson_on()
        else:
            nordson_off()

    nord_frame = tk.Frame(root)
    nord_frame.pack(pady=10)
    tk.Label(nord_frame, text="Pressurized Air: ").pack(side='left')
    tk.Radiobutton(nord_frame, text="On", variable=nord_state_var, value='On', command=set_nord).pack(side='left')
    tk.Radiobutton(nord_frame, text="Off", variable=nord_state_var, value='Off', command=set_nord).pack(side='left')

    # Query & origin
    def show_origin_ask(label):
        """Ask the user (on the main thread) whether to fine-tune this origin."""
        def _do_ask():
            result = messagebox.askyesno(
                f"{label} — Fine-tune?",
                f"The machine is now at the {label} position.\n\n"
                "Would you like to fine-tune it?\n\n"
                "  Yes  →  adjust with the movement controls,\n"
                "          then click \u2018Set Origin\u2019 to confirm\n"
                "  No   →  accept the current position and continue",
                icon='question'
            )
            notify_fine_tune_choice(result)
        root.after(0, _do_ask)

    def show_origin_prompt(label):
        root.after(0, lambda: messagebox.showinfo(
            f"{label} — Fine-tune",
            f"Fine-tune the stage position for: {label}\n\n"
            "Use the GUI movement controls to adjust, then click\n"
            "'Set Origin' to confirm and continue."
        ))

    register_origin_ask_callback(show_origin_ask)
    register_origin_prompt_callback(show_origin_prompt)

    def on_set_origin_clicked():
        set_origin_to_current()
        confirm_origin_set()

    tk.Button(root, text="Set Origin", command=on_set_origin_clicked).pack(pady=5)
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

    tk.Button(root, text="Camera Port Settings", command=lambda: open_camera_port_settings_window(root)).pack(pady=5)

    tk.Button(root, text="Query Position", command=query_all_axes_positions).pack(pady=5)

    # Button: Full Assembly
    tk.Button(root, text="Print Metal Ink Traces/Pads", command=lambda: start_routine_thread(run_full_assembly, "run_full_assembly")).pack(pady=5)

    # Full manual loop
    tk.Button(root, text="Start Wire/Laser Automation Routine", command=lambda: start_routine_thread(run_full_manual_loop, "run_full_manual_loop")).pack(side='bottom', pady=8)

    root.mainloop()

def start_camera_threads():
    """
    Launch camera0, camera1, and camera2 in separate daemon threads.
    Stores thread references so restart_cameras() can manage them.
    """
    global _camera_threads
    for i in range(3):
        t = threading.Thread(target=open_camera, args=(i,), daemon=True)
        t.start()
        _camera_threads[i] = t
    return _camera_threads[0], _camera_threads[1], _camera_threads[2]

def restart_cameras():
    """Signal all camera threads to stop cleanly, then restart with current camera_ports."""
    print("[Cameras] Stopping all camera threads...")
    for i in range(3):
        image_recognition.camera_stop_events[i].set()
    for i in range(3):
        t = _camera_threads.get(i)
        if t and t.is_alive():
            t.join(timeout=6.0)
            if t.is_alive():
                print(f"[Camera {i}] Warning: thread did not stop within timeout.")
    for i in range(3):
        image_recognition.camera_stop_events[i].clear()
    print("[Cameras] Restarting with updated port assignments...")
    start_camera_threads()