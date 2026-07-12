# app_gui.py

import threading
import queue
import tkinter as tk
import keyboard
import time
from tkinter import messagebox, Toplevel
import json
import os

# Thread-safe channel for work that MUST run on the Tk GUI thread.
# Worker/routine threads put a callable here; _poll_gui_requests (scheduled via
# root.after and therefore running on the GUI thread) drains and executes it.
# This avoids calling Tk directly from worker threads, which raises
# "main thread is not in main loop" and silently skips origin popups.
_gui_request_queue = queue.Queue()

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
    get_current_position,
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
    pnp_release,
    servo_to,
    wait_for_magnet,
)

from assembly import (
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
    microwire_origin_setup,
    reload_origins,
    full_sequence,
    reprint_feature,
    jog_to_feature_start,
    jog_to_trace_start,
    jog_to_connector_pad,
)

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
        "camera_ports": {"0": 0, "1": 1, "2": 2},
        "trace_tuning": {
            "connector_cl_drop_mm": 2.5,
            "corner_mm": 0.5,
            "cs_clear_mm": 0.6,
        },
    }
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
            for key in defaults:
                data.setdefault(key, defaults[key])
            for key in defaults["trace_tuning"]:
                data["trace_tuning"].setdefault(key, defaults["trace_tuning"][key])
            return data
        except Exception as e:
            print(f"Warning: Could not read {SETTINGS_FILE}: {e}")
    return defaults

def save_settings(pad_count, pad_spacing):
    # Read existing data so camera_ports, trace_tuning (and other keys) are not lost
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
    })
    # Remove obsolete keys
    existing.pop("offset", None)
    existing.pop("fixture_offset", None)
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Saved PCB settings to {SETTINGS_FILE}")
    except Exception as e:
        print(f"Warning: Could not write to {SETTINGS_FILE}: {e}")

def save_trace_tuning(trace_tuning):
    """Persist trace routing tuning values to pcb_settings.json."""
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing["trace_tuning"] = trace_tuning
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Saved trace tuning to {SETTINGS_FILE}: {trace_tuning}")
    except Exception as e:
        print(f"Warning: Could not save trace tuning: {e}")

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

def run_full_assembly(run_calibration=True):
    try:
        full_sequence(run_calibration=run_calibration)
    except RuntimeError as e:
        if str(e) == "Emergency stop requested.":
            print("Metal ink routine stopped by emergency stop.")
        else:
            raise

def run_full_manual_loop():
    global PAD_COUNT, PAD_SPACING

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

        # Navigate to (and fine-tune) the microwire / laser-alignment origin.
        # The confirmed position is saved to pcb_settings.json and reused next run.
        # set_origin_to_current() then locks it in so return_to_origin() throughout
        # the pad loop returns here every time.
        _abort_if_emergency_stop()
        microwire_origin_setup()
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
    pad_spacing_var = tk.StringVar(value=str(defaults.get("pad_spacing", 1000.0)))

    tk.Label(popup, text="How many pads?").pack(pady=5)
    e1 = tk.Entry(popup, textvariable=pad_count_var)
    e1.pack()

    tk.Label(popup, text="Distance between pad centers (µm):").pack(pady=5)
    e2 = tk.Entry(popup, textvariable=pad_spacing_var)
    e2.pack()

    tk.Label(
        popup,
        text="Trace routing tuning is under Settings › Trace Routing Tuning.",
        fg="gray"
    ).pack(pady=(10, 2))

    result = {"pc": 0, "ps": 0.0, "submitted": False}

    def on_ok():
        try:
            pc = int(pad_count_var.get())
            ps = float(pad_spacing_var.get())
            result["pc"] = pc
            result["ps"] = ps
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
        return (result["pc"], result["ps"])
    else:
        return (0, 0.0)

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

def open_image_adjustment_window(parent=None):
    adj_win = Toplevel(parent)
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
# NAMED ORIGINS (probe / print / microwire)
# JSON keys and axes match print.py so both systems share the same saved values.
###############################
_ORIGIN_CONFIG = {
    'probe':    ('probe_origin',        ['X', 'Y', 'r', 'Z']),
    'print':    ('print_origin_coords', ['X', 'Y', 'r', 'Z']),
    'microwire': ('microwire_origin',     ['X', 'Y', 'Z', 'r']),
}

def save_named_origin(name):
    """Read current axis positions and persist them to the print.py-compatible JSON key."""
    cfg = _ORIGIN_CONFIG.get(name)
    if not cfg:
        print(f"[Origin] Unknown origin name '{name}'")
        return
    json_key, axes = cfg
    positions = {}
    for ax in axes:
        pos = get_current_position(ax)
        if pos is not None:
            positions[ax] = pos
    if not positions:
        print(f"[Origin] Could not read any axis positions — '{name}' origin not saved.")
        return
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing[json_key] = positions
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"[Origin] Saved '{name}' origin ({json_key}): { {k: f'{v:.3f}' for k, v in positions.items()} }")
    except Exception as e:
        print(f"Warning: Could not save named origin '{name}': {e}")
        return
    # Keep print.py's in-memory state in sync so the current session sees the update.
    reload_origins()

def load_named_origin(name):
    """Return the saved axis-position dict for 'name', or None if not found."""
    cfg = _ORIGIN_CONFIG.get(name)
    if not cfg:
        return None
    json_key, _ = cfg
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
            return data.get(json_key)
    except Exception as e:
        print(f"Warning: Could not load named origin '{name}': {e}")
    return None

def _return_to_named_origin_thread(name):
    """Move axes to the stored named origin. Checks emergency stop between each axis."""
    cfg = _ORIGIN_CONFIG.get(name)
    if not cfg:
        print(f"[Origin] Unknown origin name '{name}'")
        return
    _, axes = cfg
    positions = load_named_origin(name)
    if not positions:
        print(f"[Origin] No saved origin for '{name}'. Use 'Set Origin' first.")
        return
    print(f"\n--- Moving to '{name}' origin ---")
    prev_speed = get_current_speed()
    update_speed(30)
    try:
        for ax in axes:
            if is_emergency_stop_requested():
                print(f"[Origin] Emergency stop — aborting return to '{name}' origin.")
                return
            if ax not in positions:
                continue
            current_pos = get_current_position(ax)
            if current_pos is None:
                print(f"[Origin] Axis {ax}: position unknown, skipping.")
                continue
            diff = positions[ax] - current_pos
            if abs(diff) < 0.5:
                continue
            direction = '+' if diff >= 0 else '-'
            print(f"[Origin] Moving {ax} -> {positions[ax]:.3f}")
            move_linear_stage(ax, direction, abs(diff), wait_for_stop=True, max_wait=30.0)
    finally:
        update_speed(prev_speed)
    print(f"--- Finished moving to '{name}' origin ---\n")

def open_trace_tuning_window(root):
    """Popup to edit trace routing tuning values (persisted to pcb_settings.json)."""
    win = Toplevel(root)
    win.title("Trace Routing Tuning")
    win.resizable(False, False)

    tuning = load_last_settings().get("trace_tuning", {})

    fields = [
        ("connector_cl_drop_mm", "ME row → connector row trace run (mm):", 2.5),
        ("corner_mm", "45° corner length (mm):", 0.5),
        ("cs_clear_mm", "Crossbar clearance above cs pads (mm):", 0.6),
    ]

    vars_by_key = {}
    for row, (key, label, default) in enumerate(fields):
        tk.Label(win, text=label).grid(row=row, column=0, padx=10, pady=6, sticky='w')
        var = tk.StringVar(value=str(tuning.get(key, default)))
        vars_by_key[key] = var
        tk.Entry(win, textvariable=var, width=8).grid(row=row, column=1, padx=10, pady=6)

    tk.Label(
        win,
        text="Applies to the next PCB print.",
        fg="gray"
    ).grid(row=len(fields), column=0, columnspan=2, padx=10, pady=(2, 8))

    def on_apply():
        try:
            new_tuning = {key: float(var.get()) for key, var in vars_by_key.items()}
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values.", parent=win)
            return
        save_trace_tuning(new_tuning)
        reload_origins()  # refresh trace tuning in assembly.py
        win.destroy()

    btn_frame = tk.Frame(win)
    btn_frame.grid(row=len(fields) + 1, column=0, columnspan=2, pady=8)
    tk.Button(btn_frame, text="Apply & Save", command=on_apply).pack(side='left', padx=10)
    tk.Button(btn_frame, text="Cancel", command=win.destroy).pack(side='left', padx=10)

    win.grab_set()

def open_settings_window(root):
    """Settings hub popup — opens Image Adjustments or Camera Port Settings."""
    win = Toplevel(root)
    win.title("Settings")
    win.resizable(False, False)

    tk.Label(win, text="Settings", font=("Helvetica", 13, "bold")).pack(pady=(14, 6))

    tk.Button(
        win,
        text="Image Adjustments",
        width=24,
        command=lambda: open_image_adjustment_window(win)
    ).pack(pady=6)

    tk.Button(
        win,
        text="Camera Port Settings",
        width=24,
        command=lambda: open_camera_port_settings_window(root)
    ).pack(pady=6)

    tk.Button(
        win,
        text="Trace Routing Tuning",
        width=24,
        command=lambda: open_trace_tuning_window(root)
    ).pack(pady=6)

    tk.Button(win, text="Close", command=win.destroy).pack(pady=(6, 14))


def open_reprint_window(root):
    """Popup to reprint a single feature (any pad or trace) at the current
    printhead position."""
    win = Toplevel(root)
    win.title("Reprint Feature")
    win.resizable(False, False)

    pad_count = PAD_COUNT if PAD_COUNT and PAD_COUNT > 0 else 8

    component_labels = [
        ("Full feature (ME pad → trace → connector pad)", "full"),
        ("ME pad only", "me_pad"),
        ("Trace only", "trace"),
        ("Trace + connector pad", "trace_and_connector"),
        ("Connector pad only", "connector_pad"),
    ]
    component_var = tk.StringVar(value="full")
    index_var = tk.StringVar(value="1")

    tk.Label(win, text="What to reprint:", font=("Helvetica", 10, "bold")).pack(
        anchor='w', padx=12, pady=(12, 2))
    for label, value in component_labels:
        tk.Radiobutton(win, text=label, variable=component_var, value=value).pack(
            anchor='w', padx=16)

    idx_frame = tk.Frame(win)
    idx_frame.pack(anchor='w', padx=12, pady=(8, 2))
    tk.Label(idx_frame, text="Feature number:").pack(side='left')
    tk.Spinbox(idx_frame, from_=1, to=pad_count, width=4, textvariable=index_var).pack(
        side='left', padx=8)

    tk.Label(
        win,
        text=("Jog to the feature start to auto-position (Z touches down last),\n"
              "or position the head manually. Printing begins at the current position."),
        fg="gray",
        justify='left'
    ).pack(anchor='w', padx=12, pady=(6, 8))

    def _get_index():
        try:
            index = int(index_var.get())
        except ValueError:
            messagebox.showerror("Error", "Feature number must be an integer.", parent=win)
            return None
        if not (1 <= index <= 8):
            messagebox.showerror("Error", "Feature number must be between 1 and 8.", parent=win)
            return None
        return index

    def on_jog():
        index = _get_index()
        if index is None:
            return
        # Route jog to the appropriate start point for the selected component:
        #   trace / trace_and_connector -> trace start (where trace leaves ME pad)
        #   connector_pad               -> connector pad entry point
        #   everything else             -> feature (ME-pad) start
        component = component_var.get()
        if component in ('trace', 'trace_and_connector'):
            start_routine_thread(
                lambda: jog_to_trace_start(index),
                "jog_to_trace_start"
            )
        elif component == 'connector_pad':
            start_routine_thread(
                lambda: jog_to_connector_pad(index),
                "jog_to_connector_pad"
            )
        else:
            start_routine_thread(
                lambda: jog_to_feature_start(index),
                "jog_to_feature_start"
            )

    def on_reprint():
        index = _get_index()
        if index is None:
            return
        component = component_var.get()
        win.destroy()
        start_routine_thread(
            lambda: reprint_feature(index, component),
            "reprint_feature"
        )

    btn_frame = tk.Frame(win)
    btn_frame.pack(pady=(2, 12))
    tk.Button(btn_frame, text="Jog to Start", command=on_jog).pack(side='left', padx=10)
    tk.Button(btn_frame, text="Reprint", command=on_reprint).pack(side='left', padx=10)
    tk.Button(btn_frame, text="Cancel", command=win.destroy).pack(side='left', padx=10)

    win.grab_set()


###############################
# PNP SUBSYSTEM TEST POPUP
###############################
def open_pnp_test_window(root):
    """Non-modal popup for discrete testing of each PNP subsystem."""
    win = Toplevel(root)
    win.title("PNP Subsystem Test")
    win.resizable(False, False)

    IDLE_BG = 'lightgray'
    BUSY_BG = '#ffffaa'   # yellow  — in progress / listening
    OK_BG   = '#aaffaa'   # green   — confirmed / detected
    ERR_BG  = '#ffaaaa'   # red     — error

    # ── Servo ─────────────────────────────────────────────────────────────────
    servo_lf = tk.LabelFrame(win, text="Servo", padx=10, pady=6)
    servo_lf.pack(fill='x', padx=12, pady=(10, 4))

    servo_status = tk.Label(servo_lf, text="Last command: —", width=26,
                             relief='sunken', bg=IDLE_BG, anchor='center')
    servo_status.pack(pady=(0, 6))

    def _servo_go(angle):
        servo_status.config(text=f"Sent: {angle}°", bg=OK_BG)
        threading.Thread(target=servo_to, args=(angle,), daemon=True).start()

    preset_row = tk.Frame(servo_lf)
    preset_row.pack()
    tk.Label(preset_row, text="Presets:").pack(side='left', padx=(0, 6))
    for _a in (0, 55, 71):
        tk.Button(preset_row, text=f"{_a}°", width=5,
                  command=lambda a=_a: _servo_go(a)).pack(side='left', padx=3)

    custom_row = tk.Frame(servo_lf)
    custom_row.pack(pady=(5, 0))
    tk.Label(custom_row, text="Custom (0–270):").pack(side='left')
    custom_angle_entry = tk.Entry(custom_row, width=5)
    custom_angle_entry.insert(0, "0")
    custom_angle_entry.pack(side='left', padx=4)

    def _servo_custom():
        try:
            angle = int(custom_angle_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Enter an integer angle.", parent=win)
            return
        if not (0 <= angle <= 270):
            messagebox.showerror("Error", "Angle must be 0–270.", parent=win)
            return
        _servo_go(angle)

    tk.Button(custom_row, text="Go", command=_servo_custom).pack(side='left')

    # ── DC Motor (PNP) ────────────────────────────────────────────────────────
    motor_lf = tk.LabelFrame(win, text="DC Motor (PNP)", padx=10, pady=6)
    motor_lf.pack(fill='x', padx=12, pady=4)

    motor_status = tk.Label(motor_lf, text="State: Idle", width=26,
                             relief='sunken', bg=IDLE_BG, anchor='center')
    motor_status.pack(pady=(0, 6))

    speed_row = tk.Frame(motor_lf)
    speed_row.pack()
    tk.Label(speed_row, text="Speed (1–255):").pack(side='left')
    speed_var = tk.StringVar(value="25")
    tk.Entry(speed_row, textvariable=speed_var, width=5).pack(side='left', padx=4)

    def _get_pnp_speed():
        try:
            s = int(speed_var.get())
            if 1 <= s <= 255:
                return s
        except ValueError:
            pass
        messagebox.showerror("Error", "Speed must be an integer 1–255.", parent=win)
        return None

    def _pnp_fwd():
        s = _get_pnp_speed()
        if s is None:
            return
        motor_status.config(text="State: Forward", bg='#aaddff')
        threading.Thread(target=pnp_forward, kwargs={'speed': s}, daemon=True).start()

    def _pnp_bwd():
        s = _get_pnp_speed()
        if s is None:
            return
        motor_status.config(text="State: Backward", bg=BUSY_BG)
        threading.Thread(target=pnp_backward, kwargs={'speed': s}, daemon=True).start()

    def _pnp_rel():
        motor_status.config(text="State: Released", bg=IDLE_BG)
        threading.Thread(target=pnp_release, daemon=True).start()

    motor_btn_row = tk.Frame(motor_lf)
    motor_btn_row.pack(pady=(5, 0))
    tk.Button(motor_btn_row, text="Forward",  width=10, command=_pnp_fwd).pack(side='left', padx=4)
    tk.Button(motor_btn_row, text="Backward", width=10, command=_pnp_bwd).pack(side='left', padx=4)
    tk.Button(motor_btn_row, text="Release",  width=10, command=_pnp_rel).pack(side='left', padx=4)

    # ── Hall Effect Sensor ────────────────────────────────────────────────────
    hall_lf = tk.LabelFrame(win, text="Hall Effect Sensor", padx=10, pady=6)
    hall_lf.pack(fill='x', padx=12, pady=4)

    hall_status = tk.Label(hall_lf, text="State: Idle", width=26,
                            relief='sunken', bg=IDLE_BG, anchor='center')
    hall_status.pack(pady=(0, 6))

    _hall_cancel = threading.Event()

    def _start_hall_listen():
        _hall_cancel.clear()
        hall_status.config(text="Listening for magnet...", bg=BUSY_BG)
        listen_btn.config(state='disabled')
        cancel_hall_btn.config(state='normal')

        def _listen_thread():
            result = wait_for_magnet(cancel_event=_hall_cancel)
            def _update():
                try:
                    if not win.winfo_exists():
                        return
                    if result == "Magnet Detected":
                        hall_status.config(text="Magnet Detected!", bg=OK_BG)
                    else:
                        hall_status.config(text="State: Idle (cancelled)", bg=IDLE_BG)
                    listen_btn.config(state='normal')
                    cancel_hall_btn.config(state='disabled')
                except Exception:
                    pass
            win.after(0, _update)

        threading.Thread(target=_listen_thread, daemon=True).start()

    def _cancel_hall_listen():
        _hall_cancel.set()

    hall_btn_row = tk.Frame(hall_lf)
    hall_btn_row.pack()
    listen_btn = tk.Button(hall_btn_row, text="Listen for Magnet", width=16,
                           command=_start_hall_listen)
    listen_btn.pack(side='left', padx=4)
    cancel_hall_btn = tk.Button(hall_btn_row, text="Cancel", width=8,
                                command=_cancel_hall_listen, state='disabled')
    cancel_hall_btn.pack(side='left', padx=4)

    # ── Full Sequence ─────────────────────────────────────────────────────────
    seq_lf = tk.LabelFrame(win, text="Full Sequence (print_tester)", padx=10, pady=6)
    seq_lf.pack(fill='x', padx=12, pady=(4, 8))

    seq_status = tk.Label(seq_lf, text="State: Idle", width=26,
                           relief='sunken', bg=IDLE_BG, anchor='center')
    seq_status.pack(pady=(0, 6))

    def _run_full_seq():
        seq_status.config(text="Running...", bg=BUSY_BG)
        run_seq_btn.config(state='disabled')

        def _seq_thread():
            try:
                print_tester()
                def _done():
                    try:
                        if win.winfo_exists():
                            seq_status.config(text="Completed", bg=OK_BG)
                            run_seq_btn.config(state='normal')
                    except Exception:
                        pass
                win.after(0, _done)
            except Exception as e:
                def _err():
                    try:
                        if win.winfo_exists():
                            seq_status.config(text=f"Error: {e}", bg=ERR_BG)
                            run_seq_btn.config(state='normal')
                    except Exception:
                        pass
                win.after(0, _err)

        threading.Thread(target=_seq_thread, daemon=True).start()

    run_seq_btn = tk.Button(seq_lf, text="Run Full Sequence", width=22, command=_run_full_seq)
    run_seq_btn.pack()

    # Cancel any listening if the window is closed
    win.protocol("WM_DELETE_WINDOW", lambda: (_hall_cancel.set(), win.destroy()))

    tk.Button(win, text="Close", command=lambda: (_hall_cancel.set(), win.destroy())).pack(pady=(0, 10))


###############################
# MAIN GUI LAUNCH
###############################
def launch_gui():
    global PAD_COUNT, PAD_SPACING

    root = tk.Tk()
    root.title("Motor & Camera Feed Control")

    # Hide main window, ask user for pad info
    root.withdraw()
    last_vals = load_last_settings()
    pc, ps = ask_pcb_info_popup(root, last_vals)

    PAD_COUNT = pc
    PAD_SPACING = ps

    if pc > 0:
        save_settings(pc, ps)
        reload_origins()  # also refreshes trace tuning in print.py

    # Show main window
    root.deiconify()

    # Connect motor & relay
    auto_connect_motor()
    auto_connect_relay()

    retrieve_motor_speed()

    info_label = tk.Label(
        root,
        text=(f"Pads: {PAD_COUNT}, Spacing: {PAD_SPACING} µm")
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

    stop_frame = tk.Frame(root)
    stop_frame.pack(pady=10)

    release_btn = tk.Button(
        stop_frame, text="Release Stop",
        bg="orange", fg="black", font=(15),
        state='disabled'
    )

    def on_stop_clicked():
        on_stop_motors()
        release_btn.config(state='normal')

    def on_release_clicked():
        clear_emergency_stop()
        release_btn.config(state='disabled')
        print("[GUI] Emergency stop released — system ready.")

    release_btn.config(command=on_release_clicked)

    tk.Button(
        stop_frame, text="Stop Motors",
        command=on_stop_clicked, bg="red", fg="black", font=(15)
    ).pack(side='left', padx=5)
    release_btn.pack(side='left', padx=5)

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
        """Ask the user whether to fine-tune this origin.
        Called from the routine worker thread, so the actual dialog is queued
        to run on the GUI thread via _poll_gui_requests()."""
        def _do_ask():
            root.lift()
            result = messagebox.askyesno(
                f"{label} — Fine-tune?",
                f"The machine is now at the {label} position.\n\n"
                "Would you like to fine-tune it?\n\n"
                "  Yes  →  adjust with the movement controls,\n"
                "          then click \u2018Set Origin\u2019 to confirm\n"
                "  No   →  accept the current position and continue",
                icon='question',
                parent=root,
            )
            notify_fine_tune_choice(result)
        _gui_request_queue.put(_do_ask)

    def show_origin_prompt(label):
        def _do_prompt():
            root.lift()
            messagebox.showinfo(
                f"{label} — Fine-tune",
                f"Fine-tune the stage position for: {label}\n\n"
                "Use the GUI movement controls to adjust, then click\n"
                "'Set Origin' to confirm and continue.",
                parent=root,
            )
        _gui_request_queue.put(_do_prompt)

    register_origin_ask_callback(show_origin_ask)
    register_origin_prompt_callback(show_origin_prompt)

    # Named-origin controls
    origin_mode_var = tk.StringVar(value='probe')

    def on_set_origin_clicked():
        mode = origin_mode_var.get()
        set_origin_to_current()       # keeps in-memory axis_origins in sync for automated routines
        save_named_origin(mode)        # persists to JSON and reloads print.py in-memory state
        confirm_origin_set()           # unblocks any waiting setup flow

    def on_return_to_origin_clicked():
        mode = origin_mode_var.get()
        threading.Thread(
            target=_return_to_named_origin_thread,
            args=(mode,),
            daemon=True
        ).start()

    origin_btn_frame = tk.Frame(root)
    origin_btn_frame.pack(pady=4)
    tk.Button(origin_btn_frame, text="Set Origin", command=on_set_origin_clicked).pack(side='left', padx=6)
    tk.Button(origin_btn_frame, text="Return to Origin", command=on_return_to_origin_clicked).pack(side='left', padx=6)

    origin_radio_frame = tk.Frame(root)
    origin_radio_frame.pack(pady=(0, 4))
    tk.Label(origin_radio_frame, text="Origin:").pack(side='left', padx=(0, 6))
    for _mode in ('Probe', 'Print', 'Microwire'):
        tk.Radiobutton(
            origin_radio_frame,
            text=_mode,
            variable=origin_mode_var,
            value=_mode.lower()
        ).pack(side='left')

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

    # Settings hub (image adjustments + camera ports)
    tk.Button(root, text="Settings", command=lambda: open_settings_window(root)).pack(pady=5)

    tk.Button(root, text="Query Position", command=query_all_axes_positions).pack(pady=5)

    # Button: Full Assembly
    calib_var = tk.IntVar(value=0)
    tk.Checkbutton(
        root,
        text="Run Calibration Before Printing",
        variable=calib_var
    ).pack(pady=(5, 0))
    tk.Button(
        root,
        text="Print Metal Ink Traces/Pads",
        command=lambda: start_routine_thread(
            lambda: run_full_assembly(run_calibration=bool(calib_var.get())),
            "run_full_assembly"
        )
    ).pack(pady=5)

    # Reprint a single feature (any pad or trace)
    tk.Button(
        root,
        text="Reprint Feature",
        command=lambda: open_reprint_window(root)
    ).pack(pady=5)

    # Test PNP routine
    tk.Button(
        root,
        text="Test PNP Routine",
        command=lambda: open_pnp_test_window(root)
    ).pack(pady=5)

    # Full manual loop
    tk.Button(root, text="Start Wire/Laser Automation Routine", command=lambda: start_routine_thread(run_full_manual_loop, "run_full_manual_loop")).pack(side='bottom', pady=8)

    # Drain GUI requests queued by worker threads (origin popups, etc.) on the
    # GUI thread. Reschedules itself so it runs for the lifetime of the window.
    def _poll_gui_requests():
        try:
            while True:
                job = _gui_request_queue.get_nowait()
                try:
                    job()
                except Exception as e:
                    print(f"Warning: GUI request failed: {e}")
        except queue.Empty:
            pass
        root.after(100, _poll_gui_requests)

    root.after(100, _poll_gui_requests)

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