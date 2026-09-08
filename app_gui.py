# app_gui.py

import threading
import queue
import math
import tkinter as tk
import keyboard
import time
from tkinter import messagebox, Toplevel, filedialog
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
    steps_to_µm,
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
    save_connector_print_offset,
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
    center_on_visible_area,
    x_align,
    r_align,
    compute_angle_between,
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
rec_cam_vars = {0: None, 1: None, 2: None}

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

def save_pnp_offsets(offsets_dict):
    """Persist PNP-to-nozzle offsets to pcb_settings.json."""
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing["pnp_offsets"] = offsets_dict
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Saved PNP offsets to {SETTINGS_FILE}: {offsets_dict}")
    except Exception as e:
        print(f"Warning: Could not save PNP offsets: {e}")

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

def wait_for_center_on_visible_area_done(poll_interval=0.1):
    """
    Blocks until image_recognition.center_on_visible_area_done == True.
    """
    while not image_recognition.center_on_visible_area_done:
        if is_emergency_stop_requested():
            print("Emergency stop requested during visible-area centering wait.")
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

        PAD1_SEARCH_STEPS = 20    # how many small X steps to search for pad
        PAD1_SEARCH_UM = 1500     # µm per search step

        # ── Per-loop helpers defined once, before the pad loop ────────────
        # def _find_pad(pad_key):
        #     """Inch X until pad_key appears in YOLO detections."""
        #     if image_recognition.pad_box_dict.get(pad_key) is not None:
        #         return
        #     print(f"[Loop] {pad_key} not in view — searching X...")
        #     update_speed(1)
        #     for _ in range(PAD1_SEARCH_STEPS):
        #         _abort_if_emergency_stop()
        #         time.sleep(0.5)
        #         if image_recognition.pad_box_dict.get(pad_key) is not None:
        #             print(f"[Loop] {pad_key} found.")
        #             return
        #         move_linear_stage("X", "-", PAD1_SEARCH_UM,
        #                           wait_for_stop=True, max_wait=10.0)
        #     print(f"[Loop] Warning: {pad_key} still not visible after search.")

        # def _apply_t_retraction(theta_before_deg, theta_after_deg, pad_n):
        #     """Retract/extend t to compensate for CF_Tip reach change after r_align.
        #     When the wire rotates by δθ the horizontal projection changes by
        #     L*(cos(θ_after) - cos(θ_before)); correct only when |δθ| > 1°."""
        #     if abs(theta_before_deg - theta_after_deg) <= 1.0:
        #         return
        #     cf_box = image_recognition.last_cf_box
        #     gc_box = image_recognition.last_gc_box
        #     if cf_box is None or gc_box is None:
        #         print("[Correction] Missing CF/GC boxes — skipping length correction.")
        #         return
        #     (cx_cf, cy_cf) = image_recognition.center_of_bbox(cf_box)
        #     (cx_gc, cy_gc) = image_recognition.center_of_bbox(gc_box)
        #     wire_px = math.hypot(cx_cf - cx_gc, cy_cf - cy_gc)
        #     n1 = min(pad_n, 7)
        #     c1 = image_recognition.pad_box_dict.get(f"pad{n1}")
        #     c2 = image_recognition.pad_box_dict.get(f"pad{n1 + 1}")
        #     if c1 is None or c2 is None:
        #         print("[Correction] Calibration pads not visible — skipping length correction.")
        #         return
        #     known = image_recognition.get_pad_spacing()
        #     steps_pp = image_recognition.compute_steps_per_pixel(
        #         c1, c2, axis='t', known_µm=known)
        #     if steps_pp <= 0:
        #         return
        #     th_before = math.radians(theta_before_deg)
        #     th_after  = math.radians(theta_after_deg)
        #     delta_px  = wire_px * (math.cos(th_after) - math.cos(th_before))
        #     if abs(delta_px) < 0.1:
        #         return
        #     direction  = '-' if delta_px > 0 else '+'
        #     retract_µm = steps_to_µm(abs(delta_px) * steps_pp, axis='t')
        #     print(f"[Correction] θ {theta_before_deg:.2f}°→{theta_after_deg:.2f}°  "
        #           f"wire={wire_px:.1f}px  t {'retract' if direction=='-' else 'extend'} "
        #           f"{retract_µm:.1f}µm")
        #     move_linear_stage('t', direction, retract_µm,
        #                        wait_for_stop=True, max_wait=10.0)

        for pad_num in range(1, PAD_COUNT+1):
            _abort_if_emergency_stop()
            print(f"--- Pad #{pad_num} ---")
            update_speed(3)
            move_linear_stage("Z", "-", 1220, wait_for_stop=True, max_wait=30.0)
            # Clear stale YOLO detections so alignment steps see the current camera view
            image_recognition.pad_box_dict.clear()
            image_recognition.last_cf_box = None
            image_recognition.last_gc_box = None
            time.sleep(1.5)

            # ── Initial extrude + r_align ─────────────────────────────────
            _abort_if_emergency_stop()
            extrude(pad_num)
            if not wait_for_extrude_done():
                raise RuntimeError("Emergency stop requested.")
            # Record the target pad's Y-pixel position while it's fully in view,
            # before r_align rotates it; r_align uses this to compute a calibrated
            # Y correction afterward (not a motor-controller readout comparison).
            _ref_box = image_recognition.pad_box_dict.get(f"pad{pad_num}")
            image_recognition.target_pad_ref_y = (
                image_recognition.center_of_bbox(_ref_box)[1] if _ref_box is not None else None)
            _abort_if_emergency_stop()
            r_align(target_pad_number=pad_num)
            if not wait_for_r_align_done():
                raise RuntimeError("Emergency stop requested.")

            # ── Center on visible area, then re-check Y-axis alignment ─────
            _abort_if_emergency_stop()
            center_on_visible_area(target_pad_number=pad_num)
            if not wait_for_center_on_visible_area_done():
                raise RuntimeError("Emergency stop requested.")

            # ── x_align ───────────────────────────────────────────────────
            _abort_if_emergency_stop()
            x_align(pad_num)
            if not wait_for_x_align_done():
                raise RuntimeError("Emergency stop requested.")

            _abort_if_emergency_stop()
            update_speed(10)
            move_linear_stage("Z", "+", 1720, wait_for_stop=True, max_wait=30.0)
            print(f"Laser cutting on Pad #{pad_num}")
            _abort_if_emergency_stop()
            laser_cut()

            # Return to origin after finishing this pad
            _abort_if_emergency_stop()
            # Use the microwire named-origin navigation (same as the manual
            # 'Return to Origin' button) so we truly return to the saved origin.
            _return_to_named_origin_thread('microwire')
            time.sleep(1.0)  # settle before next pad

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

# Absolute T-axis positions (µm) bounding the laser cut — the cut is run
# back-and-forth _LASER_CUT_PASSES times to ensure a clean cut over the full length.
_LASER_CUT_T_START = 2614212.500
_LASER_CUT_T_END = 2614912.500
_LASER_CUT_PASSES = 5

def _move_t_to_absolute(target_um):
    """Move the T axis to an absolute position (µm) using a relative displacement."""
    current = get_current_position("T")
    if current is None:
        print(f"[laser_cut] Warning: could not read T position; skipping move to {target_um}.")
        return
    diff = target_um - current
    if abs(diff) < 0.5:
        return
    direction = '+' if diff >= 0 else '-'
    move_linear_stage("T", direction, abs(diff), wait_for_stop=True, max_wait=30.0)

def laser_cut():
    try:
        print("--- Starting Laser Cutting Sequence ---")
        _abort_if_emergency_stop()
        update_speed(1)
        _abort_if_emergency_stop()
        move_linear_stage("Z", "+", 1800, wait_for_stop=True, max_wait=30.0)
        _abort_if_emergency_stop()
        #update_speed(30)
        #_abort_if_emergency_stop()
        _move_t_to_absolute(_LASER_CUT_T_START)
        _abort_if_emergency_stop()
        laser_relay_on()
        if laser_state_var is not None:
            laser_state_var.set('On')
        _abort_if_emergency_stop()
        update_speed(1)
        # Alternate start<->end for _LASER_CUT_PASSES passes with the laser on.
        _pass_targets = [_LASER_CUT_T_END, _LASER_CUT_T_START]
        for _pass_num in range(_LASER_CUT_PASSES):
            _abort_if_emergency_stop()
            _move_t_to_absolute(_pass_targets[_pass_num % 2])
        _abort_if_emergency_stop()
        laser_relay_off()
        if laser_state_var is not None:
            laser_state_var.set('Off')
        _abort_if_emergency_stop()
        update_speed(10)
        _abort_if_emergency_stop()
        move_linear_stage("Z", "-", 1800, wait_for_stop=True, max_wait=30.0)
        print("Laser cutting sequence completed.")
    except RuntimeError as e:
        if str(e) == "Emergency stop requested.":
            print("Laser cutting sequence aborted by emergency stop.")
            laser_relay_off()
            if laser_state_var is not None:
                laser_state_var.set('Off')
            raise
        messagebox.showerror("Error", f"An error occurred during laser_cut: {e}")
        print(f"Exception in laser_cut: {e}")
    except Exception as e:
        laser_relay_off()
        if laser_state_var is not None:
            laser_state_var.set('Off')
        messagebox.showerror("Error", f"An error occurred during laser_cut: {e}")
        print(f"Exception in laser_cut: {e}")

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
    is_on = (val == 'On')
    image_recognition.record_camera0 = is_on and bool(rec_cam_vars[0].get())
    image_recognition.record_camera1 = is_on and bool(rec_cam_vars[1].get())
    image_recognition.record_camera2 = is_on and bool(rec_cam_vars[2].get())
    image_recognition.auto_annotate   = is_on
    print(f"[GUI] Recording => {'ON' if is_on else 'OFF'} (auto-annotate follows)")


_FOLDER_PICKER_CS = r"""
using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;

[ComImport, Guid("43826D1E-E718-42EE-BC55-A1E261C37BFE"),
 InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface IShellItem {
    void BindToHandler(IntPtr pbc, ref Guid bhid, ref Guid riid, out IntPtr ppv);
    void GetParent(out IShellItem ppsi);
    void GetDisplayName(uint sigdnName, [MarshalAs(UnmanagedType.LPWStr)] out string ppszName);
    void GetAttributes(uint sfgaoMask, out uint psfgaoAttribs);
    void Compare(IShellItem psi, uint hint, out int piOrder);
}

[ComImport, Guid("B63EA76D-1F85-456F-A19C-48159EFA858B"),
 InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface IShellItemArray {
    void BindToHandler(IntPtr pbc, ref Guid bhid, ref Guid riid, out IntPtr ppv);
    void GetPropertyStore(int flags, ref Guid riid, out IntPtr ppv);
    void GetPropertyDescriptionList(IntPtr keyType, ref Guid riid, out IntPtr ppv);
    void GetAttributes(int AttribFlags, uint sfgaoMask, out uint psfgaoAttribs);
    void GetCount(out uint pdwNumItems);
    void GetItemAt(uint dwIndex, out IShellItem ppsi);
    void EnumItems(out IntPtr ppenumShellItems);
}

[ComImport, Guid("D57C7288-D4AD-4768-BE02-9D969532D960"),
 InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
interface IFileOpenDialog {
    [PreserveSig] int Show(IntPtr parent);
    void SetFileTypes(uint c, IntPtr p);
    void SetFileTypeIndex(uint i);
    void GetFileTypeIndex(out uint i);
    void Advise(IntPtr p, out uint c);
    void Unadvise(uint c);
    void SetOptions(uint fos);
    void GetOptions(out uint fos);
    void SetDefaultFolder(IShellItem psi);
    void SetFolder(IShellItem psi);
    void GetFolder(out IShellItem ppsi);
    void GetCurrentSelection(out IShellItem ppsi);
    void SetFileName([MarshalAs(UnmanagedType.LPWStr)] string s);
    void GetFileName([MarshalAs(UnmanagedType.LPWStr)] out string s);
    void SetTitle([MarshalAs(UnmanagedType.LPWStr)] string s);
    void SetOkButtonLabel([MarshalAs(UnmanagedType.LPWStr)] string s);
    void SetFileNameLabel([MarshalAs(UnmanagedType.LPWStr)] string s);
    void GetResult(out IShellItem ppsi);
    void AddPlace(IShellItem psi, int fdap);
    void SetDefaultExtension([MarshalAs(UnmanagedType.LPWStr)] string s);
    void Close(int hr);
    void SetClientGuid(ref Guid g);
    void ClearClientData();
    void SetFilter(IntPtr p);
    void GetResults(out IShellItemArray ppenum);
    void GetSelectedItems(out IShellItemArray ppsai);
}

public static class FolderPicker {
    [DllImport("ole32.dll")]
    static extern int CoCreateInstance(ref Guid rclsid, IntPtr o, uint ctx,
                                       ref Guid riid,
                                       [MarshalAs(UnmanagedType.IUnknown)] out object ppv);
    [DllImport("shell32.dll", CharSet=CharSet.Unicode)]
    static extern int SHCreateItemFromParsingName(string pszPath, IntPtr pbc, ref Guid riid, out IShellItem ppv);
    public static string[] Pick(string title, string initialDir) {
        Guid clsid = new Guid("DC1C5A9C-E88A-4DDE-A5A1-60F82A20AEF7");
        Guid iid   = new Guid("D57C7288-D4AD-4768-BE02-9D969532D960");
        object obj;
        if (CoCreateInstance(ref clsid, IntPtr.Zero, 1, ref iid, out obj) != 0)
            return new string[0];
        var dlg = (IFileOpenDialog)obj;
        uint opt; dlg.GetOptions(out opt);
        // FOS_PICKFOLDERS=0x20 | FOS_ALLOWMULTISELECT=0x200 | FOS_FORCEFILESYSTEM=0x40
        dlg.SetOptions(opt | 0x20u | 0x200u | 0x40u);
        dlg.SetTitle(title);
        if (!string.IsNullOrEmpty(initialDir)) {
            Guid siid = new Guid("43826D1E-E718-42EE-BC55-A1E261C37BFE");
            IShellItem folder;
            if (SHCreateItemFromParsingName(initialDir, IntPtr.Zero, ref siid, out folder) == 0)
                dlg.SetFolder(folder);
        }
        if (dlg.Show(IntPtr.Zero) != 0) return new string[0];
        IShellItemArray items; dlg.GetResults(out items);
        uint count; items.GetCount(out count);
        var result = new List<string>();
        for (uint i = 0; i < count; i++) {
            IShellItem item; items.GetItemAt(i, out item);
            string path; item.GetDisplayName(0x80058000u, out path);
            result.Add(path);
        }
        return result.ToArray();
    }
}
"""

def _pick_multiple_folders_windows(title="Select Folders", initialdir=r"D:\\"):
    """Windows-only: IFileOpenDialog multi-select folder picker via PowerShell.
    Returns a list of selected paths, or None if the approach is unavailable."""
    import subprocess
    escaped_title = title.replace('"', '`"')
    escaped_init  = (initialdir or "").replace('"', '`"')
    ps = f'Add-Type @\'\n{_FOLDER_PICKER_CS}\n\'@\n[FolderPicker]::Pick("{escaped_title}", "{escaped_init}") | Write-Output'
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
            capture_output=True, text=True, timeout=120,
        )
        if result.returncode != 0:
            return None
        return [p.strip() for p in result.stdout.splitlines() if p.strip()]
    except Exception as e:
        print(f"[FolderPicker] PowerShell approach failed: {e}")
        return None


def open_batch_annotate_window(root):
    """Popup that lets the user pick one or more folders and batch-annotate images."""
    win = Toplevel(root)
    win.title("Batch Annotate Folders")
    win.resizable(False, False)

    # Folder list
    tk.Label(win, text="Image Folders:").grid(row=0, column=0, padx=10, pady=(10, 2), sticky='w')

    list_frame = tk.Frame(win)
    list_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=(0, 4), sticky='ew')
    scrollbar = tk.Scrollbar(list_frame, orient='vertical')
    folder_listbox = tk.Listbox(list_frame, width=48, height=5, yscrollcommand=scrollbar.set,
                                selectmode='single')
    scrollbar.config(command=folder_listbox.yview)
    folder_listbox.pack(side='left', fill='both', expand=True)
    scrollbar.pack(side='right', fill='y')

    def add_folder():
        paths = _pick_multiple_folders_windows(
            title="Select image folders (Ctrl/Shift to multi-select)",
            initialdir=r"D:\\"
        )
        if paths is None:
            # Fallback: PowerShell unavailable
            p = filedialog.askdirectory(title="Select folder containing images",
                                        initialdir=r"D:\\", parent=win)
            paths = [p] if p else []
        existing = set(folder_listbox.get(0, 'end'))
        for path in paths:
            if path and path not in existing:
                folder_listbox.insert('end', path)
                existing.add(path)

    def remove_folder():
        sel = folder_listbox.curselection()
        if sel:
            folder_listbox.delete(sel[0])

    btn_col = tk.Frame(win)
    btn_col.grid(row=1, column=2, padx=(0, 10), pady=(0, 4), sticky='n')
    tk.Button(btn_col, text="Add\u2026",   width=10, command=add_folder).pack(pady=(0, 4))
    tk.Button(btn_col, text="Remove", width=10, command=remove_folder).pack()

    # Confidence threshold
    tk.Label(win, text="Confidence threshold:").grid(row=2, column=0, padx=10, pady=4, sticky='w')
    conf_var = tk.DoubleVar(value=0.5)
    conf_slider = tk.Scale(win, from_=0.05, to=0.95, resolution=0.05,
                           orient='horizontal', variable=conf_var, length=200)
    conf_slider.grid(row=2, column=1, columnspan=2, padx=(0, 10), pady=4, sticky='w')

    # Progress bar + status label
    progress_bar = tk.Canvas(win, width=380, height=18, bg='#ddd', highlightthickness=1,
                              highlightbackground='#aaa')
    progress_bar.grid(row=3, column=0, columnspan=3, padx=10, pady=(8, 2))
    status_var = tk.StringVar(value="Ready.")
    tk.Label(win, textvariable=status_var, anchor='w').grid(
        row=4, column=0, columnspan=3, padx=10, pady=(0, 6), sticky='w')

    _bar_fill = progress_bar.create_rectangle(0, 0, 0, 18, fill='#4caf50', width=0)

    def _update_bar(fraction):
        progress_bar.coords(_bar_fill, 0, 0, 380 * fraction, 18)

    run_btn = tk.Button(win, text="Run Annotation")
    run_btn.grid(row=5, column=0, columnspan=3, pady=(4, 12))

    def _run():
        folders = list(folder_listbox.get(0, 'end'))
        if not folders:
            messagebox.showerror("No folders", "Add at least one image folder.", parent=win)
            return
        invalid = [f for f in folders if not os.path.isdir(f)]
        if invalid:
            messagebox.showerror("Invalid folder",
                                 "The following paths are not valid directories:\n" +
                                 "\n".join(invalid), parent=win)
            return
        conf = round(conf_var.get(), 2)
        run_btn.config(state='disabled')
        status_var.set("Running…")
        _update_bar(0)

        def worker():
            total_annotated = 0
            total_skipped = 0
            try:
                for folder_idx, folder in enumerate(folders):
                    folder_name = os.path.basename(folder)

                    def progress_cb(done, total, fname, _fi=folder_idx, _fn=folder_name):
                        # Overall fraction: completed folders + current folder progress
                        overall = (_fi + (done / total if total else 1.0)) / len(folders)
                        win.after(0, lambda f=overall: _update_bar(f))
                        win.after(0, lambda: status_var.set(
                            f"[{_fi + 1}/{len(folders)}] {_fn}  —  {done}/{total}  {fname}"))

                    annotated, skipped = image_recognition.annotate_folder(
                        folder, conf=conf, progress_callback=progress_cb)
                    total_annotated += annotated
                    total_skipped += skipped

                win.after(0, lambda: _update_bar(1.0))
                win.after(0, lambda: status_var.set(
                    f"Done. {total_annotated} annotated, {total_skipped} skipped across "
                    f"{len(folders)} folder(s). Labels saved to {image_recognition.ANNOTATION_DIR}"))
            except Exception as exc:
                win.after(0, lambda: status_var.set(f"Error: {exc}"))
            finally:
                win.after(0, lambda: run_btn.config(state='normal'))

        threading.Thread(target=worker, daemon=True).start()

    run_btn.config(command=_run)

    tk.Button(
        win, text="Sort into YOLO Dataset…",
        command=lambda: open_sort_dataset_window(win, lambda: list(folder_listbox.get(0, 'end')))
    ).grid(row=6, column=0, columnspan=3, pady=(0, 4))

    tk.Button(
        win, text="Start YOLO Training…",
        command=lambda: open_training_window(win)
    ).grid(row=7, column=0, columnspan=3, pady=(0, 10))


def open_sort_dataset_window(parent_win, get_folders_fn):
    """Sort annotated image+label pairs into a YOLO train/val dataset split."""
    import random
    import shutil

    win = Toplevel(parent_win)
    win.title("Sort into YOLO Dataset")
    win.resizable(False, False)

    # Dataset root
    dest_var = tk.StringVar(value=r"D:\YOLODataset")
    tk.Label(win, text="Dataset root:").grid(row=0, column=0, padx=10, pady=(12, 4), sticky='w')
    tk.Entry(win, textvariable=dest_var, width=34).grid(row=0, column=1, padx=(0, 4), pady=(12, 4))
    def browse_dest():
        p = filedialog.askdirectory(title="Select YOLO dataset root", parent=win)
        if p:
            dest_var.set(p)
    tk.Button(win, text="Browse…", command=browse_dest).grid(row=0, column=2, padx=(0, 10), pady=(12, 4))

    # Train split
    tk.Label(win, text="Train split (%):").grid(row=1, column=0, padx=10, pady=4, sticky='w')
    split_var = tk.IntVar(value=66)
    tk.Spinbox(win, from_=10, to=90, width=5, textvariable=split_var).grid(
        row=1, column=1, padx=(0, 10), pady=4, sticky='w')

    # Progress bar + status
    progress_bar = tk.Canvas(win, width=380, height=18, bg='#ddd',
                             highlightthickness=1, highlightbackground='#aaa')
    progress_bar.grid(row=2, column=0, columnspan=3, padx=10, pady=(8, 2))
    _bar_fill = progress_bar.create_rectangle(0, 0, 0, 18, fill='#4caf50', width=0)
    def _update_bar(frac):
        progress_bar.coords(_bar_fill, 0, 0, 380 * frac, 18)
    status_var = tk.StringVar(value="Ready.")
    tk.Label(win, textvariable=status_var, anchor='w', width=52).grid(
        row=3, column=0, columnspan=3, padx=10, pady=(0, 6), sticky='w')

    sort_btn = tk.Button(win, text="Sort")
    sort_btn.grid(row=4, column=0, columnspan=3, pady=(4, 12))

    def _sort():
        folders = get_folders_fn()
        dest_root = dest_var.get().strip()

        if not dest_root:
            messagebox.showerror("Error", "Please specify a dataset root folder.", parent=win)
            return
        if not folders:
            messagebox.showerror("No folders",
                "No source folders are listed in the annotate window.", parent=win)
            return

        annotation_dir = image_recognition.ANNOTATION_DIR
        _IMG_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif'}

        # Build (image_path, label_path) pairs where the label actually exists
        pairs = []
        seen_stems = set()
        for folder in folders:
            if not os.path.isdir(folder):
                continue
            for fname in sorted(os.listdir(folder)):
                if os.path.splitext(fname)[1].lower() not in _IMG_EXTS:
                    continue
                stem = os.path.splitext(fname)[0]
                if stem in seen_stems:
                    continue  # skip duplicate stems across folders
                label_path = os.path.join(annotation_dir, stem + ".txt")
                if os.path.isfile(label_path):
                    pairs.append((os.path.join(folder, fname), label_path))
                    seen_stems.add(stem)

        if not pairs:
            messagebox.showinfo("No pairs found",
                "No image+label pairs were found.\n"
                "Run annotation first and ensure labels exist in:\n"
                f"{annotation_dir}", parent=win)
            return

        random.shuffle(pairs)
        train_pct = split_var.get()
        n_train = max(1, int(len(pairs) * train_pct / 100))
        train_pairs = pairs[:n_train]
        val_pairs   = pairs[n_train:]

        split_dirs = {
            'train_img': os.path.join(dest_root, 'images', 'train'),
            'val_img':   os.path.join(dest_root, 'images', 'val'),
            'train_lbl': os.path.join(dest_root, 'labels', 'train'),
            'val_lbl':   os.path.join(dest_root, 'labels', 'val'),
        }
        try:
            for d in split_dirs.values():
                os.makedirs(d, exist_ok=True)
        except Exception as e:
            messagebox.showerror("Error", f"Could not create destination folders:\n{e}", parent=win)
            return

        sort_btn.config(state='disabled')
        _update_bar(0)
        status_var.set("Sorting…")

        def worker():
            all_pairs = [('train', p) for p in train_pairs] + [('val', p) for p in val_pairs]
            total = len(all_pairs)
            errors = []
            for i, (split, (img_src, lbl_src)) in enumerate(all_pairs):
                try:
                    shutil.move(img_src, os.path.join(split_dirs[f'{split}_img'],
                                                      os.path.basename(img_src)))
                    shutil.move(lbl_src, os.path.join(split_dirs[f'{split}_lbl'],
                                                      os.path.basename(lbl_src)))
                except Exception as e:
                    errors.append(str(e))
                win.after(0, lambda f=(i + 1) / total: _update_bar(f))

            # Move any leftover files (videos, non-annotated images, etc.) to misc/
            misc_dir = os.path.join(dest_root, 'misc')
            misc_count = 0
            for folder in folders + [annotation_dir]:
                if not os.path.isdir(folder):
                    continue
                for fname in os.listdir(folder):
                    if fname == 'classes.txt':
                        continue
                    src = os.path.join(folder, fname)
                    if not os.path.isfile(src):
                        continue
                    os.makedirs(misc_dir, exist_ok=True)
                    try:
                        shutil.move(src, os.path.join(misc_dir, fname))
                        misc_count += 1
                    except Exception as e:
                        errors.append(str(e))

            # Delete source folders now that they are empty
            for folder in folders + [annotation_dir]:
                try:
                    if os.path.isdir(folder) and not os.listdir(folder):
                        os.rmdir(folder)
                except Exception:
                    pass

            if errors:
                win.after(0, lambda: status_var.set(f"{len(errors)} error(s) — first: {errors[0]}"))
            else:
                misc_note = f"  {misc_count} leftover file(s) → misc/." if misc_count else ""
                win.after(0, lambda: status_var.set(
                    f"Moved: {len(train_pairs)} train, {len(val_pairs)} val  →  {dest_root}.{misc_note}"))
            win.after(0, lambda: sort_btn.config(state='normal'))

        threading.Thread(target=worker, daemon=True).start()


    sort_btn.config(command=_sort)


###############################
# IMAGE ADJUSTMENT SLIDER POPUP
###############################
def open_camera_port_settings_window(root):
    """Popup to reassign which OS camera device port each logical camera role uses."""
    win = Toplevel(root)
    win.title("Camera Port Assignments")
    win.resizable(False, False)

    role_labels = {
        0: "Camera 0 — Wire Tip View",
        1: "Camera 1 — Clog Detection",
        2: "Camera 2 — Main PCB View",
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

def save_camera_adjustments(adjustments):
    """Persist per-camera image adjustments to pcb_settings.json."""
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing['camera_adjustments'] = {str(role): dict(adj) for role, adj in adjustments.items()}
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Saved camera adjustments to {SETTINGS_FILE}")
    except Exception as e:
        print(f"Warning: Could not save camera adjustments: {e}")

def open_image_adjustment_window(parent=None):
    adj_win = Toplevel(parent)
    adj_win.title("Image Adjustments")

    _FIELDS = [
        ('alpha',         'Contrast (Alpha)',  0.1,  3.0,  0.1),
        ('beta',          'Brightness (Beta)', -100, 100,  1  ),
        ('sat_factor',    'Saturation',        0.0,  2.0,  0.1),
        ('gamma',         'Gamma',             0.1,  2.5,  0.1),
        ('sharp_strength','Sharpness',         0.0,  2.0,  0.1),
    ]

    _CAM_LABELS = {0: '0 — Wire Tip', 1: '1 — Clog', 2: '2 — PCB View'}

    # Camera selector
    cam_var = tk.IntVar(value=0)
    cam_frame = tk.Frame(adj_win)
    cam_frame.pack(pady=(10, 4))
    tk.Label(cam_frame, text="Camera:").pack(side='left', padx=(0, 6))

    # Sliders — built once, values swapped when camera changes
    sliders = {}
    slider_frame = tk.Frame(adj_win)
    slider_frame.pack(fill='x', padx=10, pady=4)

    for key, label, lo, hi, res in _FIELDS:
        tk.Label(slider_frame, text=label, anchor='w').pack(fill='x')
        s = tk.Scale(
            slider_frame, from_=lo, to=hi, resolution=res,
            orient='horizontal',
            command=lambda val, k=key: _on_slider(k, float(val)),
        )
        s.pack(fill='x')
        sliders[key] = s

    def _load_sliders(role=None):
        if role is None:
            role = cam_var.get()
        adj = image_recognition.camera_adjustments.get(role, {})
        for key, s in sliders.items():
            s.set(adj.get(key, image_recognition._DEFAULT_ADJUSTMENTS[key]))

    def _on_slider(key, val):
        role = cam_var.get()
        image_recognition.camera_adjustments[role][key] = val

    def _switch_camera():
        _load_sliders(cam_var.get())

    for role, label in _CAM_LABELS.items():
        tk.Radiobutton(
            cam_frame, text=label, variable=cam_var,
            value=role, command=_switch_camera,
        ).pack(side='left', padx=4)

    _load_sliders(0)

    btn_frame = tk.Frame(adj_win)
    btn_frame.pack(pady=(6, 10))
    tk.Button(
        btn_frame, text="Save & Close",
        command=lambda: [
            save_camera_adjustments(image_recognition.camera_adjustments),
            adj_win.destroy(),
        ]
    ).pack(side='left', padx=8)
    tk.Button(btn_frame, text="Close", command=adj_win.destroy).pack(side='left', padx=8)

###############################
# NAMED ORIGINS (probe / print / microwire)
# JSON keys and axes match print.py so both systems share the same saved values.
###############################
_ORIGIN_CONFIG = {
    'probe':     ('probe_origin',        ['X', 'Y', 'r', 'Z']),
    'print':     ('print_origin_coords', ['X', 'Y', 'r', 'Z']),
    'microwire': ('microwire_origin',    ['X', 'Y', 'Z', 'r']),
    'connector': ('connector_origin',    ['X', 'Y', 'Z', 'r']),
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
    """Move axes to the stored named origin with camera-fixture-safe axis ordering.

    Arriving AT microwire:   Z drop 5000  → X → r → Z(target) → Y
    Departing FROM microwire: Z drop 15000 → Y → X → r → Z(target)
    All other moves:          Z drop 5000  → X → Y → r → Z(target)
    """
    cfg = _ORIGIN_CONFIG.get(name)
    if not cfg:
        print(f"[Origin] Unknown origin name '{name}'")
        return
    positions = load_named_origin(name)
    if not positions:
        print(f"[Origin] No saved origin for '{name}'. Use 'Set Origin' first.")
        return
    print(f"\n--- Moving to '{name}' origin ---")
    prev_speed = get_current_speed()
    update_speed(30)

    def _move(ax):
        if is_emergency_stop_requested():
            print(f"[Origin] Emergency stop — aborting return to '{name}' origin.")
            return False
        if ax not in positions:
            return True
        current_pos = get_current_position(ax)
        if current_pos is None:
            print(f"[Origin] Axis {ax}: position unknown, skipping.")
            return True
        diff = positions[ax] - current_pos
        if abs(diff) < 0.5:
            return True
        direction = '+' if diff >= 0 else '-'
        print(f"[Origin] Moving {ax} -> {positions[ax]:.3f}")
        move_linear_stage(ax, direction, abs(diff), wait_for_stop=True, max_wait=30.0)
        return True

    arriving_at_microwire = (name == 'microwire')
    departing_microwire = False
    if not arriving_at_microwire:
        microwire_pos = load_named_origin('microwire')
        if microwire_pos and 'Z' in microwire_pos:
            cur_z = get_current_position('Z')
            if cur_z is not None and abs(cur_z - microwire_pos['Z']) < 10000:
                departing_microwire = True

    z_drop = 15000 if departing_microwire else 5000

    try:
        # 1) Drop Z for clearance
        if is_emergency_stop_requested():
            print(f"[Origin] Emergency stop — aborting return to '{name}' origin.")
            return
        move_linear_stage('Z', '-', z_drop, wait_for_stop=True, max_wait=30.0)

        if arriving_at_microwire:
            # 2a) Arriving at microwire: X → r → Z → Y (Y last)
            for ax in ('X', 'r', 'Z', 'Y'):
                if not _move(ax):
                    return
        elif departing_microwire:
            # 2b) Departing from microwire: Y → X → r → Z (Y first)
            for ax in ('Y', 'X', 'r', 'Z'):
                if not _move(ax):
                    return
        else:
            # 2c) Standard: X → Y → r → Z
            for ax in ('X', 'Y', 'r', 'Z'):
                if not _move(ax):
                    return
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

def open_connector_offset_window(root):
    """Popup to edit the connector origin offset relative to the print origin (µm).

    When set, goto_pnp_origin() derives the connector position as
    print_origin + this offset instead of using the saved absolute connector_origin.
    Set all axes to 0 and click Clear to revert to the absolute connector origin.
    """
    win = Toplevel(root)
    win.title("Connector Offset from Print Origin")
    win.resizable(False, False)

    try:
        with open(SETTINGS_FILE, 'r') as f:
            current = json.load(f).get('connector_print_offset', {})
    except Exception:
        current = {}

    fields = [
        ('X', 'X offset (µm):', 0),
        ('Y', 'Y offset (µm):', 0),
        ('Z', 'Z offset (µm):', 0),
        ('r', 'r offset (µm):', 0),
    ]

    vars_by_key = {}
    for row, (key, label, default) in enumerate(fields):
        tk.Label(win, text=label).grid(row=row, column=0, padx=10, pady=6, sticky='w')
        var = tk.StringVar(value=str(current.get(key, default)))
        vars_by_key[key] = var
        tk.Entry(win, textvariable=var, width=10).grid(row=row, column=1, padx=10, pady=6)

    tk.Label(
        win,
        text=("Offset from print origin to connector pad array center.\n"
              "When set, overrides the saved absolute connector origin.\n"
              "Use 'Clear' to remove and revert to the absolute connector origin."),
        fg="gray",
        justify='left',
    ).grid(row=len(fields), column=0, columnspan=2, padx=10, pady=(2, 8))

    def on_apply():
        try:
            new_offset = {key: int(var.get()) for key, var in vars_by_key.items()}
        except ValueError:
            messagebox.showerror("Error", "Offsets must be integers (µm).", parent=win)
            return
        save_connector_print_offset(new_offset)
        reload_origins()
        print(f"[Settings] Connector offset from print origin saved: {new_offset}")
        win.destroy()

    def on_clear():
        """Remove the connector_print_offset key entirely so the absolute origin is used."""
        existing = {}
        if os.path.isfile(SETTINGS_FILE):
            try:
                with open(SETTINGS_FILE, 'r') as f:
                    existing = json.load(f)
            except Exception:
                pass
        existing.pop('connector_print_offset', None)
        try:
            with open(SETTINGS_FILE, 'w') as f:
                json.dump(existing, f, indent=2)
        except Exception as e:
            print(f"Warning: Could not clear connector offset: {e}")
            return
        save_connector_print_offset(None)
        reload_origins()
        print("[Settings] Connector offset cleared — absolute connector origin will be used.")
        win.destroy()

    btn_frame = tk.Frame(win)
    btn_frame.grid(row=len(fields) + 1, column=0, columnspan=2, pady=8)
    tk.Button(btn_frame, text="Apply & Save", command=on_apply).pack(side='left', padx=6)
    tk.Button(btn_frame, text="Clear", command=on_clear).pack(side='left', padx=6)
    tk.Button(btn_frame, text="Cancel", command=win.destroy).pack(side='left', padx=6)

    win.grab_set()


def open_pnp_offsets_window(root):
    """Popup to edit the PNP-to-nozzle axis offsets (µm), persisted to pcb_settings.json."""
    win = Toplevel(root)
    win.title("PNP Offsets")
    win.resizable(False, False)

    try:
        with open(SETTINGS_FILE, 'r') as f:
            current = json.load(f).get('pnp_offsets', {})
    except Exception:
        current = {}

    fields = [
        ('X', 'X offset (µm):', -29580),
        ('Y', 'Y offset (µm):', -2300),
        ('Z', 'Z offset (µm):', 4800),
    ]

    vars_by_key = {}
    for row, (key, label, default) in enumerate(fields):
        tk.Label(win, text=label).grid(row=row, column=0, padx=10, pady=6, sticky='w')
        var = tk.StringVar(value=str(current.get(key, default)))
        vars_by_key[key] = var
        tk.Entry(win, textvariable=var, width=10).grid(row=row, column=1, padx=10, pady=6)

    tk.Label(
        win,
        text="Applied when navigating to the PNP position.\nPositive = stage moves in + direction.",
        fg="gray",
        justify='left',
    ).grid(row=len(fields), column=0, columnspan=2, padx=10, pady=(2, 8))

    def on_apply():
        try:
            new_offsets = {key: int(var.get()) for key, var in vars_by_key.items()}
        except ValueError:
            messagebox.showerror("Error", "Offsets must be integers (µm).", parent=win)
            return
        save_pnp_offsets(new_offsets)
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
        width=34,
        command=lambda: open_image_adjustment_window(win)
    ).pack(pady=6)

    tk.Button(
        win,
        text="Camera Port Settings",
        width=34,
        command=lambda: open_camera_port_settings_window(root)
    ).pack(pady=6)

    tk.Button(
        win,
        text="Trace Routing Tuning",
        width=34,
        command=lambda: open_trace_tuning_window(root)
    ).pack(pady=6)

    tk.Button(
        win,
        text="PNP Offsets",
        width=34,
        command=lambda: open_pnp_offsets_window(root)
    ).pack(pady=6)

    tk.Button(
        win,
        text="Connector Offset from Print Origin",
        width=34,
        command=lambda: open_connector_offset_window(root)
    ).pack(pady=6)

    tk.Frame(win, height=1, bg='gray').pack(fill='x', padx=12, pady=(6, 2))

    tk.Button(
        win,
        text="Reprint Feature",
        width=34,
        command=lambda: open_reprint_window(root)
    ).pack(pady=6)

    tk.Button(
        win,
        text="Test PNP Routine",
        width=34,
        command=lambda: open_pnp_test_window(root)
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
# YOLO TRAINING WINDOW
###############################
_YOLO_EXE_DEFAULT = (
    r"C:\Users\MINI Lab\AppData\Local\Packages"
    r"\PythonSoftwareFoundation.Python.3.11_qbz5n2kfra8p0"
    r"\LocalCache\local-packages\Python311\Scripts\yolo.exe"
)
_YOLO_EXE_SETTINGS_KEY = "yolo_exe_path"

def _load_yolo_exe():
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f).get(_YOLO_EXE_SETTINGS_KEY, _YOLO_EXE_DEFAULT)
    except Exception:
        pass
    return _YOLO_EXE_DEFAULT

def _save_yolo_exe(path):
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing[_YOLO_EXE_SETTINGS_KEY] = path
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
    except Exception as e:
        print(f"Warning: could not save yolo_exe_path: {e}")

def open_training_window(root):
    """Popup to configure and launch a YOLO training run with live log output."""
    import subprocess
    from tkinter import scrolledtext

    win = Toplevel(root)
    win.title("YOLO Training")
    win.resizable(True, True)

    _proc = [None]  # mutable container so inner functions can reference it

    # ── Parameter fields ──────────────────────────────────────────────────────
    param_frame = tk.Frame(win)
    param_frame.pack(fill='x', padx=10, pady=(10, 4))

    # YOLO exe
    tk.Label(param_frame, text="YOLO exe:", anchor='w').grid(row=0, column=0, sticky='w', pady=3)
    yolo_exe_var = tk.StringVar(value=_load_yolo_exe())
    tk.Entry(param_frame, textvariable=yolo_exe_var, width=52).grid(
        row=0, column=1, columnspan=2, sticky='ew', padx=(4, 0), pady=3)
    def browse_exe():
        p = filedialog.askopenfilename(
            title="Select yolo.exe",
            filetypes=[("Executable", "*.exe"), ("All files", "*.*")],
            parent=win)
        if p:
            yolo_exe_var.set(p)
    tk.Button(param_frame, text="Browse\u2026", command=browse_exe).grid(
        row=0, column=3, padx=(4, 0), pady=3)

    # Model
    tk.Label(param_frame, text="Model:", anchor='w').grid(row=1, column=0, sticky='w', pady=3)
    model_var = tk.StringVar(value="yolov8n.pt")
    tk.Entry(param_frame, textvariable=model_var, width=18).grid(
        row=1, column=1, sticky='w', padx=(4, 0), pady=3)

    # Data YAML
    tk.Label(param_frame, text="Data YAML:", anchor='w').grid(row=1, column=2, sticky='w', padx=(10, 0), pady=3)
    data_var = tk.StringVar(value="data.yaml")
    tk.Entry(param_frame, textvariable=data_var, width=18).grid(
        row=1, column=3, sticky='w', padx=(4, 0), pady=3)
    def browse_yaml():
        p = filedialog.askopenfilename(
            title="Select data.yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
            parent=win)
        if p:
            data_var.set(p)
    tk.Button(param_frame, text="Browse\u2026", command=browse_yaml).grid(
        row=1, column=4, padx=(4, 0), pady=3)

    # Epochs + imgsz
    tk.Label(param_frame, text="Epochs:", anchor='w').grid(row=2, column=0, sticky='w', pady=3)
    epochs_var = tk.StringVar(value="100")
    tk.Entry(param_frame, textvariable=epochs_var, width=8).grid(
        row=2, column=1, sticky='w', padx=(4, 0), pady=3)

    tk.Label(param_frame, text="Image size (W,H):", anchor='w').grid(
        row=2, column=2, sticky='w', padx=(10, 0), pady=3)
    imgsz_var = tk.StringVar(value="640,480")
    tk.Entry(param_frame, textvariable=imgsz_var, width=10).grid(
        row=2, column=3, sticky='w', padx=(4, 0), pady=3)

    # ── Buttons ───────────────────────────────────────────────────────────────
    btn_frame = tk.Frame(win)
    btn_frame.pack(pady=(4, 2))

    status_var = tk.StringVar(value="Idle.")
    tk.Label(win, textvariable=status_var, anchor='w').pack(fill='x', padx=12)

    # ── Log output ────────────────────────────────────────────────────────────
    log = scrolledtext.ScrolledText(win, width=80, height=20, state='disabled',
                                    bg='#1e1e1e', fg='#d4d4d4', font=("Courier", 9))
    log.pack(fill='both', expand=True, padx=10, pady=(4, 10))

    def _append_log(text):
        log.config(state='normal')
        log.insert('end', text)
        log.see('end')
        log.config(state='disabled')

    def _stop_cameras_sync():
        """Stop all camera threads and block until they exit (called from worker thread)."""
        print("[Training] Stopping cameras to free GPU...")
        for i in range(3):
            image_recognition.camera_stop_events[i].set()
        for i in range(3):
            t = _camera_threads.get(i)
            if t and t.is_alive():
                t.join(timeout=6.0)
        print("[Training] Cameras stopped.")

    def _reader_thread(proc):
        for line in proc.stdout:
            win.after(0, lambda l=line: _append_log(l))
        ret = proc.wait()
        # Restart cameras now that the GPU is free
        print("[Training] Restarting cameras...")
        for i in range(3):
            image_recognition.camera_stop_events[i].clear()
        start_camera_threads()
        win.after(0, lambda: status_var.set(
            f"Training finished — exit code {ret}. Cameras restarted."))
        win.after(0, lambda: start_btn.config(state='normal'))
        win.after(0, lambda: stop_btn.config(state='disabled'))

    def on_start():
        exe = yolo_exe_var.get().strip()
        if not os.path.isfile(exe):
            messagebox.showerror("Not found",
                f"YOLO executable not found:\n{exe}", parent=win)
            return
        _save_yolo_exe(exe)
        cmd = [
            exe, "detect", "train",
            f"model={model_var.get().strip()}",
            f"data={data_var.get().strip()}",
            f"epochs={epochs_var.get().strip()}",
            f"imgsz={imgsz_var.get().strip()}",
        ]

        start_btn.config(state='disabled')
        stop_btn.config(state='normal')
        status_var.set("Stopping cameras…")

        def _launch():
            _stop_cameras_sync()
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                    encoding='utf-8', errors='replace',
                )
            except Exception as e:
                win.after(0, lambda: messagebox.showerror("Launch error", str(e), parent=win))
                win.after(0, lambda: start_btn.config(state='normal'))
                win.after(0, lambda: stop_btn.config(state='disabled'))
                # Restore cameras if launch failed
                for i in range(3):
                    image_recognition.camera_stop_events[i].clear()
                start_camera_threads()
                return
            _proc[0] = proc
            win.after(0, lambda: _append_log(f"$ {' '.join(cmd)}\n\n"))
            win.after(0, lambda: status_var.set("Training running\u2026"))
            _reader_thread(proc)

        threading.Thread(target=_launch, daemon=True).start()

    def on_stop():
        proc = _proc[0]
        if proc and proc.poll() is None:
            proc.terminate()
            status_var.set("Stop requested.")

    start_btn = tk.Button(btn_frame, text="Start Training", width=16, command=on_start)
    start_btn.pack(side='left', padx=8)
    stop_btn = tk.Button(btn_frame, text="Stop", width=10, command=on_stop, state='disabled')
    stop_btn.pack(side='left', padx=8)

    win.protocol("WM_DELETE_WINDOW", lambda: (on_stop(), win.destroy()))


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
        if not messagebox.askyesno(
            "Confirm Set Origin",
            f"This will overwrite the saved '{mode}' origin with the machine's\n"
            "current axis positions. This cannot be undone automatically.\n\n"
            "Are you sure the machine is positioned correctly?",
            icon='warning',
            parent=root,
        ):
            print(f"[Origin] Set Origin for '{mode}' cancelled by user.")
            return
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
    for _mode in ('Probe', 'Print', 'Connector', 'Microwire'):
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

    # RECORDING radio + per-camera checkboxes
    global record_var, rec_cam_vars
    record_var = tk.StringVar(value='Off')
    rec_cam_vars = {0: tk.IntVar(value=1), 1: tk.IntVar(value=0), 2: tk.IntVar(value=0)}
    record_frame = tk.Frame(root)
    record_frame.pack(pady=5)
    tk.Label(record_frame, text="Recording: ").pack(side='left')
    tk.Radiobutton(record_frame, text="On",  variable=record_var, value='On',  command=toggle_recording).pack(side='left')
    tk.Radiobutton(record_frame, text="Off", variable=record_var, value='Off', command=toggle_recording).pack(side='left')
    tk.Label(record_frame, text="   Cameras:").pack(side='left')
    for i in range(3):
        tk.Checkbutton(record_frame, text=f"Cam{i}",
                       variable=rec_cam_vars[i], command=toggle_recording).pack(side='left', padx=2)

    tk.Button(root, text="Annotate Image Folder…",
              command=lambda: open_batch_annotate_window(root)).pack(pady=5)

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