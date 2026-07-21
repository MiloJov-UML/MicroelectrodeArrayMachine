import time
import math
import threading
import json
import os
from motor_control import (
    move_linear_stage, 
    update_speed, 
    get_current_speed,
    stop_motor_control, 
    get_current_position,
    wait_for_axis_stop,
    mm_to_um,
    clear_emergency_stop,
    is_emergency_stop_requested,
    flush_serial,
)

from relay_control import (
    laser_relay_off,
    nordson_on, 
    nordson_off,
    servo_to,
    motor_backward, 
    motor_forward, 
    motor_release, 
    r_calibrate, 
    Z_calibrate,
    pnp_forward,
    pnp_backward,
    pnp_release,
    mag_detector,
    solenoid_relay_on,
    solenoid_relay_off
)   

#parameters for line test

x = 'X'
y = 'Y'
z = 'Z'

delay = 0.5       # Dispenser settle time

x_coord, y_coord, z_coord = None, None, None # Current coordinates
angle_dir, angle_axis, t_len = None, None, None# Angle based direction and axis, trace length
counter = 0 # Feature counter for next feature calculation

#print_z_coord = probe_z_coord - print_gap # Z coordinate for printing, set after probing based on print_gap
print_home = [0, 0, 0, 0] # X, Y, Z, R coordinate for starting point for print process
pcb_z_coord = None # Z coordinate for printing, set after probing
_has_calibrated = False  # Set True after first successful calibrate(); skipped on reruns

# ---------------------------------------------------------------------------
# Four-origin system: probe, print, microwire, connector
# Each is persisted to pcb_settings.json so every confirmed position becomes
# the starting point for the next run, making the system more accurate over time.
# ---------------------------------------------------------------------------
SETTINGS_FILE = "pcb_settings.json"

# Saved origin dicts: None means "no saved value yet, use first-run defaults"
_probe_origin    = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...}
_print_origin_saved   = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...}
_microwire_origin_saved = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...}
_connector_origin_saved = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...} — center of 4x2 connector pad array
_connector_print_offset = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...} — offset from print origin to connector origin (overrides absolute if set)

def _load_origins():
    """Load the four saved origins from pcb_settings.json at startup."""
    global _probe_origin, _print_origin_saved, _microwire_origin_saved, _connector_origin_saved, _connector_print_offset
    if not os.path.isfile(SETTINGS_FILE):
        return
    try:
        with open(SETTINGS_FILE, 'r') as f:
            data = json.load(f)
        _probe_origin             = data.get('probe_origin')            or None
        _print_origin_saved       = data.get('print_origin_coords')     or None
        _microwire_origin_saved   = data.get('microwire_origin')        or None
        _connector_origin_saved   = data.get('connector_origin')        or None
        _connector_print_offset   = data.get('connector_print_offset')  or None
        print(f"Origins loaded probe: {_probe_origin}, print: {_print_origin_saved}, microwire: {_microwire_origin_saved}, connector: {_connector_origin_saved}, connector_offset: {_connector_print_offset}")
    except Exception as e:
        print(f"Warning: could not load origins from {SETTINGS_FILE}: {e}")

def _save_origin(key, positions_dict):
    """Persist one origin dict to pcb_settings.json without overwriting other keys."""
    existing = {}
    if os.path.isfile(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                existing = json.load(f)
        except Exception:
            pass
    existing[key] = positions_dict
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(existing, f, indent=2)
        print(f"Origin '{key}' saved: {positions_dict}")
    except Exception as e:
        print(f"Warning: could not save origin '{key}': {e}")

def reload_origins():
    """Reload all four saved origins from pcb_settings.json into memory.
    Call this after the GUI saves a new origin so in-session routines see it."""
    _load_origins()

def get_connector_print_offset():
    """Return the saved connector-from-print offset dict, or None if not configured."""
    return _connector_print_offset

def save_connector_print_offset(offset_dict):
    """Persist the connector-from-print offset to pcb_settings.json and update in-memory state."""
    global _connector_print_offset
    _connector_print_offset = offset_dict if offset_dict else None
    _save_origin('connector_print_offset', offset_dict)

def _navigate_to_saved_origin(origin_dict, axes):
    """Move to a saved origin with camera-fixture-safe axis ordering.

    Arriving AT microwire:   Z drop 5000  → X → r → Z(target) → Y
    Departing FROM microwire: Z drop 15000 → Y → X → r → Z(target)
    All other moves:          Z drop 5000  → non-Z axes (given order) → Z(target)
    """
    def _move(ax, val):
        _abort_if_emergency_stop()
        cur = get_current_position(ax)
        if cur is None:
            return
        diff = val - cur
        if abs(diff) < 0.5:
            return
        move_linear_stage(ax, '+' if diff >= 0 else '-', abs(diff),
                          wait_for_stop=True, max_wait=30.0)

    arriving_at_microwire = (origin_dict is _microwire_origin_saved)
    departing_microwire = False
    if not arriving_at_microwire and _microwire_origin_saved and 'Z' in _microwire_origin_saved:
        cur_z = get_current_position('Z')
        if cur_z is not None and abs(cur_z - _microwire_origin_saved['Z']) < 10000:
            departing_microwire = True

    z_drop = 15000 if departing_microwire else 5000

    # 1) Drop Z for clearance
    _abort_if_emergency_stop()
    move_linear_stage('Z', '-', z_drop, wait_for_stop=True, max_wait=30.0)

    if arriving_at_microwire:
        # 2a) Arriving at microwire: X → r → Z → Y (Y last to clear camera fixturing)
        for ax in ('X', 'r', 'Z', 'Y'):
            val = origin_dict.get(ax)
            if val is not None:
                _move(ax, val)
    elif departing_microwire:
        # 2b) Departing from microwire: Y → X → r → Z (Y first to clear camera fixturing)
        for ax in ('Y', 'X', 'r', 'Z'):
            val = origin_dict.get(ax)
            if val is not None:
                _move(ax, val)
    else:
        # 2c) Standard: non-Z axes in given order, then Z last
        for ax in axes:
            if ax == 'Z':
                continue
            val = origin_dict.get(ax)
            if val is None:
                continue
            _move(ax, val)
        z_val = origin_dict.get('Z')
        if z_val is not None:
            _move('Z', z_val)

# Event and callback used by all three fine-tune pauses
_origin_event = threading.Event()
_origin_prompt_callback = None  # callable(label: str) registered by the GUI

# Opt-in choice: user decides whether to fine-tune or accept current position.
_origin_choice_event    = threading.Event()
_origin_fine_tune_chosen = False
_origin_ask_callback    = None  # callable(label: str) shows the Yes/No dialog

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

def register_origin_prompt_callback(callback):
    """Register a GUI callable(label: str) shown when the user chooses to fine-tune."""
    global _origin_prompt_callback
    _origin_prompt_callback = callback

def register_origin_ask_callback(callback):
    """Register a GUI callable(label: str) that asks the user: fine-tune or continue?"""
    global _origin_ask_callback
    _origin_ask_callback = callback

def confirm_origin_set():
    """Called by the GUI 'Set Origin' button to confirm the current position."""
    _origin_event.set()

def notify_fine_tune_choice(chosen: bool):
    """Called by the GUI when the user picks fine-tune (True) or skip (False)."""
    global _origin_fine_tune_chosen
    _origin_fine_tune_chosen = chosen
    _origin_choice_event.set()

def _wait_for_origin_fine_tune(label):
    """Pause the routine so the user can fine-tune the current stage position.
    Resumes when the GUI calls confirm_origin_set()."""
    _origin_event.clear()
    print(f"[{label}] Fine-tune: adjust position with GUI controls, then click 'Set Origin' to continue.")
    if _origin_prompt_callback is not None:
        try:
            _origin_prompt_callback(label)
        except Exception as e:
            print(f"Warning: failed to show origin prompt: {e}")
    while not _origin_event.wait(0.1):
        _abort_if_emergency_stop()

def _maybe_fine_tune_origin(label):
    """Ask the user whether to fine-tune this origin or accept the current position.
    If they choose to fine-tune, waits for 'Set Origin' click.
    Either way the caller should save the final position afterward."""
    if _origin_ask_callback is None:
        print(f"[{label}] Warning: no origin ask callback registered — skipping fine-tune prompt.")
        return
    _origin_choice_event.clear()
    print(f"[{label}] Prompting: fine-tune or accept current position?")
    try:
        _origin_ask_callback(label)
    except Exception as e:
        print(f"Warning: failed to show origin ask prompt: {e}")
        return
    while not _origin_choice_event.wait(0.1):
        _abort_if_emergency_stop()
    if _origin_fine_tune_chosen:
        _wait_for_origin_fine_tune(label)

# Load persisted origins immediately so they are available before the first run.
_load_origins()

# BASIC MOVES  
# Use for testing

def up(length):
    move_linear_stage(z, '+', length, wait_for_stop=True, max_wait=30.0)

def down(length):
    move_linear_stage(z, '-', length, wait_for_stop=True, max_wait=30.0)

def left(length):
    move_linear_stage(x, '-', length, wait_for_stop=True, max_wait=30.0)

def right(length):
    move_linear_stage(x, '+', length, wait_for_stop=True, max_wait=30.0)

def front(length):
    move_linear_stage(y, '-', length, wait_for_stop=True, max_wait=30.0)

def back(length):
    move_linear_stage(y, '+', length, wait_for_stop=True, max_wait=30.0)

pad_types = {

    "cs": {"l": 0.7, "w": 0.2}, # Dimensions of cable conncetor short pads, mm

    "cl": {"l": 0.7, "w": 0.2}, # Dimensions of cable conncetor long pads, mm

    "me": {"l": 1.2, "w": 0.5}    # Dimensions of microelectrode pads, mm
    
}

# ---------------------------------------------------------------------------
# PARAMETRIC TRACE GENERATION
# ME pad pitch comes from the user popup (pad_spacing in pcb_settings.json).
# Connector pad pitch is fixed at ~650 µm (1 step ‰ˆ 1 µm on the linear stages).
# Trace horizontal runs are computed so every ME pad lands on its connector pad.
# ---------------------------------------------------------------------------
CONNECTOR_PITCH_UM = 650.0        # center-to-center X pitch of connector pad columns, µm
CONNECTOR_ROW_STAGGER_UM = 1200.0 # center-to-center Y offset between the two connector rows, µm
CONN_PAD_LEN_MM = 0.7             # connector pad length (must match pad_types cs/cl "l"), mm
FINAL_ENTRY_MM = 0.3              # short final straight run entering a cl pad, mm

# User-tunable routing values (set in the startup popup, stored in pcb_settings.json).
# These are fallback defaults if the settings file has no trace_tuning section.
CONNECTOR_CL_DROP_MM = 2.5        # trace run from ME pad row to the cl-row pad entry, mm
CORNER_MM = 0.5                   # 45° corner length used on routed cs traces, mm
CS_CLEAR_MM = 0.6                 # clearance of the inner (2/7) crossbar beyond the cs pad tops, mm
CS_LAYER_MM = 0.5                 # extra onion spacing for the outer (1/8) crossbar, mm

def get_trace_tuning():
    """Read user-tunable trace routing values from pcb_settings.json."""
    tuning = {
        "connector_cl_drop_mm": CONNECTOR_CL_DROP_MM,
        "corner_mm": CORNER_MM,
        "cs_clear_mm": CS_CLEAR_MM,
        "cs_layer_mm": CS_LAYER_MM,
    }
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                saved = json.load(f).get("trace_tuning", {})
            for key in tuning:
                if key in saved:
                    tuning[key] = float(saved[key])
    except Exception as e:
        print(f"Warning: could not read trace_tuning from {SETTINGS_FILE}: {e}")
    return tuning

# Default PNP-to-nozzle offsets (µm).  Override via Settings › PNP Offsets.
PNP_OFFSET_X = -29580
PNP_OFFSET_Y = -2300
PNP_OFFSET_Z =  4800

def get_pnp_offsets():
    """Read the PNP-to-nozzle axis offsets (µm) from pcb_settings.json.
    Returns a dict with keys 'X', 'Y', 'Z'.
    Defaults: X=-29580, Y=-2300, Z=4800."""
    defaults = {'X': PNP_OFFSET_X, 'Y': PNP_OFFSET_Y, 'Z': PNP_OFFSET_Z}
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                saved = json.load(f).get('pnp_offsets', {})
            return {k: int(saved[k]) if k in saved else v for k, v in defaults.items()}
    except Exception as e:
        print(f"Warning: could not read pnp_offsets from {SETTINGS_FILE}: {e}")
    return defaults

def goto_pnp_origin():
    """Navigate the PNP head to the center of the connector pad array.

    Target position = connector origin + PNP offsets (X, Y, Z).
    Rotation (r) is taken directly from the connector origin unchanged.

    Connector origin is determined in priority order:
      1. print_origin + connector_print_offset  (if both are saved)
      2. saved absolute connector_origin

    Move order: drop Z -5000 µm first (clearance) → move XY (and r) → move Z to final target.
    """
    reload_origins()

    # Prefer print_origin + connector_print_offset when both are available.
    if _connector_print_offset is not None and _print_origin_saved is not None:
        origin = {
            ax: _print_origin_saved[ax] + _connector_print_offset.get(ax, 0)
            for ax in ('X', 'Y', 'Z', 'r')
            if ax in _print_origin_saved
        }
        print(f"[PNP] Connector position derived from print origin + connector offset: {_connector_print_offset}")
    else:
        origin = _connector_origin_saved

    if not origin or 'X' not in origin or 'Y' not in origin:
        print("[PNP] No connector position available. Set the Connector origin or configure a connector offset from Print Origin in Settings.")
        return

    offsets = get_pnp_offsets()
    target_x = origin['X'] + offsets['X']
    target_y = origin['Y'] + offsets['Y']
    target_z = origin.get('Z')
    if target_z is not None:
        target_z = target_z + offsets['Z']

    print(f"[PNP] Connector origin: X={origin['X']:.1f}, Y={origin['Y']:.1f}, Z={origin.get('Z')}")
    print(f"[PNP] PNP offsets: {offsets}")
    print(f"[PNP] Target: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z}")

    def move_to(ax, target):
        cur = get_current_position(ax)
        if cur is None:
            print(f"[PNP] Axis {ax}: position unknown, skipping.")
            return
        diff = target - cur
        if abs(diff) < 0.5:
            return
        move_linear_stage(ax, '+' if diff >= 0 else '-', abs(diff),
                          wait_for_stop=True, max_wait=30.0)

    clear_emergency_stop()
    prev_speed = get_current_speed()
    try:
        update_speed(30)
        # 1) Drop Z by 5000 µm first to clear any obstacles before any XY movement
        _abort_if_emergency_stop()
        move_linear_stage('Z', '-', 5000, wait_for_stop=True, max_wait=30.0)
        # 2) Move XY (and r if saved)
        _abort_if_emergency_stop()
        move_to('X', target_x)
        _abort_if_emergency_stop()
        move_to('Y', target_y)
        if 'r' in origin:
            _abort_if_emergency_stop()
            move_to('r', origin['r'])
        # 3) Move Z to final target last
        if target_z is not None:
            _abort_if_emergency_stop()
            move_to('Z', target_z)
        print("[PNP] At PNP origin.")
    except RuntimeError as e:
        if str(e) == "Emergency stop requested.":
            print("[PNP] Stopped by emergency stop.")
        else:
            raise
    finally:
        update_speed(prev_speed)

# Connector grid, 4x2 with no gaps (all directions in PCB view, but note the
# stage moves the PCB under a still printhead, so machine moves are mirrored;
# the code below works in trace-space where "print direction" = away from ME row):
#   Top row    (cs, printed last along each trace): pads 1, 2, 7, 8
#   Bottom row (cl, nearest the ME pads):           pads 3, 4, 5, 6
# Columns (right -> left in PCB view): (2,3) (1,4) (8,5) (7,6)
# In machine trace-space X (units of CONNECTOR_PITCH_UM from array centerline):
CONNECTOR_X_COLUMNS = {2: -1.5, 3: -1.5, 1: -0.5, 4: -0.5,
                       8: +0.5, 5: +0.5, 7: +1.5, 6: +1.5}
CS_PADS = {1, 2, 7, 8}  # top row routed around the outside (onion style)

def get_pad_spacing_um():
    """Read the ME pad pitch the user entered in the startup popup."""
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                return float(json.load(f).get("pad_spacing", 1000.0))
    except Exception as e:
        print(f"Warning: could not read pad_spacing from {SETTINGS_FILE}: {e}")
    return 1000.0

def get_pad_count():
    """Read the pad count the user entered in the startup popup (clamped 1-8)."""
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                count = int(json.load(f).get("pad_count", 8))
            return max(1, min(8, count))
    except Exception as e:
        print(f"Warning: could not read pad_count from {SETTINGS_FILE}: {e}")
    return 8

def build_traces(me_pitch_um, connector_pitch_um=CONNECTOR_PITCH_UM):
    """Generate non-intersecting traces from ME pad n to connector pad n.

    cl pads (3,4,5,6 bottom row): near-vertical trace, small 45° jog to the
    column, entering the pad from below.

    cs pads (1,8 top row, outer): routed over the top of the grid — straight
    up at the ME pad's own X (outside the grid), 45° corner, a crossbar ABOVE
    the connector grid, 45° corner, then straight down into the pad from the
    top. With pads 2/7 no longer riding over the top, this crossbar sits at the
    base clearance height only (no extra onion layer).

    cs pads (2,7 top row, outermost columns): the ME pad sits outboard of the
    connector column, so the trace rises alongside the grid and enters the
    connector pad from its outer SIDE instead of routing over the top. This
    keeps those two traces shorter and uses less ink.
    """
    c_mm = connector_pitch_um / 1000.0
    stagger_mm = CONNECTOR_ROW_STAGGER_UM / 1000.0
    tuning = get_trace_tuning()
    cl_drop = tuning["connector_cl_drop_mm"]
    corner_len = tuning["corner_mm"]
    cs_clear = tuning["cs_clear_mm"]
    corner_v = corner_len / math.sqrt(2)    # x = y component of a 45° corner
    d_cl = cl_drop                          # cl pad entry (bottom edge)
    d_cs = d_cl + stagger_mm + CONN_PAD_LEN_MM  # cs pad entry (top edge)

    traces_out = {}
    for n in range(1, 9):
        me_x = (n - 4.5) * me_pitch_um / 1000.0
        col_x = CONNECTOR_X_COLUMNS[n] * c_mm
        dx = col_x - me_x
        shift = abs(dx)
        inward_pos = dx > 0  # True => shift toward machine +x
        seg = {}
        idx = 1

        def add(angle, length):
            nonlocal idx
            if length > 1e-6:
                seg[f"a{idx}"] = angle
                seg[f"l{idx}"] = length
                idx += 1

        if n in CS_PADS:
            pad_w = pad_types["cs"]["w"]
            grid_edge = 1.5 * c_mm + 0.3
            if shift > 1e-6 and abs(me_x) < grid_edge:
                print(f"Warning: trace {n} riser at {me_x:.3f} mm is inside the "
                      f"connector grid ({grid_edge:.3f} mm); increase ME pitch.")

            if n in (2, 7):
                # SIDE ENTRY: outermost columns. Rise alongside the grid and
                # enter the connector pad from its outer side (shorter, less ink).
                center_h = d_cs - CONN_PAD_LEN_MM / 2.0   # cs pad vertical center
                up_corner = 135 if inward_pos else 45
                horiz = 180 if inward_pos else 0
                h_total = shift - pad_w / 2.0             # stop at the pad's outer edge
                if h_total < 0.0:
                    h_total = 0.0

                if h_total <= corner_v:
                    # Column nearly under the ME pad: single diagonal into the side
                    add(90.0, center_h - h_total)
                    add(up_corner, h_total * math.sqrt(2))
                else:
                    add(90.0, center_h - corner_v)
                    add(up_corner, corner_len)
                    add(horiz, h_total - corner_v)
            else:
                # Routed trace over the top of the grid (pads 1 and 8).
                # With pads 2/7 now entering from the side, nothing else rides
                # over the top, so 1/8 only need the base clearance above the cs
                # pads (no extra onion layer) — a lower, shorter crossbar.
                H = d_cs + cs_clear
                up_corner = 135 if inward_pos else 45
                horiz = 180 if inward_pos else 0
                down_corner = 225 if inward_pos else 315

                if shift <= 2 * corner_v:
                    # Columns nearly aligned: two shortened corners, no crossbar
                    cv = shift / 2.0
                    add(90.0, H - cv)
                    add(up_corner, cv * math.sqrt(2))
                    add(down_corner, cv * math.sqrt(2))
                    add(270.0, (H - cv) - d_cs)
                else:
                    add(90.0, H - corner_v)
                    add(up_corner, corner_len)
                    add(horiz, shift - 2 * corner_v)
                    add(down_corner, corner_len)
                    add(270.0, (H - corner_v) - d_cs)
        else:
            # Near-vertical cl trace
            diag_v = 0.0
            tail = []
            if shift > 1e-6:
                diag_angle = 135 if inward_pos else 45
                horiz_angle = 180 if inward_pos else 0
                if shift <= corner_v:
                    tail.append((diag_angle, shift * math.sqrt(2)))
                    diag_v = shift
                else:
                    tail.append((diag_angle, corner_len))
                    tail.append((horiz_angle, shift - corner_v))
                    diag_v = corner_v
            l1 = d_cl - diag_v - FINAL_ENTRY_MM
            if l1 < 0.1:
                print(f"Warning: trace {n} vertical clamped ({l1:.3f} mm); "
                      f"increase CONNECTOR_CL_DROP_MM.")
                l1 = 0.1
            add(90.0, l1)
            for a, l in tail:
                add(a, l)
            add(90.0, FINAL_ENTRY_MM)

        traces_out[n] = seg
    return traces_out

# Connector pad printed at the end of each trace: (pad_type, position)
# Position 9  = trace arrives from below (cl row), pad extends onward/up.
# Position 10 = trace arrives from above (cs row), pad extends back/down.
# Position 11 = trace arrives from the LEFT side (pad 2), pad extends inward/right.
# Position 12 = trace arrives from the RIGHT side (pad 7), pad extends inward/left.
PAD_SEQUENCE = [
    ("cs", 10), ("cs", 11), ("cl", 9), ("cl", 9),
    ("cl", 9), ("cl", 9), ("cs", 12), ("cs", 10),
]

def print_pcb():
    global x_coord, y_coord, counter

    counter = 0  # Always start at feature 1 regardless of any previous run

    update_speed(1)

    _abort_if_emergency_stop()
    x_coord, y_coord = get_current_position(x), get_current_position(y)

    me_pitch_um = get_pad_spacing_um()
    pad_count = get_pad_count()
    active_traces = build_traces(me_pitch_um)
    print(f"[print_pcb] Printing {pad_count} pad(s) | ME pitch: {me_pitch_um:.0f} µm "
          f"| connector pitch: {CONNECTOR_PITCH_UM:.0f} µm")
    for n in range(1, pad_count + 1):
        print(f"[print_pcb] trace {n}: {active_traces[n]}")

    for n, (pad_type, position) in enumerate(PAD_SEQUENCE[:pad_count], start=1):
        _abort_if_emergency_stop()
        print_pad(pad_types, "me", 8)
        print_trace(active_traces, n)
        print_pad(pad_types, pad_type, position)

        if n < pad_count:
            counter += 1
            advance_to_next_feature(counter, x_coord, y_coord, me_pitch_um)

    update_speed(50)
    down(10000)

# Component identifiers accepted by reprint_feature().
REPRINT_COMPONENTS = ("me_pad", "trace", "connector_pad", "trace_and_connector", "full")

def reprint_feature(index, component="full"):
    """Reprint a single feature (or one of its components) at the CURRENT head
    position, without stepping between features.

    Jog the printhead to the START of the selected component before calling:
      * 'me_pad'             — the microelectrode pad start point
      * 'trace'              — where the trace leaves the ME pad
      * 'connector_pad'      — the connector-pad entry point
      * 'trace_and_connector'— trace start (prints trace → connector pad)
      * 'full'               — the ME-pad start point (prints ME pad → trace → pad)

    index:     connector feature number 1..8 (selects the trace path and the
               connector-pad style). Ignored for a lone ME pad.
    component: one of REPRINT_COMPONENTS.
    """
    if component not in REPRINT_COMPONENTS:
        print(f"[reprint] Unknown component '{component}'.")
        return
    if not (1 <= index <= 8):
        print(f"[reprint] Feature index {index} out of range (1-8).")
        return

    clear_emergency_stop()
    update_speed(1)

    me_pitch_um = get_pad_spacing_um()
    active_traces = build_traces(me_pitch_um)
    pad_type, position = PAD_SEQUENCE[index - 1]

    print(f"[reprint] Reprinting feature {index}, component '{component}'")
    try:
        _abort_if_emergency_stop()
        if component in ("me_pad", "full"):
            print_pad(pad_types, "me", 8)
        if component in ("trace", "full", "trace_and_connector"):
            _abort_if_emergency_stop()
            print_trace(active_traces, index)
        if component in ("connector_pad", "full", "trace_and_connector"):
            _abort_if_emergency_stop()
            print_pad(pad_types, pad_type, position)
    except RuntimeError as e:
        if str(e) == "Emergency stop requested.":
            print("[reprint] Stopped by emergency stop.")
        else:
            raise
    finally:
        nordson_off()
        update_speed(50)
    print(f"[reprint] Finished feature {index}, component '{component}'")

    # Lower the PCB (effectively lifting the printhead clear) after reprinting.
    update_speed(50)
    down(8000)

# Z clearance (in stage units) held above the print height while jogging XY, so
# the pen never drags across the PCB before touching down.
JOG_Z_CLEARANCE_UM = 1000

def _jog_to_print_point(index, y_offset_um, label):
    """Jog to a print point for feature `index` (feature 1 = saved 'print'
    origin; feature n at origin_X + (n-1)*pad_spacing). `y_offset_um` shifts Y
    from the feature start (negative = front/-y). XY (and rotation) move with the
    pen held a fixed clearance above the print height, then Z touches down LAST.
    """
    if not (1 <= index <= 8):
        print(f"[jog] Feature index {index} out of range (1-8).")
        return
    reload_origins()  # pick up any origin the user just saved in the GUI
    origin = _print_origin_saved
    if not origin or 'X' not in origin or 'Y' not in origin:
        print("[jog] No saved 'print' origin. Set the Print origin first.")
        return

    clear_emergency_stop()
    me_pitch_um = get_pad_spacing_um()
    target_x = origin['X'] + (index - 1) * me_pitch_um
    target_y = origin['Y'] + y_offset_um
    print_z = origin.get('Z')

    def move_axis_to(ax, target):
        cur = get_current_position(ax)
        if cur is None:
            print(f"[jog] Axis {ax}: position unknown, skipping.")
            return
        diff = target - cur
        if abs(diff) < 0.5:
            return
        direction = '+' if diff >= 0 else '-'
        move_linear_stage(ax, direction, abs(diff), wait_for_stop=True, max_wait=30.0)

    print(f"[jog] Jogging to {label} for feature {index} "
          f"(X={target_x:.1f}, Y={target_y:.1f})")
    try:
        update_speed(30)
        # 1) Retract Z to a safe clearance below the print height (pen off the board)
        if print_z is not None:
            _abort_if_emergency_stop()
            move_axis_to(z, print_z - JOG_Z_CLEARANCE_UM)
        # 2) Position XY (and rotation) with the pen clear of the board
        _abort_if_emergency_stop()
        move_axis_to(x, target_x)
        _abort_if_emergency_stop()
        move_axis_to(y, target_y)
        if 'r' in origin:
            _abort_if_emergency_stop()
            move_axis_to('r', origin['r'])
        # 3) Touch down: lower Z onto the board LAST
        if print_z is not None:
            _abort_if_emergency_stop()
            move_axis_to(z, print_z)
        print(f"[jog] At {label} for feature {index}.")
    except RuntimeError as e:
        if str(e) == "Emergency stop requested.":
            print("[jog] Stopped by emergency stop.")
        else:
            raise
    finally:
        update_speed(50)

def jog_to_feature_start(index):
    """Automatically jog to feature `index`'s start (the ME-pad start point),
    then touch down by lowering Z to the print height LAST."""
    _jog_to_print_point(index, 0.0, "feature start")

def jog_to_trace_start(index):
    """Automatically jog to feature `index`'s trace start — where the trace
    leaves the ME pad, a quarter of the ME pad height 'up' (front / -y) from the
    feature start — then touch down by lowering Z LAST."""
    y_offset_um = -mm_to_um(pad_types["me"]["l"] / 4.0)  # front (-y) by 1/4 ME height
    _jog_to_print_point(index, y_offset_um, "trace start")

def jog_to_connector_pad(index):
    """Jog to the connector pad entry point for feature `index` (1-8), then
    touch down by lowering Z LAST. Useful for reprinting a connector pad on its
    own, or for inspecting the connector pad area.

    Connector pad positions (machine coords from saved print origin):
      X = origin_X + 3.5 * me_pitch + CONNECTOR_X_COLUMNS[n] * CONNECTOR_PITCH
      Y = origin_Y - d_cl        (cl row, pads 3-6)
        = origin_Y - d_cs        (cs top-over row, pads 1 & 8)
        = origin_Y - center_h    (cs side-entry, pads 2 & 7)
    """
    if not (1 <= index <= 8):
        print(f"[jog] Feature index {index} out of range (1-8).")
        return
    reload_origins()
    origin = _print_origin_saved
    if not origin or 'X' not in origin or 'Y' not in origin:
        print("[jog] No saved 'print' origin. Set the Print origin first.")
        return

    clear_emergency_stop()
    me_pitch_um = get_pad_spacing_um()
    tuning = get_trace_tuning()
    cl_drop = tuning["connector_cl_drop_mm"]
    stagger_mm = CONNECTOR_ROW_STAGGER_UM / 1000.0
    d_cl = cl_drop
    d_cs = d_cl + stagger_mm + CONN_PAD_LEN_MM

    # X position: independent of which ME pad column the trace starts at
    target_x = origin['X'] + 3.5 * me_pitch_um + CONNECTOR_X_COLUMNS[index] * CONNECTOR_PITCH_UM

    # Y position: depends on pad row
    if index in (2, 7):          # cs side-entry pads
        center_h = d_cs - CONN_PAD_LEN_MM / 2.0
        y_offset_um = -mm_to_um(center_h)
    elif index in CS_PADS:       # cs top-over pads (1, 8)
        y_offset_um = -mm_to_um(d_cs)
    else:                        # cl row pads (3, 4, 5, 6)
        y_offset_um = -mm_to_um(d_cl)

    target_y = origin['Y'] + y_offset_um
    print_z = origin.get('Z')

    def move_axis_to(ax, target):
        cur = get_current_position(ax)
        if cur is None:
            print(f"[jog] Axis {ax}: position unknown, skipping.")
            return
        diff = target - cur
        if abs(diff) < 0.5:
            return
        direction = '+' if diff >= 0 else '-'
        move_linear_stage(ax, direction, abs(diff), wait_for_stop=True, max_wait=30.0)

    label = f"connector pad entry (feature {index})"
    print(f"[jog] Jogging to {label} (X={target_x:.1f}, Y={target_y:.1f})")
    try:
        update_speed(30)
        # 1) Retract Z to safe clearance
        if print_z is not None:
            _abort_if_emergency_stop()
            move_axis_to(z, print_z - JOG_Z_CLEARANCE_UM)
        # 2) Move XY with pen clear of board
        _abort_if_emergency_stop()
        move_axis_to(x, target_x)
        _abort_if_emergency_stop()
        move_axis_to(y, target_y)
        if 'r' in origin:
            _abort_if_emergency_stop()
            move_axis_to('r', origin['r'])
        # 3) Touch down Z last
        if print_z is not None:
            _abort_if_emergency_stop()
            move_axis_to(z, print_z)
        print(f"[jog] At {label}.")
    except RuntimeError as e:
        if str(e) == "Emergency stop requested.":
            print("[jog] Stopped by emergency stop.")
        else:
            raise
    finally:
        update_speed(50)

# Don't modify - Phillipe's edit
def print_trace(trace_dict, index):
    global counter, angle_dir, angle_axis, t_len

    # Reset trace-state globals so a previous interrupted run never bleeds into
    # this one.  Without this, a stale t_len causes the first angle to fire
    # immediately with the wrong length, shifting every subsequent segment.
    angle_dir, angle_axis, t_len = None, None, None

    for key, value in (trace_dict.get(index)).items():
        _abort_if_emergency_stop()
        if key.find("a") != -1:
                angle = value
                angle_handler(angle)

        if key.find("l") != -1:
            t_len = mm_to_um(value)
            
        if (t_len != None) & (angle_dir != None):
            
            if angle_axis.find('d') == -1:
                nordson_on()
                update_speed(1) #ORIGINAL WAS 20, adjust for better print quality/speed tradeoff
                move_linear_stage(angle_axis, angle_dir, t_len, wait_for_stop=True, max_wait=30.0)
                
                angle_dir, angle_axis, t_len = None, None, None

            elif angle_axis.find('d') != -1:             
                diagonal_handler(angle, t_len, 3)
                
                angle_dir, angle_axis, t_len = None, None, None       

def angle_handler(angle):
    global angle_axis, angle_dir

    angle_axis = None
    angle_dir = None
    
    if angle == 0:
        angle_axis = x
        angle_dir = '-'
    elif 0 < angle < 90:
        # Q1
        angle_axis = 'd(+x,+y)'
        angle_dir = ('-', '-')
    elif angle == 90:
        angle_axis = y
        angle_dir = '-'
    elif 90 < angle < 180:
        # Q2
        angle_axis = 'd(-x,+y)'
        angle_dir = ('+', '-')  
    elif angle == 180:
        angle_axis = x
        angle_dir = '+'
    elif 180 < angle < 270:
        # Q3
        angle_axis = 'd(-x,-y)'
        angle_dir = ('+', '+')
    elif angle == 270:
        angle_axis = y
        angle_dir = '+'
    elif 270 < angle < 360:
        # Q4
        angle_axis = 'd(+x,-y)'
        angle_dir = ('-', '+')
    elif angle == 360:
        angle_axis = x
        angle_dir = '-'

def diagonal_handler(angle, t_len, div):
    # Convert angle to radians

    nordson_on() # Originally was off

    theta = math.radians(abs(angle))

    # Calculate dx and dy based on the angle
    dx = t_len * math.cos(theta)
    dy = t_len * math.sin(theta)

    xstp = round(abs(dx / div))
    ystp = round(abs(dy / div))
    update_speed(1)

    if (angle_dir[0] != None) & (angle_dir[1] != None):

        for i in range(div):
            _abort_if_emergency_stop()
            move_linear_stage(x, angle_dir[0], xstp, wait_for_stop=True, max_wait=30.0)
            move_linear_stage(y, angle_dir[1], ystp, wait_for_stop=True, max_wait=30.0)
            time.sleep(0.001)
            
def print_pad(pad_dict, pad_type, position):
    
    global x_coord, y_coord, counter

    nordson_on()
    pad_handler(pad_dict, pad_type, position)
    #nordson_off()
    
def pad_handler(pad_dict, pad_type, position):

    length = mm_to_um(pad_dict.get(pad_type).get("l"))
    width = mm_to_um(pad_dict.get(pad_type).get("w"))
    
    update_speed(1)

    if position == 8:
        # "me" pad start at center (half width, 3/4 down from top = 1/4 from bottom)
        # Path: up 3/4 h †’ right w/4 †’ down h †’ left w †’ up h
        back(length * 3 / 4)   # down 3/4 height †’ top center
        right(width / 4)       # left half width †’ top-right corner
        front(length)          # up full height †’ bottom-right corner
        right(width / 4)       # left half width †’ top-right corner
        back(length)           # down full height †’ top-left corner
        left(width)            # right full width †’ bottom-left corner
        front(length)          # up full height †’ top-left corner
        right(width / 4)       # left half width †’ top center
        back(length)           # down full height †’ bottom center
        right(width / 4)       # left half width †’ top center
        front(length)          # up full height †’ top-left corner


    elif position == 9:
        # Connector pad trace arrives from BELOW (cl row), pad extends onward.
        right(width / 2)       # †’ entry-side corner
        front(length)          # †’ far corner (direction of travel)
        left(width)            # †’ far corner, other side
        back(length)           # †’ entry-side corner, other side
        right(width / 2)       # †’ back to entry center

    elif position == 10:
        # Connector pad trace arrives from ABOVE (cs row), pad extends back
        # toward the ME row (opposite the arrival direction).
        right(width / 2)       # †’ entry-side corner
        back(length)           # †’ far corner (opposite direction of travel)
        left(width)            # †’ far corner, other side
        front(length)          # †’ entry-side corner, other side
        right(width / 2)       # †’ back to entry center

    elif position == 11:
        # Connector pad trace arrives from the LEFT side (pad 2). Pen enters at
        # the left-edge center; pad body extends inward (to the right).
        back(length / 2)       # †’ top-left corner
        right(width)           # †’ top-right corner
        front(length)          # †’ bottom-right corner
        left(width)            # †’ bottom-left corner
        back(length / 2)       # †’ back to left-edge center (entry)

    elif position == 12:
        # Connector pad trace arrives from the RIGHT side (pad 7). Pen enters at
        # the right-edge center; pad body extends inward (to the left).
        back(length / 2)       # †’ top-right corner
        left(width)            # †’ top-left corner
        front(length)          # †’ bottom-left corner
        right(width)           # †’ bottom-right corner
        back(length / 2)       # †’ back to right-edge center (entry)

def advance_to_next_feature(num, xx, yy, spacing):
    """Move directly from current position to the next feature start.
    Computes a single displacement to (xx + num*spacing, yy) instead of
    returning to origin first and then stepping right."""
    nordson_off()

    update_speed(50)
    down(1000)

    x_target = xx + num * spacing
    y_target = yy
    cur_x = get_current_position(x)
    cur_y = get_current_position(y)

    xdisp = x_target - cur_x
    ydisp = y_target - cur_y

    # Settle Y back to the origin row first, then advance X to the target column.
    if ydisp > 0:
        back(abs(ydisp))
    elif ydisp < 0:
        front(abs(ydisp))

    if xdisp > 0:
        right(abs(xdisp))
    elif xdisp < 0:
        left(abs(xdisp))

    print(f"Moving to next feature {num}")
    up(1000)
    stop_motor_control()

def get_coord():
    global x_coord, y_coord, z_coord

    x_coord = get_current_position(x)
    y_coord = get_current_position(y)
    z_coord = get_current_position(z)
    r_coord = get_current_position('r')

    print("X location: " + str(x_coord))
    print("Y_location: " + str(y_coord))
    print("Z_location: " + str(z_coord))
    print("r_location: " + str(r_coord))

# Don't modify - Phillipe's edit  
def Z_probe():
    global pcb_z_coord
    prev_speed = get_current_speed()
    _abort_if_emergency_stop()
    time.sleep(0.5)  # Wait for any vibrations to settle before probing
    update_speed(1)
    # Run the fine approach asynchronously so Z_calibrate() can stop on contact.
    move_linear_stage('Z', '+', 8000, wait_for_stop=False, max_wait=30.0)
    state = Z_calibrate()

    if state is None:
        update_speed(prev_speed)
        if is_emergency_stop_requested():
            raise RuntimeError("Emergency stop requested.")
        print("Warning: Z calibration did not return a limit state.")
        return False

    z_pos = get_current_position("Z")
    if z_pos is None:
        _sleep_with_abort(0.2)
        z_pos = get_current_position("Z")

    if z_pos is None:
        update_speed(prev_speed)
        print("Warning: Could not read Z position after Z_probe; keeping previous pcb_z_coord.")
        return False

    pcb_z_coord = z_pos
    if state == "Z limit":
        print(pcb_z_coord)
    else:
        print(f"Warning: Z calibration returned {state!r}; using current Z position {pcb_z_coord} as pcb_z_coord.")

    update_speed(prev_speed)

    return True
        
# Don't modify - Phillipe's edit
def r_limit():
    update_speed(3)
    move_linear_stage('r', '+', 100, wait_for_stop=False, max_wait=30.0)
    state = r_calibrate()
    if state == "R limit":
        rot = get_current_position("r")
        print(rot)

def r_corrector():  
    _abort_if_emergency_stop()
    r_limit()
    _sleep_with_abort(1.0)
    update_speed(30)
    move_linear_stage('r', '-', 7, wait_for_stop=False, max_wait=30.0)

def x_home():
    global print_home

    update_speed(100)
    move_linear_stage(x, '+', 60000, wait_for_stop=False, max_wait=30.0)

    if wait_for_axis_stop(x) != True:
        print_home[0] = get_current_position(x)
        print(f"X home position set at {print_home[0]}") 

def y_home():
    global print_home

    update_speed(100)
    move_linear_stage(y, '+', 60000, wait_for_stop=False, max_wait=30.0)

    if wait_for_axis_stop(y) != True:
        print_home[1] = get_current_position(y)
        print(f"Y home position set at {print_home[1]}")
    
def z_home():
    global print_home

    update_speed(100)
    move_linear_stage(z, '-', 60000, wait_for_stop=False, max_wait=30.0)

    if wait_for_axis_stop(z) != True:
        print_home[2] = get_current_position(z)
        print(f"Z home position set at {print_home[2]}")    
    
def print_origin():
    global _print_origin_saved
    if _print_origin_saved:
        # Navigate directly to the last confirmed print origin.
        print("Navigating to saved print origin...")
        _navigate_to_saved_origin(_print_origin_saved, ['X', 'Y', 'r', 'Z'])
    else:
        # First run: use default relative offsets from the probe position.
        _sleep_with_abort(1.0)
        _abort_if_emergency_stop()
        move_linear_stage(x, '+', 3500, wait_for_stop=True, max_wait=30.0)
        _sleep_with_abort(1.0)
        _abort_if_emergency_stop()
        move_linear_stage(y, '+', 1600, wait_for_stop=True, max_wait=30.0)
        _sleep_with_abort(1.0)
        _abort_if_emergency_stop()
        move_linear_stage(z, '+', 1300, wait_for_stop=True, max_wait=30.0)
    # Always give the user the option to verify / fine-tune, then save.
    _maybe_fine_tune_origin("Print Origin")
    pos = {ax: get_current_position(ax) for ax in ('X', 'Y', 'Z', 'r')}
    _print_origin_saved = {k: v for k, v in pos.items() if v is not None}
    _save_origin('print_origin_coords', _print_origin_saved)
    get_coord()

def calibrate():
    global pcb_z_coord, _probe_origin

    _abort_if_emergency_stop()
    update_speed(100)
    z_home()
    _abort_if_emergency_stop()
    y_home()
    _abort_if_emergency_stop()
    x_home()

    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()

    if _probe_origin:
        # Navigate directly to the last confirmed probe origin (X, Y, Z, r).
        print("Navigating to saved probe origin...")
        _navigate_to_saved_origin(_probe_origin, ['X', 'Y', 'r', 'Z'])
    else:
        # First run: use default relative offsets from mechanical home.
        move_linear_stage(x, '-', 27000, wait_for_stop=True, max_wait=30.0)
        _sleep_with_abort(1.0)
        _abort_if_emergency_stop()
        move_linear_stage(y, '-', 13500, wait_for_stop=True, max_wait=30.0)

    _abort_if_emergency_stop()
    r_corrector()

    update_speed(100)
    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    pcb_z_coord = None
    if not Z_probe():
        raise RuntimeError("Z probing failed: unable to read Z position.")

    _abort_if_emergency_stop()
    update_speed(100)
    down(1000)

    # Full calibrate routine is done. Now let the user optionally fine-tune
    # the probe X, Y position and save it for the next run.
    _maybe_fine_tune_origin("Probe Origin")
    pos = {ax: get_current_position(ax) for ax in ('X', 'Y', 'Z', 'r')}
    _probe_origin = {k: v for k, v in pos.items() if v is not None}
    _save_origin('probe_origin', _probe_origin)

# Add code into function to test it using the gui "Print tester" button
def print_tester():
    servo_to(71)
    solenoid_relay_on()
    time.sleep(5.0)
    servo_to(15)
    time.sleep(0.2)
    goto_pnp_origin()
    time.sleep(0.2)
    solenoid_relay_off()
    time.sleep(10)
          
# GLUE DROP & SEQUENCE
def glue_drop():
    """Dispense glue then retract slightly to stop drip."""
    motor_backward(steps=5.3)  # Dispense glue, adjust steps as needed for desired drop size
    time.sleep(4.0)  # Wait for glue to dispense
    motor_forward(steps=5.0)  # Retract to prevent dripping
    print("Glue drop complete.")
    motor_release()

def glue_sequence():
    """Glue drop sequence at 1000µm intervals left up to 8000µm."""
    print("Starting glue sequence...")
    update_speed(50)
    
    for i in range(8):  # 0, 1000, 2000 ... 7000
        _abort_if_emergency_stop()
        if i > 0:
            right(1000)
        up(3000)
        glue_drop()
        down(3000)
    
    # return_to_origin()
    print("Glue sequence complete.")

# Full assembly sequence
def sacrificial_print():
    """Print a 1 x 1 mm square at the current position to prime the nozzle
    before moving to the actual print origin."""
    sac = mm_to_um(1.0)  # 1 mm square for sacrificial print
    print("--- Sacrificial print (1 x 1 mm) ---")
    update_speed(1)
    nordson_off()
    _sleep_with_abort(delay)
    right(sac/2)
    back(sac/2)
    left(sac)
    front(sac)
    right(sac)
    nordson_off()
    update_speed(30)
    down(1000)
    print("--- Sacrificial print complete ---")

def microwire_origin_setup():
    """Navigate to the saved microwire / laser-alignment origin, pause
    for the user to fine-tune, then persist the confirmed position.
    Call this once at the start of the wire-placement routine so every
    subsequent run starts from the last confirmed position."""
    global _microwire_origin_saved

    if _microwire_origin_saved:
        print("Navigating to saved microwire origin...")
        _navigate_to_saved_origin(_microwire_origin_saved, ['X', 'Y', 'Z', 'r'])
    # If no saved origin the machine stays wherever it is; user jogs manually.

    _maybe_fine_tune_origin("Microwire Origin")
    pos = {ax: get_current_position(ax) for ax in ('X', 'Y', 'Z', 'r')}
    _microwire_origin_saved = {k: v for k, v in pos.items() if v is not None}
    _save_origin('microwire_origin', _microwire_origin_saved)

def full_sequence(run_calibration=True):
    global _has_calibrated
    clear_emergency_stop()
    # Flush the PC-side serial buffer so any bytes left over from a previous
    # stop or motor-release don't get interpreted as the start of a new command
    # by the controller, which would cause the sacrificial print to skip steps.
    flush_serial()
    laser_relay_off()
    nordson_off()

    try:
        _abort_if_emergency_stop()
        if run_calibration:
            if not _has_calibrated:
                calibrate()
                _has_calibrated = True
                print("Calibration complete. Subsequent runs will skip calibration.")
            else:
                print("Skipping calibration (already calibrated this session).")
        else:
            print("Calibration skipped by user.")
            # Navigate to probe origin so the sacrificial print is in the right place
            if _probe_origin:
                print("Navigating to probe origin for sacrificial print...")
                _navigate_to_saved_origin(_probe_origin, ['X', 'Y', 'r', 'Z'])
        _abort_if_emergency_stop()
        sacrificial_print()
        _abort_if_emergency_stop()
        print_origin()
        print_pcb()
        _abort_if_emergency_stop()
        #fill_electrode_pads()
        _abort_if_emergency_stop()
    finally:
        try:
            laser_relay_off()
            nordson_off()
        except Exception as e:
            print(f"Warning: failed to force outputs off after full_sequence: {e}")