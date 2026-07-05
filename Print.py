import time
import math
import threading
import json
import os
from motor_control import (
    move_linear_stage, 
    update_speed, 
    get_current_speed,
    return_to_origin, 
    stop_motor_control, 
    get_current_position,
    wait_for_axis_stop,
    µm_to_steps, 
    mm_to_um,
    base_displacement,
    clear_emergency_stop,
    is_emergency_stop_requested,
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
    mag_detector
)   

#parameters for line test

x = 'X'
y = 'Y'
z = 'Z'

tapl = 5000.0   # Z tap depth 
stp  = 1000.0   # Step-over between parallel lines
delay = 0.5       # Dispenser settle time

x_coord, y_coord, z_coord = None, None, None # Current coordinates
angle_dir, angle_axis, t_len = None, None, None# Angle based direction and axis, trace length
counter = 0 # Feature counter for next feature calculation
temp_location = None
temp_l = None
temp_w = None

#print_z_coord = probe_z_coord - print_gap # Z coordinate for printing, set after probing based on print_gap
wipe_y = 2123.0 # Y Position for testing, replace with actual wipe position
probe_y = 2342.0 #  Y Position for testing, replace with actual probe position
print_home = [0, 0, 0, 0] # X, Y, Z, R coordinate for starting point for print process
pcb_z_coord = None # Z coordinate for printing, set after probing
_has_calibrated = False  # Set True after first successful calibrate(); skipped on reruns

# ---------------------------------------------------------------------------
# Three-origin system: probe, print, microwire
# Each is persisted to pcb_settings.json so every confirmed position becomes
# the starting point for the next run, making the system more accurate over time.
# ---------------------------------------------------------------------------
SETTINGS_FILE = "pcb_settings.json"

# Saved origin dicts: None means "no saved value yet, use first-run defaults"
_probe_origin    = None  # {'X': ..., 'Y': ...}
_print_origin_saved   = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...}
_microwire_origin_saved = None  # {'X': ..., 'Y': ..., 'Z': ..., 'r': ...}

def _load_origins():
    """Load the three saved origins from pcb_settings.json at startup."""
    global _probe_origin, _print_origin_saved, _microwire_origin_saved
    if not os.path.isfile(SETTINGS_FILE):
        return
    try:
        with open(SETTINGS_FILE, 'r') as f:
            data = json.load(f)
        _probe_origin          = data.get('probe_origin')    or None
        _print_origin_saved    = data.get('print_origin_coords') or None
        _microwire_origin_saved = data.get('microwire_origin') or None
        print(f"Origins loaded — probe: {_probe_origin}, print: {_print_origin_saved}, microwire: {_microwire_origin_saved}")
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
    """Reload all three saved origins from pcb_settings.json into memory.
    Call this after the GUI saves a new origin so in-session routines see it."""
    _load_origins()

def _navigate_to_saved_origin(origin_dict, axes):
    """Move each listed axis to its saved absolute position."""
    for ax in axes:
        val = origin_dict.get(ax)
        if val is None:
            continue
        _abort_if_emergency_stop()
        cur = get_current_position(ax)
        if cur is None:
            continue
        diff = val - cur
        if abs(diff) < 0.5:
            continue
        move_linear_stage(ax, '+' if diff >= 0 else '-', abs(diff),
                          wait_for_stop=True, max_wait=30.0)

# Event and callback used by all three fine-tune pauses
_origin_event = threading.Event()
_origin_prompt_callback = None  # callable(label: str) — registered by the GUI

# Opt-in choice: user decides whether to fine-tune or accept current position.
_origin_choice_event    = threading.Event()
_origin_fine_tune_chosen = False
_origin_ask_callback    = None  # callable(label: str) — shows the Yes/No dialog

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
    _origin_choice_event.clear()
    print(f"[{label}] Prompting: fine-tune or accept current position?")
    if _origin_ask_callback is not None:
        try:
            _origin_ask_callback(label)
        except Exception as e:
            print(f"Warning: failed to show origin ask prompt: {e}")
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

def tap():
    down(tapl)
    up(tapl)

# Trace Dictionay, Don't modify - Phillipe's edit
# If angle is negative line has negative slope, else positive slope
# Lengths are in mm, use conversion method
traces = {

   1: {"a1": 90.0, "l1": 4.17, "a2": 135, "l2": 0.5, "a3": 180, "l3": 2.47, "a4": 225, "l4": 0.5}, # Outermost trace to the right

   2: {"a1": 90.0, "l1": 3.44, "a2": 135, "l2": 0.5 , "a3": 180, "l3": 1.0},

   3: {"a1": 90.0, "l1": 2.28, "a2": 135, "l2": 0.5},

   4: {"a1": 90.0, "l1": 2.35},
   
   5: {"a1": 90.0, "l1": 2.35},

   6: {"a1": 90.0, "l1": 2.28, "a2": 45, "l2": 0.5},

   7: {"a1": 90.0, "l1": 3.44, "a2": 45, "l2": 0.5 , "a3": 0, "l3": 1.0},

   8: {"a1": 90.0, "l1": 4.17, "a2": 45, "l2": 0.5, "a3": 0, "l3": 2.47, "a4": 315, "l4": 0.5}

}

"""traces_1 = {

   1: {"a1": 90.0, "l1": 4.17, "a2": 135, "l2": 0.5, "a3": 180, "l3": 2.47, "a4": 225, "l4": 0.5}, # Outermost trace to the right

   2: {"a1": 90.0, "l1": 3.44, "a2": 135, "l2": 0.5 , "a3": 180, "l3": 1.0},

   3: {"a1": 90.0, "l1": 2.28, "a2": 135, "l2": 0.5},

   4: {"a1": 90.0, "l1": 2.13},
   
   5: {"a1": 90.0, "l1": 2.13},

   6: {"a1": 90.0, "l1": 2.28, "a2": 45, "l2": 0.5},

   7: {"a1": 90.0, "l1": 3.44, "a2": 45, "l2": 0.5 , "a3": 0, "l3": 1.0},

   8: {"a1": 90.0, "l1": 4.17, "a2": 45, "l2": 0.5, "a3": 0, "l3": 2.47, "a4": 315, "l4": 0.5}

}"""


pad_types = {

    "cs": {"l": 0.7, "w": 0.2}, # Dimensions of cable conncetor short pads, mm

    "cl": {"l": 0.7, "w": 0.2}, # Dimensions of cable conncetor long pads, mm

    "me": {"l": 1.2, "w": 0.5}    # Dimensions of microelectrode pads, mm
    
}

"""pad_types_1 = {

    "cs": {"l": 0.75, "w": 0.38}, # Dimensions of cable conncetor short pads, mm

    "cl": {"l": 1, "w": 0.38}, # Dimensions of cable conncetor long pads, mm

    "me": {"l": 1.2, "w": 0.55}    # Dimensions of electrode pads, mm
    
}"""

"""pads = {
    
    1: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 4, "e": 4}}, 
    
    2: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 2, "e": 2}},
    
    3: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 2, "e": 2}},
    
    4: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 1, "e": 1}},
    
    5: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 7, "e": 7}},
    
    6: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 6, "e": 6}},
    
    7: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 6, "e": 6}},
    
    8: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 7, "e": 7}}

}"""

# ---------------------------------------------------------------------------
# PARAMETRIC TRACE GENERATION
# ME pad pitch comes from the user popup (pad_spacing in pcb_settings.json).
# Connector pad pitch is fixed at ~650 µm (1 step ≈ 1 µm on the linear stages).
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

# Connector grid, 4x2 with no gaps (all directions in PCB view, but note the
# stage moves the PCB under a still printhead, so machine moves are mirrored;
# the code below works in trace-space where "print direction" = away from ME row):
#   Top row    (cs, printed last along each trace): pads 1, 2, 7, 8
#   Bottom row (cl, nearest the ME pads):           pads 3, 4, 5, 6
# Columns (right -> left in PCB view): (2,3) (1,4) (8,5) (7,6)
# In machine trace-space X (units of CONNECTOR_PITCH_UM from array centerline):
CONNECTOR_X_COLUMNS = {2: -1.5, 3: -1.5, 1: -0.5, 4: -0.5,
                       8: +0.5, 5: +0.5, 7: +1.5, 6: +1.5}
CS_PADS = {1, 2, 7, 8}  # top row — routed around the outside (onion style)

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

    cl pads (3,4,5,6 — bottom row): near-vertical trace, small 45° jog to the
    column, entering the pad from below.

    cs pads (1,2,7,8 — top row): routed like onion layers so nothing crosses:
    straight up at the ME pad's own X (outside the grid), 45° corner, a
    crossbar ABOVE the whole connector grid, 45° corner, then straight down
    into the pad from the top. Traces 1/8 use the outermost/highest layer,
    2/7 the inner one.
    """
    c_mm = connector_pitch_um / 1000.0
    stagger_mm = CONNECTOR_ROW_STAGGER_UM / 1000.0
    tuning = get_trace_tuning()
    cl_drop = tuning["connector_cl_drop_mm"]
    corner_len = tuning["corner_mm"]
    cs_clear = tuning["cs_clear_mm"]
    cs_layer = tuning["cs_layer_mm"]
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
            # Routed trace over the top of the grid
            H = d_cs + cs_clear + (cs_layer if n in (1, 8) else 0.0)
            up_corner = 135 if inward_pos else 45
            horiz = 180 if inward_pos else 0
            down_corner = 225 if inward_pos else 315

            # Riser must sit outside the connector grid to avoid crossing it
            grid_edge = 1.5 * c_mm + 0.3
            if shift > 1e-6 and abs(me_x) < grid_edge:
                print(f"Warning: trace {n} riser at {me_x:.3f} mm is inside the "
                      f"connector grid ({grid_edge:.3f} mm); increase ME pitch.")

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
PAD_SEQUENCE = [
    ("cs", 10), ("cs", 10), ("cl", 9), ("cl", 9),
    ("cl", 9), ("cl", 9), ("cs", 10), ("cs", 10),
]

def print_pcb():
    global x_coord, y_coord, counter

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

# Don't modify - Phillipe's edit
def print_traces(traces_dict):
    global counter, angle_dir, angle_axis, t_len, x_coord, y_coord, z_coord

    _abort_if_emergency_stop()
    x_coord, y_coord, z_coord = get_current_position(x), get_current_position(y), get_current_position(z)

    for i in range(1, len(traces_dict) + 1, 1):
        _abort_if_emergency_stop()
        print_trace(traces_dict, i)
        counter += 1
        next_feature(counter, x_coord, y_coord, 1000)
             
def print_trace(trace_dict, index):
    global counter,angle_dir, angle_axis, t_len               
    
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
  
# Don't modify - Phillipe's edit
"""def diagonal_handler(angle, t_len, div):
    # Convert angle to radians
    
    nordson_on() # Originally was off

    theta = math.radians(abs(angle))

    # Calculate dx and dy based on the angle
    dx = 2*abs(t_len * math.cos(theta))
    dy = 2*abs(t_len * math.sin(theta))

    # Use same small-pulse approach as keyboard control so X and Y move
    # in rapid alternation instead of fully completing one axis before the other.
    # Number of steps is driven by the longer axis so the ratio is preserved.
    steps = max(1, round(max(dx, dy) / base_displacement))
    x_pulse = dx / steps
    y_pulse = dy / steps

    update_speed(150)
    
    if angle_dir[0] is not None and angle_dir[1] is not None:
        
        for _ in range(steps):
            _abort_if_emergency_stop()
            move_linear_stage(x, angle_dir[0], x_pulse, wait_for_stop=False)
            move_linear_stage(y, angle_dir[1], y_pulse, wait_for_stop=False)
            time.sleep(0.001)

        # Let both axes settle before the next move
        wait_for_axis_stop(x, max_wait=10.0)
        wait_for_axis_stop(y, max_wait=10.0)"""

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
    
    global temp_location, temp_l, temp_w

    length = mm_to_um(pad_dict.get(pad_type).get("l"))
    width = mm_to_um(pad_dict.get(pad_type).get("w"))
    
    update_speed(1)

    if position == 0:
        
        right(width/2)
        front(length)
        left(width)
        back(length)
        right(width/2)

    elif position == 1:
        front(length)
        right(width)
        back(length)
        left(width)

    elif position == 2:
        front(length/2)
        right(width)
        back(length)
        left(width)
        front(length/2)

    elif position == 3:
        right(width)
        back(length)
        left(width)
        front(length)

    elif position == 4:
        left(width/2)
        time.sleep(0.1)
        move_linear_stage(y, '+', length, wait_for_stop=True, max_wait=30.0)
        right(width)
        front(length)
        left(width/2)

    elif position == 5:
        back(length)
        left(width)
        front(length)
        right(width)    

    elif position == 6:
        back(length/2)
        left(width)
        front(length)
        right(width)
        back(length/2)

    elif position == 7:
        left(width)
        front(length)
        right(width)
        back(length)

    elif position == 8:
        # "me" pad — start at center (half width, 3/4 down from top = 1/4 from bottom)
        # Path: up 3/4 h → right w/2 → down h → left w → up h
        back(length * 3 / 4)   # up 3/4 height → top center
        right(width / 2)       # right half width → top-right corner
        front(length)          # down full height → bottom-right corner
        left(width)            # left full width → bottom-left corner
        back(length)           # up full height → top-left corner
        right(width / 2)       # right half width → top center (original start point)
        front(length)          # down full height → bottom center

    elif position == 9:
        # Connector pad — trace arrives from BELOW (cl row), pad extends onward.
        right(width / 2)       # → entry-side corner
        front(length)          # → far corner (direction of travel)
        left(width)            # → far corner, other side
        back(length)           # → entry-side corner, other side
        right(width / 2)       # → back to entry center

    elif position == 10:
        # Connector pad — trace arrives from ABOVE (cs row), pad extends back
        # toward the ME row (opposite the arrival direction).
        right(width / 2)       # → entry-side corner
        back(length)           # → far corner (opposite direction of travel)
        left(width)            # → far corner, other side
        front(length)          # → entry-side corner, other side
        right(width / 2)       # → back to entry center

def next_feature(num, xx, yy, spacing):

    nordson_off()
    
    update_speed(50)
    down(1000)
    xdisp = xx - get_current_position(x)
    ydisp = yy - get_current_position(y)
    back(abs(ydisp))
    left(abs(xdisp))    

    print(f"Moving to next feature {num}")
    move = num * spacing
    right(move)
    up(1000)
    stop_motor_control()

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

"""def probe_origin():
    global pcb_z_coord

    _abort_if_emergency_stop()
    update_speed(100)
    z_home()
    _abort_if_emergency_stop()
    y_home()
    _abort_if_emergency_stop()
    x_home()

    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    move_linear_stage(x, '-', 29000, wait_for_stop=True, max_wait=30.0)

    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    move_linear_stage(y, '-', 11500, wait_for_stop=True, max_wait=30.0)
    
    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    pcb_z_coord = None
    if not Z_probe():
        raise RuntimeError("Z probing failed: unable to read Z position.")

    _abort_if_emergency_stop()
    update_speed(100)
    down(1000)"""

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
    
    # print_trace(traces, 8)
    # print_traces(traces)
    # print_pcb()
    #calibrate()
    #print_pcb()
    servo_to(0)
    time.sleep(2.0)
    # servo_to(45)
    # time.sleep(2.0)
    # servo_to(85)
    # time.sleep(2.0)
    # servo_to(0)
    # time.sleep(2.0)
    servo_to(55)
    time.sleep(2.0)

    pnp_forward(speed=25)
    time.sleep(2.0)

    servo_to(71)
    time.sleep(2.0)
    
    servo_to(55)
    time.sleep(2.0)

    pnp_backward(speed=25)
    time.sleep(2.0)
    
    servo_to(0)
    time.sleep(2.0)
    # time.sleep(2.0)
    # mag_detector()

    # pnp_forward(speed=50)
    # time.sleep(2.0)
    # mag_detector()

    # pnp_backward(speed=50)
    # time.sleep(2.0)
    # mag_detector()

    # pnp_backward(speed=50)
    # time.sleep(2.0)
    # mag_detector()

    # update_speed(1)
    # move_linear_stage('t', '+', 600, wait_for_stop=True, max_wait=60.0)
    # move_linear_stage('t', '-', 600, wait_for_stop=True, max_wait=60.0)
    # move_linear_stage('t', '+', 600, wait_for_stop=True, max_wait=60.0)
    # move_linear_stage('t', '-', 600, wait_for_stop=True, max_wait=60.0)
          
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

# Fill electrode pads with metal ink after wire placement
def fill_electrode_pads():
    """Raster fill electrode pads — 8 passes 1000µm apart."""
    update_speed(10)

    for i in range(8):
        _abort_if_emergency_stop()
        nordson_on()
        if i % 2 == 0:
            back(1000)
        else:
            front(1000)
        nordson_off()
        down(2000)
        right(1000)
        up(2000)

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

    # """Print traces, wait for wire placement, fill electrode pads."""
    # print("Starting full sequence...")

    # # Step 1 — calibrate and print traces
    # print_tester()

    # # Step 2 — return to home X Y Z
    # z_home()
    # y_home()
    # x_home()

    # # Step 3 — rotate -90 to placement station
    # update_speed(50)
    # move_linear_stage('r', '-', 90, wait_for_stop=True, max_wait=30.0)


    # # Step 4 — adjust axes for the start of image recognition and wire placement - does need to be adjusted based on actual placement station since it changed due to the use of the r limiter.
    # move_linear_stage(x, '-', 23000, wait_for_stop=True, max_wait=30.0)
    # move_linear_stage(y, '-', 19600, wait_for_stop=True, max_wait=30.0)
    # move_linear_stage(z, '+', 13500, wait_for_stop=True, max_wait=30.0)

    #  # Step 5 — wait 20 seconds for testing
    # print("Waiting 20 seconds for wire placement...")
    # time.sleep(15)

    # move_linear_stage(x, '+', 20000, wait_for_stop=True, max_wait=30.0)
    # move_linear_stage(y, '+', 15600, wait_for_stop=True, max_wait=30.0)
    # move_linear_stage(z, '-', 10500, wait_for_stop=True, max_wait=30.0)

    # # Step 6 — rotate +90 back to print station
    # move_linear_stage('r', '+', 90, wait_for_stop=True, max_wait=30.0)

    # Step 7 — go back to print origin (same starting point as traces)

    # Step 8 — fill electrode pads

    # # Step 9 — return to home and park
    # z_home()
    # y_home()
    # x_home()
    # move_linear_stage('r', '-', 30, wait_for_stop=True, max_wait=30.0)

    # print("Full sequence complete.")
