import time
import math
import threading
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
wipe_y = 2123.0 # Y Position for testing, replace with actual wipe position, used for wiping probe after Z probe to prevent smearing ink on PCB during print process
probe_y = 2342.0 #  Y Position for testing, replace with actual probe position
print_home = [0, 0, 0, 0] # X, Y, Z, R coordinate for starting point for print process, probe to find Z
print_origin = [74410.0, 2540840.0, 2612907.5, 4022.59] # X, Y, Z, R coordinate for tarting point for print process, probe to find Z
pcb_z_coord = None # Z coordinate for printing, set after probing
_has_calibrated = False  # Set True after first successful calibrate(); skipped on reruns
_manual_origin_event = threading.Event()
_manual_origin_prompt_callback = None

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

def register_manual_origin_prompt_callback(callback):
    global _manual_origin_prompt_callback
    _manual_origin_prompt_callback = callback

def confirm_manual_origin_set():
    _manual_origin_event.set()

def _wait_for_manual_origin_set():
    _manual_origin_event.clear()
    print("Manual origin fine-tune requested. Use GUI movement controls, then click Set Origin to continue.")

    if _manual_origin_prompt_callback is not None:
        try:
            _manual_origin_prompt_callback()
        except Exception as e:
            print(f"Warning: failed to show manual-origin prompt: {e}")

    while not _manual_origin_event.wait(0.1):
        _abort_if_emergency_stop()

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

   4: {"a1": 90.0, "l1": 2.13},
   
   5: {"a1": 90.0, "l1": 2.13},

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

    "me": {"l": 1.2, "w": 0.5}    # Dimensions of electrode pads, mm
    
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

def print_pcb():
    global x_coord, y_coord, counter

    update_speed(1)

    _abort_if_emergency_stop()
    x_coord, y_coord = get_current_position(x), get_current_position(y)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 1)
    print_pad(pad_types, "cs", 4)
    
    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 2)
    print_pad(pad_types, "cs", 2)

    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 3)
    print_pad(pad_types, "cl", 2)

    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 4)
    print_pad(pad_types, "cl", 1)

    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 5)
    print_pad(pad_types, "cl", 7)

    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 6)
    print_pad(pad_types, "cl", 6)

    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 7)
    print_pad(pad_types, "cs", 6)

    counter += 1
    advance_to_next_feature(counter, x_coord, y_coord, 1000)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 8)
    print_pad(pad_types, "cs", 4)

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
    update_speed(100)
    move_linear_stage('Z', '+', 17000, wait_for_stop=True, max_wait=30.0)
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
    move_linear_stage('r', '-', 5, wait_for_stop=False, max_wait=30.0)

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
    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    move_linear_stage(x, '+', 2000, wait_for_stop=True, max_wait=30.0)

    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    move_linear_stage(y, '+', 100, wait_for_stop=True, max_wait=30.0)

    _sleep_with_abort(1.0)
    _abort_if_emergency_stop()
    move_linear_stage(z, '+', 1300, wait_for_stop=True, max_wait=30.0)
    _wait_for_manual_origin_set()
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
def full_sequence():
    global _has_calibrated
    clear_emergency_stop()
    laser_relay_off()
    nordson_off()

    try:
        _abort_if_emergency_stop()
        if not _has_calibrated:
            calibrate()
            _has_calibrated = True
            print("Calibration complete. Subsequent runs will skip calibration.")
        else:
            print("Skipping calibration (already calibrated this session).")
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