# trace_test.py
#
#   Approach A — "Staircase"       : pure alternating X/Y step
#   Approach B — "Weighted Stair"  : axis-ratio steps for any angle
#   Approach C — "Dominant Lead"   : longer axis drives, shorter fills gaps
#   Approach D — "Pulse Diagonal"  : fixed micro-steps with glue pulse per segment


import time
import math
from motor_control import move_linear_stage, update_speed, return_to_origin, stop_motor_control, get_current_position, mm_to_steps, steps_to_mm
from relay_control import nordson_on, nordson_off, motor_backward, motor_forward, motor_release, r_calibrate, Z_calibrate

#parameters for line test

x = 'X'
y = 'Y'
z = 'Z'

l    = 3000.0   # Trace length in steps
tapl = 5000.0   # Z tap depth 
stp  = 1000.0   # Step-over between parallel lines
delay = 0.5       # Dispenser settle time

SEG = 50.0      # Default segment size for diagonal interpolation (µm)

pcb_dim = (12, 6) # Dimenstions of Kapton PCB
x_coord, y_coord, z_coord = None, None, None
probe_z_coord = None
print_z_coord = None
wipe_y = 2123.0
probe_y = 2342.0 #  Fake
print_gap = 0.1 # Gap in mm from pcb surface
print_origin = (23.0, 45.0) # X, Y coordinate for tarting point for print process, probe to find Z
print_z = None # Z coordinate for printing, set after probing based on print_gap
diagonal_step = 150.0

# BASIC MOVES  
# Use for testing

def up(length):
    move_linear_stage(z, '+', length, wait_for_stop=True, max_wait=30.0)

def down(length):
    move_linear_stage(z, '-', length, wait_for_stop=True, max_wait=30.0)

def left(length):
    move_linear_stage(x, '+', length, wait_for_stop=True, max_wait=30.0)

def right(length):
    move_linear_stage(x, '-', length, wait_for_stop=True, max_wait=30.0)

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

   1: {"a1": 0, "l1": 4.17, "a2": -45.0, "l2": 0.5, "a3": +90.0, "l3": 2.47, "a4": +45.0, "l4": 0.5}, # Outermost trace to the right

   2: {"a1": 0, "l1": 3.44, "a2": -45.0, "l2": 0.5 , "a3": +90.0, "l3": 1.0},

   3: {"a1": 0, "l1": 2.28, "a2": -45.0, "l2": 0.5},

   4: {"a1": 0, "l1": 2.13}
}

pads = {

    "ccS": {"l": 0.76, "w": 0.38}, # Dimensions of cable conncetor short pads, mm

    "ccL": {"l": 1.02, "w": 0.38}, # Dimensions of cable conncetor long pads, mm

    "cf": {"l": 1.2, "w": 0.7} # Dimensions of electrode pads, mm
}

# Don't modify - Phillipe's edit
def print_trace(num):
    
    dist = None
    direction = None
    angle = None
    
    for key, t_dict in traces.items():
        
        for t, value in t_dict.items(): 
            if t.find("a") != -1:
                angle = value
                direction = dir_handler(angle)
                axis = angle_handler(angle)
            
            if t.find("l") != -1:
                length = value
                dist = mm_to_steps(length, y)
            
            if (dist != None) & (direction != None):
                if axis != 'd':
                    nordson_on()
                    move_linear_stage(axis, direction, dist, wait_for_stop=True, max_wait=30.0)
                    nordson_off()
                    dist = None
                    direction = None
                else:
                    diagonal_handler(dist, angle)
                    dist = None
                    direction = None
                    

def print_pad():
    #use 3 lines for cf pads, and 2 lines for cc pads
    for key in pads.keys():
        length = pads[key].l
        width = pads[key].w
        
        vertical_step = mm_to_steps(length)
        pass_num = None

        if key.find("cc") != -1:
            pass_num = 2
            horizontal_step = mm_to_steps(width / pass_num, x)
        elif key.find("cf") != -1:
            pass_num = 3
            horizontal_step = mm_to_steps(width / pass_num, x)
        else:
            print("Invalid width")

        for i in range(0, width, horizontal_step):
            move_linear_stage(x, '+', length, wait_for_stop=True, max_wait=30.0)
            move_linear_stage(y, '+', diagonal_step, wait_for_stop=True, max_wait=30.0)
            move_linear_stage(x, '-', length, wait_for_stop=True, max_wait=30.0)
            move_linear_stage(y, '+', diagonal_step, wait_for_stop=True, max_wait=30.0)

# direction only ussable for printing diagonal lines, for straight lines direction is determined by angle and axis of movement
def dir_handler(angle):
    angle_dir = None
    
    if angle < 0: 
        angle_dir = '+'
    elif angle >= 0:
        angle_dir = '-'
    else:
        angle_dir = 'invalid angle'
    
    return angle_dir

def angle_handler(angle):
    axis = None
    
    if abs(angle) == 0:
        axis = y
    elif abs(angle) == 45:
        axis = 'd'
    elif abs(angle) == 90:
        axis = x
    else:
        axis =  'invalid angle'

    return axis

# Don't modify - Phillipe's edit
def diagonal_handler(length, angle):
    # Convert angle to radians
    theta = abs(angle)
    direction = dir_handler(angle)
    # Calculate dx and dy based on the angle
    dx = length * math.cos(theta)
    dy = length * math.sin(theta)

    xl = mm_to_steps(dx, x)
    yl = mm_to_steps(dy, y)

    div = max(abs(xl), abs(yl)) / diagonal_step

    update_speed(150)
    
    for i in range(int(div)):
        nordson_on()
        time.sleep(delay)
        move_linear_stage(y, '-', float(yl/div), wait_for_stop=True, max_wait=30.0)
        nordson_off()
        move_linear_stage(x, direction, float(xl/div), wait_for_stop=True, max_wait=30.0)
        

# Don't modify - Phillipe's edit  
def Z_probe():
    global pcb_z_coord
    move_linear_stage('Z', '+', 50000, wait_for_stop=False, max_wait=30.0)
    state = Z_calibrate()
    if state == "Z limit":
        pcb_z_coord = get_current_position("Z")
        print(pcb_z_coord)

# Don't modify - Phillipe's edit
def r_limit():
         #Successfully created a diagonal
    move_linear_stage('r', '+', 100, wait_for_stop=False, max_wait=30.0)
    state = r_calibrate()
    if state == "R limit":
        rot = get_current_position("r")
        print(rot)

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

# Add code into function to test it using the gui "Printing tester" button
def line_test_1():
    
    """
    print("Starting line test 1...")
    update_speed(30)
    tap()
    
    # front 400 nordson on
    update_speed(5)
    nordson_on()
    time.sleep(delay)
    front(2500)
    update_speed(50)
    front(1250)
    update_speed(150)
    # diagonal front+left 800 nordson off
    for i in range(20):  # 6 steps x 100µm = 600µm
        front(100)
        time.sleep(delay)
        nordson_on()
        time.sleep(delay)
        left(100)
        nordson_off()
    """

    for i in range(0, 4, 1):
        print_trace(traces.get(i))

# OLD TESTS — from early development, not updated for new code structure
def lines_test():
     """Lay down a set of parallel horizontal glue traces."""
     print("Starting line test 1...")
     tap()
     for i in range(1, 8, 1):
         nordson_on()
         time.sleep(delay)
         update_speed(55) # speed change mod 0-150
         front(l)
         nordson_off()
         update_speed(50)
         time.sleep(delay)
        
         down(tapl)
         back(l)
         time.sleep(delay)
         left(stp)
         up(tapl)
     print("Line test 1 complete.")

#  APPROACH A — "Staircase"
def diagonal_staircase(dx: float, dy: float,
                       seg: float = SEG,
                       dispenser: str = None):
    """
    Approach A — Staircase.
    Strictly alternates equal X and Y micro-steps.
    Works best when |dx| == |dy| (45° angle).

    dispenser: None | 'nordson'
    """
    if dx == 0 or dy == 0:
        print("[Staircase] dx or dy is zero — use a straight move instead.")
        return

    n   = max(1, int(math.hypot(dx, dy) / seg))
    xs  = dx / n
    ys  = dy / n
    xd  = '+' if xs >= 0 else '-'
    yd  = '-' if ys >= 0 else '+'   # Y+  = back,  Y- = front

    print(f"[Staircase] {n} segments — alternating "
          f"X{xd}{abs(xs):.1f}µm / Y{yd}{abs(ys):.1f}µm")

    _dispenser_on(dispenser)
    for _ in range(n):
        move_linear_stage(x, xd, abs(xs), wait_for_stop=True, max_wait=15.0)
        move_linear_stage(y, yd, abs(ys), wait_for_stop=True, max_wait=15.0)
    _dispenser_off(dispenser)
    print("[Staircase] done.")

#  APPROACH B — "Weighted Stair"
#  Visual path (3:1 ratio):
#    X X X Y X X X Y  ...
def diagonal_weighted(dx: float, dy: float,
                      seg: float = SEG,
                      dispenser: str = None):
    """
    Approach B — Weighted Stair.
    Distributes X and Y steps by their ratio so any angle is accurate.

    dispenser: None | 'nordson'
    """
    if dx == 0 and dy == 0:
        return

    abs_dx, abs_dy = abs(dx), abs(dy)
    xd = '+' if dx >= 0 else '-'
    yd = '-' if dy >= 0 else '+'

    # Bresenham error accumulator
    # Treat longer axis as the "fast" axis
    if abs_dx >= abs_dy:
        n_fast   = max(1, int(abs_dx / seg))
        fast_ax  = x; fast_dir  = xd; fast_seg = abs_dx / n_fast
        slow_ax  = y; slow_dir  = yd; slow_total = abs_dy
    else:
        n_fast   = max(1, int(abs_dy / seg))
        fast_ax  = y; fast_dir  = yd; fast_seg = abs_dy / n_fast
        slow_ax  = x; slow_dir  = xd; slow_total = abs_dx

    error    = 0.0
    slow_rem = slow_total

    print(f"[WeightedStair] fast={fast_ax} × {n_fast}, "
          f"slow={slow_ax} distributed by ratio")

    _dispenser_on(dispenser)
    for i in range(n_fast):
        move_linear_stage(fast_ax, fast_dir, fast_seg,
                          wait_for_stop=True, max_wait=15.0)
        error += slow_total / n_fast
        if error >= seg and slow_rem > 0:
            step = min(error, slow_rem)
            move_linear_stage(slow_ax, slow_dir, step,
                              wait_for_stop=True, max_wait=15.0)
            slow_rem -= step
            error    -= step
    # flush any remaining slow-axis distance
    if slow_rem > 0.5:
        move_linear_stage(slow_ax, slow_dir, slow_rem,
                          wait_for_stop=True, max_wait=15.0)
    _dispenser_off(dispenser)
    print("[WeightedStair] done.")

#  APPROACH C — "Dominant Lead"
#  Visual (dx >> dy):
#    XXXXXXXXX         full X sweep
#             Y       one Y correction at end of each X block
#    XXXXXXXXX
def diagonal_dominant_lead(dx: float, dy: float,
                            block_size: float = 500.0,
                            dispenser: str = None):
    
    abs_dx, abs_dy = abs(dx), abs(dy)
    xd = '+' if dx >= 0 else '-'
    yd = '-' if dy >= 0 else '+'

    if abs_dx == 0 and abs_dy == 0:
        return

    if abs_dx >= abs_dy:
        n_blocks    = max(1, int(abs_dx / block_size))
        dom_ax, dom_dir, dom_seg  = x, xd, abs_dx / n_blocks
        sub_ax, sub_dir, sub_total = y, yd, abs_dy
    else:
        n_blocks    = max(1, int(abs_dy / block_size))
        dom_ax, dom_dir, dom_seg  = y, yd, abs_dy / n_blocks
        sub_ax, sub_dir, sub_total = x, xd, abs_dx

    sub_per_block = sub_total / n_blocks

    print(f"[DominantLead] {n_blocks} blocks — "
          f"{dom_ax}{dom_dir}{dom_seg:.1f}µm then "
          f"{sub_ax}{sub_dir}{sub_per_block:.1f}µm")

    _dispenser_on(dispenser)
    for _ in range(n_blocks):
        move_linear_stage(dom_ax, dom_dir, dom_seg,
                          wait_for_stop=True, max_wait=15.0)
        if sub_per_block > 0.5:
            move_linear_stage(sub_ax, sub_dir, sub_per_block,
                              wait_for_stop=True, max_wait=15.0)
    _dispenser_off(dispenser)
    print("[DominantLead] done.")

#  DIAGONAL MOVEMENT APPROACH D — "Micro-step Diagonal"
def diagonal_pulse(dx: float, dy: float,
                   seg: float = SEG):
   
    if dx == 0 and dy == 0:
        return

    n   = max(1, int(math.hypot(dx, dy) / seg))
    xs  = dx / n
    ys  = dy / n
    xd  = '+' if xs >= 0 else '-'
    yd  = '-' if ys >= 0 else '+'

    print(f"[MicroStep] {n} segments — X{xd}{abs(xs):.1f}µm / Y{yd}{abs(ys):.1f}µm")

    for _ in range(n):
        move_linear_stage(x, xd, abs(xs), wait_for_stop=True, max_wait=15.0)
        move_linear_stage(y, yd, abs(ys), wait_for_stop=True, max_wait=15.0)

    print("[MicroStep] done.")

#  GLUE TAP TEST — standalone, independent of all trace tests
def test_glue_tap(hold_s: float = 0.8):
    print(f"=== Glue Tap Test — hold {hold_s}s ===")
    up(tapl)
    time.sleep(hold_s)
    down(tapl)
    left(100)
    up(tapl)
    time.sleep(hold_s)
    down(tapl)

    print("[GlueTap] done.")

# GLUE DROP & SEQUENCE
def glue_drop():
    """Dispense glue for a fixed number of seconds then release."""
    motor_backward(steps=20)
    time.sleep(0.5)
    # motor_forward(steps=5000)
    # motor_forward(steps=5000)
    # time.sleep(1.0)
    motor_release()

def glue_sequence():
    """Glue tap sequence at 1000µm intervals left, up to 8000µm."""
    print("Starting glue sequence...")
    update_speed(50)
    return_to_origin()
    for i in range(1, 9):  # 1000, 2000 ... 8000
        left(1000)
        up(tapl)
        glue_drop()
        down(tapl)
    return_to_origin()
    print("Glue sequence complete.")


