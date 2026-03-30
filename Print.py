# trace_test.py
#
#   Approach A — "Staircase"       : pure alternating X/Y step
#   Approach B — "Weighted Stair"  : axis-ratio steps for any angle
#   Approach C — "Dominant Lead"   : longer axis drives, shorter fills gaps
#   Approach D — "Pulse Diagonal"  : fixed micro-steps with glue pulse per segment


import time
import math
from motor_control import (
    move_linear_stage, 
    update_speed, 
    return_to_origin, 
    stop_motor_control, 
    get_current_position, 
    µm_to_steps, 
    mm_to_um
)

from relay_control import (
    nordson_on, 
    nordson_off, 
    motor_backward, 
    motor_forward, 
    motor_release, 
    r_calibrate, 
    Z_calibrate,
)   

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
x_now, y_now, z_now = None, None, None

x_disp = 0
y_disp = 0

probe_z_coord = None
print_z_coord = None
wipe_y = 2123.0
probe_y = 2342.0 #  Fake
print_gap = 0.1 # Gap in mm from pcb surface
print_origin = [74410.0, 2540840.0, 2612907.5, 4022.59] # X, Y, Z, R coordinate for tarting point for print process, probe to find Z
print_z = None # Z coordinate for printing, set after probing based on print_gap
counter = 0

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

   1: {"a1": 0, "l1": 4.17, "a2": -45.0, "l2": 0.5, "a3": -90.0, "l3": 2.47, "a4": +45.0, "l4": 0.5}, # Outermost trace to the right

   2: {"a1": 0, "l1": 3.44, "a2": -45.0, "l2": 0.5 , "a3": -90.0, "l3": 1.0},

   3: {"a1": 0, "l1": 2.28, "a2": -45.0, "l2": 0.5},

   4: {"a1": 0, "l1": 2.13}
}

pads = {

    "ccS": {"l": 0.76, "w": 0.38}, # Dimensions of cable conncetor short pads, mm

    "ccL": {"l": 1.02, "w": 0.38}, # Dimensions of cable conncetor long pads, mm

    "cf": {"l": 1.2, "w": 0.7} # Dimensions of electrode pads, mm
}

# Don't modify - Phillipe's edit
def print_trace(traces):
    global counter
    dist = None
    direction = None
    angle = None
    
    #move_to_coord(print_origin[0], 2540840.0, 2612907.5) 

    for key, t_dict in traces.items():
        
        for t, value in t_dict.items(): 
            if t.find("a") != -1:
                angle = value
                direction = dir_handler(angle)
                axis = axis_handler(angle)
            
            if t.find("l") != -1:
                l = value
                dist = mm_to_um(l)
            
            if (dist != None) & (direction != None):
                if axis != 'd':
                    nordson_on()
                    update_speed(10)
                    #print(f"Moving {axis} {direction} for {dist}µm at angle {angle}°")
                    move_linear_stage(axis, direction, dist, wait_for_stop=True, max_wait=30.0)
                    time.sleep(0.75)
                    nordson_off()
                    disp_handler(axis, dist, angle)
                    dist = None
                    direction = None
                    time.sleep(0.5)
                elif axis == 'd':
                    diagonal_handler(dist, angle)
                    dist = None
                    direction = None
                    time.sleep(0.5)
        counter += 1
        next_feature(counter, x_disp, y_disp)
                         
def print_pad(pad_type):
    #use 3 lines for cf pads, and 2 lines for cc pads
    for key in pads.keys():
        length = pads[key].l
        width = pads[key].w
        
        vertical_step = mm_to_um(length)
        pass_num = None

        if key.find(pad_type) != -1:
            pass_num = 2
            horizontal_step = mm_to_um(width / pass_num)
            
        elif key.find(pad_type) != -1:
            pass_num = 3
            horizontal_step = mm_to_um(width / pass_num)
        else:
            print("Invalid width")

        for i in range(pass_num):
            move_linear_stage(x, '+', length, wait_for_stop=True, max_wait=30.0)
            move_linear_stage(y, '-', length, wait_for_stop=True, max_wait=30.0)
            
# direction only ussable for printing diagonal lines, for straight lines direction is determined by angle and axis of movement
def dir_handler(angle):
    angle_dir = None
    
    if angle == 0: 
        angle_dir = '-'
    elif angle == +45.0:
        angle_dir = '+'
    elif angle == -45.0:
        angle_dir = '-'
    elif angle == +90.0:
        angle_dir = '-'
    elif angle == -90.0:
        angle_dir = '+'    
    else:
        angle_dir = 'invalid angle'
    
    return angle_dir

def axis_handler(angle):
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
def diagonal_handler(size, angle):
    # Convert angle to radians
    theta = math.radians(abs(angle))
    direction = dir_handler(angle)
    # Calculate dx and dy based on the angle
    dx = size * math.cos(theta)
    dy = size * math.sin(theta)
    
    
    div = dx / 118

    xstp = round(dx / int(round(div)))
    ystp = round(dy / int(round(div)))
    update_speed(200)
    
    for i in range(int(round(div))):
        
        nordson_on()
        move_linear_stage(y, direction, ystp, wait_for_stop=True, max_wait=30.0)
        disp_handler(y, ystp, angle)
        nordson_off()
        move_linear_stage(x, '+', xstp, wait_for_stop=True, max_wait=30.0)
        disp_handler(x, xstp, angle)
        

        

        #print(f"Moving diagonal at angle {angle}° for segment {i+1}/{int(round(div))} with dx={xstp}µm and dy={ystp}µm")

def disp_handler(disp_axis, distance, angle):
    global x_disp, y_disp
    if disp_axis == x:
        x_disp += distance
    elif disp_axis == y:
        if dir_handler(angle) == '-':
            y_disp += distance
        elif dir_handler(angle) == '+':
            y_disp -= distance

# Don't modify - Phillipe's edit  
def Z_probe():
    global pcb_z_coord
    move_linear_stage('Z', '+', 40000, wait_for_stop=False, max_wait=30.0)
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

def next_feature(num, dispx, dispy):
        
    global x_disp, y_disp
    
    update_speed(50)
    
    down(1000)
    back(dispy)
    left(dispx)

    x_disp = 0
    y_disp = 0

    print(f"Moving to next feature {num}")
    move = num * 1000
    right(move)
    x_disp += move
    
    up(1000)
    stop_motor_control() 

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

    
    print_trace(traces)

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


