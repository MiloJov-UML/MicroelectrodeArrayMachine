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
print_origin = [74410.0, 2540840.0, 2612907.5, 4022.59] # X, Y, Z, R coordinate for tarting point for print process, probe to find Z
print_z = None # Z coordinate for printing, set after probing based on print_gap


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

pad_types = {

    "cs": {"l": 0.76, "w": 0.38}, # Dimensions of cable conncetor short pads, mm

    "cl": {"l": 1.02, "w": 0.38}, # Dimensions of cable conncetor long pads, mm

    "me": {"l": 1.2, "w": 0.7}    # Dimensions of electrode pads, mm
    
}

pads = {
    
    1: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 4, "e": 4}}, 
    
    2: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 2, "e": 2}},
    
    3: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 2, "e": 2}},
    
    4: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 1, "e": 1}},
    
    5: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 7, "e": 7}},
    
    6: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cl", "s": 6, "e": 6}},
    
    7: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 6, "e": 6}},
    
    8: {"start": {"t": "me", "s": 1, "e": 4}, "end": {"t": "cs", "s": 7, "e": 7}}

}

# Don't modify - Phillipe's edit
def print_traces(traces_dict):
    global counter, angle_dir, angle_axis, t_len, x_coord, y_coord, z_coord

    x_coord, y_coord, z_coord = get_current_position(x), get_current_position(y), get_current_position(z)

    for i in range(1, len(traces_dict) + 1, 1):
        print_trace(traces_dict, i)
        counter += 1
        next_feature(counter, x_coord, y_coord, 1000)
             
def print_trace(trace_dict, index):
    global counter,angle_dir, angle_axis, t_len               
    
    for key, value in (trace_dict.get(index)).items():
        if key.find("a") != -1:
                angle = value
                angle_handler(angle)

        if key.find("l") != -1:
            t_len = mm_to_um(value)
            
        if (t_len != None) & (angle_dir != None):
            
            if angle_axis.find('d') == -1:
                nordson_on()
                update_speed(20)
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
def diagonal_handler(angle, t_len, div):
    # Convert angle to radians
    
    nordson_off()

    theta = math.radians(abs(angle))

    # Calculate dx and dy based on the angle
    dx = t_len * math.cos(theta)
    dy = t_len * math.sin(theta)

    xstp = round(abs(dx / div))
    ystp = round(abs(dy / div))
    update_speed(200)
    
    if (angle_dir[0] != None) & (angle_dir[1] != None):
        
        for i in range(div):
            
            move_linear_stage(x, angle_dir[0], xstp, wait_for_stop=True, max_wait=30.0)
            
            
            move_linear_stage(y, angle_dir[1], ystp, wait_for_stop=True, max_wait=30.0)
            
def print_pad(pad_dict, pad_type, position):
    
    global temp_l, temp_w

    nordson_on()
    pad_position_handler(pad_dict, pad_type, position)
    pad_motion_handler(position, temp_l, temp_w)
    nordson_off()

def pad_position_handler(dict, type, position):
    
    global temp_location, temp_l, temp_w

    length = mm_to_um(dict.get(type).get("l"))
    linc = round(length / 5)
    temp_l = linc

    width = mm_to_um(dict.get(type).get("w"))
    winc = round(width / 5)
    temp_w = winc
    update_speed(10)

    if position == 1:
        right(winc)
        front(linc)
    elif position == 2:
        right(winc)
    elif position == 3:
        right(winc)
        back(linc)
    elif position == 4:
        back(linc)
    elif position == 5:
        left(winc)
        back(linc)
    elif position == 6:
        left(winc)
    elif position == 7:
        left(winc)
        front(linc)
    elif position == 8:
        front(winc)
    else:
        print("invalid position")

    temp_location = (get_current_position(x), get_current_position(y))

def pad_motion_handler(position, length, width):
    
    update_speed(10)
    if position == 1:
        front(length*2)
        right(width*2)
        back(length*2)
        left(length*2)
        front(length*2)

        left(width)
        back(length)
    elif position == 2:
        front(length*1)
        right(width*2)
        back(length*2)
        left(length*2)
        front(length*2)

        left(width)
    elif position == 3:
        right(width*2)
        back(length*2)
        left(width*3)
        front(length*3)

        left(width)
        front(length)
    elif position == 4:
        right(width*1)
        back(length*2)
        left(width*2)
        front(length*2)

        front(length)
    elif position == 5:
        back(length*2)
        left(width*2)
        front(length*2)
        right(width*2)

        right(width)
        front(length)
    elif position == 6:
        back(length*1)
        left(width*2)
        front(length*2)
        right(width*2)

        right(width)
    elif position == 7:
        left(width*2)
        front(length*2)
        right(width*2)
        back(length*2)

        right(width)
        back(length)
    elif position == 8:
        left(width*1)
        front(length*2)
        right(width*2)
        back(length*2)

        back(width)
    else:
        print("invalid position")

def next_feature(num, xx, yy, spacing):
    
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

# Add code into function to test it using the gui "Printing tester" button
def line_test_1():
    
    #print_trace(traces, 8)
    #print_traces(traces)

    print_pad(pad_types, "me", 4)
    print_trace(traces, 1)
    print_pad(pad_types, "cs", 4)

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
    motor_backward(steps=10)
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


