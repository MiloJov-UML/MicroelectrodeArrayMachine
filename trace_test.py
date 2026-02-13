import time

from motor_control import (move_linear_stage)
from relay_control import (nordson_on, nordson_off)

#parameters for line test
frd = '+'
bck = '-'

x = str('X')
y = str('Y')
z = str('Z')

l = 2000.0 #Trace length in steps 
tapl = 2000.0
stp = 1000.0


def tap():
    move_linear_stage(y, frd, tapl,wait_for_completion=True)
    time.sleep(2)
    move_linear_stage(y, bck, tapl,wait_for_completion=True)
    #tap

def print_trace():
    nordson_on()
    time.sleep(2)
    move_linear_stage(x, bck, l, wait_for_completion=True)
    nordson_off()
    time.sleep(2)

def side_step():
    move_linear_stage(x, frd, stp, True)
    time.sleep(2)

def line_test_1():
    """Test moving the linear stage forward and backward."""
    print("Starting line test 1...")
    
    time.sleep(3)
    move_linear_stage(z, '-', tapl, wait_for_stop=True, max_wait=30.0)
    time.sleep(2)
    move_linear_stage(z, '+', tapl, wait_for_stop=True, max_wait=30.0)
    time.sleep(2)
    #tap
    
    nordson_on()
    time.sleep(2)
    move_linear_stage(y, '-', l, wait_for_stop=True, max_wait=30.0)
    nordson_off()
    time.sleep(2)
    
    move_linear_stage(x, '+', stp, wait_for_stop=True, max_wait=30.0)
    time.sleep(2)

    print("Line test 1 complete.")
    