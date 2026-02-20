import time

from motor_control import (move_linear_stage)
from relay_control import (nordson_on, nordson_off)

#parameters for line test

x = str('X')
y = str('Y')
z = str('Z')

l = 3000.0 #Trace length in steps 
tapl = 2000.0
stp = 1000.0
delay = 2

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

def line_test_2():
    left(5000.0)

def line_test_1():
    
    """Test moving the linear stage forward and backward."""
    
    print("Starting line test 1...")
    
    tap()
    
    for i in range(1, 8, 1):
        nordson_on()
        time.sleep(delay)
        front(l)
        nordson_off()
        time.sleep(delay)

        down(tapl)
        back(l)
        time.sleep(delay)
        left(stp)
        up(tapl)
    
    
    print("Line test 1 complete.")
    