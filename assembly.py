import time
import math

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
    nordson_off
    
)

from print import (
    glue_sequence,
    print_tester,
    r_limit,
    Z_probe,
    get_coord,
    print_pcb
)

import image_recognition
from image_recognition import (
    open_camera,
    extrude,
    x_align,
    r_align
)

print_sequence = ("print", False)
placement_sequence = ("placement", False)
fill_sequence = ("fill", False)
glue_sequence = ("glue", False)
pnp_sequence = ("pnp", False)

sequences = (print_sequence, placement_sequence, cut_sequence, fill_sequence, glue_sequence, pnp_sequence)
def run_full_assembly(sequence):

    for i in range(len(sequence)+1):
        if sequence[i] == True:
            start_sequence

def start_sequence():
    # To start a sequence type, to start set sequence variable to True

def end_sequence():
    # To end a sequence type, to end set sequence variable to False

def sequence_handler(seq):
    if seq[0] == 