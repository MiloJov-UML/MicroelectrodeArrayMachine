# trace_test.py
#
#   Approach A — "Staircase"       : pure alternating X/Y step
#   Approach B — "Weighted Stair"  : axis-ratio steps for any angle
#   Approach C — "Dominant Lead"   : longer axis drives, shorter fills gaps
#   Approach D — "Pulse Diagonal"  : fixed micro-steps with glue pulse per segment


import time
import math
from motor_control import move_linear_stage, update_speed
from relay_control import nordson_on, nordson_off

#parameters for line test

x = 'X'
y = 'Y'
z = 'Z'

l    = 3000.0   # Trace length in steps
tapl = 2000.0   # Z tap depth 
stp  = 1000.0   # Step-over between parallel lines
delay = 0.7       # Dispenser settle time

SEG = 50.0      # Default segment size for diagonal interpolation (µm)


#  BASIC MOVES  

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
#
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
    """
    Standalone glue tap test.
    Z down → hold → Z up. No X/Y movement. No dispenser relay.

    hold_s : how long to hold Z down in seconds (default 0.8)
    """
    print(f"=== Glue Tap Test — hold {hold_s}s ===")
    down(tapl)
    time.sleep(hold_s)
    up(tapl)
    print("[GlueTap] done.")


#  INTERNAL HELPER — dispenser on/off

def _dispenser_on(dispenser: str):
    if dispenser == 'nordson':
        nordson_on()

def _dispenser_off(dispenser: str):
    if dispenser == 'nordson':
        nordson_off()


#  ORIGINAL TESTS  

def line_test_2():
    for i in range(1, 1000, 1):
        front(100)
        left(100)

    print("Line test 1 complete.")

def line_test_1():
    """Lay down a set of parallel horizontal glue traces."""
    print("Starting line test 1...")
    tap()
    for i in range(1, 9, 1):
        nordson_on()
        time.sleep(delay)
        update_speed(40)
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


#  DEMO TESTS — one per approach
# def test_approach_A():
#     print("=== Approach A: Staircase ===")
#     diagonal_staircase(dx=4000, dy=4000, dispenser='nordson')

# def test_approach_B():
#     print("=== Approach B: Weighted Stair ===")
#     diagonal_weighted(dx=6000.0, dy=2000, dispenser='nordson')

def test_approach_C():
    print("=== Approach C: Dominant Lead ===")
    diagonal_dominant_lead(dx=1000, dy=1000, block_size=150, dispenser='glue')

# def test_approach_D():
#     print("=== Approach D: Micro-step Diagonal ===")
#     diagonal_pulse(dx=5000, dy=5000, seg=SEG)