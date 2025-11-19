# pcb_mapping.py – diagonal + trace + test

from motor_control import move_linear_stage, get_current_position
from relay_control import motor_forward, motor_backward, motor_release
import time

# Fixed Z value used for printing contact and lifting
Z_PRINT = 300   # Lower Z by this amount to touch the PCB and raise it to stop printing


# Diagonal movement function (REQUIRED)
def move_diagonal(dx, dy):
    """
    Moves diagonally by splitting motion into small steps.
    dx > 0 → X+
    dx < 0 → X-
    dy > 0 → Y+
    dy < 0 → Y-
    """

    steps = max(abs(dx), abs(dy)) // 25
    if steps < 1:
        steps = 1

    step_x = dx / steps
    step_y = dy / steps

    for _ in range(int(steps)):
        if step_x != 0:
            move_linear_stage(
                "X",
                "+" if step_x > 0 else "-",
                abs(step_x),
                wait_for_stop=False
            )

        if step_y != 0:
            move_linear_stage(
                "Y",
                "+" if step_y > 0 else "-",
                abs(step_y),
                wait_for_stop=False
            )

        time.sleep(0.01)


# TRACE SHAPE FUNCTION

def print_trace_pattern():
    """
    Exact motion sequence requested:
    1. Lower Z to start printing
    2. Move Up (Y-)
    3. Move Up-Right diagonally
    4. Move Right (X+)
    5. Move Down-Right diagonally
    6. Move slightly Down (Y+)
    7. Lift Z to stop printing
    """

    print("Running Trace Shape")

    # Lower Z to begin printing
    move_linear_stage("Z", "-", Z_PRINT, wait_for_stop=True)   # Z down to touch PCB

    # Move Up (Y-)
    move_linear_stage("Y", "-", 200, wait_for_stop=True)       # 200 µm upward move

    # Diagonal Up-Right
    move_diagonal(200, -200)                                   # 200 µm diagonal up-right

    # Move Right (X+)
    move_linear_stage("X", "+", 200, wait_for_stop=True)       # 200 µm right move

    # Diagonal Down-Right
    move_diagonal(200, 200)                                    # 200 µm diagonal down-right

    # Slight extra Down (Y+)
    move_linear_stage("Y", "+", 100, wait_for_stop=True)       # 100 µm downward adjustment

    # Lift Z to stop printing
    move_linear_stage("Z", "+", Z_PRINT, wait_for_stop=True)   # Z up to lift needle

    print("Trace Shape Complete")


# TEST DIAGONAL MOVEMENT
def test_diagonal():
    """
    Test diagonal motion:
    - Lower Z
    - Move diagonally (X+ and Y- simultaneously)
    - Lift Z
    This verifies that diagonal movements occur at the same time, not separately.
    """

    print("Running Diagonal Test")

    # Lower Z to touch PCB
    move_linear_stage("Z", "-", Z_PRINT, wait_for_stop=True)   # Z down

    # Perform diagonal Up-Right
    move_diagonal(2000, -2000)                                   # 200 µm diagonal up-right

    # Lift Z to stop touching PCB
    move_linear_stage("Z", "+", Z_PRINT, wait_for_stop=True)   # Z up

    print("Diagonal Test Complete")