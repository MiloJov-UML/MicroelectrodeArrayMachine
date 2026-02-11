# # # pcb_mapping.py – Full 6-trace pattern with improved diagonal interpolation
# # # This assumes a fixed position and assumes the PCB is already aligned 
# # # correctly with the ink dispenser at the starting point.
# # # It can be modified later by adding a calibration setup if needed.
# # pcb_mapping.py – DIAGONAL TEST ONLY
# # All trace logic commented out for safe diagonal testing

# # pcb_mapping.py – SINGLE DIAGONAL TEST (GUI SAFE)

# import math
# from motor_control import move_linear_stage

# Z_PRINT = 300


# # ----------------------------
# # Z CONTROL
# # ----------------------------

# def start_printing():
#     move_linear_stage("Z", "+", Z_PRINT, wait_for_stop=True)


# def stop_printing():
#     move_linear_stage("Z", "-", Z_PRINT, wait_for_stop=True)


# # ----------------------------
# # FAST + SAFE DIAGONAL
# # ----------------------------

# def move_diagonal_travel(dx, dy, chunks=6):
#     if abs(dx) < 0.001 and abs(dy) < 0.001:
#         return

#     if chunks < 1:
#         chunks = 1

#     step_x = dx / chunks
#     step_y = dy / chunks

#     for _ in range(chunks):
#         if step_x != 0:
#             move_linear_stage(
#                 "X",
#                 "+" if step_x > 0 else "-",
#                 abs(step_x),
#                 wait_for_stop=True
#             )

#         if step_y != 0:
#             move_linear_stage(
#                 "Y",
#                 "+" if step_y > 0 else "-",
#                 abs(step_y),
#                 wait_for_stop=True
#             )


# # ----------------------------
# # DIAGONAL TEST (ONLY ONE MOVE)
# # ----------------------------

# def test_diagonal():
#     print("Testing single diagonal move")

#     # stop_printing()

#     move_diagonal_travel(-2000, -2000, chunks=6)

#     # start_printing()

#     print("Diagonal test complete")


# # ----------------------------
# # GUI COMPATIBILITY STUB
# # ----------------------------

# def print_trace_pattern():
#     print("print_trace_pattern() called — disabled for diagonal testing")


# if __name__ == "__main__":
#     test_diagonal()


# # import time
# # import math
# # from motor_control import move_linear_stage
# # from motor_control import get_current_position
# # from motor_control import axis_origins

# # Z_PRINT = 300

# # def start_printing():
# #     move_linear_stage("Z", "+", Z_PRINT, wait_for_stop=True)

# # def stop_printing():
# #     move_linear_stage("Z", "-", Z_PRINT, wait_for_stop=True)


# # # ============================================================
# # # FAST DIAGONAL (20 segments) – SAFE + SPEED + SMOOTH
# # # X+: left  | X-: right
# # # Y+: down | Y-: up
# # # ============================================================

# # def move_diagonal(dx, dy, segments=10):
# #     sx = dx / segments
# #     sy = dy / segments

# #     for _ in range(segments):

# #         if sx != 0:
# #             move_linear_stage("X", "+" if sx > 0 else "-", abs(sx), wait_for_stop=False)

# #         if sy != 0:
# #             move_linear_stage("Y", "+" if sy > 0 else "-", abs(sy), wait_for_stop=False)

# #         time.sleep(0.01)     # FAST + SAFE


# # # ============================================================
# # #                     RIGHT SIDE TRACES (1–4)
# # # ============================================================

# # def trace_1():
# #     # start_printing()
# #     move_linear_stage("Y", "-", 4250, wait_for_stop=True)
# #     move_diagonal(-1300, -1300)
# #     move_linear_stage("X", "-", 500, wait_for_stop=True)
# #     # move_diagonal(-2000, 2000)
# #     # move_linear_stage("Y", "+", 1000, wait_for_stop=True)
# #     # stop_printing()
# #     # start_printing()
# #     # move_linear_stage("Y", "+", 2000, wait_for_stop=True)


# # def trace_2():
# #     # start_printing()
# #     # stop_printing()
# #     move_linear_stage("Y", "-", 1500, wait_for_stop=True)
# #     move_diagonal(1500, -1500)
# #     move_linear_stage("X", "+", 1500, wait_for_stop=True)
# #     move_diagonal(1500, 1500)
# #     move_linear_stage("Y", "+", 1500, wait_for_stop=True)
# #     # start_printing()
# #     move_linear_stage("X", "-", 500, wait_for_stop=True)


# # def trace_3():
# #     # start_printing()
# #     # stop_printing()
# #     move_linear_stage("Y", "-", 1000, wait_for_stop=True)
# #     move_diagonal(-1000, -1000)
# #     move_linear_stage("X", "-", 1000, wait_for_stop=True)
# #     move_diagonal(-1000, 1000)
# #     move_linear_stage("Y", "+", 1000, wait_for_stop=True)
# #     # start_printing()
# #     move_linear_stage("X", "+", 400, wait_for_stop=True)


# # def trace_4():
# #     # start_printing()
# #     move_linear_stage("Y", "-", 800, wait_for_stop=True)
# #     move_diagonal(-800, -800)
# #     move_linear_stage("X", "-", 800, wait_for_stop=True)
# #     move_diagonal(-800, +800)
# #     move_linear_stage("Y", "+", 800, wait_for_stop=True)
# #     # stop_printing()
# #     move_linear_stage("X", "+", 600, wait_for_stop=True)


# # # ============================================================
# # #                     LEFT SIDE TRACES (5–8)
# # # ============================================================

# # def trace_5():
# #     # start_printing()
# #     move_linear_stage("Y", "-", 800, wait_for_stop=True)
# #     move_diagonal(+800, -800)
# #     move_linear_stage("X", "+", 800, wait_for_stop=True)
# #     move_diagonal(+800, +800)
# #     move_linear_stage("Y", "+", 800, wait_for_stop=True)
# #     # stop_printing()
# #     move_linear_stage("X", "+", 800, wait_for_stop=True)


# # def trace_6():
# #     # start_printing()
# #     move_linear_stage("Y", "-", 1000, wait_for_stop=True)
# #     move_diagonal(+1000, -1000)
# #     move_linear_stage("X", "+", 1000, wait_for_stop=True)
# #     move_diagonal(+1000, +1000)
# #     move_linear_stage("Y", "+", 1000, wait_for_stop=True)
# #     # stop_printing()
# #     move_linear_stage("X", "+", 1000, wait_for_stop=True)


# # def trace_7():
# #     # start_printing()
# #     move_linear_stage("Y", "-", 1500, wait_for_stop=True)
# #     move_diagonal(+1500, -1500)
# #     move_linear_stage("X", "+", 1500, wait_for_stop=True)
# #     move_diagonal(+1500, +1500)
# #     move_linear_stage("Y", "+", 1500, wait_for_stop=True)
# #     # stop_printing()
# #     move_linear_stage("X", "+", 1200, wait_for_stop=True)


# # def trace_8():
# #     # start_printing()
# #     move_linear_stage("Y", "-", 2000, wait_for_stop=True)
# #     move_diagonal(+2000, -2000)
# #     move_linear_stage("X", "+", 2000, wait_for_stop=True)
# #     move_diagonal(+2000, +2000)
# #     move_linear_stage("Y", "+", 2000, wait_for_stop=True)
# #     # stop_printing()
# #     move_linear_stage("X", "+", 1400, wait_for_stop=True)


# # # ============================================================
# # #                       RUN ALL TRACES
# # # ============================================================

# # def print_trace_pattern():
# #     print("Running 8-Trace Pattern")
# #     trace_1()
# #     # trace_2()
# #     # trace_3()
# #     # trace_4()
# #     # trace_5()
# #     # trace_6()
# #     # trace_7()
# #     # trace_8()
# #     print("8-Trace Pattern Complete")


# # def test_diagonal():
# #     # start_printing()
# #     move_diagonal(-1000, -1000)
# #     # stop_printing()
# pcb_mapping.py – Full 6-trace pattern with improved diagonal interpolation
# This assumes a fixed position and assumes the PCB is already aligned 
# correctly with the ink dispenser at the starting point.
# It can be modified later by adding a calibration setup if needed.

import time
import math
from motor_control import move_linear_stage

Z_PRINT = 300


# Z CONTROL (your hardware logic)
def start_printing():
    move_linear_stage("Z", "-", Z_PRINT, wait_for_stop=True)


def stop_printing():
    move_linear_stage("Z", "+", Z_PRINT, wait_for_stop=True)



# DIAGONAL MOVEMENT 
def move_diagonal(dx, dy, target_segment_um=50, min_segments=10, max_segments=300):

    total_dist = math.hypot(dx, dy)
    if total_dist < 0.001:
        return

    segments = int(total_dist / target_segment_um)
    if segments < min_segments:
        segments = min_segments
    if segments > max_segments:
        segments = max_segments

    step_x = dx / segments
    step_y = dy / segments

    for _ in range(segments):

        # X axis 
        if step_x != 0:
            move_linear_stage(
                "X",
                "+" if step_x > 0 else "-",   # X+: left, X-: right
                abs(step_x),
                wait_for_stop=False
            )

        # Y axis
        if step_y != 0:
            move_linear_stage(
                "Y",
                "+" if step_y > 0 else "-",   # Y+: down, Y-: up
                abs(step_y),
                wait_for_stop=False
            )

        time.sleep(0.003)  # reduce for more speed




# TRACE SEQUENCES (corrected for your inverted axes)

# RIGHT SIDE

def trace_1():
    ## frist lineer test 
    move_linear_stage("Y", "-", 1000, wait_for_stop=True)   
    stop_printing()
    ##seccond trace 90 degress
    # move_linear_stage("Y", "-", 1000, wait_for_stop=True)   
    # move_linear_stage("X", "-", 1000, wait_for_stop=True)   
    ## 3rd phase diagonal movemnt

    # move_linear_stage("X", "-", 200, wait_for_stop=True)      
    # move_diagonal(-200, 200)                                  
    # move_linear_stage("Y", "+", 100, wait_for_stop=True)      
    # stop_printing()
    # start_printing()
    # move_linear_stage("Y", "+", 150, wait_for_stop=True)       # small lift before next trace

  
# def trace_2():
#     # start_printing()
#     stop_printing()
#     move_linear_stage("Y", "+", 100, wait_for_stop=True)
#     move_diagonal(150, -150)
#     move_linear_stage("X", "+", 150, wait_for_stop=True)
#     move_diagonal(-150, 150)
#     move_linear_stage("Y", "+", 150, wait_for_stop=True)
#     # stop_printing()
#     start_printing()
#     move_linear_stage("Y", "-", 50, wait_for_stop=True)       # small lift before next trace


# def trace_3():
#     # start_printing()
#     stop_printing()
#     move_linear_stage("Y", "-", 100, wait_for_stop=True)
#     move_diagonal(-100, -100)
#     move_linear_stage("X", "-", 100, wait_for_stop=True)
#     move_diagonal(-100, 100)
#     move_linear_stage("Y", "+", 100, wait_for_stop=True)
#     # stop_printing()
#     start_printing()
#     move_linear_stage("X", "+", 400, wait_for_stop=True)       # move to left side start position



# # LEFT SIDE (mirrors, using X+ instead of X-)

# def trace_4():
#     start_printing()
#     move_linear_stage("Y", "-", 200, wait_for_stop=True)
#     move_diagonal(200, -200)                                  # LEFT + UP
#     move_linear_stage("X", "+", 200, wait_for_stop=True)      # LEFT
#     move_diagonal(200, 200)                                   # LEFT + DOWN
#     move_linear_stage("Y", "+", 100, wait_for_stop=True)
#     stop_printing()


# def trace_5():
#     start_printing()
#     move_linear_stage("Y", "-", 150, wait_for_stop=True)
#     move_diagonal(150, -150)
#     move_linear_stage("X", "+", 150, wait_for_stop=True)
#     move_diagonal(150, 150)
#     move_linear_stage("Y", "+", 100, wait_for_stop=True)
#     stop_printing()


# def trace_6():
#     start_printing()
#     move_linear_stage("Y", "-", 100, wait_for_stop=True)
#     move_diagonal(100, -100)
#     move_linear_stage("X", "+", 100, wait_for_stop=True)
#     move_diagonal(100, 100)
#     move_linear_stage("Y", "+", 100, wait_for_stop=True)
#     stop_printing()



def print_trace_pattern():
    print("Running 6-Trace Pattern")

    trace_1()
    # trace_2()
    # trace_3()

    # move_linear_stage("X", "+", 400, wait_for_stop=True)

    # trace_4()
    # trace_5()
    # trace_6()

    print("6-Trace Pattern Complete")



def test_diagonal():
    start_printing()
    move_diagonal(-200, -200)
    stop_printing()