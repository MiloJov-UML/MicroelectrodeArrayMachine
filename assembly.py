# # # assembly.py

# # import time
# # from motor_control import (
# #     update_speed,
# #     return_to_origin,
# #     stop_motor_control,
# #     move_linear_stage,
# # )
# # from relay_control import (
# #     laser_relay_on,
# #     laser_relay_off,
# # )
# # from print import (
# #     glue_sequence,
# #     print_pcb,
# #     print_pad,
# #     pad_types,
# # )
# # from image_recognition import (
# #     extrude,
# #     x_align,
# #     r_align
# # )

# # # Sequence flags — set to True to enable in full assembly run
# # print_traces_seq =  ("print",      False)  # print traces + connector pads
# # rotate_neg_seq =    ("rotate_neg", False)  # rotate -90 to placement station
# # placement_seq =     ("placement",  False)  # extrude + align + place microwires
# # rotate_pos_seq =    ("rotate_pos", False)  # rotate +90 back to print station
# # fill_pads_seq =     ("fill",       False)  # fill electrode pads with metal ink

# # sequences = (print_traces_seq, rotate_neg_seq, placement_seq, rotate_pos_seq, fill_pads_seq)

# # def fill_electrode_pads():
# #     """Fill all 8 electrode pads with metal ink after wire placement."""
# #     # to be implemented — print only electrode pads
# #     pass

# # def sequence_handler(seq):
# #     name = seq[0]
# #     if name == "print":
# #         print_pcb()
# #     elif name == "rotate_neg":
# #         # rotate -90 to placement station
# #         update_speed(50)
# #         move_linear_stage('r', '-', 90, wait_for_stop=True, max_wait=30.0)
# #     elif name == "placement":
# #         # extrude + align + place — to be implemented
# #         pass
# #     elif name == "rotate_pos":
# #         # rotate +90 back to print station
# #         update_speed(50)
# #         move_linear_stage('r', '+', 90, wait_for_stop=True, max_wait=30.0)
# #     elif name == "fill":
# #         fill_electrode_pads()

# # def run_full_assembly():
# #     for seq in sequences:
# #         if seq[1] == True:
# #             print(f"Running sequence: {seq[0]}")
# #             sequence_handler(seq)
# #     print("Full assembly complete.")

# # assembly.py

# # assembly.py

# import time
# from motor_control import (
#     update_speed,
#     move_linear_stage,
# )
# from print import (
#     print_pcb,
#     fill_electrode_pads,
#     calibrate,
#     x_home,
#     y_home,
#     z_home,
#     x, y, z,
# )

# def run_full_assembly():
#     """
#     Full assembly sequence:
#     1. Calibrate and print traces
#     2. Return to home
#     3. Rotate -90 to placement station
#     4. Wait 20 seconds (test value)
#     5. Adjust axes slightly
#     6. Rotate +90 back
#     7. Calibrate and print again
#     8. Fill electrode pads
#     """
#     print("Starting full assembly...")

#     # Step 1 — calibrate and print traces
#     calibrate()
#     print_pcb()

#     # Step 2 — return to home
#     update_speed(100)
#     z_home()
#     y_home()
#     x_home()

#     # Step 3 — rotate -90 to placement station
#     update_speed(50)
#     move_linear_stage('r', '-', 90, wait_for_stop=True, max_wait=30.0)

#     # Step 4 — wait 20 seconds for testing
#     print("Waiting 20 seconds...")
#     time.sleep(20)

#     # Step 5 — adjust axes slightly (test values)
#     move_linear_stage(x, '-', 5000, wait_for_stop=True, max_wait=30.0)
#     move_linear_stage(y, '-', 5000, wait_for_stop=True, max_wait=30.0)
#     move_linear_stage(z, '+', 5000, wait_for_stop=True, max_wait=30.0)

#     # Step 6 — rotate +90 back to print station
#     move_linear_stage('r', '+', 90, wait_for_stop=True, max_wait=30.0)

#     # Step 7 — calibrate and print again
#     calibrate()
#     print_pcb()

#     # Step 8 — fill electrode pads
#     fill_electrode_pads()

#     print("Full assembly complete.")

from print import full_sequence

def run_full_assembly():
    full_sequence()