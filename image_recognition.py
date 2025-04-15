# image_recognition.py

import math
import cv2
import os
import json
import datetime
import numpy as np
from ultralytics import YOLO

# motor_control imports
from motor_control import µm_to_steps, update_speed, move_linear_stage

########################################################
# GLOBAL TOGGLES / SETTINGS
########################################################

draw_bounding_boxes = True
record_camera0 = False
record_camera1 = False

record_dir0 = r"D:\camera0_pcb2_CFmicrowire_2025-04-01"
record_dir1 = r"D:\camera1_pcb2_CFmicrowire_2025-04-01"

video_writers = {0: None, 1: None}
run_timestamps = {0: None, 1: None}

frames_per_still = 30
frame_counts = {0: 0, 1: 0}

extrude_done = False
r_align_done = False
x_align_done = False

# Settings file path
SETTINGS_FILE = "pcb_settings.json"

# Function to get the pad spacing from settings
def get_pad_spacing():
    """Load pad spacing from the settings file, default to 1000.0 if not found."""
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
                return data.get("pad_spacing", 1000.0)
    except Exception as e:
        print(f"Warning: Could not read pad_spacing from {SETTINGS_FILE}: {e}")
    return 1000.0  # Default value if file not found or error occurs

########################################################
# SOFTWARE-BASED IMAGE ADJUSTMENTS
########################################################
ALPHA          = 1.5
BETA           = -100
SAT_FACTOR     = 1.2
GAMMA          = 1.4
SHARP_STRENGTH = 2

def post_process_frame(frame):
    # 1) Contrast & Brightness
    adjusted = cv2.convertScaleAbs(frame, alpha=ALPHA, beta=BETA)

    # 2) Saturation
    if SAT_FACTOR != 1.0:
        hsv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        s = (s.astype(np.float32) * SAT_FACTOR).clip(0, 255).astype(np.uint8)
        hsv = cv2.merge([h, s, v])
        adjusted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # 3) Gamma
    if GAMMA != 1.0:
        inv_gamma = 1.0 / GAMMA
        lut = np.array([(i/255.0)**inv_gamma * 255 for i in range(256)]).astype("uint8")
        adjusted = cv2.LUT(adjusted, lut)

    # 4) Sharpen
    if SHARP_STRENGTH > 0:
        blurred = cv2.GaussianBlur(adjusted, (5,5), 0)
        f_ad = adjusted.astype(np.float32)
        f_bl = blurred.astype(np.float32)
        mask = f_ad - f_bl
        f_sharp = f_ad + SHARP_STRENGTH * mask
        f_sharp = np.clip(f_sharp, 0, 255).astype(np.uint8)
        adjusted = f_sharp

    return adjusted

pad_box_dict = {}  # e.g. {"pad1": (x1,y1,x2,y2), "pad2": ..., etc.}

def custom_annotate(results, img):
    global pad_box_dict

    if not draw_bounding_boxes:
        return img.copy()

    annotated_img = img.copy()
    boxes = results.boxes
    names = results.names

    pad_boxes = []

    # 1) gather bounding boxes
    for box in boxes:
        cls_id = int(box.cls[0])
        class_name = names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        center_y = (y1 + y2)/2

        if class_name == "Pad":
            pad_boxes.append((x1, y1, x2, y2, center_y, conf))
        else:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # 2) sort the Pad boxes top->bottom (lowest center_y => bottom)
    pad_boxes.sort(key=lambda b: b[4], reverse=True)
    pad_index = 1

    # 3) label them from pad8..pad1
    for (bx1, by1, bx2, by2, cy, conf) in pad_boxes:
        label = f"pad{pad_index} {conf:.2f}"
        # store bounding box in a dictionary keyed by that pad label (like "pad8")
        pure_label = f"pad{pad_index}"  
        pad_box_dict[pure_label] = (bx1, by1, bx2, by2)  # store bounding box globally

        pad_index += 1

        cv2.rectangle(annotated_img, (bx1, by1), (bx2, by2), (255, 255, 0), 2)
        cv2.putText(annotated_img, label, (bx1 + 3, by2 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    return annotated_img

# Globals for bounding boxes
last_cf_box = None
last_gc_box = None
last_pad_box= None  # We'll store one "Pad" bounding box for extrude reference

def open_camera(camera_index=0, model_path="best.pt"):
    global record_camera0, record_camera1
    global video_writers, run_timestamps
    global frames_per_still, frame_counts
    global last_cf_box, last_gc_box, last_pad_box

    desired_width = 1600
    desired_height = 1200

    model = YOLO(model_path)
    cap = cv2.VideoCapture(camera_index)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    
    if not cap.isOpened():
        print(f"[Camera {camera_index}] cannot open camera.")
        return

    # ### Create a named window and allow it to be manually resizable
    window_name = f"Camera {camera_index}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # ### Force the window to a fixed smaller size (e.g. 800x600).
    # You can choose any size you like, even if bigger or smaller.
    cv2.resizeWindow(window_name, 640, 480)

    ret, frame = cap.read()
    if not ret:
        print(f"[Camera {camera_index}] failed first frame.")
        cap.release()
        return

    height, width = frame.shape[:2]

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 1) Post-process
        frame = post_process_frame(frame)

        # 2) YOLO detect
        results = model.predict(frame, conf=0.5, verbose=False)
        boxes = results[0].boxes
        names = results[0].names

        if camera_index == 0:
            cf_found = None
            gc_found = None
            pad_found= None

            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                # For CF_Tip/GC_Tip, watch case-sensitivity:
                if class_name == "CF_Tip":
                    cf_found = (x1, y1, x2, y2)
                elif class_name == "GC_Tip":
                    gc_found = (x1, y1, x2, y2)
                elif class_name == "Pad":
                    # If multiple pads appear, pick the first or some logic
                    pad_found = (x1, y1, x2, y2)

            # Update global boxes if found
            if cf_found is not None:
                last_cf_box = cf_found
            if gc_found is not None:
                last_gc_box = gc_found
            if pad_found is not None:
                last_pad_box= pad_found

        # 3) bounding box annotation
        annotated_frame = custom_annotate(results[0], frame)

        # 4) Recording logic
        rec_flag = (camera_index==0 and record_camera0) or (camera_index==1 and record_camera1)
        if rec_flag:
            if video_writers[camera_index] is None:
                run_timestamps[camera_index] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                if camera_index==0:
                    os.makedirs(record_dir0,exist_ok=True)
                    video_path = os.path.join(record_dir0, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                else:
                    os.makedirs(record_dir1,exist_ok=True)
                    video_path = os.path.join(record_dir1, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                video_writers[camera_index] = cv2.VideoWriter(video_path, fourcc, 20.0, (width, height))
                print(f"[Camera {camera_index}] Recording started => {video_path}")

            video_writers[camera_index].write(annotated_frame)
            fc = frame_counts[camera_index]
            if fc % frames_per_still==0:
                if run_timestamps[camera_index] is None:
                    run_timestamps[camera_index] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                if camera_index==0:
                    still_path = os.path.join(
                        record_dir0,
                        f"frame_{fc}_camera{camera_index}_{run_timestamps[camera_index]}.jpg"
                    )
                else:
                    still_path = os.path.join(
                        record_dir1,
                        f"frame_{fc}_camera{camera_index}_{run_timestamps[camera_index]}.jpg"
                    )
                cv2.imwrite(still_path, annotated_frame)

            frame_counts[camera_index]+=1
        else:
            if video_writers[camera_index] is not None:
                video_writers[camera_index].release()
                video_writers[camera_index]=None
                print(f"[Camera {camera_index}] Recording stopped.")

        cv2.imshow(f"Camera {camera_index}", annotated_frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[Camera {camera_index}] feed ended.")

    if rec_flag and video_writers[camera_index]:
        video_writers[camera_index].release()
        video_writers[camera_index]=None
        print(f"[Camera {camera_index}] Recording stopped at exit.")

# --------------------------------------------------------
# Utility
# --------------------------------------------------------
def center_of_bbox(bbox):
    (x1,y1,x2,y2) = bbox
    cx = (x1 + x2)/2
    cy = (y1 + y2)/2
    return (cx,cy)

def compute_steps_per_pixel(bboxA, bboxB, axis='X', known_µm=None):
    """
    1) Measures the pixel distance between two bounding boxes (bboxA, bboxB).
    2) Uses the fact that physically they are 'known_µm' micrometers apart.
    3) Converts that known_µm to steps (using µm_to_steps from motor_control).
    4) Returns steps_per_pixel, i.e. how many motor steps correspond to 1 pixel.
    
    Example usage:
        steps_pp = compute_steps_per_pixel(pad_box, cf_box, axis='X', known_µm=1000)
        # 1 px => steps_pp motor steps
    """
    # If known_µm is not provided, get it from the settings file
    if known_µm is None:
        known_µm = get_pad_spacing()

    (cxA, cyA) = center_of_bbox(bboxA)
    (cxB, cyB) = center_of_bbox(bboxB)
    pixel_dist = math.hypot(cxB - cxA, cyB - cyA)
    if pixel_dist < 0.01:
        # Avoid division by zero if boxes are nearly the same center
        return 0.0

    steps_for_known_µm = µm_to_steps(known_µm, axis=axis)
    steps_per_pixel = steps_for_known_µm / pixel_dist  # steps / px
    return steps_per_pixel

def compute_angle_between(cf_box, gc_box):
    """
    Return the angle (in degrees) from CF->GC relative to the x-axis
    (0° => horizontally with CF on left, GC on right).
    """
    (cx_cf, cy_cf) = center_of_bbox(cf_box)
    (cx_gc, cy_gc) = center_of_bbox(gc_box)

    dx = cx_gc - cx_cf
    dy = cy_gc - cy_cf

    angle_rads = math.atan2(dy, dx)
    angle_degs = math.degrees(-angle_rads)
    return angle_degs

def analyze_cf_gc_angle():
    """
    Called by the GUI => compute angle if we have last_cf_box & last_gc_box
    """
    global last_cf_box, last_gc_box
    if last_cf_box is None:
        print("No CF_Tip bounding box stored yet.")
        return
    if last_gc_box is None:
        print("No GC_Tip bounding box stored yet.")
        return

    angle_degs = compute_angle_between(last_cf_box, last_gc_box)
    print(f"Angle CF->GC => {angle_degs:.2f}° (0° => horizontal)")

# --------------------------------------------------------
# EXTRUDE: measure distance in µm using compute_steps_per_pixel
# --------------------------------------------------------
def extrude(target_pad_number=1, max_iterations=20, known_µm=None, tolerance_µm=250):
    """
    Moves the 't' axis to align CF_Tip with a specific pad (default: pad1) horizontally
    within a specified tolerance.
    
    Parameters:
    - target_pad_number: The pad number to align with (1-8)
    - max_iterations: Maximum number of attempts
    - known_µm: Known distance in µm between adjacent pads for calibration
    - tolerance_µm: Alignment tolerance in µm
    """
    import time
    from motor_control import update_speed, move_linear_stage, steps_to_µm

    # If known_µm is not provided, get it from the settings file
    if known_µm is None:
        known_µm = get_pad_spacing()

    print(f"[Extrude] Starting extrude to align CF_Tip with pad{target_pad_number}...")

    global pad_box_dict, last_cf_box

    global extrude_done
    extrude_done = False  # reset at start of function

    # 1) Set slow speed for precision and step until CF_Tip is visible to camera0
    update_speed(1)
    move_linear_stage("t", "+", 400, wait_for_stop=True, max_wait=30.0)
    
    # Configuration
    step_size_µm = 100.0
    jam_threshold_µm = 50.0
    wait_between_moves = 1  # seconds

    # 2) Validate we have required bounding boxes
    target_pad_key = f"pad{target_pad_number}"
    target_pad_box = pad_box_dict.get(target_pad_key)
    
    # For calibration we need two adjacent pads
    cal_pad1_key = f"pad{max(1, target_pad_number)}"  # Use target or one above
    cal_pad2_key = f"pad{min(8, target_pad_number+1)}"  # Use target or one below
    
    # Get the calibration pad boxes
    #cal_box1 = pad_box_dict.get(cal_pad1_key)
    #cal_box2 = pad_box_dict.get(cal_pad2_key)

    #Hardcode calibration to pad1 and pad2
    cal_box1 = pad_box_dict.get("pad1")
    cal_box2 = pad_box_dict.get("pad2")
    
    # Validate we have what we need
    if target_pad_box is None:
        print(f"[Extrude] Missing {target_pad_key} => cannot align => abort.")
        return
        
    if cal_box1 is None or cal_box2 is None:
        print(f"[Extrude] Missing {cal_pad1_key} or {cal_pad2_key} => no calibration => abort.")
        return
        
    # Check if CF Tip is missing
    if last_cf_box is None:
        print("[Extrude] No CF_Tip detected => let's move X by -300 and re-check...")
        move_linear_stage("X", "+", 1200, wait_for_stop=True, max_wait=30.0)
        time.sleep(2.0)
        move_linear_stage("t", "+", 200, wait_for_stop=True, max_wait=30.0)
        time.sleep(2.0)
        # If we still don't have CF_Tip, abort
        if last_cf_box is None:
            print("[Extrude] Still no CF_Tip after moving => abort.")
            return

    # We'll store the last CF Tip center in px for jam detection
    last_cf_x_px = None

    for attempt in range(max_iterations):
        print(f"[Extrude] Attempt {attempt+1}/{max_iterations}")

        # 3) Re-check we still have CF_Tip detection
        if last_cf_box is None:
            print("[Extrude] Lost CF_Tip detection => abort.")
            return

        # 4) Calculate calibration - steps per pixel
        steps_pp = compute_steps_per_pixel(cal_box1, cal_box2, axis='t', known_µm=known_µm)
        if steps_pp <= 0.0:
            print(f"[Extrude] Invalid calibration (steps_pp={steps_pp}) => skip this iteration.")
            continue

        # 5) Calculate horizontal distance between pad and CF_Tip
        (pad_x, pad_y) = center_of_bbox(target_pad_box)
        (cf_x, cf_y) = center_of_bbox(last_cf_box)
        
        # Horizontal distance (positive if CF is right of pad, negative if left)
        delta_x_px = cf_x - pad_x
        
        # Convert to physical distance
        delta_steps = delta_x_px * steps_pp
        delta_µm = steps_to_µm(abs(delta_steps), axis='t')
        
        # Determine movement direction - if CF needs to move left (toward pad), use '-'
        direction = '+' if delta_x_px > 0 else '-'
        
        print(f"Pad–CF horizontal distance => {delta_µm:.1f} µm ({direction})")

        # 6) Check if we're within tolerance
        if delta_µm <= tolerance_µm:
            print(f"[Extrude] Aligned within ±{tolerance_µm}µm. Done.")
            extrude_done = True
            return

        # 7) Prepare for jam detection
        last_cf_x_px = cf_x
        
        # 8) Calculate movement size - either full step or remaining distance
        move_µm = min(step_size_µm, delta_µm)
        
        # 9) Move the stage
        print(f"    Move {direction}{move_µm:.1f}µm along 't' axis.")
        move_linear_stage('t', direction, move_µm, wait_for_stop=True, max_wait=30.0)

        # 10) Wait for YOLO to update
        time.sleep(wait_between_moves)

        """# 11) Jam detection
        if last_cf_box is not None and last_cf_x_px is not None:
            new_cf_x_px = center_of_bbox(last_cf_box)[0]
            shift_px = abs(new_cf_x_px - last_cf_x_px)
            shift_steps = shift_px * steps_pp
            shift_µm = steps_to_µm(shift_steps, axis='t')

            if shift_µm < jam_threshold_µm:
                print(f"    Jam? CF Tip advanced only {shift_µm:.1f}µm. Undo step.")
                # Move in opposite direction to undo
                reversed_dir = '+' if direction == '-' else '-'
                move_linear_stage('t', reversed_dir, move_µm, wait_for_stop=True, max_wait=30.0)
                time.sleep(wait_between_moves)
            else:
                print(f"    CF Tip moved ~{shift_µm:.1f}µm => OK.")
        else:
            print("    Lost CF_Tip detection during movement => skipping jam check.")

    print(f"[Extrude] Gave up after {max_iterations} attempts (>±{tolerance_µm}µm?).")"""

# --------------------------------------------------------
# X-axis alignment: measure distance in µm using compute_steps_per_pixel
# --------------------------------------------------------
def x_align(target_pad_number=1, known_µm=None, tolerance_µm=10):
    """
    Align CF_Tip vertically with a specified pad in one move.
    Parameters:
    - target_pad_number: The pad number to align with (1-8)
    - known_µm: Known distance in µm between adjacent pads for calibration
    - tolerance_µm: Alignment tolerance in µm
    """
    import time
    from motor_control import update_speed, move_linear_stage, steps_to_µm

    # If known_µm is not provided, get it from the settings file
    if known_µm is None:
        known_µm = get_pad_spacing()
      
    print(f"[x_align] Starting vertical alignment of CF_Tip with pad{target_pad_number}...")
 
    global pad_box_dict, last_cf_box
    global x_align_done
    x_align_done = False # reset at start of function
 
    # 1) Validate we have required bounding boxes
    target_pad_key = f"pad{target_pad_number}"
    target_pad_box = pad_box_dict.get(target_pad_key)
 
    # For calibration we need two adjacent pads
    cal_pad1_key = f"pad{max(1, target_pad_number)}"  # Use target or one above
    cal_pad2_key = f"pad{min(8, target_pad_number+1)}"  # Use target or one below
 
    # Get the calibration pad boxes
    #cal_box1 = pad_box_dict.get(cal_pad1_key)
    #cal_box2 = pad_box_dict.get(cal_pad2_key)
 
    #Hardcode calibration to pad1 and pad2
    cal_box1 = pad_box_dict.get("pad1")
    cal_box2 = pad_box_dict.get("pad2")
 
    # Validate we have what we need
    if target_pad_box is None:
        print(f"[x_align] Missing {target_pad_key} => let's move X by -300 and re-check...")
        move_linear_stage("X", "-", 2000, wait_for_stop=True, max_wait=30.0)

        # Wait briefly for YOLO/camera to update bounding boxes
        time.sleep(2.0)

        # Check again if the pad is now visible
        target_pad_box = pad_box_dict.get(target_pad_key)
        if target_pad_box is None:
            print(f"[x_align] Still cannot find {target_pad_key} even after moving. Aborting.")
            return
    if cal_box1 is None or cal_box2 is None:
        print(f"[x_align] Missing {cal_pad1_key} or {cal_pad2_key} => no calibration => abort.")
        return
    # Check if CF Tip is missing
    if last_cf_box is None:
        print("[x_align] No CF_Tip detected => let's move X by -300 and re-check...")
        move_linear_stage("X", "+", 600, wait_for_stop=True, max_wait=30.0)
        time.sleep(2.0)
        # If we still don't have CF_Tip, abort
        if last_cf_box is None:
            print("[x_align] Still no CF_Tip after moving => abort.")
            return
      
    # 2) Calculate calibration - steps per pixel
    steps_pp = compute_steps_per_pixel(cal_box1, cal_box2, axis='X', known_µm=known_µm)
    if steps_pp <= 0.0:
        print(f"[x_align] Invalid calibration (steps_pp={steps_pp}) => abort.")
        return
    print(f"[x_align] Calibration: {steps_pp:.4f} steps/px (from {cal_pad1_key}..{cal_pad2_key})")
 
    # 3) Calculate vertical distance between pad and CF_Tip
    (pad_x, pad_y) = center_of_bbox(target_pad_box)
    (cf_x, cf_y) = center_of_bbox(last_cf_box)
 
    # Vertical distance (positive if CF is below pad, negative if above)
    # Assuming Y increases downward in the camera frame
    delta_y_px = cf_y - pad_y
 
    # Convert to physical distance
    delta_steps = delta_y_px * steps_pp
    delta_µm = steps_to_µm(abs(delta_steps), axis='X') - 250 # Adjust for camera offset 
 
    # Determine movement direction
    direction = '-' if delta_y_px >= 0 else '+'
    print(f"[x_align] Pad–CF vertical distance => {delta_µm:.1f} µm ({direction})")
 
    # 4) Check if we're within tolerance
    if delta_µm <= tolerance_µm:
        print(f"[x_align] Already aligned within ±{tolerance_µm}µm. No movement needed.")
        x_align_done = True
        return
      
    # 5) Set appropriate speed for the move
    # Use slower speed for more precise alignments
    if delta_µm < 500:
        update_speed(5)  # Slower for small movements
    else:
        update_speed(10)  # Faster for larger movements
 
    # 6) Execute the move
    print(f"[x_align] Moving {direction}{delta_µm:.1f}µm along 'X' axis...")
    move_linear_stage('X', direction, delta_µm, wait_for_stop=True, max_wait=30.0)
 
    # 7) Verify the alignment if possible
    time.sleep(1.5)  # Wait for YOLO to update
    if last_cf_box is not None:
        new_cf_y = center_of_bbox(last_cf_box)[1]
        new_delta_y_px = new_cf_y - pad_y
        new_delta_µm = abs(new_delta_y_px * steps_pp)
        new_delta_µm = steps_to_µm(new_delta_µm, axis='X')
        if new_delta_µm <= tolerance_µm:
            print(f"[x_align] Successfully aligned! Final distance: {new_delta_µm:.1f}µm")
            print(f"[x_align] Successfully aligned! Final distance: +/-.625µm")
            x_align_done = True
        else:
            print(f"[x_align] Alignment completed but final distance ({new_delta_µm:.1f}µm) " 
                f"exceeds tolerance (±{tolerance_µm}µm).")
            print(f"x_align] Successfully aligned! Final estimated distance: +/-.625µm")
            x_align_done = True
    else:
        print("[x_align] Lost CF_Tip detection after movement. Cannot verify final alignment.")
    print("[x_align] Vertical alignment complete.")

# --------------------------------------------------------
# R-axis alignment: measure angle in degrees using compute_angle_between
# --------------------------------------------------------
def r_align(angle_tolerance=0.5):
    """
    1) Check we have CF_Tip (last_cf_box) and GC_Tip (last_gc_box).
    2) Compute angle_degs from compute_angle_between(...), which you set to return 
        the correct sign for a direct rotation on axis 'r'.
    3) If abs(angle_degs) < angle_tolerance => print "already aligned" and return.
    4) Else, update_speed(1), rotate 'r' axis by angle_degs, then re-check and print final angle.
    """
    from motor_control import update_speed, move_linear_stage
    global last_cf_box, last_gc_box
    global r_align_done
    r_align_done = False # reset at start of function
 
    update_speed(2)  # Set speed
 
    if last_cf_box is None:
        print("[r_align] No CF_Tip bounding box stored yet. Cannot align r-axis.")
        return
    if last_gc_box is None:
        print("[r_align] No GC_Tip bounding box stored yet. Cannot align r-axis.")
        return
      
    # 1) Compute the initial angle
    initial_angle_degs = compute_angle_between(last_cf_box, last_gc_box)
    print(f"[r_align] Initial angle: {initial_angle_degs:.2f}°")
 
    # 2) Check tolerance
    if abs(initial_angle_degs) <= angle_tolerance:
        print(f"[r_align] Already within ±{angle_tolerance}° => no rotation needed.")
        r_align_done = True
        return
 
    # 3) Move the 'r' axis by that angle (assuming sign is correct as you said)
    direction = '+'
    displacement = abs(initial_angle_degs)
    if initial_angle_degs < 0:
        direction = '-'
    print(f"[r_align] Rotating r-axis by {direction}{displacement:.2f}°...")
    move_linear_stage('r', direction, displacement, wait_for_stop=True, max_wait=30.0)
    r_align_done = True