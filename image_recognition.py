# image_recognition.py

import math
import cv2
import os
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

record_dir0 = r"D:\camera0_pcb2_CFmicrowire_2025-03-27"
record_dir1 = r"D:\camera1_pcb2_CFmicrowire_2025-03-27"

video_writers = {0: None, 1: None}
run_timestamps = {0: None, 1: None}

frames_per_still = 30
frame_counts = {0: 0, 1: 0}

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


def custom_annotate(results, img):
    if not draw_bounding_boxes:
        return img.copy()

    annotated_img = img.copy()
    boxes = results.boxes
    names = results.names

    pad_boxes = []

    for box in boxes:
        cls_id = int(box.cls[0])
        class_name = names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        center_y = (y1 + y2)/2

        # If it's the main "Pad", we store it for reference
        if class_name == "Pad":
            pad_boxes.append((x1, y1, x2, y2, center_y, conf))
        else:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # Sort pad boxes top->bottom
    pad_boxes.sort(key=lambda b: b[4])
    pad_index = 8
    for (x1,y1,x2,y2,cy,conf) in pad_boxes:
        label = f"pad{pad_index} {conf:.2f}"
        pad_index-=1
        cv2.rectangle(annotated_img,(x1,y1),(x2,y2),(255,255,0),2)
        cv2.putText(annotated_img,label,(x1 + 3,y2 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,0),1)

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

    model = YOLO(model_path)
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[Camera {camera_index}] cannot open camera.")
        return

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
        results = model.predict(frame, conf=0.4, verbose=False)
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

def compute_steps_per_pixel(bboxA, bboxB, axis='X', known_um=1000):
    """
    1) Measures the pixel distance between two bounding boxes (bboxA, bboxB).
    2) Uses the fact that physically they are 'known_um' micrometers apart.
    3) Converts that known_um to steps (using µm_to_steps from motor_control).
    4) Returns steps_per_pixel, i.e. how many motor steps correspond to 1 pixel.
    
    Example usage:
        steps_pp = compute_steps_per_pixel(pad_box, cf_box, axis='X', known_um=1000)
        # 1 px => steps_pp motor steps
    """
    (cxA, cyA) = center_of_bbox(bboxA)
    (cxB, cyB) = center_of_bbox(bboxB)
    pixel_dist = math.hypot(cxB - cxA, cyB - cyA)
    if pixel_dist < 0.01:
        # Avoid division by zero if boxes are nearly the same center
        return 0.0

    steps_for_known_um = µm_to_steps(known_um, axis=axis)
    steps_per_pixel = steps_for_known_um / pixel_dist  # steps / px
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
def extrude(max_iterations=50, known_um=1000):
    """
    Moves the 't' axis in +100µm increments to align Pad & CF_Tip horizontally 
    within ±10µm. We'll measure the distance in steps or µm by:
      1) Using compute_steps_per_pixel => steps_per_pixel
      2) Dist(px) * steps_per_pixel => motor steps
         or Dist(px) * (known_um / px_dist) => real µm

    We'll do jam detection: if CF_Tip doesn't shift by >=50µm, we undo the step.
    """
    import time

    print("[Extrude] Starting extrude based on Pad–CF distance in µm...")

    from motor_control import update_speed, move_linear_stage

    global last_pad_box, last_cf_box

    # 1) slow speed
    update_speed(1)

    step_size_um = 100.0
    distance_tolerance_um = 10.0
    jam_threshold_um = 50.0
    wait_between_moves = 2.0  # seconds

    # We'll store the last CF Tip center in px for jam detection
    last_cf_x_px = None

    for attempt in range(max_iterations):
        print(f"[Extrude] Attempt {attempt+1}/{max_iterations}")

        # If we have bounding boxes for Pad & CF:
        if (last_pad_box is not None) and (last_cf_box is not None):
            # 1) measure steps_per_pixel => from known_um
            steps_pp = compute_steps_per_pixel(last_pad_box, last_cf_box, axis='X', known_um=known_um)
            # or measure px_dist directly
            (pxA, pyA) = center_of_bbox(last_pad_box)
            (pxB, pyB) = center_of_bbox(last_cf_box)
            px_dist = abs(pxB - pxA)

            if px_dist < 0.01:
                print("[Extrude] CF Tip and Pad appear at same center => done.")
                return

            # 2) Convert px_dist => real µm if we assume 'known_um' => px_dist_of_known
            # but we have compute_steps_per_pixel => steps/pixel => we can do:
            # steps_for_current_dist = px_dist * steps_pp
            # or we do simpler approach: ratio => (px_dist / known_px_dist) * known_um
            # We'll do a direct ratio approach:
            # Dist in steps => px_dist * steps_pp
            dist_in_steps = px_dist * steps_pp
            # Convert steps => µm using motor_control steps->µm
            # We invert µm_to_steps => steps => µm
            # We'll define a helper: steps_to_µm(steps, axis='X')
            from motor_control import steps_to_µm
            dist_in_um = steps_to_µm(dist_in_steps, axis='X')

            print(f"    Pad–CF horizontal distance => ~{dist_in_um:.1f} µm")

            if dist_in_um <= distance_tolerance_um:
                print(f"[Extrude] Aligned within ±{distance_tolerance_um}µm. Done.")
                return

            # jam detection reference
            last_cf_x_px = pxB
        else:
            print("    Missing bounding boxes => can't measure distance. Using old data if any.")
            # if we never had bounding boxes => we can't do anything
            if last_cf_x_px is None:
                print("    We have no prior data => continuing anyway.")
        
        # 2) Move +100µm in 't'
        print(f"    Move +{step_size_um}µm along 't' axis.")
        move_linear_stage('t', '+', step_size_um, wait_for_stop=True, max_wait=30.0)

        # 3) wait so YOLO can update
        time.sleep(wait_between_moves)

        # 4) jam detection
        if (last_pad_box is not None) and (last_cf_box is not None) and (last_cf_x_px is not None):
            new_cf_x_px = center_of_bbox(last_cf_box)[0]
            shift_in_px = abs(new_cf_x_px - last_cf_x_px)
            # We'll measure shift in px => convert to µm using the same steps_pp or ratio:
            # steps_pp again:
            steps_pp_now = compute_steps_per_pixel(last_pad_box, last_cf_box, axis='X', known_um=known_um)
            shift_in_steps = shift_in_px * steps_pp_now
            from motor_control import steps_to_µm
            shift_in_um = steps_to_µm(shift_in_steps, axis='X')

            if shift_in_um < jam_threshold_um:
                print(f"    Jam? CF Tip advanced only {shift_in_um:.1f}µm. Undo step.")
                # move -100µm
                move_linear_stage('t', '-', step_size_um, wait_for_stop=True, max_wait=30.0)
                time.sleep(wait_between_moves)
            else:
                print(f"    CF Tip advanced ~{shift_in_um:.1f}µm => OK.")
        else:
            print("    Missing bounding boxes => skipping jam detection this iteration.")

    print(f"[Extrude] Gave up after {max_iterations} attempts (>±{distance_tolerance_um}µm?).")

# --------------------------------------------------------
# R-axis alignment: measure angle in degrees using compute_angle_between
# --------------------------------------------------------
def r_align(angle_tolerance=0.5):
    """
    1) Check we have CF_Tip (last_cf_box) and GC_Tip (last_gc_box).
    2) Compute angle_degs from compute_angle_between(...), which you set to return 
       the correct sign for a direct rotation on axis 'r'.
    3) If abs(angle_degs) < angle_tolerance => print "already aligned" and return.
    4) Else, update_speed(10), rotate 'r' axis by angle_degs, then re-check and print final angle.
    """

    from motor_control import update_speed, move_linear_stage

    global last_cf_box, last_gc_box

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
        return

    # 3) Set speed to 10 for the rotation
    update_speed(1)

    # 4) Move the 'r' axis by that angle (assuming sign is correct as you said)
    direction = '+'
    displacement = abs(initial_angle_degs)
    if initial_angle_degs < 0:
        direction = '-'

    print(f"[r_align] Rotating r-axis by {direction}{displacement:.2f}°...")
    move_linear_stage('r', direction, displacement, wait_for_stop=True, max_wait=30.0)
