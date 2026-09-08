# image_recognition.py

import math
import threading
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
record_camera2 = False

def _create_unique_daily_record_dir(root_dir, folder_prefix):
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    base_name = f"{folder_prefix}_{date_str}"
    candidate = os.path.join(root_dir, base_name)

    suffix = 1
    while os.path.exists(candidate):
        candidate = os.path.join(root_dir, f"{base_name}_{suffix}")
        suffix += 1

    os.makedirs(candidate, exist_ok=False)
    return candidate

record_dir0 = None
record_dir1 = None
record_dir2 = None

video_writers = {0: None, 1: None, 2: None}  
run_timestamps = {0: None, 1: None, 2: None}

frames_per_still = 30
frame_counts = {0: 0, 1: 0, 2: 0}

extrude_done = False
r_align_done = False
x_align_done = False
center_on_visible_area_done = False
last_r_align_angle = 0.0  # signed angle of the most recent r_align rotation (0 = none)
target_pad_ref_y = None  # target pad center Y-pixel position recorded before r_align rotates it

auto_annotate = False
ANNOTATION_DIR = r"D:\Labeled_images"

def _save_yolo_annotation(still_path, results, img_w, img_h):
    """Write a YOLO-format .txt annotation alongside a saved still frame."""
    os.makedirs(ANNOTATION_DIR, exist_ok=True)
    stem = os.path.splitext(os.path.basename(still_path))[0]
    txt_path = os.path.join(ANNOTATION_DIR, stem + ".txt")
    lines = []
    for box in results.boxes:
        cls_id = int(box.cls[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx = ((x1 + x2) / 2) / img_w
        cy = ((y1 + y2) / 2) / img_h
        bw = (x2 - x1) / img_w
        bh = (y2 - y1) / img_h
        lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    with open(txt_path, 'w') as f:
        f.write('\n'.join(lines))

_IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif'}

def annotate_folder(folder_path, model_path="best.pt", conf=0.5, progress_callback=None):
    """
    Run the YOLO model on every image in folder_path and write YOLO .txt
    annotations to ANNOTATION_DIR.  progress_callback(done, total, filename)
    is called after each image if provided.
    Returns (annotated_count, skipped_count).
    """
    image_files = [
        f for f in os.listdir(folder_path)
        if os.path.splitext(f)[1].lower() in _IMAGE_EXTS
    ]
    total = len(image_files)
    model = YOLO(model_path)
    annotated = 0
    skipped = 0
    for i, fname in enumerate(image_files):
        img_path = os.path.join(folder_path, fname)
        img = cv2.imread(img_path)
        if img is None:
            skipped += 1
            if progress_callback:
                progress_callback(i + 1, total, fname)
            continue
        img_h, img_w = img.shape[:2]
        results = model.predict(img, conf=conf, verbose=False)
        _save_yolo_annotation(img_path, results[0], img_w, img_h)
        annotated += 1
        if progress_callback:
            progress_callback(i + 1, total, fname)
    return annotated, skipped

# Settings file path
SETTINGS_FILE = "pcb_settings.json"

# Camera port mapping: logical role index -> physical OS device index
# Role 0 = main PCB view, Role 1 = wire tip view, Role 2 = clog detection
camera_ports = {0: 0, 1: 1, 2: 2}

# Per-camera stop events — set to signal a running open_camera thread to exit cleanly
camera_stop_events = {0: threading.Event(), 1: threading.Event(), 2: threading.Event()}

def load_camera_ports():
    """Load camera port assignments from pcb_settings.json at startup."""
    global camera_ports
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
            ports = data.get("camera_ports", {})
            for role in [0, 1, 2]:
                camera_ports[role] = int(ports.get(str(role), role))
    except Exception as e:
        print(f"Warning: Could not load camera ports from {SETTINGS_FILE}: {e}")

load_camera_ports()

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
# SOFTWARE-BASED IMAGE ADJUSTMENTS  (per-camera)
########################################################
_DEFAULT_ADJUSTMENTS = {
    'alpha':         1.5,
    'beta':          -100.0,
    'sat_factor':    1.2,
    'gamma':         1.4,
    'sharp_strength': 2.0,
}

# Keyed by logical camera role (0, 1, 2)
camera_adjustments = {
    0: dict(_DEFAULT_ADJUSTMENTS),
    1: dict(_DEFAULT_ADJUSTMENTS),
    2: dict(_DEFAULT_ADJUSTMENTS),
}

def load_camera_adjustments():
    """Load per-camera image adjustment values from pcb_settings.json at startup."""
    global camera_adjustments
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                saved = json.load(f).get('camera_adjustments', {})
            for role in [0, 1, 2]:
                role_data = saved.get(str(role), {})
                for key, default in _DEFAULT_ADJUSTMENTS.items():
                    camera_adjustments[role][key] = float(role_data.get(key, default))
    except Exception as e:
        print(f"Warning: Could not load camera adjustments from {SETTINGS_FILE}: {e}")

load_camera_adjustments()

def post_process_frame(frame, camera_index=0):
    adj = camera_adjustments.get(camera_index, camera_adjustments[0])
    alpha  = adj['alpha']
    beta   = adj['beta']
    sat    = adj['sat_factor']
    gamma  = adj['gamma']
    sharp  = adj['sharp_strength']

    # 1) Contrast & Brightness
    adjusted = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)

    # 2) Saturation
    if sat != 1.0:
        hsv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        s = (s.astype(np.float32) * sat).clip(0, 255).astype(np.uint8)
        hsv = cv2.merge([h, s, v])
        adjusted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # 3) Gamma
    if gamma != 1.0:
        inv_gamma = 1.0 / gamma
        lut = np.array([(i/255.0)**inv_gamma * 255 for i in range(256)]).astype("uint8")
        adjusted = cv2.LUT(adjusted, lut)

    # 4) Sharpen
    if sharp > 0:
        blurred = cv2.GaussianBlur(adjusted, (5,5), 0)
        f_ad = adjusted.astype(np.float32)
        f_bl = blurred.astype(np.float32)
        mask = f_ad - f_bl
        f_sharp = f_ad + sharp * mask
        f_sharp = np.clip(f_sharp, 0, 255).astype(np.uint8)
        adjusted = f_sharp

    return adjusted

pad_box_dict = {}  # e.g. {"pad1": (x1,y1,x2,y2), "pad2": ..., etc.}
trench_start_dict = {}  # e.g. {"trenchstart1": (x1,y1,x2,y2), etc.}
trench_stop_dict = {}   # e.g. {"trenchstop1": (x1,y1,x2,y2), etc.}

def custom_annotate(results, img, camera_index=0):
    global pad_box_dict, trench_start_dict, trench_stop_dict

    if not draw_bounding_boxes:
        return img.copy()

    annotated_img = img.copy()
    boxes = results.boxes
    names = results.names

    pad_boxes = []
    trench_start_boxes = []
    trench_stop_boxes = []

    # Define allowed objects per camera
    # 0 = Wire Tip view, 1 = Clog detection, 2 = PCB view (display only, no detection)
    allowed_objects = {
        0: ["CF_Tip", "GC_Tip", "Pad", "CF_Trench", "VisibleArea"],
        1: ["Clog"],
        2: [],
    }

    # 1) gather bounding boxes
    for box in boxes:
        cls_id = int(box.cls[0])
        class_name = names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        
        # Skip if this object type shouldn't be shown on this camera
        if class_name not in allowed_objects[camera_index]:
            continue

        center_y = (y1 + y2)/2

        if class_name == "Pad":
            pad_boxes.append((x1, y1, x2, y2, center_y, conf))
        elif class_name == "TrenchStart":
            trench_start_boxes.append((x1, y1, x2, y2, center_y, conf))
        elif class_name == "TrenchStop":
            trench_stop_boxes.append((x1, y1, x2, y2, center_y, conf))
        else:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # Only process special boxes for camera 0 (Wire Tip view)
    if camera_index == 0:
        # Sort boxes top->bottom
        pad_boxes.sort(key=lambda b: b[4], reverse=True)
        trench_start_boxes.sort(key=lambda b: b[4], reverse=True)
        trench_stop_boxes.sort(key=lambda b: b[4], reverse=True)
        
        # Process Pad boxes
        for idx, (bx1, by1, bx2, by2, cy, conf) in enumerate(pad_boxes, 1):
            label = f"pad{idx} {conf:.2f}"
            pure_label = f"pad{idx}"
            pad_box_dict[pure_label] = (bx1, by1, bx2, by2)
            cv2.rectangle(annotated_img, (bx1, by1), (bx2, by2), (255, 255, 0), 2)
            cv2.putText(annotated_img, label, (bx1 + 3, by2 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Process TrenchStart boxes
        for idx, (bx1, by1, bx2, by2, cy, conf) in enumerate(trench_start_boxes, 1):
            label = f"trenchstart{idx} {conf:.2f}"
            pure_label = f"trenchstart{idx}"
            trench_start_dict[pure_label] = (bx1, by1, bx2, by2)
            cv2.rectangle(annotated_img, (bx1, by1), (bx2, by2), (255, 255, 0), 2)
            cv2.putText(annotated_img, label, (bx1 + 3, by2 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Process TrenchStop boxes
        for idx, (bx1, by1, bx2, by2, cy, conf) in enumerate(trench_stop_boxes, 1):
            label = f"trenchstop{idx} {conf:.2f}"
            pure_label = f"trenchstop{idx}"
            trench_stop_dict[pure_label] = (bx1, by1, bx2, by2)
            cv2.rectangle(annotated_img, (bx1, by1), (bx2, by2), (255, 255, 0), 2)
            cv2.putText(annotated_img, label, (bx1 + 3, by2 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return annotated_img

# Globals for bounding boxes
last_cf_box = None
last_gc_box = None
last_pad_box= None  # We'll store one "Pad" bounding box for extrude reference
last_visible_area_box = None  # Camera's usable field-of-view box
last_clog_box = None  # Exclusively updated by camera 2

def open_camera(camera_index=0, model_path="best.pt"):
    global record_camera0, record_camera1
    global video_writers, run_timestamps
    global frames_per_still, frame_counts
    global last_cf_box, last_gc_box, last_pad_box, last_visible_area_box, last_clog_box
    global record_dir0, record_dir1, record_dir2

    desired_width = 1600
    desired_height = 1200

    model = YOLO(model_path)
    device_port = camera_ports.get(camera_index, camera_index)
    cap = cv2.VideoCapture(device_port)
    print(f"[Camera {camera_index}] opening device port {device_port}")

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
        if camera_stop_events[camera_index].is_set():
            print(f"[Camera {camera_index}] Stop requested, shutting down.")
            break
        ret, frame = cap.read()
        if not ret:
            break

        # 1) Post-process
        frame = post_process_frame(frame, camera_index)

        # 2) YOLO detect
        results = model.predict(frame, conf=0.5, verbose=False)
        boxes = results[0].boxes
        names = results[0].names

        if camera_index == 0:
            # Wire Tip view: detect CF_Tip, GC_Tip, Pad, and VisibleArea
            cf_found = None
            gc_found = None
            pad_found = None
            visible_area_found = None

            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                if class_name == "CF_Tip":
                    cf_found = (x1, y1, x2, y2)
                elif class_name == "GC_Tip":
                    gc_found = (x1, y1, x2, y2)
                elif class_name == "Pad":
                    pad_found = (x1, y1, x2, y2)
                elif class_name == "VisibleArea":
                    visible_area_found = (x1, y1, x2, y2)

            if cf_found is not None:
                last_cf_box = cf_found
            if gc_found is not None:
                last_gc_box = gc_found
            if pad_found is not None:
                last_pad_box = pad_found
            if visible_area_found is not None:
                last_visible_area_box = visible_area_found

        elif camera_index == 1:
            # Clog detection view
            clog_found = None
            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                if class_name == "Clog":
                    clog_found = (x1, y1, x2, y2)
            if clog_found is not None:
                last_clog_box = clog_found
            else:
                last_clog_box = None  # Clear when no clog visible

        # camera_index == 2 is PCB view — display only, no object tracking needed

        # 3) bounding box annotation
        annotated_frame = custom_annotate(results[0], frame, camera_index)

        # 4) Recording logic - video keeps bounding boxes, spliced stills stay clean
        rec_flag = (camera_index==0 and record_camera0) or \
                  (camera_index==1 and record_camera1) or \
                  (camera_index==2 and record_camera2)
        if rec_flag:
            if video_writers[camera_index] is None:
                run_timestamps[camera_index] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                if camera_index==0:
                    record_dir0 = _create_unique_daily_record_dir("D:\\", "camera0_pcb2_CFmicrowire")
                    video_path = os.path.join(record_dir0, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                elif camera_index==1:
                    record_dir1 = _create_unique_daily_record_dir("D:\\", "camera1_pcb2_CFmicrowire")
                    video_path = os.path.join(record_dir1, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                else:  # camera_index==2
                    record_dir2 = _create_unique_daily_record_dir("D:\\", "camera2_pcb2_CFmicrowire")
                    video_path = os.path.join(record_dir2, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                video_writers[camera_index] = cv2.VideoWriter(video_path, fourcc, 20.0, (width, height))
                print(f"[Camera {camera_index}] Recording started => {video_path}")

            # Video retains bounding-box overlay
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
                elif camera_index==1:
                    still_path = os.path.join(
                        record_dir1,
                        f"frame_{fc}_camera{camera_index}_{run_timestamps[camera_index]}.jpg"
                    )
                else:  # camera_index==2
                    still_path = os.path.join(
                        record_dir2, 
                        f"frame_{fc}_camera{camera_index}_{run_timestamps[camera_index]}.jpg"
                    )
                cv2.imwrite(still_path, frame)
                if auto_annotate:
                    _save_yolo_annotation(still_path, results[0], width, height)

            frame_counts[camera_index]+=1
        else:
            if video_writers[camera_index] is not None:
                video_writers[camera_index].release()
                video_writers[camera_index]=None
                print(f"[Camera {camera_index}] Recording stopped.")

        # 5) Auto-annotation now runs inside the recording block above

        cv2.imshow(f"Camera {camera_index}", annotated_frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cap.release()
    cv2.destroyWindow(window_name)
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
def extrude(target_pad_number=1, max_iterations=20, known_µm=None, tolerance_µm=250,
            initial_extend=True):
    """
    Moves the 't' axis to align CF_Tip with a specific pad (default: pad1) horizontally
    within a specified tolerance.

    Parameters:
    - target_pad_number: The pad number to align with (1-8)
    - max_iterations: Maximum number of attempts
    - known_µm: Known distance in µm between adjacent pads for calibration
    - tolerance_µm: Alignment tolerance in µm
    - initial_extend: If True (default), extend 't' by 400 steps first to bring
                      the CF_Tip into camera view.  Set False for repeat calls
                      within the stabilisation loop where the tip is already visible.
    """
    import time
    from motor_control import update_speed, move_linear_stage, steps_to_µm

    # If known_µm is not provided, get it from the settings file
    if known_µm is None:
        known_µm = get_pad_spacing()

    print(f"[Extrude] Starting extrude to align CF_Tip with pad{target_pad_number}...")
    print(f"[Extrude] Using pad spacing for calibration: {known_µm} µm")

    global pad_box_dict, last_cf_box

    global extrude_done
    extrude_done = False  # reset at start of function

    # 1) Optionally extend 't' to bring CF_Tip into camera view.
    #    Skip on repeat calls within the stabilisation loop (initial_extend=False).
    update_speed(3)
    if initial_extend:
        move_linear_stage("t", "+", 100, wait_for_stop=True, max_wait=30.0)
    
    # Configuration
    step_size_µm = 100.0
    jam_threshold_µm = 50.0
    wait_between_moves = 1  # seconds

    # 2) Validate we have required bounding boxes
    target_pad_key = f"pad{target_pad_number}"
    target_pad_box = pad_box_dict.get(target_pad_key)
    
    # For calibration use the target pad and its neighbour — both should be
    # Calibration pair: target pad and its next neighbour, stepping together with
    # the pad index.  n1 is clamped at 7 so the pair is always padN/pad(N+1)
    # and never the same pad twice (which would give 0 px distance).
    #   pad1 → pad1/pad2 | pad4 → pad4/pad5 | pad8 → pad7/pad8
    n1 = min(target_pad_number, 7)
    cal_pad1_key = f"pad{n1}"
    cal_pad2_key = f"pad{n1 + 1}"
    cal_box1 = pad_box_dict.get(cal_pad1_key)
    cal_box2 = pad_box_dict.get(cal_pad2_key)
    print(f"[Extrude] Calibration pads: {cal_pad1_key} / {cal_pad2_key}")
    
    # Validate we have what we need
    if target_pad_box is None:
        print(f"[Extrude] Missing {target_pad_key} => cannot align => abort.")
        extrude_done = True
        return
        
    if cal_box1 is None or cal_box2 is None:
        print(f"[Extrude] Missing {cal_pad1_key} or {cal_pad2_key} => no calibration => abort.")
        extrude_done = True
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
            extrude_done = True
            return

    # We'll store the last CF Tip center in px for jam detection
    last_cf_x_px = None

    for attempt in range(max_iterations):
        print(f"[Extrude] Attempt {attempt+1}/{max_iterations}")

        # 3) Re-check we still have CF_Tip detection
        if last_cf_box is None:
            print("[Extrude] Lost CF_Tip detection => abort.")
            extrude_done = True
            return

        # 4) Calculate calibration - steps per pixel
        steps_pp = compute_steps_per_pixel(cal_box1, cal_box2, axis='t', known_µm=known_µm)
        if steps_pp <= 0.0:
            print(f"[Extrude] Invalid calibration (steps_pp={steps_pp}) => skip this iteration.")
            continue

        # 5) Calculate horizontal distance between pad center and CF_Tip.
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
            print("[Extrude] Waiting 1s for vibrations to settle...")
            time.sleep(1.0)
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
    extrude_done = True

# --------------------------------------------------------
# CENTER ON VISIBLE AREA: bring CF_Tip back within the camera's usable field
# of view via 'X' (the same axis x_align uses for vertical CF/pad alignment),
# then re-verify/re-correct CF_Tip's alignment with the target pad via 'Y'
# (same approach r_align uses after its pad search) before running x_align.
# --------------------------------------------------------
def center_on_visible_area(target_pad_number=1, known_µm=None,
                            visible_area_tolerance_µm=50, vertical_tolerance_µm=10):
    """
    Step 1: Move 'X' so CF_Tip's center matches the VisibleArea box's center,
    bringing CF_Tip back within the visible frame after r_align.
    Step 2: Re-check CF_Tip's alignment with the target pad's reference point
    (same bottom+33% point x_align uses) and correct via 'Y' if the move above
    left it out of tolerance — mirrors r_align's post-pad-search Y correction.
    """
    import time
    from motor_control import update_speed, move_linear_stage, steps_to_µm

    if known_µm is None:
        known_µm = get_pad_spacing()

    global pad_box_dict, last_cf_box, last_visible_area_box
    global center_on_visible_area_done
    center_on_visible_area_done = False  # reset at start of function

    if last_visible_area_box is None:
        print("[center_on_visible_area] No VisibleArea detected — skipping.")
        center_on_visible_area_done = True
        return
    if last_cf_box is None:
        print("[center_on_visible_area] No CF_Tip detected — skipping.")
        center_on_visible_area_done = True
        return

    n1 = min(target_pad_number, 7)
    cal_box1 = pad_box_dict.get(f"pad{n1}")
    cal_box2 = pad_box_dict.get(f"pad{n1 + 1}")
    if cal_box1 is None or cal_box2 is None:
        print(f"[center_on_visible_area] Missing pad{n1}/pad{n1 + 1} calibration — skipping.")
        center_on_visible_area_done = True
        return

    update_speed(3)
    steps_pp = compute_steps_per_pixel(cal_box1, cal_box2, axis='X', known_µm=known_µm)
    if steps_pp <= 0.0:
        print("[center_on_visible_area] Invalid 'X' calibration — skipping.")
        center_on_visible_area_done = True
        return

    # 1) Bring CF_Tip back within the visible area (via 'X' only)
    va_cy = (last_visible_area_box[1] + last_visible_area_box[3]) / 2
    cf_cy = center_of_bbox(last_cf_box)[1]
    delta_y_px = cf_cy - va_cy
    delta_µm = steps_to_µm(abs(delta_y_px * steps_pp), axis='X')
    if delta_µm <= visible_area_tolerance_µm:
        print(f"[center_on_visible_area] CF_Tip already within visible area "
              f"(±{visible_area_tolerance_µm}µm).")
    else:
        direction = '-' if delta_y_px >= 0 else '+'
        print(f"[center_on_visible_area] Centering in visible area: X {direction}{delta_µm:.1f}µm")
        move_linear_stage('X', direction, delta_µm, wait_for_stop=True, max_wait=30.0)
        time.sleep(1.0)  # let YOLO refresh detections

    # 2) Re-verify alignment with the target pad, correct via 'Y' if needed
    # (same pixel-to-step approach r_align uses for its post-pad-search Y fix)
    target_pad_box = pad_box_dict.get(f"pad{target_pad_number}")
    if target_pad_box is None or last_cf_box is None:
        print("[center_on_visible_area] Missing target pad or CF_Tip after centering "
              "— skipping pad check.")
        center_on_visible_area_done = True
        return
    steps_pp_y = compute_steps_per_pixel(cal_box1, cal_box2, axis='Y', known_µm=known_µm)
    if steps_pp_y <= 0.0:
        print("[center_on_visible_area] Invalid 'Y' calibration — skipping pad check.")
        center_on_visible_area_done = True
        return
    pad_height = target_pad_box[3] - target_pad_box[1]
    pad_y = target_pad_box[3] - 0.33 * pad_height  # same reference point x_align targets
    cf_y = center_of_bbox(last_cf_box)[1]
    delta_y_px2 = cf_y - pad_y
    delta_µm2 = steps_to_µm(abs(delta_y_px2 * steps_pp_y), axis='Y')
    if delta_µm2 <= vertical_tolerance_µm:
        print(f"[center_on_visible_area] CF_Tip still aligned with pad{target_pad_number} "
              f"(±{vertical_tolerance_µm}µm). No correction needed.")
    else:
        direction = '-' if delta_y_px2 >= 0 else '+'
        print(f"[center_on_visible_area] Correcting vs target pad: Y {direction}{delta_µm2:.1f}µm")
        move_linear_stage('Y', direction, delta_µm2, wait_for_stop=True, max_wait=30.0)

    center_on_visible_area_done = True

# --------------------------------------------------------
# X-axis alignment: measure distance in µm using compute_steps_per_pixel
# --------------------------------------------------------
def x_align(target_pad_number=1, known_µm=None, tolerance_µm=10):
    """
    Align CF_Tip's center against the target pad's center, offset 33% up from
    the bottom edge, in one move.
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
    print(f"[x_align] Using pad spacing for calibration: {known_µm} µm")
 
    global pad_box_dict, last_cf_box
    global x_align_done
    x_align_done = False # reset at start of function

    # 1) Validate we have required bounding boxes
    target_pad_key = f"pad{target_pad_number}"
    target_pad_box = pad_box_dict.get(target_pad_key)
 
    # For calibration use the target pad and its neighbour — both should be
    # visible at whatever X position the stage is currently at.
    # Calibration pair: target pad and its next neighbour, stepping together with
    # the pad index.  n1 is clamped at 7 so the pair is always padN/pad(N+1)
    # and never the same pad twice (which would give 0 px distance).
    #   pad1 → pad1/pad2 | pad4 → pad4/pad5 | pad8 → pad7/pad8
    n1 = min(target_pad_number, 7)
    cal_pad1_key = f"pad{n1}"
    cal_pad2_key = f"pad{n1 + 1}"
    cal_box1 = pad_box_dict.get(cal_pad1_key)
    cal_box2 = pad_box_dict.get(cal_pad2_key)
    print(f"[x_align] Calibration pads: {cal_pad1_key} / {cal_pad2_key}")
 
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
            x_align_done = True
            return
    if cal_box1 is None or cal_box2 is None:
        print(f"[x_align] Missing {cal_pad1_key} or {cal_pad2_key} => no calibration => abort.")
        x_align_done = True
        return
    # Check if CF Tip is missing
    if last_cf_box is None:
        print("[x_align] No CF_Tip detected => let's move X by -300 and re-check...")
        move_linear_stage("X", "+", 600, wait_for_stop=True, max_wait=30.0)
        time.sleep(2.0)
        # If we still don't have CF_Tip, abort
        if last_cf_box is None:
            print("[x_align] Still no CF_Tip after moving => abort.")
            x_align_done = True
            return
      
    # 2) Calculate calibration - steps per pixel
    steps_pp = compute_steps_per_pixel(cal_box1, cal_box2, axis='X', known_µm=known_µm)
    if steps_pp <= 0.0:
        print(f"[x_align] Invalid calibration (steps_pp={steps_pp}) => abort.")
        x_align_done = True
        return
    print(f"[x_align] Calibration: {steps_pp:.4f} steps/px (from {cal_pad1_key}..{cal_pad2_key})")
 
    # 3) Calculate vertical distance between the pad's target point (horizontally
    # centered, 33% up from the bottom edge) and CF_Tip's center
    PAD_VERTICAL_OFFSET_FRAC = 0.33  # fraction of pad height, measured up from the bottom edge
    pad_x = (target_pad_box[0] + target_pad_box[2]) / 2
    pad_height = target_pad_box[3] - target_pad_box[1]
    pad_y = target_pad_box[3] - PAD_VERTICAL_OFFSET_FRAC * pad_height
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
    if abs(delta_µm) <= tolerance_µm:
        print(f"[x_align] Already aligned within ±{tolerance_µm}µm. No movement needed.")
        x_align_done = True
        return
      
    # 5) Set speed for alignment move
    update_speed(3)
 
    # 6) Execute the move
    print(f"[x_align] Moving {direction}{abs(delta_µm):.1f}µm along 'X' axis...")
    move_linear_stage('X', direction, abs(delta_µm), wait_for_stop=True, max_wait=30.0)
 
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
        x_align_done = True
    print("[x_align] Vertical alignment complete.")

# --------------------------------------------------------
# R-axis alignment: measure angle in degrees using compute_angle_between
# --------------------------------------------------------
def r_align(angle_tolerance=0.5, reference_angle=0.0, target_pad_number=1):
    """
    Rotates the 'r' axis to bring the CF→GC angle within `angle_tolerance` degrees
    of `reference_angle` (default 0°), then re-acquires pads that may have
    shifted out of frame from the rotation (X search + calibrated Y correction).

    Pass reference_angle=<value from a previous r_align call> so that successive
    corrections in the stabilisation loop fix only the delta instead of the
    absolute angle, preventing oscillation.
    """
    import time
    from motor_control import update_speed, move_linear_stage, steps_to_µm, is_emergency_stop_requested
    global last_cf_box, last_gc_box, pad_box_dict
    global r_align_done
    global last_r_align_angle
    r_align_done = False # reset at start of function
    last_r_align_angle = 0.0  # reset; set to the signed delta only when we rotate
 
    update_speed(3)  # Set speed
 
    if last_cf_box is None:
        print("[r_align] No CF_Tip bounding box stored yet. Cannot align r-axis.")
        r_align_done = True
        return
    if last_gc_box is None:
        print("[r_align] No GC_Tip bounding box stored yet. Cannot align r-axis.")
        r_align_done = True
        return
      
    # 1) Compute current angle and delta from reference
    initial_angle_degs = compute_angle_between(last_cf_box, last_gc_box)
    angle_delta = initial_angle_degs - reference_angle
    print(f"[r_align] Current angle: {initial_angle_degs:.2f}°  "
          f"reference: {reference_angle:.2f}°  delta: {angle_delta:.2f}°")
 
    # 2) Check tolerance against delta
    if abs(angle_delta) <= angle_tolerance:
        print(f"[r_align] Within ±{angle_tolerance}° of reference => no rotation needed.")
        r_align_done = True
        return
 
    # 3) Move the 'r' axis by the delta.
    # Sign convention: positive delta => '-', negative delta => '+' (motor direction is inverted).
    last_r_align_angle = angle_delta  # remember signed rotation for reference/debugging
    direction = '-' if angle_delta >= 0 else '+'
    displacement = min(abs(angle_delta), 4.0)  # clamp to ±4° max we can accommodate
    print(f"[r_align] Rotating r-axis by {direction}{displacement:.2f}°...")
    move_linear_stage('r', direction, displacement, wait_for_stop=True, max_wait=30.0)

    # ── Pad re-acquisition after rotation ──────────────────────────────────
    # Rotating the wire can shift pads out of frame. Step X in the direction
    # matching the rotation sign (negative angle → X+, positive angle → X-)
    # in 1000µm increments until all 8 pads are visible again.
    step_dir = '+' if last_r_align_angle < 0 else '-'
    pad_box_dict.clear()  # force fresh detections so "visible" reflects the current view
    time.sleep(1.0)
    MAX_SEARCH_STEPS = 25
    for _ in range(MAX_SEARCH_STEPS):
        if is_emergency_stop_requested():
            print("[r_align] Emergency stop during pad search.")
            r_align_done = True
            return
        if all(pad_box_dict.get(f"pad{i}") is not None for i in range(1, 9)):
            print("[r_align] All 8 pads visible — proceeding.")
            break
        print(f"[r_align] Not all pads visible — stepping X {step_dir}1000µm...")
        move_linear_stage("X", step_dir, 1000, wait_for_stop=True, max_wait=30.0)
        time.sleep(1.0)  # let YOLO refresh detections
    else:
        print("[r_align] Warning: not all 8 pads visible after search; continuing.")

    # Once all pads are found, correct Y using pixel-to-step calibration (not the
    # motor controller readout) so the target pad returns to the vertical frame
    # position it had before this rotation — same approach as extrude/x_align.
    PIXEL_TOL_PX = 5
    ref_y = target_pad_ref_y
    tbox = pad_box_dict.get(f"pad{target_pad_number}")
    if ref_y is None:
        print("[r_align] No reference pad Y-position recorded — skipping Y correction.")
    elif tbox is None:
        print(f"[r_align] Target pad{target_pad_number} not detected — skipping Y correction.")
    else:
        n1 = min(target_pad_number, 7)
        cal_box1 = pad_box_dict.get(f"pad{n1}")
        cal_box2 = pad_box_dict.get(f"pad{n1 + 1}")
        if cal_box1 is None or cal_box2 is None:
            print("[r_align] Missing calibration pads — skipping Y correction.")
        else:
            steps_pp = compute_steps_per_pixel(cal_box1, cal_box2, axis='Y',
                                                known_µm=get_pad_spacing())
            if steps_pp <= 0.0:
                print("[r_align] Invalid Y calibration — skipping Y correction.")
            else:
                cur_y = center_of_bbox(tbox)[1]
                delta_y_px = cur_y - ref_y
                if abs(delta_y_px) <= PIXEL_TOL_PX:
                    print(f"[r_align] Target pad Y-position within tolerance "
                          f"({cur_y:.1f}px vs {ref_y:.1f}px). No Y correction needed.")
                else:
                    delta_steps = delta_y_px * steps_pp
                    delta_µm = steps_to_µm(abs(delta_steps), axis='Y')
                    direction = '-' if delta_y_px >= 0 else '+'
                    print(f"[r_align] Correcting Y by {direction}{delta_µm:.1f}µm "
                          f"(pad Y {cur_y:.1f}px -> target {ref_y:.1f}px)")
                    move_linear_stage("Y", direction, delta_µm, wait_for_stop=True, max_wait=30.0)

    r_align_done = True