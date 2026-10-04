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
# Index of the next frame to be written to the current recording (0-based). Reset
# when a new recording starts so it always equals the frame's position in the .avi
# and the N in the still names (frame_N_...jpg).
frame_counts = {0: 0, 1: 0, 2: 0}

# Correlation with the routine log (logs/routine_*.log): while a camera records, a
# per-frame CSV maps frame index -> wall-clock time, and log lines for camera 0 are
# tagged with the last frame written plus seconds since the recording started.
frame_time_files = {0: None, 1: None, 2: None}
record_start_times = {0: None, 1: None, 2: None}  # datetime when each recording began

def _recording_tag():
    """Prefix for routine-log lines while camera 0 records, else ''.
    'f' is the last frame written to the video; 'rec' is wall-clock seconds since the
    recording started (the .avi plays at a nominal 20 fps, so use the frame number
    rather than the video timestamp to seek)."""
    start = record_start_times[0]
    if video_writers[0] is None or start is None:
        return ""
    elapsed = (datetime.datetime.now() - start).total_seconds()
    return f"[cam0 f{max(frame_counts[0] - 1, 0)} rec+{elapsed:.2f}s] "

def _start_recording_correlation(camera_index, record_dir, video_path):
    """Log the recording start, drop a recording_info.txt next to the video, and open
    the per-frame timestamp CSV."""
    now = datetime.datetime.now()
    record_start_times[camera_index] = now
    frame_counts[camera_index] = 0
    _align_log_write(f"[camera{camera_index}:RECORDING START] video={video_path} start={now.isoformat(timespec='milliseconds')}")
    try:
        with open(os.path.join(record_dir, "recording_info.txt"), "w", encoding="utf-8") as f:
            f.write(f"camera={camera_index}\n")
            f.write(f"video={video_path}\n")
            f.write(f"recording_start={now.isoformat(timespec='milliseconds')}\n")
            f.write(f"routine_log={_align_log_path}\n")
            f.write("frame_times=" + f"frame_times_camera{camera_index}.csv\n")
        ft = open(os.path.join(record_dir, f"frame_times_camera{camera_index}.csv"), "w", encoding="utf-8")
        ft.write("frame,wall_time,epoch_s\n")
        frame_time_files[camera_index] = ft
    except Exception as e:
        print(f"Warning: could not start recording correlation files: {e}")
        frame_time_files[camera_index] = None

def _log_frame_time(camera_index, frame_index):
    ft = frame_time_files[camera_index]
    if ft is None:
        return
    try:
        now = datetime.datetime.now()
        ft.write(f"{frame_index},{now.strftime('%H:%M:%S.%f')[:-3]},{now.timestamp():.3f}\n")
        if frame_index % 20 == 0:
            ft.flush()
    except Exception:
        pass

def _stop_recording_correlation(camera_index):
    ft = frame_time_files[camera_index]
    if ft is not None:
        try:
            ft.close()
        except Exception:
            pass
        frame_time_files[camera_index] = None
    if record_start_times[camera_index] is not None:
        _align_log_write(f"[camera{camera_index}:RECORDING STOP] frames_written={frame_counts[camera_index]}")
        record_start_times[camera_index] = None

extrude_done = False
r_align_done = False
x_align_done = False
center_on_visible_area_done = False
last_r_align_angle = 0.0  # signed angle of the most recent r_align rotation (0 = none)
target_pad_ref_y = None  # target pad center Y-pixel position recorded before r_align rotates it
cf_tip_ref_px = None  # CF_Tip center (px) captured post-extrude; fixed in frame thereafter

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

# Function to get the pad count from settings
def get_pad_count():
    """Load pad count from the settings file, default to 8 if not found."""
    try:
        if os.path.isfile(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
                return int(data.get("pad_count", 8))
    except Exception as e:
        print(f"Warning: Could not read pad_count from {SETTINGS_FILE}: {e}")
    return 8  # Default value if file not found or error occurs

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

def draw_axis_compass(img):
    """Overlay a compass showing which motor axis moves the stage each way.

    Up=-X, Down=+X, Left=+Y, Right=-Y.
    """
    h, w = img.shape[:2]
    arm = 60                 # arrow length in pixels
    margin = 30              # gap from the frame edges
    label_pad = 40           # extra room on the right for the "-Y" label
    cx, cy = w - margin - arm - label_pad, margin + arm   # center in the top-right region
    color = (0, 255, 255)    # yellow
    thickness = 3
    font = cv2.FONT_HERSHEY_SIMPLEX
    fscale = 0.8

    # (dx, dy, label) for each direction. dy is negative going up (screen coords).
    directions = [
        (0, -1, "-X"),   # up
        (0,  1, "X"),    # down
        (-1, 0, "Y"),    # left
        (1,  0, "-Y"),   # right
    ]

    for dx, dy, label in directions:
        end = (cx + dx * arm, cy + dy * arm)
        cv2.arrowedLine(img, (cx, cy), end, color, thickness, tipLength=0.3)

        (tw, th), _ = cv2.getTextSize(label, font, fscale, 2)
        tx = end[0] + (8 if dx > 0 else -tw - 8 if dx < 0 else -tw // 2)
        ty = end[1] + (th + 8 if dy > 0 else -8 if dy < 0 else th // 2)
        # keep text inside the frame
        tx = max(2, min(tx, w - tw - 2))
        ty = max(th + 2, min(ty, h - 2))
        cv2.putText(img, label, (tx, ty), font, fscale, color, 2, cv2.LINE_AA)

    return img

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
                _start_recording_correlation(camera_index, os.path.dirname(video_path), video_path)
                print(f"[Camera {camera_index}] Recording started => {video_path}")

            # Video retains bounding-box overlay
            video_writers[camera_index].write(annotated_frame)
            fc = frame_counts[camera_index]
            _log_frame_time(camera_index, fc)
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
                _stop_recording_correlation(camera_index)
                print(f"[Camera {camera_index}] Recording stopped.")

        # 5) Auto-annotation now runs inside the recording block above

        display_frame = annotated_frame
        if camera_index == 0:
            display_frame = draw_axis_compass(annotated_frame.copy())

        cv2.imshow(f"Camera {camera_index}", display_frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cap.release()
    cv2.destroyWindow(window_name)
    print(f"[Camera {camera_index}] feed ended.")

    if rec_flag and video_writers[camera_index]:
        video_writers[camera_index].release()
        video_writers[camera_index]=None
        _stop_recording_correlation(camera_index)
        print(f"[Camera {camera_index}] Recording stopped at exit.")

# --------------------------------------------------------
# Utility
# --------------------------------------------------------
def center_of_bbox(bbox):
    (x1,y1,x2,y2) = bbox
    cx = (x1 + x2)/2
    cy = (y1 + y2)/2
    return (cx,cy)

# Cached pixel distance between adjacent calibration pads, measured once per pad
# and reused across extrude/r_align/x_align/center_on_visible_area (see
# get_steps_per_pixel).
_pad_pixel_spacing = None

def reset_pad_pixel_spacing():
    """Clear the cached pad pixel-spacing so the next get_steps_per_pixel call
    re-measures it. Called at the start of each pad."""
    global _pad_pixel_spacing
    _pad_pixel_spacing = None

def get_steps_per_pixel(bboxA, bboxB, axis='X', known_µm=None):
    """Steps/pixel for `axis`, reusing a pad pixel-spacing measured once per pad.

    Measures the pixel distance between two calibration bounding boxes on the
    first call after a reset and caches it (a fixed camera-scale property), then
    converts the known pad spacing to steps via µm_to_steps for the requested
    axis: 1 px => returned value motor steps. X and Y share the same steps/µm,
    while 't' uses different microstepping.
    """
    global _pad_pixel_spacing
    if known_µm is None:
        known_µm = get_pad_spacing()
    if _pad_pixel_spacing is None:
        (cxA, cyA) = center_of_bbox(bboxA)
        (cxB, cyB) = center_of_bbox(bboxB)
        dist = math.hypot(cxB - cxA, cyB - cyA)
        if dist < 0.01:
            # Avoid division by zero if boxes are nearly the same center
            return 0.0
        _pad_pixel_spacing = dist
    return µm_to_steps(known_µm, axis=axis) / _pad_pixel_spacing

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
# EXTRUDE: measure distance in µm using get_steps_per_pixel
# --------------------------------------------------------
def extrude(target_pad_number=1, max_iterations=20, known_µm=None, tolerance_µm=400,
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
    log_report("extrude", "START", f"pad{target_pad_number}: extend 't' until CF_Tip is within ±{tolerance_µm}µm of pad center (calibration {known_µm}µm).")

    global pad_box_dict, last_cf_box

    global extrude_done
    extrude_done = False  # reset at start of function

    # 1) Optionally extend 't' to bring CF_Tip into camera view.
    #    Skip on repeat calls within the stabilisation loop (initial_extend=False).
    update_speed(3)
    if initial_extend:
        log_move("extrude", "INIT", "t", "+", "100", "extend wire to bring CF_Tip into camera view")
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
        # move_linear_stage("X", "+", 1200, wait_for_stop=True, max_wait=30.0)
        # time.sleep(2.0)
        # move_linear_stage("t", "+", 200, wait_for_stop=True, max_wait=30.0)
        # time.sleep(2.0)
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

        # 4) Calibration - steps per pixel (pad pixel-spacing cached once per pad)
        steps_pp = get_steps_per_pixel(cal_box1, cal_box2, axis='t', known_µm=known_µm)
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
            log_report("extrude", "DONE", f"pad{target_pad_number}: CF_Tip within ±{tolerance_µm}µm of pad center after {attempt + 1} attempt(s).")
            time.sleep(1.0)
            extrude_done = True
            return

        # 7) Prepare for jam detection
        last_cf_x_px = cf_x
        
        # 8) Calculate movement size - either full step or remaining distance
        move_µm = min(step_size_µm, delta_µm)
        
        # 9) Move the stage
        print(f"    Move {direction}{move_µm:.1f}µm along 't' axis.")
        log_move("extrude", f"ATTEMPT {attempt + 1}", "t", direction, f"{move_µm:.1f}µm",
                 f"close horizontal gap to pad{target_pad_number} ({delta_µm:.1f}µm remaining)")
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
    The CF_Tip is printhead-mounted and fixed in the camera frame; r_align
    rotates the PCB, which swings the pads and the VisibleArea marker out of
    view (the tip never moves).
    Step 1: Move 'X' so the VisibleArea box's center lines back up with the
    (fixed) CF_Tip center, bringing the pads/VisibleArea back into frame.
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
    steps_pp = get_steps_per_pixel(cal_box1, cal_box2, axis='X', known_µm=known_µm)
    if steps_pp <= 0.0:
        print("[center_on_visible_area] Invalid 'X' calibration — skipping.")
        center_on_visible_area_done = True
        return

    # 1) Bring the VisibleArea/pads back under the fixed CF_Tip (via 'X' only)
    va_cy = (last_visible_area_box[1] + last_visible_area_box[3]) / 2
    cf_cy = center_of_bbox(last_cf_box)[1]
    delta_y_px = cf_cy - va_cy
    delta_µm = steps_to_µm(abs(delta_y_px * steps_pp), axis='X')
    if delta_µm <= visible_area_tolerance_µm:
        print(f"[center_on_visible_area] VisibleArea already centered under CF_Tip "
              f"(±{visible_area_tolerance_µm}µm).")
    else:
        direction = '-' if delta_y_px >= 0 else '+'
        print(f"[center_on_visible_area] Recentering VisibleArea/pads under CF_Tip: X {direction}{delta_µm:.1f}µm")
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
    steps_pp_y = get_steps_per_pixel(cal_box1, cal_box2, axis='Y', known_µm=known_µm)
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
# X-axis alignment: measure distance in µm using get_steps_per_pixel
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
      
    # 2) Calibration - steps per pixel (pad pixel-spacing cached once per pad)
    steps_pp = get_steps_per_pixel(cal_box1, cal_box2, axis='X', known_µm=known_µm)
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
# Live routine log file (created lazily on first report), in logs/ next to this
# file. Shared by extrude/r_align and the pad-loop moves in app_gui.py via
# log_report/log_move so the whole manual routine narrates to one session log.
_align_log_path = None

def _align_log_write(line):
    """Append a timestamped line to the per-session routine log file."""
    global _align_log_path
    try:
        if _align_log_path is None:
            log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            _align_log_path = os.path.join(log_dir, f"routine_{stamp}.log")
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(_align_log_path, "a", encoding="utf-8") as f:
            f.write(f"{ts}  {_recording_tag()}{line}\n")
    except Exception as e:
        print(f"Warning: could not write routine log: {e}")

def _align_report(phase, msg):
    """Live report line for an alignment phase (no motion)."""
    line = f"[r_align:{phase}] {msg}"
    print(line)
    _align_log_write(line)

def _align_move_report(phase, axis, direction, amount, reason):
    """Live report of one move: which phase, axis/direction/amount, and why."""
    line = f"[r_align:{phase}] MOVE {axis} {direction}{amount}  —  {reason}"
    print(line)
    _align_log_write(line)

def log_report(category, phase, msg):
    """Generic live report line for any routine phase (no motion).
    `category` is a short tag (e.g. 'extrude', 'pad-loop') so callers outside
    r_align (like extrude, or app_gui.py's pad loop) share the same log/format.
    """
    line = f"[{category}:{phase}] {msg}"
    print(line)
    _align_log_write(line)

def log_move(category, phase, axis, direction, amount, reason):
    """Generic live report of one move: category/phase, axis/direction/amount, and why."""
    line = f"[{category}:{phase}] MOVE {axis} {direction}{amount}  —  {reason}"
    print(line)
    _align_log_write(line)

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

    _align_report("START", f"pad{target_pad_number}: level wire → re-acquire pads → restore pad Y → "
                            f"recover VisibleArea → rough-center tip → fine Y then X align.")

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
    _align_report("ANGLE", f"CF→GC {initial_angle_degs:.2f}° vs reference {reference_angle:.2f}° (Δ{angle_delta:.2f}°, tol ±{angle_tolerance}°).")
 
    # 2) Check tolerance against delta
    if abs(angle_delta) <= angle_tolerance:
        _align_report("ANGLE", "within tolerance — no rotation needed; skipping the rest of r_align.")
        r_align_done = True
        return

    # Compute the Y pixel-to-step calibration NOW, before rotating — while the
    # calibration pads are in the same view where target_pad_ref_y was recorded.
    # Reused for the post-rotation Y correction below (pad_box_dict is cleared
    # during the pad search, so it can't be recomputed reliably afterward).
    _n1_cal = min(target_pad_number, 7)
    _cal_box1_start = pad_box_dict.get(f"pad{_n1_cal}")
    _cal_box2_start = pad_box_dict.get(f"pad{_n1_cal + 1}")
    y_steps_pp = None
    if _cal_box1_start is not None and _cal_box2_start is not None:
        y_steps_pp = get_steps_per_pixel(_cal_box1_start, _cal_box2_start, axis='Y',
                                         known_µm=get_pad_spacing())
    else:
        print("[r_align] Missing calibration pads at start — Y calibration unavailable.")

    # Record the VisibleArea's area BEFORE rotating; after the rotation the box is
    # often partially out of frame, so Step 2 steps X until its area recovers.
    va_area_before = None
    if last_visible_area_box is not None:
        _va0 = last_visible_area_box
        va_area_before = abs((_va0[2] - _va0[0]) * (_va0[3] - _va0[1]))
 
    # 3) Move the 'r' axis by the delta.
    # Sign convention: positive delta => '-', negative delta => '+' (motor direction is inverted).
    last_r_align_angle = angle_delta  # remember signed rotation for reference/debugging
    direction = '-' if angle_delta >= 0 else '+'
    displacement = min(abs(angle_delta), 4.0)  # clamp to ±4° max we can accommodate
    _align_move_report("ROTATE", "r", direction, f"{displacement:.2f}°",
                       f"level CF→GC wire {initial_angle_degs:.2f}° → {reference_angle:.2f}° (Δ{angle_delta:.2f}°)")
    move_linear_stage('r', direction, displacement, wait_for_stop=True, max_wait=30.0)

    # ── Pad re-acquisition after rotation ──────────────────────────────────
    # Rotating the PCB swings pads out of frame. Step X in the direction
    # matching the rotation sign (negative angle → X+, positive angle → X-)
    # in 1000µm increments until all pads are visible again.
    step_dir = '+' if last_r_align_angle < 0 else '-'
    pad_count = get_pad_count()
    pad_box_dict.clear()  # force fresh detections so "visible" reflects the current view
    time.sleep(1.0)
    _align_report("PAD-SEARCH", f"re-acquiring pads: stepping X {step_dir} until all {pad_count} pads are visible.")
    MAX_SEARCH_STEPS = 25
    for _ in range(MAX_SEARCH_STEPS):
        if is_emergency_stop_requested():
            print("[r_align] Emergency stop during pad search.")
            r_align_done = True
            return
        if all(pad_box_dict.get(f"pad{i}") is not None for i in range(1, pad_count + 1)):
            _align_move_report("PAD-SEARCH", "X", step_dir, "1000µm",
                               f"all {pad_count} pads visible — one extra step for margin")
            move_linear_stage("X", step_dir, 1000, wait_for_stop=True, max_wait=30.0)
            time.sleep(1.0)  # let YOLO refresh detections
            break
        _align_move_report("PAD-SEARCH", "X", step_dir, "1000µm",
                           "not all pads visible yet — searching to re-acquire pads shifted by the rotation")
        move_linear_stage("X", step_dir, 1000, wait_for_stop=True, max_wait=30.0)
        time.sleep(1.0)  # let YOLO refresh detections
    else:
        _align_report("PAD-SEARCH", f"WARNING: not all {pad_count} pads visible after {MAX_SEARCH_STEPS} steps; continuing.")

    # ── Step 1: restore the target pad to the Y-pixel it had before rotation ──
    # Uses y_steps_pp (measured pre-rotation) so the pad returns to its original
    # vertical frame position via the 'Y' stage.
    PIXEL_TOL_PX = 5
    ref_y = target_pad_ref_y
    tbox = pad_box_dict.get(f"pad{target_pad_number}")
    if ref_y is None:
        _align_report("STEP1 Y-RESTORE", "no reference pad Y recorded — skipping.")
    elif tbox is None:
        _align_report("STEP1 Y-RESTORE", f"target pad{target_pad_number} not detected — skipping.")
    elif y_steps_pp is None or y_steps_pp <= 0.0:
        _align_report("STEP1 Y-RESTORE", "invalid/unavailable Y calibration — skipping.")
    else:
        cur_y = center_of_bbox(tbox)[1]
        delta_y_px = cur_y - ref_y
        if abs(delta_y_px) <= PIXEL_TOL_PX:
            _align_report("STEP1 Y-RESTORE", f"pad already at pre-rotation row ({cur_y:.1f}px vs ref {ref_y:.1f}px) — no move.")
        else:
            delta_µm = steps_to_µm(abs(delta_y_px * y_steps_pp), axis='Y')
            direction = '-' if delta_y_px >= 0 else '+'
            _align_move_report("STEP1 Y-RESTORE", "Y", direction, f"{delta_µm:.1f}µm",
                               f"return pad to its pre-rotation row ({cur_y:.1f}px → ref {ref_y:.1f}px)")
            move_linear_stage("Y", direction, delta_µm, wait_for_stop=True, max_wait=30.0)
            time.sleep(1.0)

    # ── Step 2: recover the VisibleArea back into frame ──────────────────────
    # The rotation (and pad search / Step 1) often leaves the VisibleArea only
    # partially in frame. Keep stepping X 1600µm in the rotation-decided direction
    # until the live VisibleArea area is >=50% of its pre-rotation area (>100% is
    # fine). This gets both objects visible for the fine alignment below.
    VA_AREA_FRAC = 0.50
    VA_STEP_UM = 1600
    VA_MAX_STEPS = 25
    if va_area_before is not None and va_area_before > 0:
        _align_report("STEP2 VA-RECOVER", f"stepping X {step_dir} until VisibleArea returns to >={VA_AREA_FRAC * 100:.0f}% of its pre-rotation size.")
        for _ in range(VA_MAX_STEPS):
            if is_emergency_stop_requested():
                print("[r_align] Emergency stop during VisibleArea recovery.")
                r_align_done = True
                return
            va = last_visible_area_box  # live detection
            va_area_now = abs((va[2] - va[0]) * (va[3] - va[1])) if va is not None else 0.0
            frac = va_area_now / va_area_before
            if frac >= VA_AREA_FRAC:
                _align_report("STEP2 VA-RECOVER", f"VisibleArea back to {frac * 100:.0f}% of pre-rotation area — recovered.")
                break
            _align_move_report("STEP2 VA-RECOVER", "X", step_dir, f"{VA_STEP_UM}µm",
                               f"VisibleArea only {frac * 100:.0f}% in frame (<{VA_AREA_FRAC * 100:.0f}%) — stepping to bring it back")
            move_linear_stage("X", step_dir, VA_STEP_UM, wait_for_stop=True, max_wait=30.0)
            time.sleep(1.0)  # let YOLO refresh
        else:
            _align_report("STEP2 VA-RECOVER", f"WARNING: VisibleArea below {VA_AREA_FRAC * 100:.0f}% after {VA_MAX_STEPS} steps; continuing.")
    else:
        _align_report("STEP2 VA-RECOVER", "no pre-rotation VisibleArea area recorded — skipping recovery.")

    # Now that at least half of the VisibleArea is back in frame, rough-center the CF_Tip in it
    # along the X axis only (vertical). Use the live CF_Tip if it's detectable
    # again, else the saved fixed pixel. Large tolerance — just get the tip into
    # the detectable region. Compass: +X moves content down (+pixel-y), -X up.
    VISIBLE_AREA_TOL_UM = 500
    _n1v = min(target_pad_number, 7)
    _cv1 = pad_box_dict.get(f"pad{_n1v}")
    _cv2 = pad_box_dict.get(f"pad{_n1v + 1}")
    steps_pp_v = get_steps_per_pixel(_cv1, _cv2, axis='X', known_µm=get_pad_spacing()) if (_cv1 and _cv2) else 0.0
    cf_src = center_of_bbox(last_cf_box) if last_cf_box is not None else cf_tip_ref_px
    if cf_src is not None and last_visible_area_box is not None and steps_pp_v > 0.0:
        va = last_visible_area_box  # live detection
        # Target the top 25% of the VisibleArea (a quarter down), not the center.
        va_cy = va[1] + 0.25 * (va[3] - va[1])
        _src_lbl = "live" if last_cf_box is not None else "saved"

        delta_y_px = cf_src[1] - va_cy
        delta_y_µm = steps_to_µm(abs(delta_y_px * steps_pp_v), axis='X')
        if delta_y_µm > VISIBLE_AREA_TOL_UM:
            dir_y = '+' if delta_y_px >= 0 else '-'
            _align_move_report("STEP2 ROUGH-CENTER", "X", dir_y, f"{delta_y_µm:.1f}µm",
                               f"bring {_src_lbl} CF_Tip to VisibleArea top-25% so tip & pad are both detectable")
            update_speed(3)
            move_linear_stage('X', dir_y, delta_y_µm, wait_for_stop=True, max_wait=30.0)
            time.sleep(1.5)  # let YOLO refresh
        else:
            _align_report("STEP2 ROUGH-CENTER", f"CF_Tip already at VisibleArea top-25% (±{VISIBLE_AREA_TOL_UM}µm) — no move.")
    else:
        _align_report("STEP2 ROUGH-CENTER", "no CF_Tip ref / VisibleArea / calibration — skipping.")

    # ── Step 3: fine-align CF_Tip to the live target pad center ──────────────
    # Y align (horizontal pixels -> 'Y' stage) first, closed loop on live detections.
    # Then X align (vertical pixels -> 'X' stage): snapshot the CF_Tip and target pad,
    # compute the vertical pixel distance once, and move that distance open-loop — the
    # CF_Tip leaves the view during this move so the loop can't be closed.
    FINE_TOL_UM = 100
    tbox = pad_box_dict.get(f"pad{target_pad_number}")
    _n1c = min(target_pad_number, 7)
    _c1 = pad_box_dict.get(f"pad{_n1c}")
    _c2 = pad_box_dict.get(f"pad{_n1c + 1}")
    steps_pp_f = get_steps_per_pixel(_c1, _c2, axis='X', known_µm=get_pad_spacing()) if (_c1 and _c2) else 0.0
    if tbox is None:
        _align_report("STEP3", f"target pad{target_pad_number} not detected — skipping fine alignment.")
    elif steps_pp_f <= 0.0:
        _align_report("STEP3", "invalid calibration — skipping fine alignment.")
    else:
        # Y align via the 'Y' stage, which moves content left/right in the image (compass:
        # Left=+Y, Right=-Y), so it closes the HORIZONTAL pixel difference between the live
        # CF_Tip and the live target pad. Closed loop: re-read both after every move until
        # within tolerance. The move direction is verified against the measured result; if
        # a move makes the error worse, the sign is flipped.
        Y_ALIGN_TOL_UM = 250
        Y_ALIGN_MAX_ITERS = 15
        Y_ALIGN_MAX_MISSES = 3
        Y_ALIGN_MAX_STEP_UM = 500    # small capped steps: re-check live after each so it can't overshoot
        Y_ALIGN_WORSE_UM = 100       # error growth that counts as "moved the wrong way"
        sign_flip = 1                # +1: cf right of pad -> '-Y' (hardware-tested); -1: flipped by feedback
        prev_err_µm = None
        prev_move_µm = 0.0
        misses = 0
        iters = 0
        while iters < Y_ALIGN_MAX_ITERS:
            if is_emergency_stop_requested():
                _align_report("STEP3 Y-ALIGN", "emergency stop requested — aborting.")
                r_align_done = True
                return
            live_pad = pad_box_dict.get(f"pad{target_pad_number}")
            live_cf = last_cf_box
            if live_pad is None or live_cf is None:
                misses += 1
                if misses >= Y_ALIGN_MAX_MISSES:
                    _align_report("STEP3 Y-ALIGN", "live CF_Tip / target pad not detected — stopping Y align.")
                    break
                _align_report("STEP3 Y-ALIGN", "live CF_Tip / target pad not detected — waiting for a fresh detection.")
                time.sleep(1.0)
                continue
            misses = 0
            pad_cx = center_of_bbox(live_pad)[0]
            cf_x = center_of_bbox(live_cf)[0]
            delta_x_px = cf_x - pad_cx
            delta_x_µm = steps_to_µm(abs(delta_x_px * steps_pp_f), axis='Y')
            if delta_x_µm <= Y_ALIGN_TOL_UM:
                _align_report("STEP3 Y-ALIGN", f"CF_Tip on pad column within ±{Y_ALIGN_TOL_UM}µm (off by {delta_x_µm:.1f}µm) — aligned.")
                break
            if prev_err_µm is not None and prev_move_µm > 0 and delta_x_µm > prev_err_µm + Y_ALIGN_WORSE_UM:
                sign_flip = -sign_flip
                _align_report("STEP3 Y-ALIGN", f"error grew {prev_err_µm:.1f}µm → {delta_x_µm:.1f}µm after last move — reversing direction.")
            move_µm = min(delta_x_µm, Y_ALIGN_MAX_STEP_UM)
            dir_y = '-' if (delta_x_px >= 0) == (sign_flip > 0) else '+'
            _align_move_report("STEP3 Y-ALIGN", "Y", dir_y, f"{move_µm:.1f}µm",
                               f"match tip column to pad center (tip {cf_x:.1f}px vs pad {pad_cx:.1f}px, live, off {delta_x_µm:.1f}µm)")
            update_speed(3)
            move_linear_stage('Y', dir_y, move_µm, wait_for_stop=True, max_wait=30.0)
            prev_err_µm = delta_x_µm
            prev_move_µm = move_µm
            time.sleep(1.5)  # let YOLO refresh before re-checking
            iters += 1
        else:
            _align_report("STEP3 Y-ALIGN", f"WARNING: not within ±{Y_ALIGN_TOL_UM}µm after {Y_ALIGN_MAX_ITERS} steps; continuing.")

        # X align via the 'X' stage (moves content up/down) — closes the VERTICAL pixel
        # difference; snapshot then open-loop move.
        # Snapshot CF_Tip live if visible, else use the saved fixed location.
        tbox = pad_box_dict.get(f"pad{target_pad_number}")
        cf_snap = center_of_bbox(last_cf_box) if last_cf_box is not None else cf_tip_ref_px
        _snap_lbl = "live" if last_cf_box is not None else "saved"
        if tbox is None or cf_snap is None:
            _align_report("STEP3 X-ALIGN", "missing pad/CF_Tip snapshot — skipping X align.")
        else:
            # Same target point and camera-offset correction as x_align(): 33% up from
            # the pad's bottom edge, minus 250µm.
            pad_cy = tbox[3] - 0.33 * (tbox[3] - tbox[1])
            delta_y_px = cf_snap[1] - pad_cy
            delta_y_µm = steps_to_µm(abs(delta_y_px * steps_pp_f), axis='X') - 250
            if abs(delta_y_µm) <= FINE_TOL_UM:
                _align_report("STEP3 X-ALIGN", f"CF_Tip already on pad row (±{FINE_TOL_UM}µm) — no move.")
            else:
                # Same convention as x_align(): tip below the pad -> '-X', above -> '+X'.
                dir_x = '-' if delta_y_px >= 0 else '+'
                _align_move_report("STEP3 X-ALIGN", "X", dir_x, f"{abs(delta_y_µm):.1f}µm",
                                   f"open-loop final approach ({_snap_lbl} tip {cf_snap[1]:.1f}px vs pad target {pad_cy:.1f}px); tip leaves view mid-move")
                update_speed(3)
                move_linear_stage('X', dir_x, abs(delta_y_µm), wait_for_stop=True, max_wait=30.0)

    _align_report("DONE", f"pad{target_pad_number} alignment sequence complete.")
    r_align_done = True