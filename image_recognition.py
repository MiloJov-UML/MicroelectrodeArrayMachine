# image_recognition.py

import math
import cv2
import os
import datetime
import numpy as np
from ultralytics import YOLO
from motor_control import µm_to_steps

# ------------------------------------------------------------------
# GLOBAL TOGGLES / SETTINGS
# ------------------------------------------------------------------

# 1) Bounding box toggle
draw_bounding_boxes = False

# 2) Recording toggles for each camera
record_camera0 = False
record_camera1 = False

# 3) Output directories for recorded video & still images
record_dir0 = r"D:\camera0_pcb2_CFmicrowire_2025-03-17_2"
record_dir1 = r"D:\camera1_pcb2_CFmicrowire_2025-03-17_2"

# 4) Per-camera video writers & timestamps
video_writers = {0: None, 1: None}
run_timestamps = {0: None, 1: None}

# 5) Still image logic
frames_per_still = 30
frame_counts = {0: 0, 1: 0}

# ------------------------------------------------------------------
# SOFTWARE-BASED IMAGE ADJUSTMENTS
# ------------------------------------------------------------------
ALPHA         = 1.1   # Contrast factor (1.0 => no change, >1 => more contrast)
BETA          = -100   # Brightness offset
SAT_FACTOR    = 1.2   # Saturation factor (1.0 => no change, >1 => more saturated)
GAMMA         = 1.4   # Gamma value (1.0 => no change, <1 => lighten midtones, >1 => darken)
SHARP_STRENGTH= 2   # Unsharp mask strength (0 => none, >0 => sharper)

def post_process_frame(frame):
    """
    1) Adjust contrast/brightness with ALPHA/BETA.
    2) Adjust saturation in HSV space with SAT_FACTOR.
    3) Apply gamma correction if GAMMA != 1.0.
    4) Sharpen via unsharp mask using SHARP_STRENGTH.
    Returns the final adjusted frame.
    """
    # 1) Contrast & Brightness
    adjusted = cv2.convertScaleAbs(frame, alpha=ALPHA, beta=BETA)

    # 2) Saturation
    if SAT_FACTOR != 1.0:
        hsv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        # multiply s-channel by SAT_FACTOR, then clip
        s = s.astype(np.float32) * SAT_FACTOR
        s = np.clip(s, 0, 255).astype(np.uint8)
        hsv = cv2.merge([h, s, v])
        adjusted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # 3) Gamma Correction
    if GAMMA != 1.0:
        inv_gamma = 1.0 / GAMMA
        lut = np.array([(i / 255.0) ** inv_gamma * 255 for i in range(256)]).astype("uint8")
        adjusted = cv2.LUT(adjusted, lut)

    # 4) Sharpen (Unsharp Mask)
    if SHARP_STRENGTH > 0.0:
        blurred = cv2.GaussianBlur(adjusted, (5, 5), 0)
        f_ad = adjusted.astype(np.float32)
        f_bl = blurred.astype(np.float32)
        mask = f_ad - f_bl
        f_sharp = f_ad + SHARP_STRENGTH * mask
        f_sharp = np.clip(f_sharp, 0, 255).astype(np.uint8)
        adjusted = f_sharp

    return adjusted

# ------------------------------------------------------------------
# BOUNDING BOX ANNOTATION LOGIC
# ------------------------------------------------------------------
def custom_annotate(results, img):
    """
    If draw_bounding_boxes == False, returns a copy unmodified.
    Otherwise:
      - We label 'Pad' bounding boxes top->bottom as pad8..pad1
      - Others are drawn in a different color
    """
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
        center_y = (y1 + y2) / 2

        if class_name == "Pad":
            pad_boxes.append((x1, y1, x2, y2, center_y, conf))
        else:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # Sort 'Pad' boxes from top -> bottom (lowest center_y first)
    pad_boxes.sort(key=lambda b: b[4])
    pad_index = 8
    for (x1, y1, x2, y2, center_y, conf) in pad_boxes:
        label = f"pad{pad_index} {conf:.2f}"
        pad_index -= 1
        cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(annotated_img, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    return annotated_img


def open_camera(camera_index=0, model_path="best.pt"):
    """
    1) Captures frames from camera_index.
    2) Adjust each frame's contrast/brightness/saturation/gamma/sharpness.
    3) YOLO detect => custom_annotate => possibly record frames & save still images.
    4) Press 'q' to exit the camera loop.
    """
    global draw_bounding_boxes
    global record_camera0, record_camera1
    global record_dir0, record_dir1
    global video_writers, run_timestamps
    global frames_per_still, frame_counts

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

        # (1) Software-based approach: post-process for contrast, brightness, etc.
        frame = post_process_frame(frame)

        # (2) YOLO detection
        results = model.predict(frame, conf=0.3, verbose=False)

        # (3) Annotate bounding boxes if toggled
        annotated_frame = custom_annotate(results[0], frame)

        # (4) Recording logic
        # Determine if camera0 or camera1 is toggled to record
        rec_flag = (camera_index == 0 and record_camera0) or (camera_index == 1 and record_camera1)
        if rec_flag:
            # If not already recording, start now (timestamped file)
            if video_writers[camera_index] is None:
                run_timestamps[camera_index] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                if camera_index == 0:
                    os.makedirs(record_dir0, exist_ok=True)
                    video_path = os.path.join(
                        record_dir0,
                        f"camera{camera_index}_{run_timestamps[camera_index]}.avi"
                    )
                else:
                    os.makedirs(record_dir1, exist_ok=True)
                    video_path = os.path.join(
                        record_dir1,
                        f"camera{camera_index}_{run_timestamps[camera_index]}.avi"
                    )
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writers[camera_index] = cv2.VideoWriter(video_path, fourcc, 20.0, (width, height))
                print(f"[Camera {camera_index}] Recording started => {video_path}")

            # Write annotated frame
            video_writers[camera_index].write(annotated_frame)

            # Save still image every N frames
            fc = frame_counts[camera_index]
            if fc % frames_per_still == 0:
                if run_timestamps[camera_index] is None:
                    run_timestamps[camera_index] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                if camera_index == 0:
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

            frame_counts[camera_index] += 1
        else:
            # Not recording => close writer if open
            if video_writers[camera_index] is not None:
                video_writers[camera_index].release()
                video_writers[camera_index] = None
                print(f"[Camera {camera_index}] Recording stopped.")

        # (5) Show feed
        cv2.imshow(f"Camera {camera_index}", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print(f"[Camera {camera_index}] feed ended.")

    # finalize if user didn't manually stop
    if rec_flag and video_writers[camera_index]:
        video_writers[camera_index].release()
        video_writers[camera_index] = None
        print(f"[Camera {camera_index}] Recording stopped at exit.")


# ------------------------------------------------------------------
# UTILITY FUNCTIONS
# ------------------------------------------------------------------

def center_of_bbox(bbox):
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def compute_steps_per_pixel(bboxA, bboxB, axis='X', known_um=1000):
    dist = math.hypot(
        center_of_bbox(bboxB)[0] - center_of_bbox(bboxA)[0],
        center_of_bbox(bboxB)[1] - center_of_bbox(bboxA)[1]
    )
    steps = µm_to_steps(known_um, axis)
    return steps / dist
