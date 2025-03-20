# image_recognition.py

import math
import cv2
import os
import datetime
import numpy as np
from ultralytics import YOLO
from motor_control import µm_to_steps

########################################################
# GLOBAL TOGGLES / SETTINGS
########################################################

draw_bounding_boxes = False
record_camera0 = False
record_camera1 = False

record_dir0 = r"D:\camera0_pcb2_CFmicrowire_2025-03-18"
record_dir1 = r"D:\camera1_pcb2_CFmicrowire_2025-03-18"

video_writers = {0: None, 1: None}
run_timestamps = {0: None, 1: None}

frames_per_still = 30
frame_counts = {0: 0, 1: 0}

########################################################
# SOFTWARE-BASED IMAGE ADJUSTMENTS
########################################################
ALPHA          = 1.1
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
        cv2.putText(annotated_img,label,(x1,y2+15),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,0),1)

    return annotated_img

# Store the last bounding boxes for CF_tip & GC_tip
last_cf_box = None
last_gc_box = None

def open_camera(camera_index=0, model_path="best.pt"):
    global record_camera0, record_camera1
    global video_writers, run_timestamps
    global frames_per_still, frame_counts

    global last_cf_box, last_gc_box

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
        results = model.predict(frame, conf=0.5, verbose=False)
        boxes = results[0].boxes
        names = results[0].names

        if camera_index == 0:
            cf_found = None
            gc_found = None
            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                # **Case sensitive**: "CF_Tip" & "GC_Tip" from your trained model
                if class_name == "CF_Tip":
                    cf_found = (x1, y1, x2, y2)
                elif class_name == "GC_Tip":
                    gc_found = (x1, y1, x2, y2)

            if cf_found is not None:
                last_cf_box = cf_found
            if gc_found is not None:
                last_gc_box = gc_found

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


# Utility
def center_of_bbox(bbox):
    (x1,y1,x2,y2) = bbox
    cx = (x1 + x2)/2
    cy = (y1 + y2)/2
    return (cx,cy)

def compute_angle_between(cf_box, gc_box):
    """
    Return the angle (in degrees) from CF->GC relative to the x-axis
    (0° => horizontally with CF on left, GC on right).
    """
    (cx_cf, cy_cf) = center_of_bbox(cf_box)
    (cx_gc, cy_gc) = center_of_bbox(gc_box)

    dx = cx_gc - cx_cf
    dy = cy_gc - cy_cf

    angle_rads = math.atan2(dy, dx)  # -pi..pi
    angle_degs = math.degrees(angle_rads)
    return angle_degs

def analyze_cf_gc_angle():
    """
    Called by the GUI button => compute angle if we have last_cf_box & last_gc_box
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

