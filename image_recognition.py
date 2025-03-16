# image_recognition.py

import math
import cv2
import os
import datetime
from ultralytics import YOLO
from motor_control import µm_to_steps

draw_bounding_boxes = False
record_camera0 = False
record_camera1 = False

record_dir0 = r"D:\camera0_pcb2_CFmicrowire_2025-03-15"
record_dir1 = r"D:\camera1_pcb2_CFmicrowire_2025-03-15"

video_writers = {0: None, 1: None}
run_timestamps = {0: None, 1: None}

frames_per_still = 30
frame_counts = {0: 0, 1: 0}

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
            pad_boxes.append((x1,y1,x2,y2,center_y,conf))
        else:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(annotated_img,(x1,y1),(x2,y2),(255,0,0),2)
            cv2.putText(annotated_img,label,(x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)

    # sort top->bottom
    pad_boxes.sort(key=lambda b: b[4])
    pad_index = 8
    for (x1,y1,x2,y2,cy,conf) in pad_boxes:
        label = f"pad{pad_index} {conf:.2f}"
        pad_index -= 1
        cv2.rectangle(annotated_img,(x1,y1),(x2,y2),(255,255,0),2)
        cv2.putText(annotated_img,label,(x1,y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,0),1)

    return annotated_img

def open_camera(camera_index=0, model_path="best.pt"):
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

        results = model.predict(frame, conf=0.5, verbose=False)
        annotated_frame = custom_annotate(results[0], frame)

        # Are we recording camera_index?
        rec_flag = False
        if camera_index == 0:
            rec_flag = record_camera0
        elif camera_index == 1:
            rec_flag = record_camera1

        if rec_flag:
            if video_writers[camera_index] is None:
                # new run => create run timestamp
                run_timestamps[camera_index] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                if camera_index == 0:
                    os.makedirs(record_dir0, exist_ok=True)
                    video_path = os.path.join(record_dir0, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    video_writers[camera_index] = cv2.VideoWriter(video_path, fourcc, 20.0, (width, height))
                    print(f"[Camera {camera_index}] Recording started => {video_path}")
                else:
                    os.makedirs(record_dir1, exist_ok=True)
                    video_path = os.path.join(record_dir1, f"camera{camera_index}_{run_timestamps[camera_index]}.avi")
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    video_writers[camera_index] = cv2.VideoWriter(video_path, fourcc, 20.0, (width, height))
                    print(f"[Camera {camera_index}] Recording started => {video_path}")

            # write frames
            video_writers[camera_index].write(annotated_frame)

            # every N frames => still
            fc = frame_counts[camera_index]
            if fc % frames_per_still == 0:
                # we embed "camera{camera_index}" plus run timestamp
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
            frame_counts[camera_index] += 1

        else:
            # not recording => close if open
            if video_writers[camera_index] is not None:
                video_writers[camera_index].release()
                video_writers[camera_index] = None
                print(f"[Camera {camera_index}] Recording stopped.")

        cv2.imshow(f"Camera {camera_index}", annotated_frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[Camera {camera_index}] feed ended.")

    # finalize if still recording
    if rec_flag and video_writers[camera_index]:
        video_writers[camera_index].release()
        video_writers[camera_index]=None
        print(f"[Camera {camera_index}] Recording stopped at exit.")


#####################################
# UTILITY
#####################################
def center_of_bbox(bbox):
    x1,y1,x2,y2 = bbox
    return ((x1 + x2)/2, (y1 + y2)/2)

def compute_steps_per_pixel(bboxA, bboxB, axis='X', known_um=1000):
    dist = math.hypot(
        center_of_bbox(bboxB)[0]-center_of_bbox(bboxA)[0],
        center_of_bbox(bboxB)[1]-center_of_bbox(bboxA)[1]
    )
    steps = µm_to_steps(known_um, axis)
    return steps / dist
