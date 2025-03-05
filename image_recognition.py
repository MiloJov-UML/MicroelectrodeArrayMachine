# image_recognition.py

import math
import cv2
from ultralytics import YOLO

# Import the motor control function for µm->steps
from motor_control import µm_to_steps

def custom_annotate(results, img):
    """
    1) Gather bounding boxes
       - if class == "Pad", store for special top->bottom labeling
       - else, draw immediately in a standard style
    2) Sort the pad boxes from top->bottom, labeling them pad8..padN
    3) Optionally compute steps-per-pixel if 2+ pads exist.
    """
    boxes = results.boxes
    names = results.names
    annotated_img = img.copy()

    pad_boxes = []  # store (x1, y1, x2, y2, center_y, conf)
    
    # 1) Handle non-Pad classes immediately
    for box in boxes:
        cls_id = int(box.cls[0])
        class_name = names[cls_id]
        conf = float(box.conf[0])

        x1, y1, x2, y2 = box.xyxy[0].tolist()
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])  
        center_y = (y1 + y2) / 2

        if class_name == "Pad":
            pad_boxes.append((x1, y1, x2, y2, center_y, conf))
        else:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # 2) Sort the pads by center_y ascending -> top to bottom
    pad_boxes.sort(key=lambda b: b[4])  # b[4] is center_y

    pad_index = 8
    for (x1, y1, x2, y2, center_y, conf) in pad_boxes:
        label = f"pad{pad_index} {conf:.2f}"
        pad_index -= 1

        cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(annotated_img, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 3) If 2+ pads found, you can do calibration or further logic
    if len(pad_boxes) >= 2:
        (x1A, y1A, x2A, y2A, cyA, confA) = pad_boxes[0]
        (x1B, y1B, x2B, y2B, cyB, confB) = pad_boxes[1]

        bboxA = (x1A,y1A,x2A,y2A)
        bboxB = (x1B,y1B,x2B,y2B)

        # axis='X' or 'Y' depending on your actual movement axis
        # known_um=1000 if these two pads are physically 1000um center-to-center
        steps_pp = compute_steps_per_pixel(bboxA, bboxB, axis='X', known_um=1000)

        # Print to terminal
        # print(f"Steps/pixel (based on the bottom 2 pads) = {steps_pp:.3f}")

    return annotated_img

def center_of_bbox(bbox):
    """
    Given a YOLO bounding box (x1, y1, x2, y2), return its (cx, cy) center.
    """
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    return cx, cy

def compute_steps_per_pixel(bboxA, bboxB, axis='X', known_um=1000):
    """
    1) Measures the pixel distance between two bounding boxes (bboxA, bboxB).
    2) Uses the fact that physically they are 'known_um' micrometers apart.
    3) Converts that known_um to steps (using µm_to_steps from motor_control).
    4) Returns steps_per_pixel, i.e. how many motor steps correspond to 1 pixel.
    
    bboxA, bboxB: (x1, y1, x2, y2) from YOLO detection
    axis: 'X' or 'Y' or whichever axis your stage moves
    known_um: e.g. 1000µm if these two pads are known to be 1000µm apart
    """
    # 1) Pixel distance
    cxA, cyA = center_of_bbox(bboxA)
    cxB, cyB = center_of_bbox(bboxB)
    
    # For vertical alignment, you might do abs(cyB - cyA).
    # For horizontal, abs(cxB - cxA).
    # Or a general Euclidean distance:
    pixel_dist = math.hypot(cxB - cxA, cyB - cyA)

    # 2) Convert the known distance (1000µm) to steps
    steps_for_known_um = µm_to_steps(known_um, axis=axis)

    # 3) steps_per_pixel
    steps_per_pixel = steps_for_known_um / pixel_dist  # steps / px

    return steps_per_pixel

def open_camera(camera_index=0, model_path="best.pt"):
    """
    Opens camera feed using YOLOv8 for detection.
    We do custom_annotate for bounding boxes:
      - "Pad" boxes are sorted bottom->top, labeled pad1..padN.
      - Other classes are drawn as YOLO sees them.
      - If at least two pads found, we compute steps_per_pixel and print it out.
    """
    model = YOLO(model_path)
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(frame, conf=0.45, verbose=False)
        annotated_frame = custom_annotate(results[0], frame)

        cv2.imshow(f"Camera {camera_index}", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[Camera {camera_index}] feed ended.")
