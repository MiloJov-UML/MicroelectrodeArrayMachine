# image_recognition.py

import cv2
from ultralytics import YOLO

def open_camera(camera_index, model_path="best.pt", conf=0.25):
    """
    Opens the camera at 'camera_index', runs YOLOv8 inference on each frame
    using 'model_path' weights, and displays the annotated feed in a window.
    Press 'q' to quit the stream.
    """
    # 1) Load the YOLOv8 model
    model = YOLO(model_path)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return

    print(f"[Camera {camera_index}] Press 'q' in the window to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from camera {camera_index}")
            break

        # 2) Run YOLOv8 inference on this frame
        #    'verbose=False' to silence console logs
        results = model.predict(source=frame, conf=conf, verbose=False)

        # 3) Annotate (draw boxes, labels) on the frame
        #    YOLOv8 can automatically produce an annotated image in results[0].plot()
        annotated_frame = results[0].plot()  # returns a numpy array with boxes and labels drawn

        # 4) Show the annotated frame
        cv2.imshow(f"Camera {camera_index}", annotated_frame)

        # 5) Press 'q' in the camera window to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[Camera {camera_index}] Stream ended.")
