import cv2
import os

def video_to_images(video_path, output_folder="D:\\PCB2_carbon_fiber_microwire_2025-03-12"):
    """
    Converts a video into individual image frames, but only saves every 75th frame,
    then writes those frames to `output_folder`.
    """
    # Create the output directory if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)

    # Initialize VideoCapture with your video file
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    frame_num = 0      # Counts every frame read
    saved_count = 0    # Counts how many frames we actually save

    while True:
        ret, frame = cap.read()
        if not ret:
            # We've reached the end of the video or there's an error
            break

        # Save this frame only if frame_num is divisible by 75
        if frame_num % 30 == 0:
            # e.g. D:\extracted_frames\frame_0000.png
            filename = os.path.join(output_folder, f"frame_{frame_num:04d}.png")
            cv2.imwrite(filename, frame)
            saved_count += 1

        frame_num += 1

    # Release the video to free resources
    cap.release()
    print(f"Done! Saved {saved_count} frames (every 75th) to {output_folder} from {video_path}.")

# Example usage:
if __name__ == "__main__":
    # Note: Use properly escaped backslashes or a raw string for Windows paths.
    video_to_images("D:\\PCB2_carbon_fiber_microwire_2025-03-12\\Recording 2025-03-13 210441.mp4", 
                    output_folder="D:\\camera0_pcb2_CFmicrowire_2025-03-12")
