# main.py

import threading
from app_gui import launch_gui, continuous_motor_control, start_camera_threads

def main():
    print("Welcome to the Microelectrode Fabrication App!")
    
    # Start the GUI in its own thread
    threading.Thread(target=launch_gui).start()

    # Start keyboard control
    threading.Thread(target=continuous_motor_control).start()

    # Start camera threads
    cam0, cam1 = start_camera_threads()
    cam0.join()
    cam1.join()

if __name__ == "__main__":
    main()
