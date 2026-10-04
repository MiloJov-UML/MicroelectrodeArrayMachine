# main.py

import threading
from app_gui import launch_gui, continuous_motor_control, start_camera_threads

def main():
    print("Welcome to the Microelectrode Fabrication App!")
    
    # Start the GUI in its own thread
    gui_thread = threading.Thread(target=launch_gui)
    gui_thread.start()

    # Start keyboard control
    threading.Thread(target=continuous_motor_control, daemon=True).start()

    # Start camera threads (managed as daemon threads; restart_cameras() handles lifecycle)
    start_camera_threads()

    # Keep main alive until the GUI window is closed
    gui_thread.join()

if __name__ == "__main__":
    main()
 