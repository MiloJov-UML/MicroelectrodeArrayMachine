# Microelectrode Array Machine

https://github.com/MiloJov-UML/MicroelectrodeArrayMachine

## Overview
This repository contains the Python code for a Microelectrode Array Machine capstone project. The system integrates motor control, 
relay control, and image recognition to automate the process of working with microelectrode arrays. The machine uses computer vision 
for alignment and precise positioning, and includes laser cutting capabilities for microelectrode processing.

## Features
- **Motor Control**: Precise multi-axis control system (X, Y, Z, r, t, T) for accurate positioning
- **Automated Alignment**: Computer vision-based alignment system using YOLOv8 for object detection
- **Laser Control**: Integrated laser cutting with relay control for precise microelectrode processing
- **User Interface**: GUI for manual and automated control of all machine functions
- **Keyboard Control**: Direct keyboard input for fine adjustment of motor positions
- **Image Processing**: Real-time image processing with contrast, brightness, and other adjustments
- **Video Recording**: Option to record camera feeds for documentation or analysis/training image recognition framework

## Project Structure
- **`app_gui.py`**: Main GUI application with controls for all system functions
- **`main.py`**: Entry point for the application
- **`motor_control.py`**: Motor control functions for precision movement
- **`relay_control.py`**: Relay control for laser operation
- **`image_recognition.py`**: Computer vision system using YOLOv8 for alignment
- **`data.yaml`**: Configuration for the YOLOv8 model
- **`best.pt`**: Trained YOLOv8 model for object detection
- **`arduino/`**: Arduino code for additional hardware interfaces
- **`data collection/`**: Data collection scripts and collected data
- **`runs/`**: Model training runs and results

## Dependencies
The project requires the following Python packages:

```
# Core dependencies
numpy>=1.20.0
opencv-python>=4.5.0
pillow>=8.0.0
pyserial>=3.5
tkinter>=8.6
keyboard>=0.13.5

# For image recognition
ultralytics>=8.0.0  # For YOLOv8
torch>=1.10.0
torchvision>=0.11.0

# Optional for advanced features
matplotlib>=3.4.0  # For plotting/visualization
pandas>=1.3.0      # For data handling
```

## Hardware Requirements
- PC with USB ports and Python support
- USB-SERIAL CH340 motor controller
- USB relay controller
- Two USB cameras for vision system
- Stepper motors (for X, Y, Z, r, t, T axes)
- Laser cutting system
- Microelectrode array hardware fixture

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/MiloJov-UML/MicroelectrodeArrayMachine.git
cd MicroelectrodeArrayMachine
```

### 2. Create and activate a virtual environment (recommended)
```bash
# On Windows
python -m venv venv
venv\Scripts\activate

# On macOS/Linux
python3 -m venv venv
source venv/bin/activate
```

### 3. Install dependencies
```bash
pip install numpy opencv-python pillow pyserial keyboard ultralytics torch torchvision matplotlib pandas
```

### 4. Hardware setup
1. Connect the USB-SERIAL CH340 motor controller to a USB port
2. Connect the USB relay controller to a USB port
3. Connect both USB cameras to available USB ports (make sure camera0 is the vertical camera watching the PCB 
pads/capillary/carbon fiber and Camera1 is the horizontal camera above the laser/Camera2 is the vertical camera 
watching the wheels of the extruder for clogs)
4. Ensure all stepper motors are properly connected to the controller

## Usage

### Starting the application
```bash
python main.py
```

### PCB Configuration
Upon startup, you'll be prompted to enter:
- Number of pads to process
- Distance between pad centers (µm)
- Distance from reference point to first pad (µm) (this field is now redundant)

### Basic Controls
- **Motor Control**: Enter axis (X, Y, Z, r, t, T), direction, and displacement
- **Speed Control**: Adjust motor speed (0-150)
- **Laser Control**: Toggle laser on/off
- **Origin Control**: Set current position as origin or return to origin
- **Keyboard Mode**: Enable for direct keyboard control using WASD and other keys

### Keyboard Controls
When keyboard mode is enabled:
- `W/S`: Move X axis -/+
- `A/D`: Move Y axis +/-
- `Shift/Ctrl`: Move Z axis +/-
- `Q/E`: Rotate r axis -/+
- `Z/X`: Move t axis -/+
- `R/F`: Move T axis -/+
- Hold `Space`: Reduce step size for finer control

### Advanced Features
- **Image Adjustments**: Modify contrast, brightness, saturation, gamma, and sharpness
- **Bounding Boxes**: Toggle detection visualization
- **Recording**: Toggle camera feed recording
- **Automation**: Run complete alignment and cutting sequence

## Troubleshooting

### Common Issues
1. **Motor connection fails**:
   - Ensure the USB-SERIAL CH340 driver is installed
   - Check USB port connections
   - Verify the motor controller is powered on

2. **Camera not detected**:
   - Check USB connections
   - Ensure cameras are compatible with OpenCV
   - Try different USB ports

3. **Movement issues**:
   - Check for mechanical obstructions
   - Verify motor connections and power
   - Try reducing speed for problematic movements

### Error Messages
- "Motor control device not found": Check USB connections and drivers
- "ERR from Motor Controller": Check command syntax or motor configuration
- "No response from Motor Controller": Check connections and power

## License
This project is the property of the University of Massachusetts Lowell and is used for educational purposes.

## Contributors
- Milovan Jovic, Josiah Concepcion, Maxwell Vinzi, Ryan Kinney (MiloJov-UML)