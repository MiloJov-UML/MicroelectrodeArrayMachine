# relay_control.py

import serial
import time
from tkinter import messagebox
from motor_control import serial_lock, send_command, find_port, stop_motor_control

relay_ser = None

def auto_connect_relay():
    """Auto-detect and connect to the relay device (Arduino)."""
    global relay_ser
    relay_port = find_port("Arduino Uno")  # Adjust if your Arduino is labeled differently
    if relay_port:
        connect_relay(relay_port)
    else:
        messagebox.showerror("Error", "Relay device not found.")

def connect_relay(port):
    """Connect to the relay device (Arduino) on the given COM port."""
    global relay_ser
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        relay_ser = ser
        print(f"Connected to Relay on {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to relay on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to relay on {port}")
        return None

def laser_relay_on():
    """Turn the laser on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_On", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def laser_relay_off():
    """Turn the laser off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_Off", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

# Phill's edit
def solenoid_relay_on():
    """Turn the solenoid on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Solenoid_Relay_On", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def solenoid_relay_off():
    """Turn the solenoid off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Solenoid_Relay_Off", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def nordson_on():
    """Turn the Nordson on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Nordson_On", "Relay Controller")
        #relay_ser.write(("Nordson_On\n").encode())
        #print("Nordson ON")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def nordson_off():
    """Turn the Nordson off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Nordson_Off", "Relay Controller")
        #relay_ser.write(("Nordson_Off\n").encode())
        #print("Nordson OFF")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def servo_to(angle: int):
    """Move the servo to the specified angle (0-180 degrees)."""
    global relay_ser
    if relay_ser:
        command = f"Servo_To_{angle}"
        send_command(relay_ser, command, "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def pnp_forward(speed: int):
    """Move the pick-and-place mechanism forward at the specified speed (0-100)."""
    global relay_ser
    if relay_ser:
        command = f"PNP_Forward_{speed}"
        send_command(relay_ser, command, "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def pnp_backward(speed: int):
    """Move the pick-and-place mechanism backward at the specified speed (0-100)."""
    global relay_ser
    if relay_ser:
        command = f"PNP_Backward_{speed}"
        send_command(relay_ser, command, "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def pnp_release():
    """Release the pick-and-place mechanism."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "PNP_Release", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")


def motor_forward(steps=100, wait_for_completion=True, timeout=30):
    """Move the stepper motor forward by the specified number of steps.
    
    Args:
        steps (int): Number of steps to move forward (default: 100, max: 10000)
        wait_for_completion (bool): Wait for motor to complete before returning (default: True)
        timeout (float): Maximum seconds to wait for completion (default: 30)
    """
    global relay_ser
    if not relay_ser:
        messagebox.showerror("Error", "Not connected to relay device.")
        return False
    
    command = f"Motor_Forward_{steps}"
    
    if wait_for_completion:
        # Send command and wait for completion message
        with serial_lock:
            relay_ser.write((command + '\n').encode())
            print(f"Sent command to Relay Controller: {command}")
            
            start_time = time.time()
            response_buffer = ""
            
            while time.time() - start_time < timeout:
                if relay_ser.in_waiting > 0:
                    data = relay_ser.read(relay_ser.in_waiting).decode()
                    response_buffer += data
                    print(f"Relay Controller response: {data.strip()}")
                    
                    # Check if we got the completion message
                    if "Motor forward complete" in response_buffer:
                        print("Motor forward movement completed")
                        return True
                
                time.sleep(0.1)  # Small delay between checks
            
            print(f"Warning: Motor forward command timed out after {timeout}s")
            return False
    else:
        send_command(relay_ser, command, "Relay Controller")
        return True

def motor_backward(steps=100, wait_for_completion=True, timeout=30):
    """Move the stepper motor backward by the specified number of steps.
    
    Args:
        steps (int): Number of steps to move backward (default: 100, max: 10000)
        wait_for_completion (bool): Wait for motor to complete before returning (default: True)
        timeout (float): Maximum seconds to wait for completion (default: 30)
    """
    global relay_ser
    if not relay_ser:
        messagebox.showerror("Error", "Not connected to relay device.")
        return False
    
    command = f"Motor_Backward_{steps}"
    
    if wait_for_completion:
        # Send command and wait for completion message
        with serial_lock:
            relay_ser.write((command + '\n').encode())
            print(f"Sent command to Relay Controller: {command}")
            
            start_time = time.time()
            response_buffer = ""
            
            while time.time() - start_time < timeout:
                if relay_ser.in_waiting > 0:
                    data = relay_ser.read(relay_ser.in_waiting).decode()
                    response_buffer += data
                    print(f"Relay Controller response: {data.strip()}")
                    
                    # Check if we got the completion message
                    if "Motor backward complete" in response_buffer:
                        print("Motor backward movement completed")
                        return True
                
                time.sleep(0.1)  # Small delay between checks
            
            print(f"Warning: Motor backward command timed out after {timeout}s")
            return False
    else:
        send_command(relay_ser, command, "Relay Controller")
        return True

def motor_release():
    """Release the stepper motor to stop holding torque and eliminate idle pulsing."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Motor_Release", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")


# Don't modify - Phillipe's edit
def start_r_poll():
    """Turn the R limit switch monitor on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Start_R_Poll", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def end_r_poll():
    """Turn the R limit switch monitor off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "End_R_Poll", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

# Don't modify - Phillipe's edit
def start_z_poll():
    """Turn the Z limit switch monitor on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Start_Z_Poll", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def end_z_poll():
    """Turn the Z limit switch monitor off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "End_Z_Poll", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def r_calibrate():

    while True:
        relay_ser.reset_input_buffer()  # Clear any existing data in the buffer
        resp = relay_ser.data = relay_ser.read_until(b'R limit').decode("utf-8").strip()
        print(str(resp))
        
        if str(resp) != None:
            if str(resp) == "R limit":
                stop_motor_control()
                print('R limit reached')
                return resp
          
def Z_calibrate():
    
    while True:
        relay_ser.reset_input_buffer()  # Clear any existing data in the buffer
        respo = relay_ser.data = relay_ser.read_until(b'Z limit').decode("utf-8").strip()
        print(str(respo))
        
        if str(respo) != None:
            if str(respo) == "Z limit":
                stop_motor_control()
                print('Z limit reached')
                return respo

def mag_detector():

    while True:
        relay_ser.reset_input_buffer()  # Clear any existing data in the buffer
        resp = relay_ser.data = relay_ser.read_until(b'Magnet Detected').decode("utf-8").strip()
        print(str(resp))
        
        if str(resp) != None:
            if str(resp) == "Magnet Detected":
                pnp_release()
                print('Connector Available')
                return resp           
           
        
            
    
    
        

