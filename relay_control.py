# relay_control.py

import serial
import time
from tkinter import messagebox
from motor_control import serial_lock, direct_command, send_command, find_port

relay_ser = None
sol_ser = None
nord_ser = None

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

def solenoid_relay_on():
    """Turn the laser on."""
    global relay_ser
    direct_command(relay_ser, "Solenoid_Relay_On")
    

def solenoid_relay_off():
    """Turn the laser off."""
    global relay_ser
    direct_command(relay_ser, "Solenoid_Relay_Off")


def nordson_on():
    """Turn the Nordson on."""
    global relay_ser
    direct_command(relay_ser, "Nordson_On")
    

def nordson_off():
    """Turn the Nordson off."""
    global relay_ser
    direct_command(relay_ser, "Nordson_Off")
    

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
