# relay_control.py

import serial
from tkinter import messagebox
from motor_control import serial_lock, send_command, find_port

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
