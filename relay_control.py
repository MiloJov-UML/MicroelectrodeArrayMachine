# relay_control.py

import serial
import time
import threading
import queue
from tkinter import messagebox
from motor_control import (
    serial_lock,
    send_command,
    find_port,
    stop_motor_control,
    is_emergency_stop_requested,
    set_axis_limit,
    clear_axis_limit,
)

relay_ser = None

# ---------------------------------------------------------------------------
# Always-on relay serial monitor
# ---------------------------------------------------------------------------
# Events set by the monitor thread when specific messages arrive.
_r_limit_event   = threading.Event()
_z_limit_event   = threading.Event()
_magnet_event    = threading.Event()
_servo_done_event = threading.Event()
# Queue for stepper-motor completion messages (motor_forward / motor_backward).
_motor_done_queue = queue.Queue()

_relay_monitor_thread = None

# How long (seconds) with no limit messages before we treat the switch as released.
_LIMIT_RELEASE_TIMEOUT = 0.5

def _relay_monitor_loop():
    """Daemon thread: reads every line from relay_ser and dispatches events.
    Runs for the lifetime of the process; safe to start once after connection.

    Limit-switch handling uses edge detection so the motor is stopped exactly
    once on first contact.  The blocked direction is recorded so backing out
    (moving the opposite way) is always permitted.  The block is cleared after
    _LIMIT_RELEASE_TIMEOUT seconds of silence from that switch."""
    global relay_ser

    # Per-axis limit tracking (local to this thread — no shared state needed).
    _r_at_limit = False
    _r_limit_last_seen = 0.0
    _z_at_limit = False
    _z_limit_last_seen = 0.0

    while True:
        ser = relay_ser
        if ser is None:
            time.sleep(0.1)
            continue
        try:
            raw = ser.readline()
        except Exception:
            time.sleep(0.1)
            continue

        now = time.time()

        # Release detection: if the switch has been silent long enough, un-block.
        if _r_at_limit and (now - _r_limit_last_seen) > _LIMIT_RELEASE_TIMEOUT:
            _r_at_limit = False
            clear_axis_limit('r')
            _r_limit_event.clear()
            print("Relay monitor: R limit switch released — R axis unblocked.")
        if _z_at_limit and (now - _z_limit_last_seen) > _LIMIT_RELEASE_TIMEOUT:
            _z_at_limit = False
            clear_axis_limit('Z')
            _z_limit_event.clear()
            print("Relay monitor: Z limit switch released — Z axis unblocked.")

        if not raw:
            continue
        try:
            line = raw.decode('utf-8', errors='replace').strip()
        except Exception:
            continue
        if not line:
            continue
        print(f"[Relay monitor] {line}")

        if "R limit" in line:
            _r_limit_last_seen = now
            if not _r_at_limit:
                # Edge: first contact — stop the motor and block the '+' direction.
                stop_motor_control()
                set_axis_limit('r', '+')
                _r_at_limit = True
                _r_limit_event.set()
                print("Relay monitor: R limit hit — motor stopped, R+ blocked.")

        if "Z limit" in line:
            _z_limit_last_seen = now
            if not _z_at_limit:
                stop_motor_control()
                set_axis_limit('Z', '+')
                _z_at_limit = True
                _z_limit_event.set()
                print("Relay monitor: Z limit hit — motor stopped, Z+ blocked.")

        if "Magnet Detected" in line:
            _magnet_event.set()
        if "Motion complete" in line:
            _servo_done_event.set()
        if any(kw in line for kw in ("Motor forward complete", "Motor backward complete", "Motor released")):
            _motor_done_queue.put(line)

def start_relay_monitor():
    """Start the background relay monitor.  Safe to call multiple times."""
    global _relay_monitor_thread
    if _relay_monitor_thread is not None and _relay_monitor_thread.is_alive():
        return
    t = threading.Thread(target=_relay_monitor_loop, daemon=True, name="relay-monitor")
    t.start()
    _relay_monitor_thread = t
    print("Relay monitor thread started.")

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
        # Use a short timeout so the monitor thread returns quickly when idle.
        ser = serial.Serial(port, 9600, timeout=0.1)
        relay_ser = ser
        print(f"Connected to Relay on {port}")
        start_relay_monitor()
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to relay on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to relay on {port}")
        return None

def laser_relay_on():
    """Turn the laser on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_On", "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def laser_relay_off():
    """Turn the laser off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_Off", "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

# Phill's edit
def solenoid_relay_on():
    """Turn the solenoid on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Solenoid_Relay_On", "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def solenoid_relay_off():
    """Turn the solenoid off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Solenoid_Relay_Off", "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def nordson_on():
    """Turn the Nordson on."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Nordson_On", "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def nordson_off():
    """Turn the Nordson off."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "Nordson_Off", "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def servo_to(angle: int, step_ms: int = 15):
    """Move the servo to angle, stepping step_ms milliseconds per degree (default 15).

    The Arduino sweeps 1° at a time so the move is smooth and gentle.  This
    function blocks until the Arduino signals 'Motion complete' or the estimated
    worst-case travel time elapses.
    """
    global relay_ser
    if relay_ser:
        command = f"Servo_To_{angle}_{step_ms}"
        _servo_done_event.clear()
        send_command(relay_ser, command, "Relay Controller", blocking=False)
        # Wait for the monitor thread to receive 'Motion complete'.
        # Worst case: full 270° sweep + 1 s serial latency buffer.
        timeout = (270 * step_ms / 1000.0) + 1.0
        elapsed = 0.0
        while elapsed < timeout:
            if is_emergency_stop_requested():
                print("servo_to: aborted by emergency stop.")
                return
            if _servo_done_event.wait(timeout=0.1):
                return
            elapsed += 0.1
        print(f"Warning: servo_to({angle}) timed out after {timeout:.1f}s")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def pnp_forward(speed: int):
    """Move the pick-and-place mechanism forward at the specified speed (0-100)."""
    global relay_ser
    if relay_ser:
        command = f"PNP_Backward_{speed}"
        # The DC motor starts running and never sends a completion signal — use
        # blocking=False like laser/solenoid to avoid racing with _relay_monitor_loop.
        send_command(relay_ser, command, "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def pnp_backward(speed: int):
    """Move the pick-and-place mechanism backward at the specified speed (0-100)."""
    global relay_ser
    if relay_ser:
        command = f"PNP_Forward_{speed}"
        send_command(relay_ser, command, "Relay Controller", blocking=False)
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def pnp_release():
    """Release the pick-and-place mechanism."""
    global relay_ser
    if relay_ser:
        send_command(relay_ser, "PNP_Release", "Relay Controller", blocking=False)
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
        # Drain any stale completions before sending so we don't pick up an old one.
        while not _motor_done_queue.empty():
            _motor_done_queue.get_nowait()
        with serial_lock:
            relay_ser.write((command + '\n').encode())
            print(f"Sent command to Relay Controller: {command}")
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                msg = _motor_done_queue.get(timeout=0.2)
                if "Motor forward complete" in msg:
                    print("Motor forward movement completed")
                    return True
            except queue.Empty:
                pass
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
        # Drain any stale completions before sending so we don't pick up an old one.
        while not _motor_done_queue.empty():
            _motor_done_queue.get_nowait()
        with serial_lock:
            relay_ser.write((command + '\n').encode())
            print(f"Sent command to Relay Controller: {command}")
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                msg = _motor_done_queue.get(timeout=0.2)
                if "Motor backward complete" in msg:
                    print("Motor backward movement completed")
                    return True
            except queue.Empty:
                pass
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
    # The relay monitor thread fires stop_motor_control() and sets _r_limit_event
    # the instant "R limit" arrives on the serial line.  We just wait here.
    _r_limit_event.clear()
    while True:
        if is_emergency_stop_requested():
            print("R calibration cancelled by emergency stop.")
            return None
        if _r_limit_event.wait(timeout=0.2):
            _r_limit_event.clear()
            print("R limit reached")
            return "R limit"
          
def Z_calibrate():
    # The relay monitor thread fires stop_motor_control() and sets _z_limit_event
    # the instant "Z limit" arrives on the serial line.  We just wait here.
    _z_limit_event.clear()
    while True:
        if is_emergency_stop_requested():
            print("Z calibration cancelled by emergency stop.")
            return None
        if _z_limit_event.wait(timeout=0.2):
            _z_limit_event.clear()
            print("Z limit reached")
            return "Z limit"

def mag_detector():
    # The relay monitor thread sets _magnet_event when "Magnet Detected" arrives.
    _magnet_event.clear()
    while True:
        if is_emergency_stop_requested():
            print("Magnet detection cancelled by emergency stop.")
            return None
        if _magnet_event.wait(timeout=0.2):
            _magnet_event.clear()
            pnp_release()
            print("Connector Available")
            return "Magnet Detected"

def wait_for_magnet(cancel_event=None, poll_interval=0.2):
    """Block until a magnet is detected, cancel_event is set, or emergency stop.

    Unlike mag_detector(), does not call pnp_release() on detection and accepts
    an external threading.Event so the caller can cancel without triggering an
    emergency stop.  Returns 'Magnet Detected' or None if cancelled.
    """
    _magnet_event.clear()
    while True:
        if cancel_event is not None and cancel_event.is_set():
            return None
        if is_emergency_stop_requested():
            return None
        if _magnet_event.wait(timeout=poll_interval):
            _magnet_event.clear()
            print("Hall effect: magnet detected.")
            return "Magnet Detected"
