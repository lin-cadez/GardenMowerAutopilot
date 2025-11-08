#!/usr/bin/env python3
"""
===========================================================
Autonomous Mower Safety Controller (Raspberry Pi 4B)
===========================================================

FUNCTION:
- Reads RC receiver PWM inputs (CH1, CH2, CH5).
- Mirrors CH1 and CH2 to mower control outputs (CH1_OUT, CH2_OUT).
- Detects people with PiCamera + MobileNetSSD model.
- When AUTOPILOT mode is active (CH5 > 1900 Âµs), the mower will automatically STOP (1500 Âµs neutral) if a person is detected.
- When AUTOPILOT is off (CH5 < 1100 Âµs), the user has full control even if a person is detected.

PINOUT (BCM numbering):
------------------------------------------------------------
| Function        | GPIO | Physical Pin | Direction | Description             |
|-----------------|------|--------------|-----------|-------------------------|
| CH1 input       | 17   | 11           | Input     | RC receiver channel 1   |
| CH2 input       | 27   | 13           | Input     | RC receiver channel 2   |
| CH5 input       | 22   | 15           | Input     | RC receiver channel 5   |
| CH1 output      | 23   | 16           | Output    | Motor controller 1      |
| CH2 output      | 24   | 18           | Output    | Motor controller 2      |
| GND (common)    | ---  | Any GND pin  | ---       | Shared ground           |
------------------------------------------------------------

REQUIREMENTS:
sudo apt install python3-opencv python3-picamera2 python3-pigpio
sudo pigpiod
Place MobileNetSSD_deploy.prototxt and MobileNetSSD_deploy.caffemodel in the same folder.

"""
import time
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
import pigpio

# ------------------- Configuration -------------------
CONF_THRESH = 0.5
PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"

TARGETS = {"person", "cat", "dog"}

CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

# --- GPIO pin assignments ---
CH1_IN = 17
CH2_IN = 27
CH5_IN = 22
CH1_OUT = 23
CH2_OUT = 24

# --- PWM thresholds ---
MIN_PULSE = 900
MAX_PULSE = 2100
NEUTRAL_PULSE = 1500

# --- Mode thresholds ---
AUTOPILOT_ON = 1900
AUTOPILOT_OFF = 1100

# -----------------------------------------------------

pi = pigpio.pi()
if not pi.connected:
    raise OSError("Cannot connect to pigpio daemon. Run: sudo pigpiod")

# Initialize outputs
pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)

# State variables
tagret_detected = False
autopilot_mode = False

_last_tick = {}
_pulse_widths = {CH1_IN: NEUTRAL_PULSE, CH2_IN: NEUTRAL_PULSE, CH5_IN: NEUTRAL_PULSE}


def create_callback(gpio):
    def cb(gpio, level, tick):
        global _last_tick, _pulse_widths
        if level == 1:
            _last_tick[gpio] = tick
        elif level == 0 and gpio in _last_tick:
            pulse_len = pigpio.tickDiff(_last_tick[gpio], tick)
            if MIN_PULSE <= pulse_len <= MAX_PULSE:
                _pulse_widths[gpio] = pulse_len
    return cb


for pin in (CH1_IN, CH2_IN, CH5_IN):
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
    pi.callback(pin, pigpio.EITHER_EDGE, create_callback(pin))

# =====================================================
# Camera Thread
# =====================================================

def camera_thread():
    global tagret_detected

    print("[INFO] Loading MobileNetSSD model...")
    net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (320, 240)}))
    cam.start()
    time.sleep(1)

    print("[INFO] Camera active. Scanning for people...")
 
                      
    while True:
        
        if not autopilot_mode:
            time.sleep(0.3);
            continue
        #OD TU NAPREJ DELA SAMO V STANJU AUTOPLIOTA:    
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
 
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843,
                                     (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()

        detected_now = any(
            float(detections[0, 0, i, 2]) >= CONF_THRESH and
            CLASSES[int(detections[0, 0, i, 1])] in TARGETS
            for i in range(detections.shape[2])
)

        if detected_now != tagret_detected:
            target_detected = detected_now
            print("[SAFETY] Target detected!" if target_detected else "[INFO] Target clear.")

        time.sleep(0.05)

# =====================================================
# Control Loop
# =====================================================

def control_loop():
    global autopilot_mode, tagret_detected

    print("[INFO] Control loop active.")

    while True:
        ch1 = _pulse_widths.get(CH1_IN, NEUTRAL_PULSE)
        ch2 = _pulse_widths.get(CH2_IN, NEUTRAL_PULSE)
        ch5 = _pulse_widths.get(CH5_IN, NEUTRAL_PULSE)

        if ch5 > AUTOPILOT_ON and not autopilot_mode:
            autopilot_mode = True
            print("[MODE] Autopilot ON")
            if autopilot_mode and tagret_detected:
                pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
                pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
            else:
                pi.set_servo_pulsewidth(CH1_OUT, ch1)
                pi.set_servo_pulsewidth(CH2_OUT, ch2)
                
        elif ch5 < AUTOPILOT_OFF and autopilot_mode:
            autopilot_mode = False
            print("[MODE] Autopilot OFF")

        

        time.sleep(0.02)  # ~50 Hz


# =====================================================
# Main
# =====================================================

if __name__ == "__main__":
    try:
        threading.Thread(target=camera_thread, daemon=True).start()
        control_loop()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting safely...")
    finally:
        pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
        pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
        pi.stop()
        print("[INFO] pigpio stopped. Goodbye!")
