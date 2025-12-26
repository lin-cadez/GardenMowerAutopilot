#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===========================================================
Autonomous Mower Safety Controller (Raspberry Pi 4B)
===========================================================

FUNCTION:
- Reads RC receiver PWM inputs (CH1, CH2, CH5).
- ALWAYS mirrors CH1 and CH2 to mower control outputs (CH1_OUT, CH2_OUT).
- AUTOPILOT mode is considered ON when CH5 > 1900us, OFF when CH5 < 1100us (hysteresis).
- SAFETY rule (requested):
  When AUTOPILOT is ON and a target is detected:
    - mower is forced to STOP (1500/1500) immediately
    - mower stays STOPPED until target is clear continuously for 1 second
      (if target reappears during that second, the 1s timer restarts)

PINOUT (BCM numbering):
CH1 input  GPIO17
CH2 input  GPIO27
CH5 input  GPIO22
CH1 output GPIO23
CH2 output GPIO24

REQUIREMENTS:
sudo apt install python3-opencv python3-picamera2 python3-pigpio
sudo pigpiod
Place MobileNetSSD_deploy.prototxt and MobileNetSSD_deploy.caffemodel in the same folder.
"""

import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import cv2
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

# --- Safety clear delay ---
CLEAR_DELAY_S = 1.0

# -----------------------------------------------------

pi = pigpio.pi()
if not pi.connected:
    raise OSError("Cannot connect to pigpio daemon. Run: sudo pigpiod")

# Initialize outputs
pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)

# State variables
target_detected = False
autopilot_mode = False
current_frame = None

# Safety latch states (NEW)
safety_latched = False
target_clear_since = None

_last_tick = {}
_pulse_widths = {CH1_IN: NEUTRAL_PULSE, CH2_IN: NEUTRAL_PULSE, CH5_IN: NEUTRAL_PULSE}


def create_callback(gpio):
    def cb(gpio, level, tick):
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
    global target_detected, current_frame

    print("[INFO] Loading MobileNetSSD model...")
    net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
    cam.start()
    time.sleep(1)

    print("[INFO] Camera active. Scanning for targets...")

    frame_count = 0
    last_state = None

    while True:
        frame = cam.capture_array()
        current_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        frame_count += 1
        if frame_count % 2 != 0:
            continue

        small = cv2.resize(current_frame, (160, 160))
        blob = cv2.dnn.blobFromImage(small, 0.007843, (160, 160), 127.5)

        net.setInput(blob)
        detections = net.forward()

        detected_now = any(
            float(detections[0, 0, i, 2]) >= CONF_THRESH and
            CLASSES[int(detections[0, 0, i, 1])] in TARGETS
            for i in range(detections.shape[2])
        )

        if detected_now != last_state:
            last_state = detected_now
            target_detected = detected_now
            print("[SAFETY] Target detected!" if target_detected else "[INFO] Target clear.")

        time.sleep(0.05)


# =====================================================
# Control Loop
# =====================================================
def control_loop():
    global autopilot_mode, safety_latched, target_clear_since

    print("[INFO] Control loop active.")

    last_mode = None
    last_latched = None

    while True:
        ch1 = _pulse_widths.get(CH1_IN, NEUTRAL_PULSE)
        ch2 = _pulse_widths.get(CH2_IN, NEUTRAL_PULSE)
        ch5 = _pulse_widths.get(CH5_IN, NEUTRAL_PULSE)

        # Determine autopilot mode (with hysteresis thresholds)
        if ch5 > AUTOPILOT_ON:
            autopilot_mode = True
        elif ch5 < AUTOPILOT_OFF:
            autopilot_mode = False

        if autopilot_mode != last_mode:
            print("[MODE] Autopilot ON" if autopilot_mode else "[MODE] Autopilot OFF")
            last_mode = autopilot_mode

        now = time.time()

        # Safety latch logic with 1s clear delay
        if autopilot_mode:
            if target_detected:
                safety_latched = True
                target_clear_since = None
            else:
                if safety_latched:
                    if target_clear_since is None:
                        target_clear_since = now
                    elif (now - target_clear_since) >= CLEAR_DELAY_S:
                        safety_latched = False
                        target_clear_since = None
        else:
            # If autopilot is OFF, do not latch safety
            safety_latched = False
            target_clear_since = None

        if safety_latched != last_latched:
            if safety_latched:
                print("[SAFETY] STOP LATCHED (waiting for clear)")
            else:
                print("[SAFETY] Cleared -> RC control enabled")
            last_latched = safety_latched

        # Output decision:
        # - If autopilot ON AND (target present OR latched) => STOP
        # - Else => mirror RC
        if autopilot_mode and (target_detected or safety_latched):
            pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
            pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
        else:
            pi.set_servo_pulsewidth(CH1_OUT, ch1)
            pi.set_servo_pulsewidth(CH2_OUT, ch2)

        time.sleep(0.02)  # ~50 Hz


# =====================================================
# MJPEG Web Stream Thread
# =====================================================
def mjpeg_stream_thread():

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != "/stream":
                self.send_response(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                if current_frame is None:
                    time.sleep(0.05)
                    continue

                ok, jpeg = cv2.imencode(".jpg", current_frame, [cv2.IMWRITE_JPEG_QUALITY, 45])
                if not ok:
                    continue

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b"\r\n")
                except BrokenPipeError:
                    break

    server = HTTPServer(("0.0.0.0", 8090), Handler)
    print("[WEB] Video available at: http://<PI-IP>:8090/stream")
    server.serve_forever()


# =====================================================
# Main
# =====================================================
if __name__ == "__main__":
    try:
        threading.Thread(target=camera_thread, daemon=True).start()
        threading.Thread(target=mjpeg_stream_thread, daemon=True).start()
        control_loop()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting safely...")
    finally:
        pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
        pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
        pi.stop()
        print("[INFO] pigpio stopped. Goodbye!")
