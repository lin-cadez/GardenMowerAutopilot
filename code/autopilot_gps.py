#!/usr/bin/env python3

import time
import json
import threading
import math
import serial
import cv2
import numpy as np
import pigpio
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
from shapely.geometry import Polygon
from picamera2 import Picamera2

# ================= CONFIGURATION =================
# Hardware
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080

# Mower Settings (Meters)
MOWER_WIDTH_M = 1.2
OVERLAP_M = 0.4
EFFECTIVE_WIDTH = MOWER_WIDTH_M - OVERLAP_M

# RC / GPIO PINS
CH1_IN = 17   # Steering Input (RC Receiver)
CH2_IN = 27   # Throttle Input (RC Receiver)
CH5_IN = 22   # Switch (RC Receiver)
CH1_OUT = 23  # Steering Output (Motor Controller)
CH2_OUT = 24  # Throttle Output (Motor Controller)

# PWM Constants
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000
# Auto Drive Settings
AUTO_THROTTLE = 1600  # Slow mowing speed
STEER_GAIN = 400      # PID P-gain for steering

# Safety Camera
CONF_THRESH = 0.5
PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
TARGETS = {"person", "cat", "dog"}
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
           "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
           "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# ================= STATE MANAGEMENT =================
state = {
    "lat": 0.0, "lon": 0.0, "heading": 0.0, "fix": "0", "accuracy_ok": False,
    "mode": "MANUAL",  # MANUAL, RECORDING, AUTO_MOW
    "perimeter": [],   
    "mow_path": [],    
    "target_idx": 0,   
    "safety_stop": False,
    "mowed_trail": []  
}
state_lock = threading.Lock()

# RC Inputs (Shared Global)
_last_tick = {}
_pulse_widths = {CH1_IN: PWM_NEUTRAL, CH2_IN: PWM_NEUTRAL, CH5_IN: PWM_NEUTRAL}

current_frame = None
pi = pigpio.pi()

# ================= HELPER FUNCTIONS =================
def get_xy_from_latlon(lat, lon, anchor_lat, anchor_lon):
    r = 6371000
    x = r * math.radians(lon - anchor_lon) * math.cos(math.radians(anchor_lat))
    y = r * math.radians(lat - anchor_lat)
    return x, y

def get_latlon_from_xy(x, y, anchor_lat, anchor_lon):
    r = 6371000
    lat = anchor_lat + math.degrees(y / r)
    lon = anchor_lon + math.degrees(x / (r * math.cos(math.radians(anchor_lat))))
    return lat, lon

def generate_snail_path(perimeter_latlon):
    if len(perimeter_latlon) < 3: return []
    anchor = perimeter_latlon[0]
    poly_pts = [get_xy_from_latlon(lat, lon, anchor[0], anchor[1]) for lat, lon in perimeter_latlon]
    poly = Polygon(poly_pts)
    if not poly.is_valid: poly = poly.buffer(0)

    path_xy = []
    current_poly = poly
    offset_dist = -EFFECTIVE_WIDTH

    while True:
        next_poly = current_poly.buffer(offset_dist)
        if next_poly.is_empty: break
        if next_poly.geom_type == 'MultiPolygon':
            next_poly = max(next_poly.geoms, key=lambda a: a.area)
        
        coords = list(next_poly.exterior.coords)
        path_xy.extend(coords)
        current_poly = next_poly
    
    return [get_latlon_from_xy(x, y, anchor[0], anchor[1]) for x, y in path_xy]

def nmea_to_deg(value, hemi):
    if not value: return 0.0
    d = float(value)
    deg = int(d // 100)
    coord = deg + (d - deg * 100) / 60.0
    return -coord if hemi in ("S","W") else coord

# ================= GNSS THREAD =================
def gnss_thread():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    last_pos = None

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                p = line.split(",")
                if len(p) > 9 and p[2] and p[4]:
                    lat = nmea_to_deg(p[2], p[3])
                    lon = nmea_to_deg(p[4], p[5])
                    fix = p[6]
                    
                    heading = state["heading"]
                    if last_pos:
                        dist = math.sqrt((lat-last_pos[0])**2 + (lon-last_pos[1])**2)
                        if dist > 0.000005: # Update heading if moved > ~0.5m
                            y = math.sin(math.radians(lon - last_pos[1])) * math.cos(math.radians(lat))
                            x = math.cos(math.radians(last_pos[0])) * math.sin(math.radians(lat)) - \
                                math.sin(math.radians(last_pos[0])) * math.cos(math.radians(lat)) * math.cos(math.radians(lon - last_pos[1]))
                            heading = (math.degrees(math.atan2(y, x)) + 360) % 360
                    last_pos = (lat, lon)

                    with state_lock:
                        state["lat"] = lat
                        state["lon"] = lon
                        state["fix"] = fix
                        state["heading"] = heading
                        state["accuracy_ok"] = (fix in ['4', '5'])
                        
                        if state["mode"] == "RECORDING":
                            if not state["perimeter"] or math.dist(state["perimeter"][-1], [lat, lon]) > 0.00001:
                                state["perimeter"].append([lat, lon])
                        if state["mode"] == "AUTO_MOW":
                            state["mowed_trail"].append([lat, lon])
        except: time.sleep(1)

# ================= CAMERA THREAD =================
def camera_thread():
    global current_frame
    net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
    cam.start()
    time.sleep(1)
    
    frame_skip = 0
    while True:
        img = cam.capture_array()
        current_frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        frame_skip += 1
        if frame_skip % 3 != 0: continue

        with state_lock: mode = state["mode"]
        
        # Only process object detection if we are Mowing
        if mode == "AUTO_MOW":
            blob = cv2.dnn.blobFromImage(cv2.resize(current_frame, (300, 300)), 0.007843, (300, 300), 127.5)
            net.setInput(blob)
            dets = net.forward()
            
            danger = False
            for i in range(dets.shape[2]):
                if dets[0, 0, i, 2] > CONF_THRESH and CLASSES[int(dets[0, 0, i, 1])] in TARGETS:
                    danger = True
                    break
            
            with state_lock: state["safety_stop"] = danger
            if danger: print("[SAFETY] STOP! Obstacle Detected")
        else:
            with state_lock: state["safety_stop"] = False

# ================= RC INPUT & CONTROL =================
def create_callback(gpio):
    def cb(gpio, level, tick):
        global _last_tick, _pulse_widths
        if level == 1:
            _last_tick[gpio] = tick
        elif level == 0 and gpio in _last_tick:
            width = pigpio.tickDiff(_last_tick[gpio], tick)
            if 900 <= width <= 2100:
                _pulse_widths[gpio] = width
    return cb

def control_thread():
    # 1. Setup Input Listeners
    for pin in (CH1_IN, CH2_IN, CH5_IN):
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
        pi.callback(pin, pigpio.EITHER_EDGE, create_callback(pin))

    # 2. Setup Outputs
    pi.set_servo_pulsewidth(CH1_OUT, PWM_NEUTRAL)
    pi.set_servo_pulsewidth(CH2_OUT, PWM_NEUTRAL)

    print("[CONTROL] RC Passthrough & Auto-Drive Active")

    while True:
        # Read current RC Stick positions
        rc_steer = _pulse_widths.get(CH1_IN, PWM_NEUTRAL)
        rc_throttle = _pulse_widths.get(CH2_IN, PWM_NEUTRAL)
        rc_switch = _pulse_widths.get(CH5_IN, PWM_NEUTRAL)

        with state_lock:
            mode = state["mode"]
            lat = state["lat"]
            lon = state["lon"]
            heading = state["heading"]
            path = state["mow_path"]
            t_idx = state["target_idx"]
            safety = state["safety_stop"]

        # Default outputs (Manual Mode)
        out_steer = rc_steer
        out_throttle = rc_throttle

        # --- AUTO MODE LOGIC ---
        if mode == "AUTO_MOW":
            # Check RC Override (Emergency Switch on Remote)
            if rc_switch < 1100: 
                with state_lock: state["mode"] = "MANUAL"
                print("[OVERRIDE] Manual Switch Detected")
            
            elif safety:
                out_throttle = PWM_NEUTRAL # STOP
                out_steer = PWM_NEUTRAL
            
            elif t_idx < len(path):
                target = path[t_idx]
                dist = math.sqrt((target[0]-lat)**2 + (target[1]-lon)**2) * 111139
                
                if dist < 0.5:
                    with state_lock: state["target_idx"] += 1
                else:
                    # Navigation
                    y = math.sin(math.radians(target[1] - lon)) * math.cos(math.radians(target[0]))
                    x = math.cos(math.radians(lat)) * math.sin(math.radians(target[0])) - \
                        math.sin(math.radians(lat)) * math.cos(math.radians(target[0])) * math.cos(math.radians(target[1] - lon))
                    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
                    
                    err = bearing - heading
                    if err > 180: err -= 360
                    if err < -180: err += 360
                    
                    steer_corr = err * STEER_GAIN
                    out_steer = max(PWM_MIN, min(PWM_MAX, PWM_NEUTRAL + steer_corr))
                    out_throttle = AUTO_THROTTLE
            else:
                # Finished
                out_throttle = PWM_NEUTRAL
                with state_lock: state["mode"] = "MANUAL"

        # Apply Output
        pi.set_servo_pulsewidth(CH1_OUT, out_steer)
        pi.set_servo_pulsewidth(CH2_OUT, out_throttle)
        time.sleep(0.05)

# ================= WEB SERVER =================
class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_response(200); self.send_header("Content-type", "text/html"); self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif parsed.path == "/status":
            with state_lock:
                d = {
                    "lat": state["lat"], "lon": state["lon"], "fix": state["fix"],
                    "mode": state["mode"], "safety": state["safety_stop"],
                    "perimeter": state["perimeter"], "path": state["mow_path"],
                    "trail": state["mowed_trail"]
                }
            self.send_response(200); self.send_header("Content-type", "application/json"); self.end_headers()
            self.wfile.write(json.dumps(d).encode())
        elif parsed.path == "/stream":
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                if current_frame is None: time.sleep(0.1); continue
                try:
                    _, jpeg = cv2.imencode('.jpg', current_frame)
                    self.wfile.write(b'--frame\r\n'); self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes()); self.wfile.write(b'\r\n')
                    time.sleep(0.1)
                except: break

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/cmd":
            l = int(self.headers.get('Content-Length'))
            data = json.loads(self.rfile.read(l))
            cmd = data.get("command")
            
            with state_lock:
                if cmd == "emergency_stop":
                    state["mode"] = "MANUAL" # Force Manual immediately
                    print("[WEB] EMERGENCY STOP TRIGGERED")
                elif cmd == "start_rec":
                    state["perimeter"] = []; state["mode"] = "RECORDING"
                elif cmd == "stop_rec":
                    state["mode"] = "MANUAL"
                    state["mow_path"] = generate_snail_path(state["perimeter"])
                elif cmd == "start_mow":
                    if state["mow_path"]: state["target_idx"] = 0; state["mode"] = "AUTO_MOW"
            
            self.send_response(200); self.end_headers(); self.wfile.write(b"OK")

# ================= HTML UI =================
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>RC Mower</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <style>
        body { font-family: sans-serif; text-align: center; background: #eee; }
        #map { height: 50vh; width: 100%; border-bottom: 2px solid #333; }
        #video { width: 320px; height: 240px; background: #000; margin: 10px; border: 2px solid #333; }
        .btn { padding: 15px 25px; font-size: 16px; margin: 5px; cursor: pointer; border: none; border-radius: 5px; color: white;}
        .estop { background-color: #ff0000; font-weight: bold; font-size: 20px; padding: 20px 40px; width: 80%; border: 4px solid darkred; animation: pulse 1s infinite; }
        .rec { background: #2196F3; } .gen { background: #FF9800; } .go { background: #4CAF50; }
        @keyframes pulse { 0% { transform: scale(1); } 50% { transform: scale(1.02); } 100% { transform: scale(1); } }
        #info { margin: 10px; font-weight: bold; font-size: 18px; }
    </style>
</head>
<body>
    <div id="map"></div>
    <div id="info">Mode: <span id="mode">-</span> | Fix: <span id="fix">-</span> | Safety: <span id="safe">-</span></div>
    
    <button class="btn estop" onclick="sendCmd('emergency_stop')">🚨 EMERGENCY STOP & MANUAL 🚨</button>
    <br>
    <img id="video" src="/stream">
    <br>
    <button class="btn rec" onclick="sendCmd('start_rec')">1. Record Perimeter</button>
    <button class="btn gen" onclick="sendCmd('stop_rec')">2. Stop Rec / Gen Path</button>
    <button class="btn go" onclick="sendCmd('start_mow')">3. START AUTO MOW</button>

    <script>
        var map = L.map('map').setView([0,0], 19);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom: 22}).addTo(map);
        var car = L.marker([0,0]).addTo(map);
        var poly = L.polygon([], {color: 'red'}).addTo(map);
        var path = L.polyline([], {color: 'green'}).addTo(map);

        function update() {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('mode').innerText = d.mode;
                document.getElementById('fix').innerText = d.fix;
                document.getElementById('safe').innerText = d.safety ? "OBSTACLE!" : "OK";
                document.getElementById('safe').style.color = d.safety ? "red" : "green";
                
                if(d.lat) { car.setLatLng([d.lat, d.lon]); if(d.mode!=='MANUAL') map.panTo([d.lat, d.lon]); }
                if(d.perimeter.length) poly.setLatLngs(d.perimeter);
                if(d.path.length) path.setLatLngs(d.path);
            });
        }
        setInterval(update, 500);
        function sendCmd(c) { fetch('/cmd', {method: 'POST', body: JSON.stringify({command: c})}); }
    </script>
</body>
</html>
"""

if __name__ == "__main__":
    try:
        threading.Thread(target=gnss_thread, daemon=True).start()
        threading.Thread(target=camera_thread, daemon=True).start()
        threading.Thread(target=control_thread, daemon=True).start()
        server = HTTPServer(("0.0.0.0", HTTP_PORT), WebHandler)
        print(f"Control UI: http://<PI_IP>:{HTTP_PORT}")
        server.serve_forever()
    except KeyboardInterrupt:
        pi.stop()
