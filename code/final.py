#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===========================================================
FINAL.PY — Mower Safety + DGPS Map + Live Stream Dashboard
===========================================================

WHAT IT DOES (all-in-one):
1) RC passthrough + safety stop (camera)
   - Mirrors RC CH1/CH2 to outputs CH1_OUT/CH2_OUT always
   - AUTOPILOT ON when CH5 > 1900us, OFF when CH5 < 1100us (hysteresis)
   - If AUTOPILOT ON and target detected:
       - STOP immediately (1500/1500)
       - remain stopped until target is clear continuously for 1 second

2) GNSS + DGPS (NTRIP) -> /pos endpoint + map
   - Reads GGA from /dev/serial0
   - Sends latest GGA to NTRIP caster (RTK2go)
   - Receives RTCM and injects back into GNSS UART
   - Serves dashboard + /pos JSON on http://<PI-IP>:8080/

3) MJPEG live stream
   - Serves camera stream on http://<PI-IP>:8090/stream

REQUIREMENTS:
sudo apt install python3-opencv python3-picamera2 python3-pigpio python3-serial
sudo pigpiod
Place MobileNetSSD_deploy.prototxt and MobileNetSSD_deploy.caffemodel in same folder.
"""

import base64
import json
import socket
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import serial
import cv2
from picamera2 import Picamera2
import pigpio

# =======================
# GNSS / DGPS (NTRIP)
# =======================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200

NTRIP_HOST = "rtk2go.com"
NTRIP_PORT = 2101
MOUNTPOINT = "FRELIH"          # case-sensitive

USERNAME = "test@email.com"    # RTK2go ignores password, but wants "basic auth"
PASSWORD = "none"
GGA_SEND_HZ = 1.0

HTTP_PORT = 8080

FIX_MAP = {
    "0": "NO FIX",
    "1": "GNSS",
    "2": "DGPS",
    "4": "RTK FIXED",
    "5": "RTK FLOAT",
}

state_lock = threading.Lock()
latest = {
    "lat": None, "lon": None,
    "fix": None, "fix_text": None,
    "sats": None, "hdop": None,
    "alt": None, "ts": None, "raw": None
}
latest_gga_line = None  # used by NTRIP sender

# =======================
# RC / Safety / Camera
# =======================
CONF_THRESH = 0.5
PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"

TARGETS = {"person", "cat", "dog"}
CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

CH1_IN = 17
CH2_IN = 27
CH5_IN = 22
CH1_OUT = 23
CH2_OUT = 24

MIN_PULSE = 900
MAX_PULSE = 2100
NEUTRAL_PULSE = 1500

AUTOPILOT_ON = 1900
AUTOPILOT_OFF = 1100

CLEAR_DELAY_S = 1.0

pi = pigpio.pi()
if not pi.connected:
    raise OSError("Cannot connect to pigpio daemon. Run: sudo pigpiod")

pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)

target_detected = False
autopilot_mode = False
current_frame = None

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


# ================= Utilities =================
def nmea_to_deg(v, h):
    if not v:
        return None
    try:
        d = float(v)
        deg = int(d // 100)
        m = d - deg * 100
        val = deg + m / 60.0
        return -val if h in ("S", "W") else val
    except ValueError:
        return None


# ================= GNSS Reader (single reader for serial) =================
def gnss_reader_thread(ser):
    global latest, latest_gga_line
    print("[INFO] GNSS reader started")

    while True:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
            continue

        # Save latest GGA for NTRIP sender
        with state_lock:
            latest_gga_line = line

        p = line.split(",")
        if len(p) < 10:
            continue

        lat = nmea_to_deg(p[2], p[3])
        lon = nmea_to_deg(p[4], p[5])
        if lat is None or lon is None:
            continue

        fix = p[6] or "0"
        sats = p[7]
        hdop = p[8]
        alt = p[9]
        now = time.time()

        with state_lock:
            latest = {
                "lat": lat, "lon": lon,
                "fix": fix,
                "fix_text": FIX_MAP.get(fix, fix),
                "sats": sats, "hdop": hdop,
                "alt": alt, "ts": now, "raw": line
            }


# ================= NTRIP / DGPS =================
def open_ntrip():
    sock = socket.create_connection((NTRIP_HOST, NTRIP_PORT), timeout=10)
    sock.settimeout(10)

    auth = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()
    req = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"User-Agent: Python-NTRIP\r\n"
        f"Authorization: Basic {auth}\r\n\r\n"
    )
    sock.sendall(req.encode("ascii", errors="ignore"))
    resp = sock.recv(2048).decode("ascii", errors="ignore")

    if "SOURCETABLE" in resp:
        raise RuntimeError("Got SOURCETABLE. Check mountpoint (case-sensitive).")
    if ("200 OK" not in resp) and ("ICY 200 OK" not in resp):
        raise RuntimeError("NTRIP connection failed:\n" + resp)

    print("[INFO] NTRIP connected:", MOUNTPOINT)
    return sock


def gga_sender_thread(sock, stop_flag):
    last_sent = 0.0
    while not stop_flag["stop"]:
        time.sleep(0.05)
        with state_lock:
            gga = latest_gga_line
        if not gga:
            continue

        now = time.time()
        if (now - last_sent) >= (1.0 / GGA_SEND_HZ):
            try:
                sock.sendall((gga + "\r\n").encode("ascii", errors="ignore"))
                last_sent = now
            except (BrokenPipeError, ConnectionResetError, OSError):
                stop_flag["stop"] = True


def rtcm_receiver_thread(ser, sock, stop_flag):
    while not stop_flag["stop"]:
        try:
            data = sock.recv(4096)
            if not data:
                stop_flag["stop"] = True
                break
            ser.write(data)
        except socket.timeout:
            continue
        except (BrokenPipeError, ConnectionResetError, OSError):
            stop_flag["stop"] = True


def ntrip_manager_thread(ser):
    while True:
        try:
            sock = open_ntrip()
            stop_flag = {"stop": False}

            threading.Thread(target=gga_sender_thread, args=(sock, stop_flag), daemon=True).start()
            threading.Thread(target=rtcm_receiver_thread, args=(ser, sock, stop_flag), daemon=True).start()

            while not stop_flag["stop"]:
                time.sleep(0.2)

            try:
                sock.close()
            except Exception:
                pass

            print("[WARN] NTRIP disconnected. Reconnecting in 2s...")
            time.sleep(2)

        except Exception as e:
            print("[ERROR] NTRIP:", e)
            time.sleep(5)


# ================= Dashboard (Map + Stream) =================
INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>Mower Dashboard — Live Stream + DGPS Map</title>
  <meta name="viewport" content="width=device-width, initial-scale=1" />

  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

  <style>
    :root{
      --bg:#0b1020; --panel:#121a33; --text:#e7ecff; --muted:#a9b2da;
      --ok:#35d07f; --warn:#ffcc00; --bad:#ff5a6b; --border:rgba(255,255,255,.08);
      --shadow: 0 10px 30px rgba(0,0,0,.35); --radius: 16px; --gap: 14px;
      --font: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, "Noto Sans";
    }
    *{box-sizing:border-box}
    html,body{height:100%}
    body{
      margin:0; font-family:var(--font); color:var(--text);
      background: radial-gradient(1200px 600px at 20% 0%, rgba(87,117,255,.18), transparent 60%),
                  radial-gradient(1000px 500px at 100% 30%, rgba(53,208,127,.12), transparent 55%),
                  var(--bg);
    }
    header{
      position:sticky; top:0; z-index:50;
      backdrop-filter: blur(10px);
      background: linear-gradient(to bottom, rgba(11,16,32,.85), rgba(11,16,32,.45));
      border-bottom: 1px solid var(--border);
    }
    .wrap{max-width:1200px;margin:0 auto;padding:14px 16px;}
    .topbar{display:flex;align-items:center;justify-content:space-between;gap:12px;}
    .title{display:flex;align-items:center;gap:10px;font-weight:700;letter-spacing:.2px;}
    .badge{
      font-size:12px;padding:4px 10px;border-radius:999px;
      background:rgba(255,255,255,.06);border:1px solid var(--border);color:var(--muted);
    }
    .statusRow{display:flex;flex-wrap:wrap;gap:10px;margin-top:10px;}
    .chip{
      display:flex;align-items:center;gap:8px;padding:8px 10px;
      background:rgba(255,255,255,.05);border:1px solid var(--border);
      border-radius:999px;font-size:13px;color:var(--muted);
    }
    .dot{width:10px;height:10px;border-radius:50%;background:var(--warn);
         box-shadow:0 0 0 4px rgba(255,204,0,.12);}
    .dot.ok{background:var(--ok);box-shadow:0 0 0 4px rgba(53,208,127,.12);}
    .dot.bad{background:var(--bad);box-shadow:0 0 0 4px rgba(255,90,107,.12);}
    main .wrap{padding-top:16px;padding-bottom:22px;}
    .grid{display:grid;grid-template-columns:1.15fr 0.85fr;gap:var(--gap);align-items:stretch;}
    @media (max-width:920px){.grid{grid-template-columns:1fr;}}
    .card{
      background:linear-gradient(180deg, rgba(255,255,255,.06), rgba(255,255,255,.035));
      border:1px solid var(--border);border-radius:var(--radius);
      box-shadow:var(--shadow);overflow:hidden;position:relative;min-height:340px;
    }
    .cardHeader{
      display:flex;align-items:center;justify-content:space-between;
      padding:12px 14px;border-bottom:1px solid var(--border);background:rgba(0,0,0,.15);
    }
    .cardHeader h2{margin:0;font-size:14px;letter-spacing:.3px;font-weight:700;display:flex;gap:10px;align-items:center;}
    .pill{
      font-size:12px;padding:4px 10px;border-radius:999px;
      background:rgba(255,255,255,.06);border:1px solid var(--border);color:var(--muted);
    }
    .streamWrap{position:relative;width:100%;height:calc(100% - 49px);background:#000;}
    .streamImg{position:absolute;inset:0;width:100%;height:100%;object-fit:cover;background:#000;}
    .streamOverlay{position:absolute;inset:auto 12px 12px 12px;display:flex;gap:10px;flex-wrap:wrap;pointer-events:none;}
    .overlayTag{background:rgba(0,0,0,.55);border:1px solid rgba(255,255,255,.12);color:#fff;
                padding:6px 10px;border-radius:999px;font-size:12px;letter-spacing:.2px;}
    #map{width:100%;height:calc(100% - 49px);min-height:360px;}
  </style>
</head>
<body>
<header>
  <div class="wrap">
    <div class="topbar">
      <div class="title"><span style="font-size:18px">🧭</span><span>Mower Dashboard</span><span class="badge">Live Stream + DGPS Map</span></div>
      <div class="badge" id="clock">--:--:--</div>
    </div>
    <div class="statusRow">
      <div class="chip"><span class="dot" id="fixDot"></span><span><b id="fixText">Fix:</b> <span id="fixVal">--</span></span></div>
      <div class="chip"><span>🛰️ <b>Sats:</b> <span id="satsVal">--</span></span></div>
      <div class="chip"><span>📉 <b>HDOP:</b> <span id="hdopVal">--</span></span></div>
      <div class="chip"><span>⛰️ <b>Alt:</b> <span id="altVal">--</span></span></div>
      <div class="chip"><span>⏱️ <b>Age:</b> <span id="ageVal">--</span>s</span></div>
    </div>
  </div>
</header>

<main>
  <div class="wrap">
    <div class="grid">
      <section class="card">
        <div class="cardHeader"><h2>📷 Live Camera</h2><span class="pill" id="streamPill">Stream: connecting…</span></div>
        <div class="streamWrap">
          <img id="streamImg" class="streamImg" alt="Live stream" src="" />
          <div class="streamOverlay">
            <div class="overlayTag">http://<span id="hostA">PI</span>:8090/stream</div>
            <div class="overlayTag" id="streamFpsTag">FPS: --</div>
          </div>
        </div>
      </section>

      <section class="card">
        <div class="cardHeader"><h2>🗺️ DGPS Position</h2><span class="pill" id="posPill">/pos: waiting…</span></div>
        <div id="map"></div>
      </section>
    </div>
  </div>
</main>

<script>
  function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

  function fixDotByText(fixText){
    const dot = document.getElementById('fixDot');
    dot.classList.remove('ok','bad');
    if(!fixText) return;
    const t = String(fixText).toUpperCase();
    if(t.includes('NO FIX')) dot.classList.add('bad');
    else if(t.includes('RTK')) dot.classList.add('ok');
  }

  setInterval(() => {
    const d = new Date();
    const hh = String(d.getHours()).padStart(2,'0');
    const mm = String(d.getMinutes()).padStart(2,'0');
    const ss = String(d.getSeconds()).padStart(2,'0');
    document.getElementById('clock').textContent = `${hh}:${mm}:${ss}`;
  }, 250);

  const host = window.location.hostname || 'localhost';
  document.getElementById('hostA').textContent = host;

  const streamUrl = `http://${host}:8090/stream`;
  const streamImg = document.getElementById('streamImg');
  const streamPill = document.getElementById('streamPill');

  streamImg.src = streamUrl;
  streamImg.onload = () => streamPill.textContent = 'Stream: OK';
  streamImg.onerror = () => streamPill.textContent = 'Stream: ERROR';

  let frameCount = 0;
  let lastFpsTs = performance.now();
  streamImg.addEventListener('load', () => {
    frameCount++;
    const now = performance.now();
    const dt = (now - lastFpsTs) / 1000;
    if(dt >= 1.0){
      const fps = frameCount / dt;
      document.getElementById('streamFpsTag').textContent = `FPS: ${fps.toFixed(1)}`;
      frameCount = 0;
      lastFpsTs = now;
    }
  });

  const map = L.map('map', { zoomControl: true }).setView([46.0569, 14.5058], 16);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 20, attribution: '&copy; OpenStreetMap contributors' }).addTo(map);
  const marker = L.marker([46.0569, 14.5058]).addTo(map);
  const trail = L.polyline([], { weight: 3, opacity: 0.7 }).addTo(map);
  let firstFix = true;

  const posPill = document.getElementById('posPill');
  const fixVal  = document.getElementById('fixVal');
  const satsVal = document.getElementById('satsVal');
  const hdopVal = document.getElementById('hdopVal');
  const altVal  = document.getElementById('altVal');
  const ageVal  = document.getElementById('ageVal');

  async function pollPos(){
    try{
      const r = await fetch('/pos', { cache:'no-store' });
      if(!r.ok) throw new Error('HTTP ' + r.status);
      const d = await r.json();

      if(d.lat == null || d.lon == null){
        posPill.textContent = '/pos: no data';
        return;
      }

      posPill.textContent = '/pos: OK';
      fixVal.textContent  = d.fix_text ?? d.fix ?? '--';
      satsVal.textContent = d.sats ?? '--';
      hdopVal.textContent = d.hdop ?? '--';
      altVal.textContent  = d.alt ?? '--';

      const age = (d.ts != null) ? (Date.now()/1000 - d.ts) : null;
      ageVal.textContent = (age != null) ? clamp(age, 0, 999).toFixed(1) : '--';
      fixDotByText(d.fix_text);

      const lat = d.lat, lon = d.lon;
      marker.setLatLng([lat, lon]);

      const pts = trail.getLatLngs();
      const last = pts.length ? pts[pts.length - 1] : null;
      if(!last || Math.hypot(last.lat - lat, last.lng - lon) > 0.00001){
        pts.push(L.latLng(lat, lon));
        if(pts.length > 250) pts.splice(0, pts.length - 250);
        trail.setLatLngs(pts);
      }

      if(firstFix){
        map.setView([lat, lon], 18);
        firstFix = false;
      }

    }catch(e){
      posPill.textContent = '/pos: ERROR';
    }
  }

  setInterval(pollPos, 500);
  pollPos();
</script>
</body>
</html>
""".encode("utf-8")


class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
            return

        if self.path == "/pos":
            with state_lock:
                data = json.dumps(latest).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, *args):
        return


# ================= Camera / Vision =================
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


# ================= MJPEG stream (8090) =================
def mjpeg_stream_thread():
    class StreamHandler(BaseHTTPRequestHandler):
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

        def log_message(self, *args):
            return

    server = HTTPServer(("0.0.0.0", 8090), StreamHandler)
    print("[WEB] Stream: http://<PI-IP>:8090/stream")
    server.serve_forever()


# ================= Control loop (RC passthrough + safety) =================
def control_loop():
    global autopilot_mode, safety_latched, target_clear_since

    print("[INFO] Control loop active (RC passthrough + safety).")
    last_mode = None
    last_latched = None

    while True:
        ch1 = _pulse_widths.get(CH1_IN, NEUTRAL_PULSE)
        ch2 = _pulse_widths.get(CH2_IN, NEUTRAL_PULSE)
        ch5 = _pulse_widths.get(CH5_IN, NEUTRAL_PULSE)

        if ch5 > AUTOPILOT_ON:
            autopilot_mode = True
        elif ch5 < AUTOPILOT_OFF:
            autopilot_mode = False

        if autopilot_mode != last_mode:
            print("[MODE] Autopilot ON" if autopilot_mode else "[MODE] Autopilot OFF")
            last_mode = autopilot_mode

        now = time.time()

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
            safety_latched = False
            target_clear_since = None

        if safety_latched != last_latched:
            print("[SAFETY] STOP LATCHED (waiting for clear)" if safety_latched else "[SAFETY] Cleared -> RC enabled")
            last_latched = safety_latched

        if autopilot_mode and (target_detected or safety_latched):
            pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
            pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
        else:
            pi.set_servo_pulsewidth(CH1_OUT, ch1)
            pi.set_servo_pulsewidth(CH2_OUT, ch2)

        time.sleep(0.02)


# ================= MAIN =================
def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    print("[INFO] GNSS serial open:", SERIAL_PORT, SERIAL_BAUD)

    # GNSS + NTRIP
    threading.Thread(target=gnss_reader_thread, args=(ser,), daemon=True).start()
    threading.Thread(target=ntrip_manager_thread, args=(ser,), daemon=True).start()

    # Camera + stream
    threading.Thread(target=camera_thread, daemon=True).start()
    threading.Thread(target=mjpeg_stream_thread, daemon=True).start()

    # Dashboard web server (8080)
    threading.Thread(
        target=lambda: HTTPServer(("0.0.0.0", HTTP_PORT), DashboardHandler).serve_forever(),
        daemon=True
    ).start()
    print(f"[WEB] Dashboard: http://<PI-IP>:{HTTP_PORT}/")

    # RC control loop blocks
    control_loop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting safely...")
    finally:
        pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
        pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
        pi.stop()
        print("[INFO] pigpio stopped. Goodbye!")
