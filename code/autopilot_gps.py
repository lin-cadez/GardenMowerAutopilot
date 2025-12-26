#!/usr/bin/env python3
"""
===========================================================
Autonomous Mower Controller (Raspberry Pi 4B) - GNSS + RC + Web UI
===========================================================

Key rules (as requested):
- RC passthrough is ALWAYS ON, except when autonomous mowing is ACTIVE.
- CH5 LOW  (<1100us) forces manual mode and cancels autonomy immediately.
- CH5 HIGH (>1900us) arms autonomy (allowed), but autonomy only runs when user presses "Start Autonomous".

Workflow:
1) Drive perimeter manually -> record GNSS points -> polygon shown filled red
2) Compute mowing rings using car width & overlap -> step = 0.80m
3) Press Start Autonomous -> mower follows outer ring then inward

NOTE:
- "Polygon shrink" is implemented as a simple centroid-based inward offset in local XY.
  It’s a practical approximation, not a perfect GIS polygon offset.
"""

import json
import math
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse
import serial
import pigpio

# ===================== RC / GPIO =====================
CH1_IN = 17
CH2_IN = 27
CH5_IN = 22
CH1_OUT = 23
CH2_OUT = 24

MIN_PULSE = 900
MAX_PULSE = 2100
NEUTRAL_PULSE = 1500

AUTOPILOT_ON = 1900   # CH5 high = ARMED
AUTOPILOT_OFF = 1100  # CH5 low  = FORCE MANUAL

# ===================== GNSS / WEB =====================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080

# ===================== Mowing geometry =====================
CAR_WIDTH_M = 1.20
OVERLAP_M = 0.40
STEP_M = max(0.10, CAR_WIDTH_M - OVERLAP_M)  # 0.80m

# ===================== Guidance constants =====================
# Very simple heading PID over GNSS (tune for your chassis + update rate)
Kp = 2.0
Ki = 0.02
Kd = 0.15

BASE_SPEED = 1600      # forward "throttle" PWM
TURN_LIMIT = 250       # max +/- correction PWM
WAYPOINT_REACH_M = 0.8 # how close to switch to next waypoint
GNSS_MIN_MOVE_M = 0.3  # reject jitter moves smaller than this

# ===================== Storage =====================
BASE_DIR = Path(__file__).resolve().parent
PATH_DIR = BASE_DIR / "mowing_paths"
PATH_DIR.mkdir(exist_ok=True)

FIX_MAP = {
    "0": "NO FIX",
    "1": "GNSS",
    "2": "DGPS",
    "4": "RTK FIXED",
    "5": "RTK FLOAT",
}

# ===================== Global state =====================
lock = threading.Lock()

latest = {
    "lat": None, "lon": None, "fix": None, "fix_text": None,
    "sats": None, "hdop": None, "alt": None, "ts": None, "raw": None,
}

perimeter = {
    "recording": False,
    "points": [],      # recorded raw points
    "polygon": [],     # closed polygon for display
    "area_m2": 0.0,
    "rings": [],       # list of rings (each ring is closed)
}

# CH5-high means "armed". Autonomy only runs when guidance["active"] is True.
autopilot_armed = False

guidance = {
    "active": False,
    "ring_idx": 0,
    "wp_idx": 0,
}

# Safety stop while autonomous is active:
target_detected = False  # plug your vision detector into this boolean

# RC input capture
_last_tick = {}
_pulse_widths = {CH1_IN: NEUTRAL_PULSE, CH2_IN: NEUTRAL_PULSE, CH5_IN: NEUTRAL_PULSE}

_record_fp = None

# PID state
_pid_integral = 0.0
_pid_last_err = 0.0

# pigpio init
pi = pigpio.pi()
if not pi.connected:
    raise OSError("Cannot connect to pigpio daemon. Run: sudo pigpiod")

pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)

for pin in (CH1_IN, CH2_IN, CH5_IN):
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_DOWN)


# ===================== RC input callbacks =====================
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
    pi.callback(pin, pigpio.EITHER_EDGE, create_callback(pin))


# ===================== GNSS =====================
def nmea_to_deg(v, h):
    if not v:
        return None
    d = float(v)
    deg = int(d // 100)
    m = d - deg * 100
    c = deg + m / 60
    return -c if h in ("S", "W") else c

def gnss_thread():
    global latest
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    print("[INFO] GNSS opened on", SERIAL_PORT)

    while True:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line.startswith("$GNGGA") and not line.startswith("$GPGGA"):
            continue

        p = line.split(",")
        if len(p) < 10:
            continue

        lat = nmea_to_deg(p[2], p[3])
        lon = nmea_to_deg(p[4], p[5])
        if lat is None or lon is None:
            continue

        fix = p[6] if p[6] else "0"

        with lock:
            latest = {
                "lat": lat, "lon": lon,
                "fix": fix,
                "fix_text": FIX_MAP.get(fix, fix),
                "sats": p[7], "hdop": p[8], "alt": p[9],
                "ts": time.time(),
                "raw": line,
            }

            if perimeter["recording"]:
                perimeter["points"].append([lat, lon])
                if _record_fp:
                    _record_fp.write(f"{time.time():.3f},{lat},{lon}\n")


# ===================== Geometry helpers =====================
def haversine_m(a, b):
    # a,b are [lat,lon]
    R = 6371000.0
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    s = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(s))

def simplify_min_dist(points, min_dist_m=0.4):
    if not points:
        return []
    out = [points[0]]
    for pt in points[1:]:
        if haversine_m(out[-1], pt) >= min_dist_m:
            out.append(pt)
    # also drop last if it’s basically same as first
    if len(out) > 2 and haversine_m(out[0], out[-1]) < min_dist_m:
        out.pop()
    return out

def to_xy(pts):
    lat0 = sum(p[0] for p in pts) / len(pts)
    lon0 = sum(p[1] for p in pts) / len(pts)
    R = 6371000.0
    xy = []
    for lat, lon in pts:
        x = R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
        y = R * math.radians(lat - lat0)
        xy.append((x, y))
    return xy, (lat0, lon0)

def from_xy(xy, origin):
    lat0, lon0 = origin
    R = 6371000.0
    out = []
    for x, y in xy:
        lat = lat0 + math.degrees(y / R)
        lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
        out.append([lat, lon])
    return out

def polygon_area_m2(poly):
    if len(poly) < 3:
        return 0.0
    xy, _ = to_xy(poly)
    a = 0.0
    for i in range(len(xy)):
        x1, y1 = xy[i]
        x2, y2 = xy[(i + 1) % len(xy)]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0

def shrink_polygon_centroid(poly, step_m):
    """
    Simple inward offset: move each vertex toward polygon centroid in XY by step_m.
    Not perfect for concave polygons but works as a practical ring generator.
    """
    if len(poly) < 3:
        return []

    xy, org = to_xy(poly)
    cx = sum(x for x, y in xy) / len(xy)
    cy = sum(y for x, y in xy) / len(xy)

    out = []
    for x, y in xy:
        dx = cx - x
        dy = cy - y
        d = math.hypot(dx, dy)
        if d < step_m * 1.3:  # too small to shrink further safely
            return []
        out.append((x + (dx / d) * step_m, y + (dy / d) * step_m))

    return from_xy(out, org)

def close_ring(ring):
    if not ring:
        return ring
    if ring[0] != ring[-1]:
        return ring + [ring[0]]
    return ring


# ===================== Guidance helpers =====================
def bearing_deg(a, b):
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def angle_diff_deg(a, b):
    # minimal signed difference a-b
    return (a - b + 180) % 360 - 180

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ===================== Autopilot / Guidance thread =====================
def guidance_thread():
    global _pid_integral, _pid_last_err

    last_pos = None

    while True:
        time.sleep(0.10)  # 10 Hz GNSS guidance loop

        with lock:
            active = guidance["active"]
            armed = autopilot_armed
            rings = perimeter["rings"]
            lat = latest["lat"]
            lon = latest["lon"]
            safety = target_detected

            ring_idx = guidance["ring_idx"]
            wp_idx = guidance["wp_idx"]

        # Guidance runs ONLY when: active AND armed AND we have rings AND GNSS fix
        if (not active) or (not armed) or (not rings) or (lat is None) or (lon is None):
            _pid_integral = 0.0
            _pid_last_err = 0.0
            last_pos = None
            continue

        if safety:
            # stop only while autonomous is active
            pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
            pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
            continue

        cur = [lat, lon]

        if last_pos is None:
            last_pos = cur
            continue

        moved = haversine_m(last_pos, cur)
        if moved < GNSS_MIN_MOVE_M:
            # too jittery / not moving, don't update heading
            continue

        current_hdg = bearing_deg(last_pos, cur)
        last_pos = cur

        # Ensure indices valid
        ring_idx = min(ring_idx, len(rings) - 1)
        ring = rings[ring_idx]
        if len(ring) < 3:
            continue

        # waypoint list should NOT include last duplicate point for navigation
        nav_pts = ring[:-1] if ring[0] == ring[-1] else ring
        if not nav_pts:
            continue

        wp_idx = min(wp_idx, len(nav_pts) - 1)
        target = nav_pts[wp_idx]

        # Advance waypoint if close
        if haversine_m(cur, target) <= WAYPOINT_REACH_M:
            wp_idx += 1
            if wp_idx >= len(nav_pts):
                # next ring inward
                ring_idx += 1
                wp_idx = 0
                if ring_idx >= len(rings):
                    # finished mowing
                    with lock:
                        guidance["active"] = False
                    print("[AUTO] Finished all rings. Returning to manual passthrough.")
                    continue

            with lock:
                guidance["ring_idx"] = ring_idx
                guidance["wp_idx"] = wp_idx

            ring_idx = min(ring_idx, len(rings) - 1)
            ring = rings[ring_idx]
            nav_pts = ring[:-1] if ring[0] == ring[-1] else ring
            if not nav_pts:
                continue
            target = nav_pts[min(wp_idx, len(nav_pts) - 1)]

        target_hdg = bearing_deg(cur, target)
        err = angle_diff_deg(target_hdg, current_hdg)

        # PID
        _pid_integral = clamp(_pid_integral + err, -60.0, 60.0)
        deriv = err - _pid_last_err
        _pid_last_err = err

        corr = (Kp * err) + (Ki * _pid_integral) + (Kd * deriv)
        corr = clamp(corr, -TURN_LIMIT, TURN_LIMIT)

        left = int(clamp(BASE_SPEED + corr, MIN_PULSE, MAX_PULSE))
        right = int(clamp(BASE_SPEED - corr, MIN_PULSE, MAX_PULSE))

        # Motors are controlled ONLY during active autonomy.
        pi.set_servo_pulsewidth(CH1_OUT, left)
        pi.set_servo_pulsewidth(CH2_OUT, right)


# ===================== RC passthrough + CH5 arming =====================
def rc_thread():
    global autopilot_armed

    print("[INFO] RC thread active (passthrough always ON unless autonomy is active).")

    while True:
        time.sleep(0.02)  # 50 Hz

        ch1 = _pulse_widths.get(CH1_IN, NEUTRAL_PULSE)
        ch2 = _pulse_widths.get(CH2_IN, NEUTRAL_PULSE)
        ch5 = _pulse_widths.get(CH5_IN, NEUTRAL_PULSE)

        with lock:
            # Update armed state from CH5
            if ch5 > AUTOPILOT_ON and not autopilot_armed:
                autopilot_armed = True
                print("[MODE] Autopilot ARMED (CH5 HIGH)")

            if ch5 < AUTOPILOT_OFF and autopilot_armed:
                autopilot_armed = False
                guidance["active"] = False  # force cancel autonomy
                print("[MODE] Autopilot DISARMED (CH5 LOW) -> Manual passthrough")

            autonomy_active = guidance["active"] and autopilot_armed

        # Passthrough ALWAYS unless autonomy is actively running
        if not autonomy_active:
            pi.set_servo_pulsewidth(CH1_OUT, ch1)
            pi.set_servo_pulsewidth(CH2_OUT, ch2)


# ===================== HTTP server =====================
class Handler(BaseHTTPRequestHandler):
    def _json(self, o):
        d = json.dumps(o).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(d)))
        self.end_headers()
        self.wfile.write(d)

    def do_GET(self):
        p = urlparse(self.path)

        if p.path == "/":
            h = (BASE_DIR / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(h)
            return

        if p.path == "/state":
            with lock:
                self._json({
                    "latest": latest,
                    "perimeter": perimeter,
                    "guidance": guidance,
                    "autopilot_armed": autopilot_armed,
                    "target_detected": target_detected,
                    "step_m": STEP_M,
                    "rc_inputs": {
                        "ch1": _pulse_widths.get(CH1_IN),
                        "ch2": _pulse_widths.get(CH2_IN),
                        "ch5": _pulse_widths.get(CH5_IN),
                    }
                })
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        global _record_fp

        p = urlparse(self.path)

        # --- Perimeter recording ---
        if p.path == "/perimeter/start":
            with lock:
                perimeter["recording"] = True
                perimeter["points"] = []
                perimeter["polygon"] = []
                perimeter["rings"] = []
                perimeter["area_m2"] = 0.0

            f = PATH_DIR / f"perimeter_{int(time.time())}.csv"
            _record_fp = open(f, "w")
            _record_fp.write("ts,lat,lon\n")

            print("[PERIM] Recording started ->", f)
            self._json({"ok": True})
            return

        if p.path == "/perimeter/stop":
            with lock:
                perimeter["recording"] = False

            if _record_fp:
                _record_fp.close()
                _record_fp = None

            # Build polygon + rings
            with lock:
                pts = simplify_min_dist(perimeter["points"], min_dist_m=0.6)

            if len(pts) < 3:
                with lock:
                    perimeter["polygon"] = []
                    perimeter["rings"] = []
                    perimeter["area_m2"] = 0.0
                self._json({"ok": False, "error": "Not enough perimeter points."})
                return

            poly = pts[:]  # keep driven perimeter (NOT convex hull!)
            area_m2 = polygon_area_m2(poly)

            rings = []
            cur = poly
            for _ in range(250):
                rings.append(close_ring(cur))
                cur = shrink_polygon_centroid(cur, STEP_M)
                if len(cur) < 3:
                    break

            with lock:
                perimeter["polygon"] = close_ring(poly)
                perimeter["rings"] = rings
                perimeter["area_m2"] = area_m2

            print(f"[PERIM] Recording stopped. Area ≈ {area_m2:.1f} m², rings={len(rings)}")
            self._json({"ok": True, "area_m2": area_m2, "rings": len(rings)})
            return

        # --- Autonomy controls ---
        if p.path == "/autonomy/start":
            with lock:
                if not autopilot_armed:
                    self._json({"ok": False, "error": "CH5 not armed (set CH5 HIGH first)."})
                    return
                if not perimeter["rings"]:
                    self._json({"ok": False, "error": "No rings computed. Record perimeter first."})
                    return
                guidance["active"] = True
                guidance["ring_idx"] = 0
                guidance["wp_idx"] = 0

            print("[AUTO] Autonomous mowing START")
            self._json({"ok": True})
            return

        if p.path == "/autonomy/stop":
            with lock:
                guidance["active"] = False
            print("[AUTO] Autonomous mowing STOP -> Manual passthrough")
            self._json({"ok": True})
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, fmt, *args):
        return


# ===================== MAIN =====================
if __name__ == "__main__":
    try:
        threading.Thread(target=gnss_thread, daemon=True).start()
        threading.Thread(target=guidance_thread, daemon=True).start()
        threading.Thread(target=rc_thread, daemon=True).start()

        print(f"[WEB] http://<PI-IP>:{HTTP_PORT}")
        print("[INFO] CH5 LOW  -> manual (forces autonomy off)")
        print("[INFO] CH5 HIGH -> armed (allows autonomy), still manual until you press Start Autonomous")
        HTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()

    except KeyboardInterrupt:
        print("\n[INFO] Exiting safely...")

    finally:
        pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
        pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
        pi.stop()
        print("[INFO] pigpio stopped.")
