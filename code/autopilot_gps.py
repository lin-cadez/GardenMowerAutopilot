#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===========================================================
Autonomous Mower Controller (Raspberry Pi 4B) - GNSS + RC + Web UI
===========================================================

FIXED CONTROLS (your truth table):
- Forward:  out27=2000, out17=1050
- Neutral:  out27=1500, out17=1500
- Backward: out27=1050, out17=2000
- Left:     out17=2000, out27=2000
- Right:    out17=1050, out27=1050

CH5 (GPIO22) MOMENTARY pulses:
- ~1000us pulse -> autopilot ARMED (allowed)
- ~2000us pulse -> RC passthrough (DISARM + stop autonomy)

RC passthrough:
- ALWAYS ON unless autonomy is ACTIVE (guidance["active"] and autopilot_armed).

RC signal loss (NEW):
- If ANY of CH1/CH2/CH5 pulse width is < 900us => RC is OFF:
    - immediately stop motors (1500/1500)
    - disarm autopilot + stop autonomy
    - pause until signal returns

Note:
- Guidance is simple GNSS waypoint following over rings. Tune PID constants to your chassis.
- Hook your vision/person detection by updating target_detected True/False.
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

# RC axis meaning (CHANGE if your radio channels are swapped):
RC_STEER_GPIO = CH1_IN     # steering stick axis
RC_THROTTLE_GPIO = CH2_IN  # throttle stick axis

MIN_PULSE = 900
MAX_PULSE = 2100
NEUTRAL_PULSE = 1500

# RC OFF threshold (your request)
RC_OFF_US = 900  # if any channel pulse < 900 => RC OFF / signal lost

# ===================== CH5 pulse mode switching =====================
PULSE_LOW_TRIG = 1150
PULSE_HIGH_TRIG = 1850
PULSE_HOLD_MIN_S = 0.05
PULSE_COOLDOWN_S = 0.40

# ===================== GNSS / WEB =====================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080

# ===================== Mowing geometry =====================
CAR_WIDTH_M = 1.20
OVERLAP_M = 0.40
STEP_M = max(0.10, CAR_WIDTH_M - OVERLAP_M)  # 0.80m

# ===================== Guidance constants =====================
Kp = 2.0
Ki = 0.02
Kd = 0.15

BASE_SPEED = 0.70          # [0..1] forward intent while autonomous
TURN_GAIN = 0.90           # [0..1] turn intent scaling

WAYPOINT_REACH_M = 0.9
GNSS_MIN_MOVE_M = 0.3

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
    "points": [],
    "polygon": [],
    "area_m2": 0.0,
    "rings": [],
}

autopilot_armed = False
guidance = {"active": False, "ring_idx": 0, "wp_idx": 0}

# Safety stop only during autonomy
target_detected = False  # set from your vision thread

# RC signal state (NEW)
rc_signal_ok = False  # True when RC pulses are valid

# RC input capture
_last_tick = {}
_pulse_widths = {CH1_IN: 0, CH2_IN: 0, CH5_IN: 0}  # start "no signal"

_record_fp = None

# PID state
_pid_integral = 0.0
_pid_last_err = 0.0

# ===================== pigpio init =====================
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
            # Store even small values (so we can detect <900 => RC OFF)
            if 0 < pulse_len < 3000:
                _pulse_widths[gpio] = pulse_len
    return cb

for pin in (CH1_IN, CH2_IN, CH5_IN):
    pi.callback(pin, pigpio.EITHER_EDGE, create_callback(pin))

# ===================== Control mapping (YOUR truth table) =====================
DEADBAND = 60
MIN_US = 1050
MAX_US = 2000
MID_US = 1500

def norm_axis(us: int) -> float:
    """PWM us -> [-1..+1], with deadband around 1500."""
    us = int(us)
    if abs(us - MID_US) <= DEADBAND:
        return 0.0
    if us > MID_US:
        return min(1.0, (us - MID_US) / (MAX_US - MID_US))
    return max(-1.0, (us - MID_US) / (MID_US - MIN_US))

def mix_to_outputs(throttle: float, turn: float) -> tuple[int, int]:
    # choose command based on which has larger magnitude, but no cross-conditions
    if abs(turn) >= abs(throttle) and abs(turn) > 0.15:
        return (MAX_US, MAX_US) if turn > 0 else (MIN_US, MIN_US)  # left / right
    if abs(throttle) > 0.15:
        return (MIN_US, MAX_US) if throttle > 0 else (MAX_US, MIN_US)  # fwd / back
    return (MID_US, MID_US)


def motor_write(out17: int, out27: int) -> None:
    """
    Motor outputs on GPIO23/GPIO24.
    If your physical motor channels are swapped, swap these two lines.
    """
    pi.set_servo_pulsewidth(CH1_OUT, int(out17))
    pi.set_servo_pulsewidth(CH2_OUT, int(out27))

def write_motors_from_rc():
    steer_us = _pulse_widths.get(RC_STEER_GPIO, MID_US)
    thr_us = _pulse_widths.get(RC_THROTTLE_GPIO, MID_US)
    steer = norm_axis(steer_us)  # +left, -right
    thr = norm_axis(thr_us)      # +forward, -back
    out17, out27 = mix_to_outputs(thr, steer)
    motor_write(out17, out27)

# ===================== RC signal check (NEW) =====================
def rc_is_on() -> bool:
    ch1 = _pulse_widths.get(CH1_IN, 0)
    ch2 = _pulse_widths.get(CH2_IN, 0)
    ch5 = _pulse_widths.get(CH5_IN, 0)
    return (ch1 >= RC_OFF_US) and (ch2 >= RC_OFF_US) and (ch5 >= RC_OFF_US)

def rc_watchdog_thread():
    """
    If RC is OFF (any channel < 900us):
    - stop motors
    - disarm autopilot + stop autonomy
    - pause until RC returns
    """
    global autopilot_armed, rc_signal_ok

    last_ok = None
    while True:
        time.sleep(0.05)

        ok = rc_is_on()
        if ok:
            if last_ok is False:
                print("[RC] Signal restored.")
            rc_signal_ok = True
            last_ok = True
            continue

        # RC OFF
        if last_ok is not False:
            print("[RC] Signal LOST (pulse < 900us). Stopping motors and pausing autonomy.")
        last_ok = False
        rc_signal_ok = False

        with lock:
            autopilot_armed = False
            guidance["active"] = False

        motor_write(MID_US, MID_US)

        # Pause until RC returns
        while not rc_is_on():
            time.sleep(0.1)

        print("[RC] Signal restored. Manual control resumed.")
        rc_signal_ok = True
        last_ok = True

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
    R = 6371000.0
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    s = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(math.sqrt(s))

def simplify_min_dist(points, min_dist_m=0.6):
    if not points:
        return []
    out = [points[0]]
    for pt in points[1:]:
        if haversine_m(out[-1], pt) >= min_dist_m:
            out.append(pt)
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
        if d < step_m * 1.3:
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
    return (a - b + 180) % 360 - 180

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ===================== Autopilot / Guidance thread =====================
def guidance_thread():
    global _pid_integral, _pid_last_err

    last_pos = None

    while True:
        time.sleep(0.10)

        with lock:
            active = guidance["active"]
            armed = autopilot_armed
            ok = rc_signal_ok
            rings = perimeter["rings"]
            lat = latest["lat"]
            lon = latest["lon"]
            safety = target_detected
            ring_idx = guidance["ring_idx"]
            wp_idx = guidance["wp_idx"]

        if (not ok) or (not active) or (not armed) or (not rings) or (lat is None) or (lon is None):
            _pid_integral = 0.0
            _pid_last_err = 0.0
            last_pos = None
            continue

        if safety:
            motor_write(MID_US, MID_US)
            continue

        cur = [lat, lon]

        if last_pos is None:
            last_pos = cur
            continue

        moved = haversine_m(last_pos, cur)
        if moved < GNSS_MIN_MOVE_M:
            continue

        current_hdg = bearing_deg(last_pos, cur)
        last_pos = cur

        ring_idx = min(ring_idx, len(rings) - 1)
        ring = rings[ring_idx]
        if len(ring) < 3:
            continue

        nav_pts = ring[:-1] if ring[0] == ring[-1] else ring
        if not nav_pts:
            continue

        wp_idx = min(wp_idx, len(nav_pts) - 1)
        target = nav_pts[wp_idx]

        if haversine_m(cur, target) <= WAYPOINT_REACH_M:
            wp_idx += 1
            if wp_idx >= len(nav_pts):
                ring_idx += 1
                wp_idx = 0
                if ring_idx >= len(rings):
                    with lock:
                        guidance["active"] = False
                    print("[AUTO] Finished all rings. Returning to RC passthrough.")
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

        _pid_integral = clamp(_pid_integral + err, -60.0, 60.0)
        deriv = err - _pid_last_err
        _pid_last_err = err

        corr = (Kp * err) + (Ki * _pid_integral) + (Kd * deriv)

        turn_intent = clamp((corr / 90.0) * TURN_GAIN, -1.0, 1.0)
        throttle_intent = clamp(BASE_SPEED, 0.0, 1.0)

        out17, out27 = mix_to_outputs(throttle_intent, turn_intent)
        motor_write(out17, out27)

# ===================== CH5 pulse thread =====================
def ch5_pulse_thread():
    global autopilot_armed

    last_state = None
    stable_since = time.time()
    last_action = 0.0

    while True:
        time.sleep(0.01)

        with lock:
            ok = rc_signal_ok
        if not ok:
            continue

        ch5 = _pulse_widths.get(CH5_IN, MID_US)

        if ch5 <= PULSE_LOW_TRIG:
            state = "LOWPULSE"
        elif ch5 >= PULSE_HIGH_TRIG:
            state = "HIGHPULSE"
        else:
            state = "MID"

        now = time.time()
        if state != last_state:
            last_state = state
            stable_since = now
            continue

        if (now - stable_since) < PULSE_HOLD_MIN_S:
            continue
        if (now - last_action) < PULSE_COOLDOWN_S:
            continue

        if state == "LOWPULSE":
            with lock:
                autopilot_armed = True
            print("[MODE] Autopilot ARMED (CH5 pulse ~1000us)")
            last_action = now

        elif state == "HIGHPULSE":
            with lock:
                autopilot_armed = False
                guidance["active"] = False
            print("[MODE] RC passthrough (CH5 pulse ~2000us) -> autonomy stopped")
            last_action = now

# ===================== RC passthrough thread =====================
def rc_thread():
    print("[INFO] RC passthrough active (mapped). Autonomy overrides only when ACTIVE + ARMED.")
    while True:
        time.sleep(0.02)

        with lock:
            ok = rc_signal_ok
            autonomy_active = guidance["active"] and autopilot_armed

        if not ok:
            continue

        if autonomy_active:
            continue

        write_motors_from_rc()

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
                    "rc_signal_ok": rc_signal_ok,
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

            with lock:
                pts = simplify_min_dist(perimeter["points"], min_dist_m=0.6)

            if len(pts) < 3:
                with lock:
                    perimeter["polygon"] = []
                    perimeter["rings"] = []
                    perimeter["area_m2"] = 0.0
                self._json({"ok": False, "error": "Not enough perimeter points."})
                return

            poly = pts[:]
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

            print(f"[PERIM] Recording stopped. Area ~= {area_m2:.1f} m^2, rings={len(rings)}")
            self._json({"ok": True, "area_m2": area_m2, "rings": len(rings)})
            return

        if p.path == "/autonomy/start":
            with lock:
                if not rc_signal_ok:
                    self._json({"ok": False, "error": "RC signal is OFF (pulse < 900us)."})
                    return
                if not autopilot_armed:
                    self._json({"ok": False, "error": "Autopilot not armed. Send CH5 pulse ~1000us."})
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
            print("[AUTO] Autonomous mowing STOP -> RC passthrough")
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
        threading.Thread(target=rc_watchdog_thread, daemon=True).start()
        threading.Thread(target=ch5_pulse_thread, daemon=True).start()
        threading.Thread(target=rc_thread, daemon=True).start()

        print(f"[WEB] http://<PI-IP>:{HTTP_PORT}")
        print("[INFO] CH5 pulse ~1000us -> autopilot ARMED")
        print("[INFO] CH5 pulse ~2000us -> RC passthrough (disarm + stop autonomy)")
        print("[INFO] RC OFF if any channel pulse < 900us (program pauses motors until return)")
        HTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()

    except KeyboardInterrupt:
        print("\n[INFO] Exiting safely...")

    finally:
        motor_write(MID_US, MID_US)
        pi.stop()
        print("[INFO] pigpio stopped.")
