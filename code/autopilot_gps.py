
#!/usr/bin/env python3
import json
import math
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs
import serial
import pigpio

# --- GPIO Pin Assignments ---
CH1_OUT = 23
CH2_OUT = 24

# --- PWM Constants ---
MIN_PULSE = 900
MAX_PULSE = 2100
NEUTRAL_PULSE = 1500

# Initialize Hardware Connection
pi = pigpio.pi()
if not pi.connected:
    print("[ERROR] Could not connect to pigpio. Did you run 'sudo pigpiod'?")
else:
    # Set initial neutral state for safety
    pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
    pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)

# ================= CONFIG =================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080

autopilot_mode = False
target_detected = False

CAR_WIDTH_M = 1.20
OVERLAP_M   = 0.40
STEP_M      = CAR_WIDTH_M - OVERLAP_M  # 0.80 m

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
# ==========================================

lock = threading.Lock()

latest = {
    "lat": None, "lon": None, "fix": None, "fix_text": None,
    "sats": None, "hdop": None, "alt": None, "ts": None, "raw": None,
}

perimeter = {
    "recording": False,
    "filename": None,
    "points": [],
    "polygon": [],
    "area_m2": 0.0,
    "rings": [],
}

guidance = {
    "active": False,
    "ring": 0,
}

_record_fp = None
_last_pos = None
_last_msg = None

# ================= GNSS ===================
def nmea_to_deg(v, h):
    if not v:
        return None
    d = float(v)
    deg = int(d // 100)
    m = d - deg * 100
    c = deg + m / 60
    return -c if h in ("S","W") else c

def gnss_thread():
    global latest
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    print("[INFO] GNSS opened")

    while True:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line.startswith("$GNGGA"):
            continue
        p = line.split(",")
        if len(p) < 10:
            continue

        lat = nmea_to_deg(p[2], p[3])
        lon = nmea_to_deg(p[4], p[5])
        if lat is None or lon is None:
            continue

        fix = p[6]
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

# ================ GEOMETRY =================
def to_xy(pts):
    lat0 = sum(p[0] for p in pts)/len(pts)
    lon0 = sum(p[1] for p in pts)/len(pts)
    R = 6371000
    xy=[]
    for lat,lon in pts:
        x = R*math.radians(lon-lon0)*math.cos(math.radians(lat0))
        y = R*math.radians(lat-lat0)
        xy.append((x,y))
    return xy,(lat0,lon0)

def from_xy(xy, origin):
    lat0,lon0 = origin
    R=6371000
    out=[]
    for x,y in xy:
        lat = lat0 + math.degrees(y/R)
        lon = lon0 + math.degrees(x/(R*math.cos(math.radians(lat0))))
        out.append([lat,lon])
    return out

def convex_hull(pts):
    pts=sorted(set(tuple(p) for p in pts))
    if len(pts)<=2: return [list(p) for p in pts]
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[]
    for p in pts:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0: lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(pts):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0: upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1]+upper[:-1]]

def area(poly):
    if len(poly)<3: return 0
    xy,_=to_xy(poly)
    a=0
    for i in range(len(xy)):
        x1,y1=xy[i]
        x2,y2=xy[(i+1)%len(xy)]
        a+=x1*y2-x2*y1
    return abs(a)/2

def shrink(poly, step):
    xy,org=to_xy(poly)
    cx=sum(x for x,y in xy)/len(xy)
    cy=sum(y for x,y in xy)/len(xy)
    out=[]
    for x,y in xy:
        dx,dy=cx-x,cy-y
        d=math.hypot(dx,dy)
        if d<step*1.3: return []
        out.append((x+dx/d*step,y+dy/d*step))
    return from_xy(out,org)

# ================ GUIDANCE =================
def bearing(a,b):
    lat1,lon1=map(math.radians,a)
    lat2,lon2=map(math.radians,b)
    dlon=lon2-lon1
    y=math.sin(dlon)*math.cos(lat2)
    x=math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    return (math.degrees(math.atan2(y,x))+360)%360

def angle_diff(a,b): return (a-b+180)%360-180

# --- PID Constants (Tune these for your mower) ---
Kp = 1.5   # Proportional: How hard to turn based on current error
Ki = 0.01  # Integral: Corrects long-term drift
Kd = 0.1   # Derivative: Prevents overshooting/oscillations

# PID State Variables
integral = 0
last_error = 0
base_speed = 1600  # Base forward speed when on track
def guidance_thread():
    global _last_pos, _last_msg, integral, last_error, autopilot_mode
    
    while True:
        time.sleep(0.1)
        
        with lock:
            # FIX: Check if we actually have data and are in autopilot
            if not guidance["active"] or not autopilot_mode or not perimeter["rings"] or latest["lat"] is None:
                integral = 0
                last_error = 0
                continue
            
            cur = [latest["lat"], latest["lon"]]
            if _last_pos is None:
                _last_pos = cur; continue
            
            # Distance check to prevent GPS jitter from ruining heading
            dist = math.hypot(cur[0]-_last_pos[0], cur[1]-_last_pos[1])
            if dist < 0.000005: # Roughly 0.5 meters
                continue

            current_hdg = bearing(_last_pos, cur)
            _last_pos = cur
            
            # IMPROVEMENT: Target the specific segment of the ring based on progress
            # For now, we use a simplified index, but you may need a 'target_point_index'
            ring = perimeter["rings"][min(guidance["ring"], len(perimeter["rings"])-1)]
            
            # Simple look-ahead: Target the next point in the ring
            target_hdg = bearing(cur, ring[1]) 
            
            error = angle_diff(target_hdg, current_hdg)
            
            # PID Math
            integral = max(min(integral + error, 50), -50)
            derivative = error - last_error
            last_error = error
            
            steering_correction = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # Apply to motors ONLY if safety allows
            if not target_detected:
                left_motor = max(min(base_speed + steering_correction, MAX_PULSE), MIN_PULSE)
                right_motor = max(min(base_speed - steering_correction, MAX_PULSE), MIN_PULSE)
                pi.set_servo_pulsewidth(CH1_OUT, int(left_motor))
                pi.set_servo_pulsewidth(CH2_OUT, int(right_motor))
            else:
                # E-STOP
                pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
                pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
                
                
# ================ HTTP =====================
class Handler(BaseHTTPRequestHandler):
    def _json(self,o):
        d=json.dumps(o).encode()
        self.send_response(200)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length",str(len(d)))
        self.end_headers(); self.wfile.write(d)

    def do_GET(self):
        p=urlparse(self.path)
        if p.path=="/":
            h=(BASE_DIR/"index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type","text/html")
            self.end_headers(); self.wfile.write(h)
            return
        if p.path=="/state":
            with lock:
                self._json({"latest":latest,"perimeter":perimeter,"guidance":guidance})
            return
        self.send_response(404); self.end_headers()

    def do_POST(self):
        global _record_fp
        p=urlparse(self.path)
        if p.path=="/perimeter/start":
            with lock:
                perimeter["recording"]=True
                perimeter["points"]=[]; perimeter["rings"]=[]; perimeter["polygon"]=[]
            f=PATH_DIR/f"perimeter_{int(time.time())}.txt"
            _record_fp=open(f,"w")
            self._json({"ok":True}); return
        if p.path == "/guidance/manual_override":
            global autopilot_mode
            with lock:
                guidance["active"] = False
                autopilot_mode = False
                # Force hardware back to neutral/RC passthrough immediately
                pi.set_servo_pulsewidth(CH1_OUT, NEUTRAL_PULSE)
                pi.set_servo_pulsewidth(CH2_OUT, NEUTRAL_PULSE)
            self._json({"ok": True, "status": "Manual Control Engaged"})
            return
            
            
        if p.path=="/perimeter/stop":
            with lock:
                perimeter["recording"]=False
            if _record_fp:
                _record_fp.close()
            with lock:
                hull=convex_hull(perimeter["points"])
                perimeter["polygon"]=hull+[hull[0]]
                perimeter["area_m2"]=area(hull)
                rings=[]
                cur=hull
                for _ in range(100):
                    rings.append(cur+[cur[0]])
                    cur=shrink(cur,STEP_M)
                    if len(cur)<3: break
                perimeter["rings"]=rings
            self._json({"ok":True}); return

        if p.path=="/guidance/on":
            guidance["active"]=True; self._json({"ok":True}); return
        if p.path=="/guidance/off":
            guidance["active"]=False; self._json({"ok":True}); return

        self.send_response(404); self.end_headers()

# ================= MAIN ====================
if __name__=="__main__":
    threading.Thread(target=gnss_thread,daemon=True).start()
    threading.Thread(target=guidance_thread,daemon=True).start()
    print(f"[WEB] http://<PI-IP>:{HTTP_PORT}")
    HTTPServer(("0.0.0.0",HTTP_PORT),Handler).serve_forever()
