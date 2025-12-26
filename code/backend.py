#!/usr/bin/env python3

#123
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs
import math
import serial

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080

BASE_DIR = Path(__file__).resolve().parent
PATH_DIR = BASE_DIR / "mowing_paths"
PATH_DIR.mkdir(exist_ok=True)

FIX_MAP = {"0":"NO FIX","1":"GNSS","2":"DGPS","4":"RTK FIXED","5":"RTK FLOAT"}

state_lock = threading.Lock()

latest = {"lat":None,"lon":None,"fix":None,"fix_text":None,"sats":None,"hdop":None,"alt":None,"ts":None,"raw":None}
recording = {"active":False,"filename":None,"started_ts":None,"points":[],"count":0}
_record_fp = None


def nmea_to_deg(value: str, hemi: str):
    if not value:
        return None
    try:
        d = float(value)
        deg = int(d // 100)
        minutes = d - deg * 100
        coord = deg + minutes / 60.0
        return -coord if hemi in ("S","W") else coord
    except ValueError:
        return None


def new_record_filename():
    return time.strftime("%Y%m%d_%H%M%S") + ".txt"


def append_point_to_file(epoch, lat, lon, fix, sats, hdop, alt):
    line = f"{epoch:.3f},{repr(lat)},{repr(lon)},{fix},{sats},{hdop},{alt}\n"
    if _record_fp:
        _record_fp.write(line)


def start_recording():
    global _record_fp
    with state_lock:
        if recording["active"]:
            return {"ok": False, "error": "Already recording"}
        fname = new_record_filename()
        fpath = PATH_DIR / fname
        _record_fp = open(fpath, "w", buffering=1)
        _record_fp.write("# epoch,lat,lon,fix,sats,hdop,alt\n")
        recording.update({"active":True,"filename":str(fpath),"started_ts":time.time(),"points":[],"count":0})
        return {"ok": True, "filename": str(fpath)}


def stop_recording():
    global _record_fp
    with state_lock:
        if not recording["active"]:
            return {"ok": False, "error": "Not recording"}
        recording["active"] = False
        try:
            if _record_fp:
                _record_fp.flush()
                _record_fp.close()
        finally:
            _record_fp = None

        pts = list(recording["points"])
        outer, area_m2 = hull_and_area(pts)
        return {"ok": True, "filename": recording["filename"], "count": recording["count"], "points": pts, "outer": outer, "area_m2": area_m2}


def list_path_files():
    files = sorted(PATH_DIR.glob("*.txt"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [p.name for p in files]


def load_points_from_file(filename: str):
    safe = Path(filename).name
    fpath = PATH_DIR / safe
    if not fpath.exists() or not fpath.is_file():
        return {"ok": False, "error": "File not found"}

    pts = []
    with fpath.open("r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 3:
                continue
            try:
                lat = float(parts[1]); lon = float(parts[2])
            except ValueError:
                continue
            pts.append([lat, lon])

    outer, area_m2 = hull_and_area(pts)
    return {"ok": True, "filename": str(fpath), "count": len(pts), "points": pts, "outer": outer, "area_m2": area_m2}


# --------- Convex hull (outer boundary) + area in m² ---------
def _to_xy(points):
    if not points:
        return [], 0.0
    lat0 = sum(p[0] for p in points) / len(points)
    lon0 = sum(p[1] for p in points) / len(points)
    lat0r = math.radians(lat0)
    R = 6371000.0
    xy = []
    for lat, lon in points:
        x = R * math.cos(lat0r) * math.radians(lon - lon0)
        y = R * math.radians(lat - lat0)
        xy.append((x, y, lat, lon))
    return xy, lat0r

def _cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def convex_hull_latlon(points):
    uniq = sorted(set((p[0], p[1]) for p in points))
    if len(uniq) <= 2:
        return [[lat, lon] for lat, lon in uniq]

    xy, _ = _to_xy([[lat, lon] for lat, lon in uniq])
    xy_sorted = sorted(xy, key=lambda t: (t[0], t[1]))

    lower = []
    for p in xy_sorted:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(xy_sorted):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    hull_xy = lower[:-1] + upper[:-1]
    return [[p[2], p[3]] for p in hull_xy]

def polygon_area_m2(points_latlon):
    if len(points_latlon) < 3:
        return 0.0
    xy, _ = _to_xy(points_latlon)
    xs = [p[0] for p in xy]; ys = [p[1] for p in xy]
    area = 0.0
    n = len(xs)
    for i in range(n):
        j = (i + 1) % n
        area += xs[i]*ys[j] - xs[j]*ys[i]
    return abs(area) * 0.5

def hull_and_area(points):
    if len(points) < 2:
        return [], 0.0
    hull = convex_hull_latlon(points)
    area = polygon_area_m2(hull)
    # closed polyline (connected on both ends)
    if len(hull) >= 3:
        hull_closed = hull + [hull[0]]
    else:
        hull_closed = hull[:]
    return hull_closed, area


def gnss_reader_thread():
    global latest
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    print(f"[INFO] GNSS serial open: {SERIAL_PORT} {SERIAL_BAUD}")

    while True:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
            continue

        parts = line.split(",")
        if len(parts) < 10:
            continue

        lat = nmea_to_deg(parts[2], parts[3])
        lon = nmea_to_deg(parts[4], parts[5])
        if lat is None or lon is None:
            continue

        fix = parts[6]; sats = parts[7]; hdop = parts[8]; alt = parts[9]
        now = time.time()

        with state_lock:
            latest = {"lat":lat,"lon":lon,"fix":fix,"fix_text":FIX_MAP.get(fix,fix),
                      "sats":sats,"hdop":hdop,"alt":alt,"ts":now,"raw":line}
            if recording["active"]:
                recording["points"].append([lat, lon])
                recording["count"] += 1
                append_point_to_file(now, lat, lon, fix, sats, hdop, alt)


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, obj, status=200):
        data = (json.dumps(obj) + "\n").encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            html_path = BASE_DIR / "index.html"
            if not html_path.exists():
                self.send_response(404); self.end_headers(); self.wfile.write(b"Missing index.html"); return
            body = html_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/files":
            self._send_json({"ok": True, "files": list_path_files()})
            return

        if path == "/load":
            qs = parse_qs(parsed.query)
            fn = (qs.get("file") or [""])[0]
            if not fn:
                self._send_json({"ok": False, "error": "Missing file="}, status=400); return
            res = load_points_from_file(fn)
            self._send_json(res, status=200 if res.get("ok") else 404)
            return

        if path == "/pos":
            with state_lock:
                self._send_json(latest)
            return

        if path == "/status":
            with state_lock:
                self._send_json({
                    "active": recording["active"],
                    "filename": recording["filename"],
                    "started_ts": recording["started_ts"],
                    "count": recording["count"],
                    "points": recording["points"],
                })
            return

        self.send_response(404); self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/start":
            self._send_json(start_recording()); return
        if path == "/stop":
            self._send_json(stop_recording()); return

        self.send_response(404); self.end_headers()

    def log_message(self, format, *args):
        return


def main():
    threading.Thread(target=gnss_reader_thread, daemon=True).start()
    server = HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[WEB] Open: http://<PI-IP>:{HTTP_PORT}/")
    print(f"[WEB] Files saved to: {PATH_DIR}")
    server.serve_forever()

if __name__ == "__main__":
    main()
