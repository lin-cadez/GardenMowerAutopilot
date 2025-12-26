#!/usr/bin/env python3
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

import serial

# ------------------- Config -------------------
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080

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

state_lock = threading.Lock()

latest = {
    "lat": None, "lon": None, "fix": None, "fix_text": None,
    "sats": None, "hdop": None, "alt": None, "ts": None, "raw": None,
}

recording = {
    "active": False,
    "filename": None,
    "started_ts": None,
    "points": [],  # list of [lat, lon]
    "count": 0,
}

_record_fp = None


def nmea_to_deg(value: str, hemi: str):
    if not value:
        return None
    try:
        d = float(value)
        deg = int(d // 100)
        minutes = d - deg * 100
        coord = deg + minutes / 60.0
        if hemi in ("S", "W"):
            coord = -coord
        return coord
    except ValueError:
        return None


def new_record_filename():
    return time.strftime("%Y%m%d_%H%M%S") + ".txt"


def start_recording():
    global _record_fp
    with state_lock:
        if recording["active"]:
            return {"ok": False, "error": "Already recording"}

        fname = new_record_filename()
        fpath = PATH_DIR / fname
        _record_fp = open(fpath, "w", buffering=1)  # line-buffered
        _record_fp.write("# epoch,lat,lon,fix,sats,hdop,alt\n")

        recording["active"] = True
        recording["filename"] = str(fpath)
        recording["started_ts"] = time.time()
        recording["points"] = []
        recording["count"] = 0
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
        return {"ok": True, "filename": recording["filename"], "points": pts, "count": recording["count"]}


def append_point_to_file(epoch, lat, lon, fix, sats, hdop, alt):
    # max precision: repr() preserves full float precision
    line = f"{epoch:.3f},{repr(lat)},{repr(lon)},{fix},{sats},{hdop},{alt}\n"
    if _record_fp:
        _record_fp.write(line)


def list_path_files():
    files = sorted(PATH_DIR.glob("*.txt"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [p.name for p in files]


def load_points_from_file(filename: str):
    # Prevent path traversal: only allow plain filenames that exist in mowing_paths
    safe = Path(filename).name
    fpath = PATH_DIR / safe
    if not fpath.exists() or not fpath.is_file():
        return {"ok": False, "error": "File not found"}

    pts = []
    count = 0
    with fpath.open("r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # epoch,lat,lon,fix,sats,hdop,alt
            parts = line.split(",")
            if len(parts) < 3:
                continue
            try:
                lat = float(parts[1])
                lon = float(parts[2])
            except ValueError:
                continue
            pts.append([lat, lon])
            count += 1

    return {"ok": True, "filename": str(fpath), "points": pts, "count": count}


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
        fix = parts[6]
        sats = parts[7]
        hdop = parts[8]
        alt = parts[9]

        if lat is None or lon is None:
            continue

        now = time.time()
        with state_lock:
            latest = {
                "lat": lat,
                "lon": lon,
                "fix": fix,
                "fix_text": FIX_MAP.get(fix, fix),
                "sats": sats,
                "hdop": hdop,
                "alt": alt,
                "ts": now,
                "raw": line,
            }

            if recording["active"]:
                recording["points"].append([lat, lon])
                recording["count"] += 1
                append_point_to_file(now, lat, lon, fix, sats, hdop, alt)


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, obj, status=200):
        data = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index.html"):
            html_path = BASE_DIR / "index.html"
            if not html_path.exists():
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"Missing index.html")
                return
            body = html_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/pos"):
            with state_lock:
                self._send_json(latest)
            return

        if self.path.startswith("/status"):
            with state_lock:
                self._send_json({
                    "active": recording["active"],
                    "filename": recording["filename"],
                    "started_ts": recording["started_ts"],
                    "count": recording["count"],
                    "points": recording["points"],  # live polyline points
                })
            return

        if self.path.startswith("/files"):
            self._send_json({"ok": True, "files": list_path_files()})
            return

        if self.path.startswith("/load"):
            # query: /load?file=XYZ.txt
            try:
                _, q = self.path.split("?", 1)
            except ValueError:
                self._send_json({"ok": False, "error": "Missing query ?file="}, status=400)
                return

            params = {}
            for kv in q.split("&"):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    params[k] = v

            fn = params.get("file", "")
            # very small decode for spaces etc.
            fn = fn.replace("%20", " ")
            res = load_points_from_file(fn)
            self._send_json(res, status=200 if res.get("ok") else 404)
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        if self.path.startswith("/start"):
            self._send_json(start_recording())
            return

        if self.path.startswith("/stop"):
            self._send_json(stop_recording())
            return

        self.send_response(404)
        self.end_headers()

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
