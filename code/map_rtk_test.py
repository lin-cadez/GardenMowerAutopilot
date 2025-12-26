
#!/usr/bin/env python3
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import serial

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
HTTP_PORT = 8080
UPDATE_MS = 1000  # browser update interval

latest = {
    "lat": None,
    "lon": None,
    "fix": None,
    "sats": None,
    "hdop": None,
    "alt": None,
    "ts": None,
    "raw": None,
}

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

FIX_MAP = {
    "0": "NO FIX",
    "1": "GNSS",
    "2": "DGPS",
    "4": "RTK FIXED",
    "5": "RTK FLOAT",
}

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

        # parts[6]=fix, [7]=sats, [8]=hdop, [9]=alt
        latest = {
            "lat": lat,
            "lon": lon,
            "fix": parts[6],
            "fix_text": FIX_MAP.get(parts[6], parts[6]),
            "sats": parts[7],
            "hdop": parts[8],
            "alt": parts[9],
            "ts": time.time(),
            "raw": line,
        }

HTML_PAGE = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>LC29H Live Map</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    body {{ margin:0; font-family: system-ui, Arial; }}
    #map {{ width: 100vw; height: 100vh; }}
    .hud {{
      position: absolute; top: 12px; left: 12px; z-index: 9999;
      background: rgba(255,255,255,0.92); padding: 10px 12px;
      border-radius: 12px; box-shadow: 0 6px 18px rgba(0,0,0,0.15);
      max-width: 360px; font-size: 14px;
    }}
    .row {{ display:flex; gap:10px; flex-wrap:wrap; }}
    .pill {{
      padding: 2px 8px; border-radius: 999px; background: #eee;
      font-variant-numeric: tabular-nums;
    }}
  </style>
</head>
<body>
  <div id="map"></div>
  <div class="hud">
    <div><b>LC29H Live Position</b></div>
    <div class="row" style="margin-top:6px;">
      <div class="pill" id="fix">Fix: -</div>
      <div class="pill" id="sats">Sats: -</div>
      <div class="pill" id="hdop">HDOP: -</div>
      <div class="pill" id="alt">Alt: -</div>
    </div>
    <div style="margin-top:6px; font-size:12px; color:#444;">
      <div id="latlon">Lat/Lon: -</div>
      <div id="age">Last update: -</div>
    </div>
  </div>

<script>
  const map = L.map("map").setView([46.1178, 14.0213], 17);
  L.tileLayer("https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png", {{
    maxZoom: 20,
    attribution: "&copy; OpenStreetMap contributors"
  }}).addTo(map);

  const marker = L.marker([46.1178, 14.0213]).addTo(map);
  const trail = L.polyline([], {{}}).addTo(map);

  function fmt(n, d=6) {{
    if (n === null || n === undefined) return "-";
    return Number(n).toFixed(d);
  }}

  async function update() {{
    try {{
      const r = await fetch("/pos", {{ cache: "no-store" }});
      const d = await r.json();

      if (d.lat && d.lon) {{
        const lat = d.lat, lon = d.lon;

        marker.setLatLng([lat, lon]);
        map.panTo([lat, lon], {{ animate: true }});

        // trail
        const pts = trail.getLatLngs();
        pts.push([lat, lon]);
        if (pts.length > 200) pts.shift();
        trail.setLatLngs(pts);

        document.getElementById("fix").textContent = "Fix: " + (d.fix_text || d.fix || "-");
        document.getElementById("sats").textContent = "Sats: " + (d.sats ?? "-");
        document.getElementById("hdop").textContent = "HDOP: " + (d.hdop ?? "-");
        document.getElementById("alt").textContent = "Alt: " + (d.alt ?? "-") + " m";
        document.getElementById("latlon").textContent = "Lat/Lon: " + fmt(lat, 6) + ", " + fmt(lon, 6);

        const age = d.ts ? (Date.now()/1000 - d.ts) : null;
        document.getElementById("age").textContent = "Last update: " + (age !== null ? age.toFixed(1) + " s" : "-");
      }}
    }} catch (e) {{
      // ignore transient errors
    }}
  }}

  update();
  setInterval(update, {UPDATE_MS});
</script>
</body>
</html>
"""

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
            return

        if self.path.startswith("/pos"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(json.dumps(latest).encode("utf-8"))
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        # silence default HTTP logs
        return

def main():
    threading.Thread(target=gnss_reader_thread, daemon=True).start()
    server = HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[WEB] Open: http://<PI-IP>:{HTTP_PORT}/")
    server.serve_forever()

if __name__ == "__main__":
    main()
