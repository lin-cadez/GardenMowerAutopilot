#!/usr/bin/env python3

import time
import threading
import cv2
from picamera2 import Picamera2
from http.server import BaseHTTPRequestHandler, HTTPServer

current_frame = None  # Shared frame buffer


# =====================================================
# Camera Grab Thread
# =====================================================
def camera_thread():
    global current_frame

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
    cam.start()

    time.sleep(1)
    print("[CAM] Camera streaming started.")

    while True:
        frame = cam.capture_array()
        current_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        time.sleep(0.05)  # ~100 FPS max


# =====================================================
# MJPEG Web Stream Thread
# =====================================================
def mjpeg_stream_thread():

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != '/stream':
                self.send_response(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            while True:
                if current_frame is None:
                    time.sleep(0.05)
                    continue

                ok, jpeg = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])

                if not ok:
                    continue

                try:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
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

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
