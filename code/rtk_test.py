#!/usr/bin/env python3
import base64
import socket
import threading
import time
import serial

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200

NTRIP_HOST = "rtk2go.com"
NTRIP_PORT = 2101
MOUNTPOINT = "FRELIH"          # <-- Slovenia mountpoint (case-sensitive)

USERNAME = "test@email.com"    # valid-looking email (RTK2go ignores password)
PASSWORD = "none"

GGA_SEND_HZ = 1.0              # send at most 1 GGA per second


def open_ntrip():
    sock = socket.create_connection((NTRIP_HOST, NTRIP_PORT), timeout=10)
    sock.settimeout(10)

    auth = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()

    request = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"User-Agent: NTRIP PythonClient\r\n"
        f"Authorization: Basic {auth}\r\n"
        f"\r\n"
    )

    sock.sendall(request.encode("ascii", errors="ignore"))
    resp = sock.recv(2048)
    text = resp.decode("ascii", errors="ignore")
    print(text.strip())

    # If you see SOURCETABLE here, you are NOT on the stream
    if "SOURCETABLE" in text:
        raise RuntimeError("Got SOURCETABLE instead of RTCM stream. Check mountpoint name (case-sensitive).")

    if ("200 OK" not in text) and ("ICY 200 OK" not in text):
        raise RuntimeError("NTRIP connection failed:\n" + text)

    print("[INFO] Connected to RTK2go mountpoint:", MOUNTPOINT)
    return sock


def gga_sender(ser, sock, stop_flag):
    """
    Read NMEA and send only 1 GGA per second (max).
    """
    last_sent = 0.0
    latest_gga = None

    while not stop_flag["stop"]:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
            latest_gga = line + "\r\n"

        now = time.time()
        if latest_gga and (now - last_sent) >= (1.0 / GGA_SEND_HZ):
            try:
                sock.sendall(latest_gga.encode("ascii", errors="ignore"))
                print("[GGA -> NTRIP]", latest_gga.strip())
                last_sent = now
            except (BrokenPipeError, ConnectionResetError, OSError):
                stop_flag["stop"] = True
                break


def rtcm_receiver(ser, sock, stop_flag):
    """
    Receive RTCM bytes and forward to GNSS over UART.
    """
    while not stop_flag["stop"]:
        try:
            data = sock.recv(4096)
            if not data:
                stop_flag["stop"] = True
                break
            ser.write(data)
            # keep log minimal; uncomment if you want:
            print(f"[RTCM <- NTRIP] {len(data)} bytes")
        except socket.timeout:
            continue
        except (BrokenPipeError, ConnectionResetError, OSError):
            stop_flag["stop"] = True
            break


def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    print("[INFO] GNSS serial open:", SERIAL_PORT, SERIAL_BAUD)

    while True:
        try:
            sock = open_ntrip()

            stop_flag = {"stop": False}
            t_gga = threading.Thread(target=gga_sender, args=(ser, sock, stop_flag), daemon=True)
            t_rtcm = threading.Thread(target=rtcm_receiver, args=(ser, sock, stop_flag), daemon=True)

            t_gga.start()
            t_rtcm.start()

            while not stop_flag["stop"]:
                time.sleep(0.2)

            try:
                sock.close()
            except Exception:
                pass

            print("[WARN] Disconnected. Reconnecting in 2 seconds...")
            time.sleep(2)

        except Exception as e:
            print("[ERROR]", e)
            print("[WARN] Retry in 5 seconds...")
            time.sleep(5)


if __name__ == "__main__":
    main()
