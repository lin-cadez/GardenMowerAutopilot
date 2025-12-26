#!/usr/bin/env python3

import time
import threading
import collections

import pigpio
import matplotlib.pyplot as plt

# --- GPIO pins (BCM) ---
CH1_IN = 17
CH2_IN = 27
CH5_IN = 22

# --- PWM validity ---
MIN_PULSE = 900
MAX_PULSE = 2100

# --- Plot window ---
HISTORY_SECONDS = 10.0     # last N seconds on screen
SAMPLE_HZ = 20             # how often to sample stored pulse widths into the plot buffers

# --------------------------------

pi = pigpio.pi()
if not pi.connected:
    raise OSError("Cannot connect to pigpio. Run: sudo pigpiod")

_last_tick = {}
_pulse_widths = {CH1_IN: 1500, CH2_IN: 1500, CH5_IN: 1500}
lock = threading.Lock()

def create_callback(gpio):
    def cb(gpio, level, tick):
        if level == 1:
            _last_tick[gpio] = tick
        elif level == 0 and gpio in _last_tick:
            pw = pigpio.tickDiff(_last_tick[gpio], tick)
            if MIN_PULSE <= pw <= MAX_PULSE:
                with lock:
                    _pulse_widths[gpio] = pw
    return cb

for pin in (CH1_IN, CH2_IN, CH5_IN):
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
    pi.callback(pin, pigpio.EITHER_EDGE, create_callback(pin))

# Buffers for plotting
maxlen = int(HISTORY_SECONDS * SAMPLE_HZ)
tbuf  = collections.deque(maxlen=maxlen)
ch1buf = collections.deque(maxlen=maxlen)
ch2buf = collections.deque(maxlen=maxlen)
ch5buf = collections.deque(maxlen=maxlen)

start = time.time()

def sampler():
    period = 1.0 / SAMPLE_HZ
    while True:
        now = time.time() - start
        with lock:
            ch1 = _pulse_widths[CH1_IN]
            ch2 = _pulse_widths[CH2_IN]
            ch5 = _pulse_widths[CH5_IN]
        tbuf.append(now)
        ch1buf.append(ch1)
        ch2buf.append(ch2)
        ch5buf.append(ch5)
        time.sleep(period)

threading.Thread(target=sampler, daemon=True).start()

# --- Matplotlib live plot ---
plt.ion()
fig, ax = plt.subplots()
l1, = ax.plot([], [], label="CH1 (GPIO17)")
l2, = ax.plot([], [], label="CH2 (GPIO27)")
l5, = ax.plot([], [], label="CH5 (GPIO22)")

ax.set_title("RC PWM pulse width (µs)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Pulse width (µs)")
ax.set_ylim(800, 2200)
ax.grid(True)
ax.legend(loc="upper right")

try:
    while True:
        if len(tbuf) < 2:
            time.sleep(0.05)
            continue

        # show last HISTORY_SECONDS
        t0 = tbuf[-1] - HISTORY_SECONDS
        # find first index where t >= t0
        # (simple linear scan is fine for these sizes)
        idx = 0
        for i, tv in enumerate(tbuf):
            if tv >= t0:
                idx = i
                break

        tt  = list(tbuf)[idx:]
        y1  = list(ch1buf)[idx:]
        y2  = list(ch2buf)[idx:]
        y5  = list(ch5buf)[idx:]

        l1.set_data(tt, y1)
        l2.set_data(tt, y2)
        l5.set_data(tt, y5)

        ax.set_xlim(tt[0], tt[-1])

        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    pi.stop()
