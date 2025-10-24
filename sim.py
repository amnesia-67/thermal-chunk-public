"""

Authors     : Anish Subash, Caden Wate
Date        : Sept 9th, 2025 (end date)
Description : Unnofficial sim (used pre-deployement of harware for fine tuning). 
Usage       : python (required).  python <filename>

=====================

WARNING     : INACCURATE & MEANT ONLY FOR PRE-DEPLOYEMENT 
ACCURACY    : ~ 88% - 92%

=====================

"""

"""
simulator.py
Emulates the ESP32 TEC controller in software.
Outputs the same CSV serial stream that heatmap_step.py expects.
"""
import time, math, threading, sys
import numpy as np
import serial

# ---------------- SIMULATION CONFIG ----------------
PORT = "COM4"        # <-- create this COM port via com0com, tty0tty, or similar
BAUD = 115200

AMBIENT_TEMP = 24.0
BOARD_SIZE = 50
THERMAL_DIFFUSIVITY = 0.12
TEC_POWER = 3.8          # °C/s at 100% duty
ENV_LOSS = 0.003         # coupling to ambient
DT = 0.2                 # simulation timestep (s)

# ---------------- CONTROLLER STATE ----------------
setpointC = 35.0
pidEnabled = True
ctrlBus = "A"
ctrlIdx = 1
lastMode = "S"
lastDutyPct = 0
time_s = 0.0

# ---------------- BOARD SIM ----------------
class VirtualBoard:
    def __init__(self, N=BOARD_SIZE, ambient=AMBIENT_TEMP):
        self.N = N
        self.temp = np.full((N, N), ambient)
        self.ambient = ambient

    def diffuse(self):
        lap = (
            np.roll(self.temp, 1, 0) + np.roll(self.temp, -1, 0) +
            np.roll(self.temp, 1, 1) + np.roll(self.temp, -1, 1) -
            4 * self.temp
        )
        self.temp += THERMAL_DIFFUSIVITY * lap

    def tec_effect(self, duty, polarity):
        cx = cy = self.N // 2
        r = self.N // 6
        y, x = np.ogrid[-cx:self.N-cx, -cy:self.N-cy]
        mask = x*x + y*y <= r*r
        self.temp[mask] += polarity * duty * TEC_POWER * DT

    def environment_loss(self):
        self.temp += ENV_LOSS * (self.ambient - self.temp)

    def step(self, duty, polarity):
        self.diffuse()
        self.tec_effect(duty, polarity)
        self.environment_loss()

    def sensor_readings(self):
        # match geometry of 9 sensors
        coords = [
            (25, 5), (41, 15), (45, 25), (41, 35),
            (25, 45), (9, 35), (5, 25), (9, 15),
            (25, 25)
        ]
        vals = [self.temp[y, x] + np.random.normal(0, 0.05) for x, y in coords]
        return vals

board = VirtualBoard()

# ---------------- SIM CONTROL LOGIC ----------------
def control_step(pv):
    global lastDutyPct, lastMode
    if not pidEnabled:
        lastDutyPct = 0
        lastMode = "S"
        return 0, 0

    err = setpointC - pv
    if abs(err) < 0.3:
        duty = 0
        lastMode = "S"
        return 0, 0

    polarity = 1 if err > 0 else -1
    duty = min(max(abs(err) * 3, 15), 85) / 100.0  # approximate proportional response
    lastDutyPct = int(duty * 100)
    lastMode = "H" if polarity > 0 else "C"
    return duty, polarity

# ---------------- SERIAL EMULATION ----------------
ser = serial.Serial(PORT, BAUD, timeout=0.05)
print(f"Simulator running on {PORT}")

def serial_reader():
    global setpointC, pidEnabled, ctrlBus, ctrlIdx
    while True:
        try:
            line = ser.readline().decode("utf-8", "ignore").strip()
            if not line: 
                continue
            if line.startswith("t"):
                try:
                    setpointC = float(line[1:])
                    ser.write(f"Setpoint = {setpointC:.1f} °C\n".encode())
                except: pass
            elif line == "p0":
                pidEnabled = False
                ser.write(b"PID: OFF\n")
            elif line == "p1":
                pidEnabled = True
                ser.write(b"PID: ON\n")
            elif line.startswith("sel"):
                ser.write(b"Control PV set\n")
            elif line == "m":
                ser.write(b"Sim menu\n")
        except Exception:
            pass

reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# ---------------- MAIN SIM LOOP ----------------
t0 = time.time()
while True:
    time.sleep(DT)
    time_s = time.time() - t0

    # compute average as PV
    sensors = board.sensor_readings()
    pv = sensors[-1]  # center as main PV
    duty, polarity = control_step(pv)
    board.step(duty, polarity)

    # Fake A/B bus arrangement: A[0..3], B[0..5]
    busA = [sensors[0], sensors[-1], AMBIENT_TEMP, sensors[1]]
    busB = sensors[1:7]

    csv = (
        f"{time_s:.1f},{pv:.2f},{setpointC:.2f},{setpointC:.2f},"
        f"{'|'.join(f'{x:.2f}' for x in busA)},"
        f"{'|'.join(f'{x:.2f}' for x in busB)},"
        f"{lastMode},{lastDutyPct}\n"
    )
    try:
        ser.write(csv.encode("utf-8"))
    except Exception as e:
        pass
