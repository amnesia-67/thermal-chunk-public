'''

Authors     : Anish Subash, Caden Wate
Date        : 20 October, 2025 (end date)
Description : Supplementary file to visalize data outputted over COM3 from the ESP32 running "base code"
Usage       : python heatmap.py (Run only while ESP32 is running)

'''


# heatmap_step.py
import time, math
import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox, RadioButtons
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle

# ---------------- CONFIG ----------------
PORT = 'COM3'           # Current ESP32 PORT
BAUD = 115200
UPDATE_INTERVAL = 0.25  # seconds between GUI refreshes
DEFAULT_TEMP = 24.0

# ---------------- SENSOR LAYOUT ----------------
# Coordinates are normalized to a unit disk
sensor_positions = np.array([
    [ 0.00,  1.00],   # ring 0  (top)
    [ 0.86,  0.50],   # ring 1
    [ 0.86, -0.50],   # ring 2
    [ 0.00, -1.00],   # ring 3  (bottom)
    [-0.86, -0.50],   # ring 4
    [-0.86,  0.50],   # ring 5
    [ 0.00,  0.00],   # center
], dtype=float)

# busA = [prox, center, room, disk], busB = ring temps
RING_COUNT = 6
CENTER_FROM_A = 1   # busA[1] is center (A1) per your Arduino print
DISK_FROM_A   = 3   # busA[3] used as center fallback

# ---------------- SERIAL OPEN ----------------
def open_serial(port, baud):
    s = serial.Serial(port, baud, timeout=0.05)
    try:
        s.setDTR(False)
        s.setRTS(False)
    except Exception:
        pass
    time.sleep(1.8)   # let ESP32 boot if it just reset
    return s

ser = open_serial(PORT, BAUD)
print(f"Connected to {PORT}")

# Send a sane starting state
def send(cmd):
    try:
        ser.write((cmd + "\n").encode('utf-8'))
    except Exception:
        pass

for cmd in ["p1", "sel A1", f"t{DEFAULT_TEMP}"]:
    send(cmd)
    time.sleep(0.08)

# ---------------- COLORMAP ----------------
colors = ["#001040", "#004870", "#009050", "#C0C000", "#FF6000", "#800000"]
heat_cmap = LinearSegmentedColormap.from_list("custom_heat", colors, N=256)
# Make NaNs fully transparent
heat_cmap = heat_cmap.with_extremes(bad=(0,0,0,0))

# ---------------- FIGURE ----------------
fig = plt.figure(figsize=(9,7))
plt.subplots_adjust(left=0.25, bottom=0.23, top=0.9)

tab_ax = plt.axes([0.05, 0.50, 0.15, 0.10])
radio_tabs = RadioButtons(tab_ax, ("2D Heat Map", "Step Response"))
radio_tabs.set_active(0)

ax = fig.add_axes([0.25, 0.2, 0.7, 0.7])                 # heatmap axes
ax2 = fig.add_axes([0.25, 0.2, 0.7, 0.7], frameon=True)  # step response axes
ax2.set_visible(False)

# Heatmap base
RES = 200
grid_x, grid_y = np.mgrid[-1:1:complex(0, RES), -1:1:complex(0, RES)]
r = np.sqrt(grid_x**2 + grid_y**2)
mask = r > 1.0
blank = np.ma.array(np.zeros_like(grid_x), mask=mask)  # masked outside circle

img = ax.imshow(blank, extent=(-1,1,-1,1), origin='lower',
                cmap=heat_cmap, vmin=0, vmax=100, interpolation='bilinear')

# Clip to a circle (prevents any rectangular bleed)
circle = Circle((0,0), radius=1.0, transform=ax.transData, fill=False) # fill clamps to circle w/o external fill
ax.add_patch(Circle((0,0),1.0, fill=False, color='black', lw=2))
img.set_clip_path(circle)

# Show sensor dots
ax.scatter(sensor_positions[:,0], sensor_positions[:,1],
           c='white', s=50, edgecolors='black', zorder=3)
ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2)
ax.set_aspect('equal'); ax.axis('off')
ax.set_title("Real-Time Circular Temperature Map")

# Text overlays
prox_text = ax.text(-1.55, 0.90,
    "A0 (Prox): -- °C\nA1 (Center): -- °C\nA2 (Room): -- °C",
    fontsize=10, ha='left', color='black', transform=ax.transData)
status_ax = fig.add_axes([0.02, 0.93, 0.22, 0.04]); status_ax.axis('off')
status = status_ax.text(0, 0.5, "Setpoint: -- °C\nPID: ON | Control: A1",
                        ha='left', va='center', color='black', fontsize=9)

cb = plt.colorbar(img, ax=ax, fraction=0.046, pad=0.04)
cb.set_label("Temperature (°C)")

# Step response plot
ax2.set_title("Step Response")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Temperature (°C)")
ax2.grid(True)
line_sp, = ax2.plot([], [], 'r--', label='SP (target or SPcmd)')
line_pv, = ax2.plot([], [], 'b-',  label='PV')
ax2.legend()

# ---------------- UI ----------------
ax_tbox = plt.axes([0.25, 0.05, 0.15, 0.07])
ax_sbox = plt.axes([0.43, 0.05, 0.15, 0.07])
ax_p1   = plt.axes([0.70, 0.05, 0.12, 0.07])
ax_p0   = plt.axes([0.84, 0.05, 0.12, 0.07])

tbox = TextBox(ax_tbox, 'Temp °C', initial=str(DEFAULT_TEMP))
sbox = TextBox(ax_sbox, 'Sensor',  initial='A1')
b_on = Button(ax_p1, 'PID ON (p1)')
b_off= Button(ax_p0, 'PID OFF (p0)')

def t_submit(txt): send(f"t{txt}")
def s_submit(txt): send(f"sel {txt.strip().upper()}")
def pid_on(_):     send("p1")
def pid_off(_):    send("p0")

tbox.on_submit(t_submit)
sbox.on_submit(s_submit)
b_on.on_clicked(pid_on)
b_off.on_clicked(pid_off)

# ---------------- PRECOMPUTE GAUSSIAN WEIGHTS ----------------
# Fast blending: grid = sum_i w_i * T_i / sum_i w_i  (masked outside circle)
SIGMA2 = 0.18  # adjust blur; smaller = sharper blobs
W = []
for (sx, sy) in sensor_positions:
    d2 = (grid_x - sx)**2 + (grid_y - sy)**2
    w  = np.exp(-d2 / max(1e-6, SIGMA2))
    w[mask] = 0.0
    W.append(w)
W = np.stack(W, axis=0)   # shape (N_sensors, RES, RES)
Wsum = np.maximum(W.sum(axis=0), 1e-6)

# ---------------- DATA BUFFERS ----------------
times, pvs, sps = [], [], []
t0 = time.time()

def parse_bus(s):
    s = s.strip()
    if not s: return []
    return [float(x) for x in s.split('|') if x.strip()]

def read_csv_line():
    """Read one CSV line from Arduino and parse flexibly.
    time,PV,SP,SPcmd,A,B,mode,duty (8 fields) or time,PV,SP,A,B,mode,duty (7 fields, legacy)"""
    for _ in range(100):
        ln = ser.readline().decode('utf-8', 'ignore').strip()
        if not ln or ',' not in ln or ln.startswith(("BOOT","CSV","Commands")):
            continue
        parts = ln.split(',')
        try:
            t    = float(parts[0])
            pv   = float(parts[1]) if parts[1] != "NaN" else math.nan
            sp   = float(parts[2])
            idx  = 3
            spcmd = None
            if len(parts) >= 8:
                spcmd = float(parts[3]); idx = 4  # new format adds SPcmd
            busA = parse_bus(parts[idx]);   idx += 1
            busB = parse_bus(parts[idx]);   idx += 1
            mode = parts[idx].strip();      idx += 1
            duty = parts[idx].strip()
            return dict(t=t, pv=pv, sp=sp, spcmd=spcmd, busA=busA, busB=busB, mode=mode, duty=duty)
        except Exception:
            continue
    return None

def build_temp_vector(busA, busB):
    # Produce an array of 7 temps in the same order as sensor_positions.
    # center preference: A1, else A3(disk), else PV fallback
    center = None
    if len(busA) > CENTER_FROM_A and not math.isnan(busA[CENTER_FROM_A]):
        center = busA[CENTER_FROM_A]
    elif len(busA) > DISK_FROM_A and not math.isnan(busA[DISK_FROM_A]):
        center = busA[DISK_FROM_A]
    else:
        center = float('nan')

    ring = list(busB[:RING_COUNT])
    # If only 5 ring values present, synthesize the 6th:
    if len(ring) < RING_COUNT:
        if len(busB) >= 2:
            ring.append(0.5*(busB[0] + busB[1]))  # simple fill
        else:
            while len(ring) < RING_COUNT:
                ring.append(center if not math.isnan(center) else 25.0)

    temps = ring[:RING_COUNT] + [center]
    return np.array(temps, dtype=float)

def update_heatmap(temps):
    # temps: array length 7 >> (6 ring + center)
    # grid = sum_i W[i]*T[i] / Wsum
    # reorder temps to match sensor_positions rows:
    if temps.shape[0] != sensor_positions.shape[0]:
        return
    num = np.tensordot(temps, W, axes=(0,0))  # (RES, RES)
    grid = num / Wsum
    mgrid = np.ma.array(grid, mask=mask)
    img.set_data(mgrid)

def animate(_):
    d = read_csv_line()
    if not d:
        return []

    pv   = d['pv']
    sp   = d['sp']
    spcmd= d['spcmd'] if d['spcmd'] is not None else sp
    busA = d['busA']
    busB = d['busB']

    # Update heatmap
    temps = build_temp_vector(busA, busB)
    update_heatmap(temps)

    # Proximity / center / room text (if present)
    prox  = busA[0] if len(busA) > 0 else float('nan')
    center= busA[1] if len(busA) > 1 else float('nan')
    room  = busA[2] if len(busA) > 2 else float('nan')
    prox_text.set_text(
        f"A0 (Prox): {prox if not math.isnan(prox) else '--':>5}\n"
        f"A1 (Center): {center if not math.isnan(center) else '--':>5}\n"
        f"A2 (Room): {room if not math.isnan(room) else '--':>5}"
    )
    status.set_text(f"Setpoint: {sp:.1f} °C\nPID: ON | Control: A1")

    # Step response data
    ts = d['t'] - t0
    times.append(ts); pvs.append(pv); sps.append(spcmd)
    if len(times) > 1200:  # cap memory (~5 min at 0.25 s)
        times.pop(0); pvs.pop(0); sps.pop(0)

    line_sp.set_data(times, sps)
    line_pv.set_data(times, pvs)
    ax2.relim(); ax2.autoscale_view()

    return [img, prox_text, status, line_sp, line_pv]

ani = FuncAnimation(
    fig, animate,
    interval=int(UPDATE_INTERVAL * 1000),
    blit=False,
    cache_frame_data=False
)

def switch_tab(label):
    if label == "2D Heat Map":
        ax.set_visible(True);  cb.ax.set_visible(True)
        ax2.set_visible(False)
    else:
        ax.set_visible(False); cb.ax.set_visible(False)
        ax2.set_visible(True)

radio_tabs.on_clicked(switch_tab)

def on_close(_):
    try:
        ser.close()
        print("Serial closed.")
    except Exception:
        pass

fig.canvas.mpl_connect('close_event', on_close)

plt.show()
