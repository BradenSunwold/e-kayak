import re
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import datetime as dt
from collections import deque

# log file path
log_file = "rfLog.txt"

# regex to capture values
patterns = {
    "roll": re.compile(r"Roll:\s+([-+]?\d*\.\d+|\d+)"),
    "pitch": re.compile(r"Pitch:\s+([-+]?\d*\.\d+|\d+)"),
    "yaw": re.compile(r"Yaw:\s+([-+]?\d*\.\d+|\d+)"),
    "ax": re.compile(r"X Acceleration:\s+([-+]?\d*\.\d+|\d+)"),
    "ay": re.compile(r"Y Acceleration:\s+([-+]?\d*\.\d+|\d+)"),
    "az": re.compile(r"Z Acceleration:\s+([-+]?\d*\.\d+|\d+)"),
    "gx": re.compile(r"X Gyroscope:\s+([-+]?\d*\.\d+|\d+)"),
    "gy": re.compile(r"Y Gyroscope:\s+([-+]?\d*\.\d+|\d+)"),
    "gz": re.compile(r"Z Gyroscope:\s+([-+]?\d*\.\d+|\d+)"),
}

# keep last N points
MAXLEN = 500
time_data = deque(maxlen=MAXLEN)
roll, pitch, yaw = deque(maxlen=MAXLEN), deque(maxlen=MAXLEN), deque(maxlen=MAXLEN)
ax, ay, az = deque(maxlen=MAXLEN), deque(maxlen=MAXLEN), deque(maxlen=MAXLEN)
gx, gy, gz = deque(maxlen=MAXLEN), deque(maxlen=MAXLEN), deque(maxlen=MAXLEN)

# figure setup
fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# top plot: roll/pitch/yaw
line_roll, = axs[0].plot([], [], label="Roll")
line_pitch, = axs[0].plot([], [], label="Pitch")
line_yaw, = axs[0].plot([], [], label="Yaw")
axs[0].legend(loc="upper left")
axs[0].set_ylabel("Degrees")

# bottom plot: accel + gyro (stacked)
line_ax, = axs[1].plot([], [], label="Ax")
line_ay, = axs[1].plot([], [], label="Ay")
line_az, = axs[1].plot([], [], label="Az")
line_gx, = axs[1].plot([], [], label="Gx", linestyle="--")
line_gy, = axs[1].plot([], [], label="Gy", linestyle="--")
line_gz, = axs[1].plot([], [], label="Gz", linestyle="--")
axs[1].legend(loc="upper left")
axs[1].set_ylabel("Accel/Gyro")
axs[1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

plt.tight_layout()

def follow(file):
    file.seek(0, 2)  # seek to end
    while True:
        line = file.readline()
        if not line:
            continue
        yield line

def parse_line(line):
    try:
        ts_str = line.split(" - ")[0]
        timestamp = dt.datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S,%f")
    except Exception:
        return None, None

    for key, pat in patterns.items():
        m = pat.search(line)
        if m:
            return timestamp, (key, float(m.group(1)))
    return None, None

def update(_):
    while True:
        try:
            line = next(lines)
            ts, val = parse_line(line)
            if ts and val:
                key, value = val
                if ts not in time_data:
                    time_data.append(ts)
                if key == "roll": roll.append(value)
                elif key == "pitch": pitch.append(value)
                elif key == "yaw": yaw.append(value)
                elif key == "ax": ax.append(value)
                elif key == "ay": ay.append(value)
                elif key == "az": az.append(value)
                elif key == "gx": gx.append(value)
                elif key == "gy": gy.append(value)
                elif key == "gz": gz.append(value)
        except StopIteration:
            break

    if not time_data:
        return []

    # update plots
    line_roll.set_data(time_data, roll)
    line_pitch.set_data(time_data, pitch)
    line_yaw.set_data(time_data, yaw)
    line_ax.set_data(time_data, ax)
    line_ay.set_data(time_data, ay)
    line_az.set_data(time_data, az)
    line_gx.set_data(time_data, gx)
    line_gy.set_data(time_data, gy)
    line_gz.set_data(time_data, gz)

    for axx in axs:
        axx.relim()
        axx.autoscale_view()

    return [line_roll, line_pitch, line_yaw,
            line_ax, line_ay, line_az,
            line_gx, line_gy, line_gz]

import matplotlib.animation as animation
with open(log_file) as f:
    lines = follow(f)
    ani = animation.FuncAnimation(fig, update, interval=200, blit=False)
    plt.show()
