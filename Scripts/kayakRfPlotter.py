#!/usr/bin/env python3
import re
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.dates import AutoDateLocator, DateFormatter

LOG_PATH = "rfLog.txt"  # change if needed

# --- Regex patterns for loggerRf entries ---
PATTERNS = {
    "roll":   re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Roll: (?P<val>[-\d\.]+)"),
    "pitch":  re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Pitch: (?P<val>[-\d\.]+)"),
    "yaw":    re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Yaw: (?P<val>[-\d\.]+)"),

    "acc_x":  re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - X Acceleration: (?P<val>[-\d\.]+)"),
    "acc_y":  re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Y Acceleration: (?P<val>[-\d\.]+)"),
    "acc_z":  re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Z Acceleration: (?P<val>[-\d\.]+)"),

    "gyro_x": re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - X Gyroscope: (?P<val>[-\d\.]+)"),
    "gyro_y": re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Y Gyroscope: (?P<val>[-\d\.]+)"),
    "gyro_z": re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Z Gyroscope: (?P<val>[-\d\.]+)"),

    "rx_timing": re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Rx message timing: (?P<val>[-\d\.]+)"),
    "tx_timing": re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerRf - INFO - Tx message timing: (?P<val>[-\d\.]+)"),
}

# Run separator
START_MARKER = re.compile(
    r"^\s*\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2},\d{3}\s+- loggerRf - INFO - \*{4}\s*Motor Manager Starting Up\s*\*{5,}\s*$"
)

def parse_ts(ts_str: str) -> datetime:
    return datetime.strptime(ts_str.strip(), "%Y-%m-%d %H:%M:%S,%f")

def new_run_dict():
    return {k: {"t": [], "v": []} for k in PATTERNS.keys()}

def parse_runs(path: str):
    runs = []
    run = None

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            # Start of new run
            if START_MARKER.search(line):
                if run and any(run[k]["t"] for k in run):
                    runs.append(run)
                run = new_run_dict()
                continue

            if run is None:
                continue

            # Match line against patterns
            for key, rx in PATTERNS.items():
                m = rx.search(line)
                if m:
                    ts = parse_ts(m.group("ts"))
                    val = float(m.group("val"))
                    run[key]["t"].append(ts)
                    run[key]["v"].append(val)
                    break

    if run and any(run[k]["t"] for k in run):
        runs.append(run)

    return runs

def safe_plot(ax, series, label):
    if series["t"]:
        ax.plot(series["t"], series["v"], label=label)

def plot_runs(runs):
    if not runs:
        print("No runs found in log.")
        return

    for i, run in enumerate(runs, start=1):
        fig, axs = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
        fig.suptitle(f"RF Log — Run {i}")

        # 1) Roll / Pitch / Yaw
        safe_plot(axs[0], run["roll"], "Roll")
        safe_plot(axs[0], run["pitch"], "Pitch")
        safe_plot(axs[0], run["yaw"], "Yaw")
        axs[0].set_ylabel("Degrees")
        axs[0].legend()
        axs[0].grid(True, alpha=0.3)

        # 2) Acceleration
        safe_plot(axs[1], run["acc_x"], "X Acc")
        safe_plot(axs[1], run["acc_y"], "Y Acc")
        safe_plot(axs[1], run["acc_z"], "Z Acc")
        axs[1].set_ylabel("m/s²")
        axs[1].legend()
        axs[1].grid(True, alpha=0.3)

        # 3) Gyroscope
        safe_plot(axs[2], run["gyro_x"], "X Gyro")
        safe_plot(axs[2], run["gyro_y"], "Y Gyro")
        safe_plot(axs[2], run["gyro_z"], "Z Gyro")
        axs[2].set_ylabel("°/s")
        axs[2].legend()
        axs[2].grid(True, alpha=0.3)

        # 4) Rx message timing
        safe_plot(axs[3], run["rx_timing"], "Rx timing")
        axs[3].set_ylabel("s")
        axs[3].legend()
        axs[3].grid(True, alpha=0.3)

        # 5) Tx message timing
        safe_plot(axs[4], run["tx_timing"], "Tx timing")
        axs[4].set_ylabel("s")
        axs[4].set_xlabel("Timestamp")
        axs[4].legend()
        axs[4].grid(True, alpha=0.3)

        # Format timestamps nicely
        locator = AutoDateLocator()
        formatter = DateFormatter("%H:%M:%S.%f")
        axs[-1].xaxis.set_major_locator(locator)
        axs[-1].xaxis.set_major_formatter(formatter)
        fig.autofmt_xdate()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()

if __name__ == "__main__":
    runs = parse_runs(LOG_PATH)
    plot_runs(runs)
