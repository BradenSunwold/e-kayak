#!/usr/bin/env python3
import re
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.dates import AutoDateLocator, DateFormatter

LOG_PATH = "motorLog.txt"  # change if needed

# --- Regex patterns for the exact lines you provided ---
PATTERNS = {
    "rpm_cmd":    re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Rpm Step Command: (?P<val>[-\d\.]+)"),
    "rpm_filt":   re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Filtered RPM: (?P<val>[-\d\.]+)"),
    "rpm":        re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - RPM: (?P<val>[-\d\.]+)"),
    "fet_temp":   re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Fet Temperature: (?P<val>[-\d\.]+)"),
    "vin_meas":   re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - VESC measured Input Voltage: (?P<val>[-\d\.]+)"),
    "vin_cal":    re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Calibrated Input Voltage: (?P<val>[-\d\.]+)"),
    "curr_in":    re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Current In: (?P<val>[-\d\.]+)"),
    "curr_motor": re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Motor Current: (?P<val>[-\d\.]+)"),
    "power":      re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Power: (?P<val>[-\d\.]+)"),
    "wh":         re.compile(r"^(?P<ts>[\d\-:\.,\s]+) - loggerMotor - INFO - Watt Hours: (?P<val>[-\d\.]+)"),
}

# Robust start-of-run marker (accept 5+ trailing asterisks just in case)
START_MARKER = re.compile(
    r"^\s*\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2},\d{3}\s+- loggerMotor - INFO - \*{4}\s*Motor Manager Starting Up\s*\*{5,}\s*$"
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
            # Start a new run when we see the startup banner
            if START_MARKER.search(line):
                if run and any(run[k]["t"] for k in run):
                    runs.append(run)
                run = new_run_dict()
                continue

            if run is None:
                # Ignore lines until we see the first startup
                continue

            # Try each metric
            for key, rx in PATTERNS.items():
                m = rx.search(line)
                if m:
                    ts = parse_ts(m.group("ts"))
                    val = float(m.group("val"))
                    run[key]["t"].append(ts)
                    run[key]["v"].append(val)
                    break

    # Append the final run if it has any data
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
        fig, axs = plt.subplots(6, 1, figsize=(12, 16), sharex=True)
        fig.suptitle(f"Motor Log — Run {i}")

        # 1) RPMs (commanded + filtered + actual feedback)
        safe_plot(axs[0], run["rpm_cmd"], "RPM Command")
        safe_plot(axs[0], run["rpm_filt"], "Filtered RPM")
        safe_plot(axs[0], run["rpm"],     "RPM Feedback")
        axs[0].set_ylabel("RPM")
        axs[0].legend()
        axs[0].grid(True, alpha=0.3)

        # 2) Temperature
        safe_plot(axs[1], run["fet_temp"], "FET Temp")
        axs[1].set_ylabel("°C")
        axs[1].legend()
        axs[1].grid(True, alpha=0.3)

        # 3) Voltage (measured + calibrated)
        safe_plot(axs[2], run["vin_meas"], "Input Voltage (measured)")
        safe_plot(axs[2], run["vin_cal"],  "Input Voltage (calibrated)")
        axs[2].set_ylabel("V")
        axs[2].legend()
        axs[2].grid(True, alpha=0.3)

        # 4) Currents (same plot)
        safe_plot(axs[3], run["curr_in"],    "Current In")
        safe_plot(axs[3], run["curr_motor"], "Motor Current")
        axs[3].set_ylabel("A")
        axs[3].legend()
        axs[3].grid(True, alpha=0.3)

        # 5) Power
        safe_plot(axs[4], run["power"], "Power")
        axs[4].set_ylabel("W")
        axs[4].legend()
        axs[4].grid(True, alpha=0.3)

        # 6) Watt-hours (cumulative)
        safe_plot(axs[5], run["wh"], "Watt Hours")
        axs[5].set_ylabel("Wh")
        axs[5].set_xlabel("Timestamp")
        axs[5].legend()
        axs[5].grid(True, alpha=0.3)

        # Shared time formatting
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
