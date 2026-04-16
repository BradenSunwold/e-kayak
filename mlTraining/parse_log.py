"""
Parse RF log files from the Pi into clean CSVs for training.

Each IMU sample in the log is a group of 8 consecutive lines:
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - Auto mode packet
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - Speed: 0
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - X Acceleration: 0.1234
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - X Gyroscope: 0.0100
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - Y Acceleration: -9.7800
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - Y Gyroscope: -0.0200
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - Z Acceleration: 0.3400
    2026-04-14 10:00:00,123 - loggerRf - DEBUG - Z Gyroscope: 0.0300

Usage:
    python parse_log.py data/raw/rfLog_2026-04-14_10:00.00.log \
        --label stroke --start "10:00:30" --end "10:01:30" \
        --output data/stroke_session1.csv

The label isn't stored in the CSV -- it's encoded in the filename prefix
(e.g. stroke_*, no_stroke_*) so the dataset loader can read it from the name.
"""

import argparse
import csv
import re
from datetime import datetime

from config import CHANNEL_NAMES, DATA_DIR

# Regex to extract timestamp and message from a log line
# Format: "2026-04-14 10:00:00,123 - loggerRf - DEBUG - <message>"
LOG_LINE_RE = re.compile(
    r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) - loggerRf - DEBUG - (.+)$"
)

# Patterns for extracting values from each line in an IMU group
VALUE_PATTERNS = {
    "accel_x": re.compile(r"^X Acceleration: (.+)$"),
    "gyro_x":  re.compile(r"^X Gyroscope: (.+)$"),
    "accel_y": re.compile(r"^Y Acceleration: (.+)$"),
    "gyro_y":  re.compile(r"^Y Gyroscope: (.+)$"),
    "accel_z": re.compile(r"^Z Acceleration: (.+)$"),
    "gyro_z":  re.compile(r"^Z Gyroscope: (.+)$"),
}


def parse_timestamp(ts_str):
    """Parse '2026-04-14 10:00:00,123' into a datetime."""
    return datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S,%f")


def time_of_day(ts_str):
    """Extract just the HH:MM:SS portion for --start/--end comparison."""
    return parse_timestamp(ts_str).strftime("%H:%M:%S")


def parse_log_file(log_path, start_time=None, end_time=None):
    """
    Parse an RF log file and yield one dict per IMU sample.

    Each dict has keys: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    Args:
        log_path: Path to the RF log file
        start_time: Optional "HH:MM:SS" -- skip samples before this time
        end_time: Optional "HH:MM:SS" -- skip samples after this time
    """
    current_sample = {}
    current_ts = None

    with open(log_path) as f:
        for line in f:
            line = line.strip()
            match = LOG_LINE_RE.match(line)
            if not match:
                continue

            ts_str, message = match.groups()

            # "Auto mode packet" marks the start of a new IMU sample group
            if message == "Auto mode packet":
                # Emit the previous sample if complete
                if current_ts and len(current_sample) == 6:
                    # Replace comma with period in timestamp so it doesn't
                    # break CSV parsing (e.g. "08:05:51,828" -> "08:05:51.828")
                    current_sample["timestamp"] = current_ts.replace(",", ".")
                    yield current_sample

                # Start a new sample
                tod = time_of_day(ts_str)
                if start_time and tod < start_time:
                    current_sample = {}
                    current_ts = None
                    continue
                if end_time and tod > end_time:
                    current_sample = {}
                    current_ts = None
                    continue

                current_sample = {}
                current_ts = ts_str
                continue

            # Try to match each value pattern
            if current_ts is None:
                continue
            for channel, pattern in VALUE_PATTERNS.items():
                val_match = pattern.match(message)
                if val_match:
                    current_sample[channel] = float(val_match.group(1))
                    break

    # Don't forget the last sample
    if current_ts and len(current_sample) == 6:
        current_sample["timestamp"] = current_ts
        yield current_sample


def main():
    parser = argparse.ArgumentParser(
        description="Parse RF log files into CSVs for ML training"
    )
    parser.add_argument("log_file", help="Path to RF log file")
    parser.add_argument("--label", required=True,
                        help="Label for this segment (e.g. 'stroke', 'no_stroke')")
    parser.add_argument("--start", default=None,
                        help="Start time filter HH:MM:SS")
    parser.add_argument("--end", default=None,
                        help="End time filter HH:MM:SS")
    parser.add_argument("--output", default=None,
                        help="Output CSV path (default: data/<label>_<logname>.csv)")
    args = parser.parse_args()

    if args.output:
        out_path = args.output
    else:
        from pathlib import Path
        log_name = Path(args.log_file).stem
        out_path = DATA_DIR / f"{args.label}_{log_name}.csv"

    samples = list(parse_log_file(args.log_file, args.start, args.end))

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp"] + CHANNEL_NAMES)
        writer.writeheader()
        writer.writerows(samples)

    print(f"Wrote {len(samples)} samples to {out_path}")


if __name__ == "__main__":
    main()
