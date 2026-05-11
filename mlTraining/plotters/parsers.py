"""
Parsers for the four log file types produced by the kayak Pi.

Each parser returns a pandas DataFrame indexed by a tz-naive ``timestamp``
column (datetime64[ns]). Designed to be reused by both the offline plotter
and the auto-labeling pipeline.

Log formats (from KayakLogs/5-4-26/logs):
  imuLog   — kayak BNO055: heading, roll, pitch, accel=[x,y,z], gyro=[x,y,z]
  rfLog    — paddle IMU streamed over RF, one field per line, grouped per packet
  mlLog    — inference output: stroke_probability, inference_latency_ms
  motorLog — In Mode <NAME>, Filtered RPM, Kayak heading, FinCtrl <mode> fin=...
"""

from __future__ import annotations

import re
from pathlib import Path

import pandas as pd

# 2026-05-04 09:32:51,711
_TS_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})")
_TS_FMT = "%Y-%m-%d %H:%M:%S,%f"


def _parse_ts(s: str) -> pd.Timestamp:
    return pd.to_datetime(s, format=_TS_FMT)


def _iter_lines(path: Path):
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            m = _TS_RE.match(line)
            if not m:
                continue
            yield _parse_ts(m.group(1)), line[m.end():].rstrip("\n")


# ── Kayak IMU ──────────────────────────────────────────────────────────────
_KAYAK_RE = re.compile(
    r"Kayak IMU\s+heading=(?P<heading>-?\d+\.\d+)\s+"
    r"roll=(?P<roll>-?\d+\.\d+)\s+"
    r"pitch=(?P<pitch>-?\d+\.\d+)\s+"
    r"accel=\[(?P<ax>-?\d+\.\d+),\s*(?P<ay>-?\d+\.\d+),\s*(?P<az>-?\d+\.\d+)\]\s+"
    r"gyro=\[(?P<gx>-?\d+\.\d+),\s*(?P<gy>-?\d+\.\d+),\s*(?P<gz>-?\d+\.\d+)\]"
)


def parse_imu_log(path: Path | str) -> pd.DataFrame:
    """Kayak IMU log. Columns: heading, roll, pitch, accel_x/y/z, gyro_x/y/z."""
    rows = []
    for ts, body in _iter_lines(Path(path)):
        m = _KAYAK_RE.search(body)
        if not m:
            continue
        rows.append({
            "timestamp": ts,
            "heading": float(m["heading"]),
            "roll": float(m["roll"]),
            "pitch": float(m["pitch"]),
            "accel_x": float(m["ax"]),
            "accel_y": float(m["ay"]),
            "accel_z": float(m["az"]),
            "gyro_x": float(m["gx"]),
            "gyro_y": float(m["gy"]),
            "gyro_z": float(m["gz"]),
        })
    return pd.DataFrame(rows)


# ── Paddle IMU over RF ─────────────────────────────────────────────────────
# Each packet is split across 7 consecutive log lines that share a timestamp:
#   Speed: 0
#   X Acceleration: -1.6641
#   X Gyroscope: 1.0527
#   Y Acceleration: -0.1914
#   Y Gyroscope: -0.7832
#   Z Acceleration: 9.7422
#   Z Gyroscope: -0.1895
_RF_FIELD_RE = re.compile(
    r"^(?P<field>Speed|X Acceleration|X Gyroscope|"
    r"Y Acceleration|Y Gyroscope|Z Acceleration|Z Gyroscope):\s*(?P<val>-?\d+(?:\.\d+)?)"
)
_RF_FIELD_TO_COL = {
    "Speed": "speed",
    "X Acceleration": "accel_x",
    "Y Acceleration": "accel_y",
    "Z Acceleration": "accel_z",
    "X Gyroscope": "gyro_x",
    "Y Gyroscope": "gyro_y",
    "Z Gyroscope": "gyro_z",
}
_RF_MODE_RE = re.compile(r"(Auto|Manual|Training) mode packet", re.IGNORECASE)


def parse_rf_log(path: Path | str) -> pd.DataFrame:
    """Paddle IMU log. Columns: speed, accel_x/y/z, gyro_x/y/z, paddle_mode.

    Coalesces the 7-line-per-packet format into one row per packet, keyed on
    the last seen 'mode packet' marker.
    """
    rows: list[dict] = []
    current: dict | None = None
    current_mode: str | None = None

    def flush():
        nonlocal current
        if current is not None:
            current.setdefault("paddle_mode", current_mode)
            rows.append(current)
            current = None

    for ts, body in _iter_lines(Path(path)):
        body = body.lstrip(" -")
        body = re.sub(r"^loggerRf - (?:DEBUG|INFO|WARNING|ERROR) - ", "", body)

        mode_m = _RF_MODE_RE.search(body)
        if mode_m:
            flush()
            current_mode = mode_m.group(1).capitalize()
            current = {"timestamp": ts}
            continue

        field_m = _RF_FIELD_RE.match(body)
        if not field_m:
            continue
        col = _RF_FIELD_TO_COL[field_m["field"]]
        if current is None:
            current = {"timestamp": ts}
        current[col] = float(field_m["val"])

        if col == "gyro_z":
            flush()

    flush()
    df = pd.DataFrame(rows)
    return df


# ── ML inference log ───────────────────────────────────────────────────────
_ML_RE = re.compile(
    r"Prediction:\s*\S+\s*\(stroke_probability=(?P<p>-?\d+\.\d+)\)"
    r"\s*inference_latency=(?P<lat>-?\d+\.\d+)ms"
)


def parse_ml_log(path: Path | str) -> pd.DataFrame:
    """ML inference log. Columns: stroke_probability, inference_latency_ms."""
    rows = []
    for ts, body in _iter_lines(Path(path)):
        m = _ML_RE.search(body)
        if not m:
            continue
        rows.append({
            "timestamp": ts,
            "stroke_probability": float(m["p"]),
            "inference_latency_ms": float(m["lat"]),
        })
    return pd.DataFrame(rows)


# ── Motor log ──────────────────────────────────────────────────────────────
_MOTOR_MODE_RE = re.compile(r"In Mode\s+(?P<mode>[A-Z_]+)")
_MOTOR_RPM_RE = re.compile(r"Filtered RPM:\s*(?P<rpm>-?\d+(?:\.\d+)?)")
_MOTOR_HEADING_RE = re.compile(r"Kayak heading:\s*(?P<h>-?\d+(?:\.\d+)?)")
_MOTOR_FIN_RE = re.compile(r"FinCtrl\s+\S+\s+fin=(?P<fin>-?\d+(?:\.\d+)?)")


def parse_motor_log(path: Path | str) -> pd.DataFrame:
    """Motor log. Columns: mode, filtered_rpm, kayak_heading, fin.

    Each field is on its own log line. Returned as a long DataFrame with one
    row per line (NaN where that line didn't contain that field). Use
    ``df['mode'].ffill()`` etc. when plotting if you want a continuous trace.
    """
    rows = []
    for ts, body in _iter_lines(Path(path)):
        row = {"timestamp": ts}
        if (m := _MOTOR_MODE_RE.search(body)):
            row["mode"] = m["mode"]
        if (m := _MOTOR_RPM_RE.search(body)):
            row["filtered_rpm"] = float(m["rpm"])
        if (m := _MOTOR_HEADING_RE.search(body)):
            row["kayak_heading"] = float(m["h"])
        if (m := _MOTOR_FIN_RE.search(body)):
            row["fin"] = float(m["fin"])
        if len(row) > 1:
            rows.append(row)
    return pd.DataFrame(rows)


# ── Convenience dispatcher ─────────────────────────────────────────────────
PARSERS = {
    "imu": parse_imu_log,
    "rf": parse_rf_log,
    "ml": parse_ml_log,
    "motor": parse_motor_log,
}


def parse(source: str, path: Path | str) -> pd.DataFrame:
    if source not in PARSERS:
        raise ValueError(f"Unknown source '{source}'. Expected one of {list(PARSERS)}.")
    return PARSERS[source](path)
