"""
Interactive 2D playback of a kayak session.

What it shows:
  - Top: 2D top-down view of the boat. Faint full trajectory is drawn
    upfront; an oriented boat marker plus a brighter recent-trail line
    show the current playback position.
  - Below: time-synced subpanels for paddle IMU, kayak forward accel +
    assist label, kayak yaw rate + turn label. A vertical playhead line
    tracks the current sample.
  - Bottom: slider for scrubbing, play/pause button.

Usage:
  python -m plotters.playback plotters/configs/raw_signals.yaml
  python -m plotters.playback plotters/configs/raw_signals.yaml \
      --start 30 --end 120        # seconds from session start
  python -m plotters.playback plotters/configs/raw_signals.yaml \
      --speed 2.0                 # 2x real-time playback

Uses the same YAML config format as plot_session.py — only the session
and log_dir fields are required; the labels_config block is optional and
controls the overlaid label traces.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyArrow
from matplotlib.widgets import Button, Slider

from plotters import parsers
from plotters.labels import LabelConfig, compute_labels
from plotters.sim_trajectory import TrajectoryConfig, compute_trajectory


SOURCE_TO_PREFIX = {
    "imu": "imuLog",
    "rf": "rfLog",
    "ml": "mlLog",
    "motor": "motorLog",
}


def _log_path(log_dir: Path, source: str, session: str) -> Path:
    return log_dir / f"{SOURCE_TO_PREFIX[source]}_{session}.log"


def _parse_time_arg(arg, session_start):
    """Same accepted forms as plot_session: HH:MM:SS, seconds-from-start, or None."""
    if arg is None:
        return None
    try:
        offset = float(arg)
        return session_start + pd.Timedelta(seconds=offset)
    except ValueError:
        pass
    date = session_start.strftime("%Y-%m-%d")
    return pd.to_datetime(f"{date} {arg}")


def _slice_to_window(df, start_ts, end_ts):
    if df.empty:
        return df
    mask = pd.Series(True, index=df.index)
    if start_ts is not None:
        mask &= df["timestamp"] >= start_ts
    if end_ts is not None:
        mask &= df["timestamp"] <= end_ts
    return df.loc[mask].reset_index(drop=True)


def _load_yaml(path):
    with open(path) as fh:
        cfg = yaml.safe_load(fh)
    labels_cfg = LabelConfig(**cfg["labels_config"]) if cfg.get("labels_config") else LabelConfig()
    return cfg["session"], Path(cfg["log_dir"]).expanduser(), labels_cfg


def _draw_boat(ax, x, y, heading_rad, length=2.0, color="tab:red"):
    """Draw an oriented boat marker as an arrow. Returns the artist so it
    can be removed/redrawn each frame."""
    dx = length * np.cos(heading_rad)
    dy = length * np.sin(heading_rad)
    return ax.add_patch(FancyArrow(
        x - dx / 2, y - dy / 2, dx, dy,
        width=length * 0.25, head_width=length * 0.5,
        head_length=length * 0.4, length_includes_head=True,
        color=color, zorder=5,
    ))


def run_playback(session, log_dir, labels_cfg, start_arg, end_arg, playback_speed):
    # ── Load & align all sources ───────────────────────────────────────────
    imu_path = _log_path(log_dir, "imu", session)
    rf_path = _log_path(log_dir, "rf", session)
    if not imu_path.exists():
        raise SystemExit(f"Kayak IMU log missing: {imu_path}")

    kayak_df = parsers.parse_imu_log(imu_path)
    paddle_df = parsers.parse_rf_log(rf_path) if rf_path.exists() else pd.DataFrame()
    print(f"[ok] kayak IMU: {len(kayak_df)} rows")
    print(f"[ok] paddle IMU: {len(paddle_df)} rows")

    # Find session start across whatever loaded successfully, then slice both.
    starts = [df["timestamp"].iloc[0] for df in (kayak_df, paddle_df) if not df.empty]
    if not starts:
        raise SystemExit("No usable IMU data — nothing to play back.")
    session_start = min(starts)
    start_ts = _parse_time_arg(start_arg, session_start)
    end_ts = _parse_time_arg(end_arg, session_start)
    kayak_df = _slice_to_window(kayak_df, start_ts, end_ts)
    paddle_df = _slice_to_window(paddle_df, start_ts, end_ts)
    if len(kayak_df) < 2:
        raise SystemExit("Time window selected too few kayak samples to play back.")

    # ── Compute trajectory + labels ───────────────────────────────────────
    traj = compute_trajectory(kayak_df, TrajectoryConfig())
    labels_df = compute_labels(kayak_df, labels_cfg)
    print(f"[ok] trajectory: x range [{traj.x.min():.1f}, {traj.x.max():.1f}], "
          f"y range [{traj.y.min():.1f}, {traj.y.max():.1f}]")

    # ── Figure layout ─────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 9))
    gs = GridSpec(
        5, 2, figure=fig,
        width_ratios=[2, 3],
        height_ratios=[1, 1, 1, 1, 0.18],
        hspace=0.45, wspace=0.18,
    )
    ax_sim = fig.add_subplot(gs[0:4, 0])
    ax_paddle = fig.add_subplot(gs[0, 1])
    ax_fwd = fig.add_subplot(gs[1, 1], sharex=ax_paddle)
    ax_yaw = fig.add_subplot(gs[2, 1], sharex=ax_paddle)
    ax_lbl = fig.add_subplot(gs[3, 1], sharex=ax_paddle)
    ax_slider = fig.add_subplot(gs[4, :])

    # ── 2D simulation pane ────────────────────────────────────────────────
    ax_sim.plot(traj.x, traj.y, color="lightgray", linewidth=0.8, label="full path")
    recent_trail_line, = ax_sim.plot([], [], color="tab:red", linewidth=2.0, label="recent")
    ax_sim.set_aspect("equal", adjustable="datalim")
    ax_sim.set_title("Kayak path (top-down, integrated from IMU)")
    ax_sim.set_xlabel("x (drift units, not metric)")
    ax_sim.set_ylabel("y (drift units, not metric)")
    ax_sim.grid(True, alpha=0.3)
    ax_sim.legend(loc="upper left", fontsize=8)

    # Boat marker — we recreate it each frame because FancyArrow doesn't
    # support in-place updates of position+rotation cleanly.
    boat_state = {"artist": _draw_boat(ax_sim, traj.x.iloc[0], traj.y.iloc[0],
                                       traj.heading_rad.iloc[0])}

    # ── Time-series subpanels ─────────────────────────────────────────────
    if not paddle_df.empty:
        for col, lab in (("accel_x", "ax"), ("accel_y", "ay"), ("accel_z", "az")):
            ax_paddle.plot(paddle_df["timestamp"], paddle_df[col], linewidth=0.8, label=lab)
        ax_paddle.legend(loc="upper right", fontsize=7)
    ax_paddle.set_title("Paddle IMU accel")
    ax_paddle.grid(True, alpha=0.3)

    ax_fwd.plot(kayak_df["timestamp"], kayak_df["accel_x"],
                color="tab:blue", linewidth=0.9, label="kayak accel_x (forward)")
    if "assist_label" in labels_df.columns:
        ax_fwd.plot(labels_df["timestamp"], labels_df["assist_label"],
                    color="tab:orange", linewidth=1.4, label="assist_label")
    ax_fwd.legend(loc="upper right", fontsize=7)
    ax_fwd.set_title("Forward axis: signal vs. label")
    ax_fwd.grid(True, alpha=0.3)

    ax_yaw.plot(kayak_df["timestamp"], kayak_df["gyro_z"],
                color="tab:blue", linewidth=0.9, label="kayak gyro_z (yaw rate)")
    if "turn_label" in labels_df.columns:
        ax_yaw.plot(labels_df["timestamp"], labels_df["turn_label"],
                    color="tab:orange", linewidth=1.4, label="turn_label")
    ax_yaw.legend(loc="upper right", fontsize=7)
    ax_yaw.set_title("Yaw axis: signal vs. label")
    ax_yaw.grid(True, alpha=0.3)

    ax_lbl.plot(traj["timestamp"], traj["velocity"],
                color="tab:green", linewidth=1.0, label="sim forward velocity")
    ax_lbl.plot(traj["timestamp"], np.degrees(traj["heading_rad"]),
                color="tab:purple", linewidth=1.0, label="sim heading (deg, integrated)")

    # BNO055 fused heading — anchored at the session start and unwrapped so
    # the trace doesn't jump at the 0/360 boundary. Negated to match the
    # math convention used by our gyro_z integration: BNO055 reports
    # compass heading (positive clockwise from north), while our integrated
    # heading follows the math convention (positive counterclockwise).
    # After this flip, the gap between the two traces is gyro drift.
    if "heading" in kayak_df.columns and not kayak_df["heading"].isna().all():
        measured_unwrapped = np.unwrap(kayak_df["heading"].to_numpy(dtype=float), period=360.0)
        measured_relative = -(measured_unwrapped - measured_unwrapped[0])
        ax_lbl.plot(kayak_df["timestamp"], measured_relative,
                    color="tab:olive", linewidth=1.0, linestyle="--",
                    label="measured heading (BNO055, deg, sign-flipped)")

    ax_lbl.legend(loc="upper right", fontsize=7)
    ax_lbl.set_title("Simulator state — integrated heading vs. measured")
    ax_lbl.set_xlabel("time")
    ax_lbl.grid(True, alpha=0.3)

    # Playhead vertical lines on each subpanel.
    playhead_lines = [ax.axvline(traj["timestamp"].iloc[0], color="k",
                                 linewidth=0.8, alpha=0.6)
                      for ax in (ax_paddle, ax_fwd, ax_yaw, ax_lbl)]

    # ── Slider + play button ──────────────────────────────────────────────
    num_frames = len(traj)
    slider = Slider(ax_slider, "frame", 0, num_frames - 1, valinit=0, valstep=1)
    button_ax = fig.add_axes([0.01, 0.01, 0.06, 0.04])
    play_button = Button(button_ax, "Play")

    # Trail "recent" window = last ~3 seconds. Helps the eye track where the
    # boat is on the faint full-path background.
    recent_n = max(10, int(3.0 * 20))  # ~3 s at 20 Hz

    state = {"playing": False, "timer": None}

    def update_frame(i):
        i = int(i)
        i = max(0, min(num_frames - 1, i))

        # Update boat position+orientation (recreate the patch — cheap).
        boat_state["artist"].remove()
        boat_state["artist"] = _draw_boat(
            ax_sim, traj.x.iloc[i], traj.y.iloc[i], traj.heading_rad.iloc[i])

        # Update brighter "recent" trail.
        lo = max(0, i - recent_n)
        recent_trail_line.set_data(traj.x.iloc[lo:i + 1], traj.y.iloc[lo:i + 1])

        # Move playhead lines on every time-series subpanel.
        t = traj["timestamp"].iloc[i]
        for line in playhead_lines:
            line.set_xdata([t, t])

        fig.canvas.draw_idle()

    slider.on_changed(update_frame)

    def step_play(_evt=None):
        if not state["playing"]:
            return
        i = int(slider.val) + 1
        if i >= num_frames:
            i = 0
        slider.set_val(i)   # triggers update_frame
        # Re-arm timer for the next step. Real-time at 20 Hz = 50 ms per frame.
        delay_ms = max(1, int(50 / max(playback_speed, 0.01)))
        state["timer"] = fig.canvas.new_timer(interval=delay_ms)
        state["timer"].add_callback(step_play)
        state["timer"].start()

    def toggle_play(_evt):
        if state["playing"]:
            state["playing"] = False
            if state["timer"] is not None:
                state["timer"].stop()
            play_button.label.set_text("Play")
        else:
            state["playing"] = True
            play_button.label.set_text("Pause")
            step_play()

    play_button.on_clicked(toggle_play)

    fig.suptitle(f"Session {session}", fontsize=11)
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Interactive kayak session playback.")
    ap.add_argument("config", type=Path, help="Path to YAML plot config.")
    ap.add_argument("--start", default=None,
                    help="Start of playback window. HH:MM:SS or seconds-from-start.")
    ap.add_argument("--end", default=None,
                    help="End of playback window. HH:MM:SS or seconds-from-start.")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Playback speed multiplier (1.0 = real-time).")
    args = ap.parse_args()

    session, log_dir, labels_cfg = _load_yaml(args.config)
    run_playback(session, log_dir, labels_cfg, args.start, args.end, args.speed)


if __name__ == "__main__":
    main()
