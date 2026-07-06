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
  python -m plotters.playback plotters/configs/raw_signals.yaml \
      --ghost model               # ghost driven by the trained ONNX models
  python -m plotters.playback plotters/configs/raw_signals.yaml \
      --ghost both                # label ghost AND model ghost overlaid
  python -m plotters.playback plotters/configs/raw_signals.yaml \
      --ghost off                 # disable ghost overlays

Uses the same YAML config format as plot_session.py — only the session
and log_dir fields are required; the labels_config block is optional and
controls the overlaid label traces.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyArrow
from matplotlib.widgets import Button, Slider

from config import CHANNEL_NAMES, CHECKPOINT_DIR
from plotters import parsers
from plotters.labels import LabelConfig, compute_labels
from plotters.sim_trajectory import (
    TrajectoryConfig,
    compute_ghost_trajectory_from_labels,
    compute_trajectory,
)


# Ghost-boat source selector.
#   'label' — ghost driven by the regression labels (sanity-checks the
#             labeling pipeline: this is the best a perfect model could do).
#   'model' — ghost driven by the exported ONNX model predictions run over
#             the paddle IMU stream (shows what the model actually learned).
#   'both'  — label ghost and model ghost overlaid. The gap between them is
#             model error; the gap between label ghost and the real boat is
#             labeling/physics error.
#   'off'   — no ghosts.
GHOST_CHOICES = ("label", "model", "both", "off")

# Colors per ghost source: (faint full-path color, bright recent-trail color).
GHOST_COLORS = {
    "label": ("lightsteelblue", "tab:blue"),
    "model": ("darkseagreen", "tab:green"),
}

# How close a kayak-frame timestamp must be to a paddle-side prediction to
# pair them; larger gaps (RF packet loss) fall back to zero prediction.
_PREDICTION_ALIGNMENT_TOLERANCE = pd.Timedelta(milliseconds=100)

# Batch size for offline ONNX inference. Big enough to amortize onnxruntime
# call overhead; the (batch, 6, window) tensor stays tiny either way.
_INFERENCE_BATCH_SIZE = 2048


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


def _predict_model_series(model_type, paddle_df):
    """Run an exported regression ONNX model over a whole session of paddle IMU.

    This is the offline, batched twin of what MlManager does sample-by-sample
    on the Pi: slide a window over the paddle stream, z-score normalize each
    window with the training-time channel statistics, predict at every sample
    (stride 1).

    Returns a DataFrame with columns:
      timestamp             — paddle sample time the window ends at
      prediction            — raw model output (normalized label units)
      prediction_physical   — prediction * label_norm_scale (physical units)

    The first (window_size - 1) samples can't fill a window; their prediction
    is 0 — the "buffer warming up" state on the Pi.

    Raises FileNotFoundError if the model's exported artifacts are missing.
    """
    import onnxruntime as ort

    onnx_path = CHECKPOINT_DIR / f"{model_type}_best.onnx"
    norm_stats_path = CHECKPOINT_DIR / f"{model_type}_norm_stats.json"
    meta_path = CHECKPOINT_DIR / f"{model_type}_onnx_meta.json"
    for path in (onnx_path, norm_stats_path, meta_path):
        if not path.exists():
            raise FileNotFoundError(
                f"missing {model_type} model artifact {path.name} "
                f"(train + export first: python train.py --model {model_type} ... "
                f"&& python export_onnx.py --model {model_type})")

    with open(meta_path) as f:
        meta = json.load(f)
    window_size = meta["window_size"]
    label_norm_scale = meta["label_norm_scale"]

    with open(norm_stats_path) as f:
        stats = json.load(f)
    channel_means = np.asarray(stats["means"], dtype=np.float32)
    channel_stds = np.asarray(stats["stds"], dtype=np.float32)

    # Drop incomplete RF packets — same cleanup the training dataset does.
    clean = paddle_df.dropna(subset=CHANNEL_NAMES).reset_index(drop=True)
    if len(clean) < window_size:
        return pd.DataFrame(columns=["timestamp", "prediction", "prediction_physical"])

    raw = clean[CHANNEL_NAMES].to_numpy(dtype=np.float32)
    normalized = (raw - channel_means) / channel_stds

    # All windows at stride 1 in one shot: shape (num_windows, channels, window),
    # which is exactly the (batch, channels, time) layout the model expects.
    windows = np.lib.stride_tricks.sliding_window_view(
        normalized, window_size, axis=0)
    num_windows = windows.shape[0]

    onnx_session = ort.InferenceSession(onnx_path.as_posix(),
                                        providers=["CPUExecutionProvider"])
    input_name = onnx_session.get_inputs()[0].name

    predictions = np.zeros(len(clean), dtype=np.float32)
    for start in range(0, num_windows, _INFERENCE_BATCH_SIZE):
        end = min(start + _INFERENCE_BATCH_SIZE, num_windows)
        batch = np.ascontiguousarray(windows[start:end], dtype=np.float32)
        (out,) = onnx_session.run(None, {input_name: batch})
        # Window k covers samples [k .. k + window_size - 1], so its
        # prediction belongs to the sample it ends at.
        predictions[start + window_size - 1:end + window_size - 1] = out

    return pd.DataFrame({
        "timestamp": clean["timestamp"],
        "prediction": predictions,
        "prediction_physical": predictions * label_norm_scale,
    })


def _align_to_kayak(kayak_df, prediction_df):
    """Nearest-in-time match of paddle-side predictions onto kayak timestamps.

    Kayak frames with no prediction within the tolerance (RF dropouts, buffer
    warmup at session start) get 0 — the ghost coasts through those gaps.
    """
    aligned = pd.merge_asof(
        kayak_df[["timestamp"]],
        prediction_df[["timestamp", "prediction_physical"]],
        on="timestamp",
        direction="nearest",
        tolerance=_PREDICTION_ALIGNMENT_TOLERANCE,
    )
    return aligned["prediction_physical"].fillna(0.0)


def _compute_model_ghost(kayak_df, paddle_df, labels_df, labels_cfg, traj_cfg):
    """Ghost trajectory driven by ONNX model predictions.

    Forward accel comes from the assist model (required). Yaw rate comes from
    the turn model if its artifacts exist; otherwise it falls back to the
    de-normalized turn *label* so the ghost still steers plausibly — with a
    printed warning, since heading is then truth-derived, not model-derived.

    Returns the trajectory DataFrame (frame-aligned with the kayak IMU), or
    None if the assist model artifacts are missing or there is no paddle data.
    """
    if paddle_df.empty:
        print("[warn] model ghost disabled: no paddle IMU data in this window")
        return None

    try:
        assist_pred = _predict_model_series("assist", paddle_df)
    except FileNotFoundError as e:
        print(f"[warn] model ghost disabled: {e}")
        return None
    forward_accel = _align_to_kayak(kayak_df, assist_pred)

    try:
        turn_pred = _predict_model_series("turn", paddle_df)
        yaw_rate = _align_to_kayak(kayak_df, turn_pred)
        print("[ok] model ghost: assist + turn predictions")
    except FileNotFoundError as e:
        yaw_rate = labels_df["turn_label"].fillna(0.0) * labels_cfg.turn_norm_scale
        print(f"[warn] model ghost heading falls back to turn labels: {e}")

    ghost_input = pd.DataFrame({
        "timestamp": kayak_df["timestamp"].reset_index(drop=True),
        traj_cfg.forward_accel_column: forward_accel.reset_index(drop=True),
        traj_cfg.yaw_rate_column: yaw_rate.reset_index(drop=True),
    })
    return compute_trajectory(ghost_input, traj_cfg)


def run_playback(session, log_dir, labels_cfg, start_arg, end_arg, playback_speed, ghost_source):
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
    traj_cfg = TrajectoryConfig()
    traj = compute_trajectory(kayak_df, traj_cfg)
    labels_df = compute_labels(kayak_df, labels_cfg)
    print(f"[ok] trajectory: x range [{traj.x.min():.1f}, {traj.x.max():.1f}], "
          f"y range [{traj.y.min():.1f}, {traj.y.max():.1f}]")

    # Ghost trajectories: same integrator, fed by labels and/or model
    # predictions instead of the raw IMU. All boats share the same physics
    # (and therefore the same drift), so divergence between paths reflects
    # source-vs-truth differences rather than integration artifacts.
    # Each entry: {"name", "traj"} — drawing state is attached later.
    ghosts = []
    if ghost_source in ("label", "both"):
        label_ghost = compute_ghost_trajectory_from_labels(
            labels_df, traj_cfg,
            assist_norm_scale=labels_cfg.assist_norm_scale,
            turn_norm_scale=labels_cfg.turn_norm_scale)
        ghosts.append({"name": "label", "traj": label_ghost})
    if ghost_source in ("model", "both"):
        model_ghost = _compute_model_ghost(
            kayak_df, paddle_df, labels_df, labels_cfg, traj_cfg)
        if model_ghost is not None:
            ghosts.append({"name": "model", "traj": model_ghost})
    for ghost in ghosts:
        print(f"[ok] ghost ({ghost['name']}) trajectory: "
              f"x range [{ghost['traj'].x.min():.1f}, {ghost['traj'].x.max():.1f}], "
              f"y range [{ghost['traj'].y.min():.1f}, {ghost['traj'].y.max():.1f}]")

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
    ax_sim.plot(traj.x, traj.y, color="lightgray", linewidth=0.8, label="real (full)")
    recent_trail_line, = ax_sim.plot([], [], color="tab:red", linewidth=2.0, label="real (recent)")
    for ghost in ghosts:
        full_color, recent_color = GHOST_COLORS[ghost["name"]]
        ax_sim.plot(ghost["traj"].x, ghost["traj"].y,
                    color=full_color, linewidth=0.8, linestyle="--",
                    label=f"ghost-{ghost['name']} (full)")
        ghost["recent_line"], = ax_sim.plot(
            [], [], color=recent_color, linewidth=2.0, alpha=0.7,
            label=f"ghost-{ghost['name']} (recent)")
    ax_sim.set_aspect("equal", adjustable="datalim")
    ax_sim.set_title("Kayak path (top-down, integrated from IMU)")
    ax_sim.set_xlabel("x (drift units, not metric)")
    ax_sim.set_ylabel("y (drift units, not metric)")
    ax_sim.grid(True, alpha=0.3)
    ax_sim.legend(loc="upper left", fontsize=8)

    # Boat marker(s) — recreated each frame because FancyArrow doesn't
    # support in-place updates of position+rotation cleanly.
    boat_state = {"real": _draw_boat(ax_sim, traj.x.iloc[0], traj.y.iloc[0],
                                     traj.heading_rad.iloc[0], color="tab:red")}
    for ghost in ghosts:
        boat_state[ghost["name"]] = _draw_boat(
            ax_sim, ghost["traj"].x.iloc[0], ghost["traj"].y.iloc[0],
            ghost["traj"].heading_rad.iloc[0], color=GHOST_COLORS[ghost["name"]][1])

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

        # Update real boat (recreate the patch — cheap).
        boat_state["real"].remove()
        boat_state["real"] = _draw_boat(
            ax_sim, traj.x.iloc[i], traj.y.iloc[i], traj.heading_rad.iloc[i],
            color="tab:red")

        # Update brighter "recent" trail for the real boat.
        lo = max(0, i - recent_n)
        recent_trail_line.set_data(traj.x.iloc[lo:i + 1], traj.y.iloc[lo:i + 1])

        # Update each active ghost boat and its trail.
        for ghost in ghosts:
            g_traj = ghost["traj"]
            boat_state[ghost["name"]].remove()
            boat_state[ghost["name"]] = _draw_boat(
                ax_sim, g_traj.x.iloc[i], g_traj.y.iloc[i],
                g_traj.heading_rad.iloc[i],
                color=GHOST_COLORS[ghost["name"]][1])
            ghost["recent_line"].set_data(
                g_traj.x.iloc[lo:i + 1], g_traj.y.iloc[lo:i + 1])

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
    ap.add_argument("--ghost", choices=GHOST_CHOICES, default="label",
                    help="Ghost-boat overlay source. 'label' drives the ghost "
                         "from regression labels, 'model' from the exported "
                         "ONNX model predictions, 'both' overlays the two, "
                         "'off' hides them.")
    args = ap.parse_args()

    session, log_dir, labels_cfg = _load_yaml(args.config)
    run_playback(session, log_dir, labels_cfg, args.start, args.end, args.speed, args.ghost)


if __name__ == "__main__":
    main()
