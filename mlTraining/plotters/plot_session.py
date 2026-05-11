"""
Offline interactive plotter for kayak training sessions.

Usage:
    python -m plotters.plot_session plotters/configs/raw_signals.yaml
    python -m plotters.plot_session plotters/configs/raw_signals.yaml \
        --start 09:33:00 --end 09:34:00
    python -m plotters.plot_session plotters/configs/raw_signals.yaml \
        --start 30 --end 90        # seconds from session start

Config schema (YAML):
    session: "2026-05-04_09:32.46"          # log filename suffix
    log_dir: "/abs/path/to/logs"
    plots:
      - title: "Paddle Accel"
        source: rf                          # imu | rf | ml | motor
        signals: [accel_x, accel_y, accel_z]
        # optional: kind: line (default) | step | scatter
        # optional: ylabel: "m/s^2"
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import yaml

from plotters import parsers


SOURCE_TO_PREFIX = {
    "imu": "imuLog",
    "rf": "rfLog",
    "ml": "mlLog",
    "motor": "motorLog",
}


@dataclass
class PlotSpec:
    title: str
    source: str
    signals: list[str]
    kind: str = "line"  # line | step | scatter
    ylabel: str | None = None


def _load_config(path: Path) -> tuple[str, Path, list[PlotSpec]]:
    with open(path) as fh:
        cfg = yaml.safe_load(fh)
    session = cfg["session"]
    log_dir = Path(cfg["log_dir"]).expanduser()
    plots = [PlotSpec(**p) for p in cfg["plots"]]
    return session, log_dir, plots


def _log_path(log_dir: Path, source: str, session: str) -> Path:
    return log_dir / f"{SOURCE_TO_PREFIX[source]}_{session}.log"


def _parse_time_arg(arg: str | None, session_start: pd.Timestamp) -> pd.Timestamp | None:
    """Accepts:
      - HH:MM:SS or HH:MM:SS.fff       → absolute time on the session date
      - bare number (int/float)        → seconds offset from session_start
      - None                           → None
    """
    if arg is None:
        return None
    try:
        offset = float(arg)
        return session_start + pd.Timedelta(seconds=offset)
    except ValueError:
        pass
    date = session_start.strftime("%Y-%m-%d")
    return pd.to_datetime(f"{date} {arg}")


def _slice(df: pd.DataFrame, start: pd.Timestamp | None, end: pd.Timestamp | None) -> pd.DataFrame:
    if df.empty:
        return df
    mask = pd.Series(True, index=df.index)
    if start is not None:
        mask &= df["timestamp"] >= start
    if end is not None:
        mask &= df["timestamp"] <= end
    return df.loc[mask].reset_index(drop=True)


def _draw(ax: plt.Axes, df: pd.DataFrame, spec: PlotSpec) -> None:
    if df.empty:
        ax.text(0.5, 0.5, f"(no data for {spec.source})",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_title(spec.title)
        return

    ts = df["timestamp"]
    for sig in spec.signals:
        if sig not in df.columns:
            ax.plot([], [], label=f"{sig} (missing)")
            continue

        # Drop rows where this signal is NaN (motor log fields are sparse —
        # most rows only carry one of mode/rpm/heading/fin).
        valid = df[["timestamp", sig]].dropna(subset=[sig])
        if valid.empty:
            ax.plot([], [], label=f"{sig} (no data)")
            continue
        v_ts = valid["timestamp"]
        series = valid[sig]

        first_val = series.iloc[0]
        is_categorical = isinstance(first_val, (str, bytes))

        if is_categorical:
            categories = list(dict.fromkeys(series.tolist()))
            cat_to_int = {c: i for i, c in enumerate(categories)}
            y = series.map(cat_to_int)
            ax.step(v_ts, y, where="post", label=sig)
            ax.set_yticks(range(len(categories)))
            ax.set_yticklabels(categories)
        elif spec.kind == "step":
            ax.step(v_ts, series, where="post", label=sig)
        elif spec.kind == "scatter":
            ax.scatter(v_ts, series, s=6, label=sig)
        else:
            ax.plot(v_ts, series, label=sig, linewidth=0.9)

    ax.set_title(spec.title)
    if spec.ylabel:
        ax.set_ylabel(spec.ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)


def main() -> None:
    ap = argparse.ArgumentParser(description="Offline kayak session plotter.")
    ap.add_argument("config", type=Path, help="Path to YAML plot config.")
    ap.add_argument("--start", default=None,
                    help="Start of time window. HH:MM:SS or seconds-from-start.")
    ap.add_argument("--end", default=None,
                    help="End of time window. HH:MM:SS or seconds-from-start.")
    args = ap.parse_args()

    session, log_dir, specs = _load_config(args.config)

    # Parse every required source once and cache
    needed_sources = sorted({s.source for s in specs})
    dfs: dict[str, pd.DataFrame] = {}
    for src in needed_sources:
        path = _log_path(log_dir, src, session)
        if not path.exists():
            print(f"[warn] {src} log missing: {path}")
            dfs[src] = pd.DataFrame()
            continue
        dfs[src] = parsers.parse(src, path)
        print(f"[ok] parsed {src}: {len(dfs[src])} rows from {path.name}")

    # Determine session start from earliest timestamp across parsed logs
    starts = [df["timestamp"].iloc[0] for df in dfs.values() if not df.empty]
    if not starts:
        raise SystemExit("No data parsed from any log; nothing to plot.")
    session_start = min(starts)
    start_ts = _parse_time_arg(args.start, session_start)
    end_ts = _parse_time_arg(args.end, session_start)

    fig, axes = plt.subplots(
        len(specs), 1,
        figsize=(13, max(2.2 * len(specs), 4)),
        sharex=True,
    )
    if len(specs) == 1:
        axes = [axes]

    for ax, spec in zip(axes, specs):
        df = _slice(dfs[spec.source], start_ts, end_ts)
        _draw(ax, df, spec)

    axes[-1].set_xlabel("time")
    fig.suptitle(f"Session {session}", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


if __name__ == "__main__":
    main()
