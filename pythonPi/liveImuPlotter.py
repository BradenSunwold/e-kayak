import re
import glob
import os
import json
import datetime as dt
from flask import Flask, Response, render_template_string

log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def get_latest_log():
    rf_logs = sorted(glob.glob(os.path.join(log_dir, "rfLog_*.log")))
    return rf_logs[-1] if rf_logs else None

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


class LogTailer:
    def __init__(self):
        self.path = None
        self.pos = 0
        self.partial = ""

    def read_new_lines(self):
        latest = get_latest_log()
        if latest is None:
            return []
        # new log file appeared — switch to it
        if latest != self.path:
            self.path = latest
            self.pos = 0
            self.partial = ""
            print(f"Tailing log: {self.path}")
        size = os.path.getsize(self.path)
        if size <= self.pos:
            return []
        with open(self.path, "r") as f:
            f.seek(self.pos)
            raw = f.read(size - self.pos)
            self.pos = size
        text = self.partial + raw
        lines = text.split("\n")
        self.partial = lines.pop()
        return lines


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


tailer = LogTailer()
app = Flask(__name__)

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Live IMU Plotter</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns"></script>
    <style>
        body { margin: 20px; background: #1a1a1a; color: #eee; font-family: sans-serif; }
        canvas { background: #222; border-radius: 8px; margin-bottom: 20px; }
        h1 { font-size: 1.2em; margin-bottom: 5px; }
        .status { font-size: 0.85em; color: #888; margin-bottom: 15px; }
    </style>
</head>
<body>
    <h1>Live IMU Plotter</h1>
    <div class="status" id="logStatus">Waiting for log file... | Polling every 50ms</div>
    <canvas id="rpyChart" height="120"></canvas>
    <canvas id="accelGyroChart" height="120"></canvas>
    <script>
    const MAXLEN = 500;
    const commonOpts = {
        animation: false,
        responsive: true,
        scales: {
            x: {
                type: 'time',
                time: { tooltipFormat: 'HH:mm:ss', displayFormats: { second: 'HH:mm:ss' } },
                ticks: { color: '#aaa' },
                grid: { color: '#333' }
            },
            y: { ticks: { color: '#aaa' }, grid: { color: '#333' } }
        },
        plugins: { legend: { labels: { color: '#ccc' } } }
    };

    function makeDataset(label, color, dashed) {
        return { label, borderColor: color, backgroundColor: color + '33',
                 data: [], borderWidth: 2, pointRadius: 0,
                 borderDash: dashed ? [5, 3] : [] };
    }

    const rpyChart = new Chart(document.getElementById('rpyChart'), {
        type: 'line',
        data: {
            datasets: [
                makeDataset('Roll', '#ff6384', false),
                makeDataset('Pitch', '#36a2eb', false),
                makeDataset('Yaw', '#ffce56', false)
            ]
        },
        options: { ...commonOpts, scales: { ...commonOpts.scales,
            y: { ...commonOpts.scales.y, title: { display: true, text: 'Degrees', color: '#aaa' } } } }
    });

    const agChart = new Chart(document.getElementById('accelGyroChart'), {
        type: 'line',
        data: {
            datasets: [
                makeDataset('Ax', '#ff6384', false),
                makeDataset('Ay', '#36a2eb', false),
                makeDataset('Az', '#ffce56', false),
                makeDataset('Gx', '#ff6384', true),
                makeDataset('Gy', '#36a2eb', true),
                makeDataset('Gz', '#ffce56', true)
            ]
        },
        options: { ...commonOpts, scales: { ...commonOpts.scales,
            y: { ...commonOpts.scales.y, title: { display: true, text: 'Accel / Gyro', color: '#aaa' } } } }
    });

    const keyMap = {
        roll: [rpyChart, 0], pitch: [rpyChart, 1], yaw: [rpyChart, 2],
        ax: [agChart, 0], ay: [agChart, 1], az: [agChart, 2],
        gx: [agChart, 3], gy: [agChart, 4], gz: [agChart, 5]
    };

    async function poll() {
        try {
            const resp = await fetch('/data');
            const result = await resp.json();
            if (result.log_file) {
                document.getElementById('logStatus').textContent =
                    'Log: ' + result.log_file + ' | Polling every 50ms';
            }
            for (const p of result.points) {
                const [chart, idx] = keyMap[p.key];
                const ds = chart.data.datasets[idx];
                ds.data.push({ x: new Date(p.ts), y: p.value });
                if (ds.data.length > MAXLEN) ds.data.shift();
            }
            rpyChart.update();
            agChart.update();
        } catch (e) {}
        setTimeout(poll, 50);
    }
    poll();
    </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/data")
def data():
    points = []
    for line in tailer.read_new_lines():
        ts, val = parse_line(line)
        if ts and val:
            key, value = val
            points.append({"ts": ts.isoformat(), "key": key, "value": value})
    log_name = os.path.basename(tailer.path) if tailer.path else None
    return Response(json.dumps({"points": points, "log_file": log_name}), mimetype="application/json")


if __name__ == "__main__":
    print("Open http://localhost:5000 in your browser")
    print("(Use: ssh -L 5000:localhost:5000 bradensunwold@<pi-ip>)")
    app.run(host="0.0.0.0", port=5000)
