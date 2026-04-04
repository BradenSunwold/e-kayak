#!/usr/bin/env bash
# setup_influx_grafana.sh — Install and configure InfluxDB 2.x + Grafana on Raspberry Pi 5
# Usage: sudo bash setup_influx_grafana.sh
set -euo pipefail

# ── Configurable defaults ────────────────────────────────────────────────────
INFLUX_ORG="ekayak"
INFLUX_BUCKET="telemetry"
INFLUX_RETENTION="2w"          # 2 weeks; change to e.g. "30d", "0" for infinite
INFLUX_USER="admin"
INFLUX_PASSWORD="ekayak-admin" # Change after initial setup if desired

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/config/influxdb_setup.json"

echo "=== E-Kayak Telemetry Stack Setup ==="
echo "  InfluxDB 2.x + Grafana OSS"
echo "  Target: Raspberry Pi 5 (aarch64 / Debian Bookworm)"
echo ""

# ── Must run as root ─────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    echo "ERROR: Please run with sudo:  sudo bash $0"
    exit 1
fi

# ── 1. Install InfluxDB 2.x ─────────────────────────────────────────────────
echo "--- Installing InfluxDB 2.x ---"
if ! command -v influxd &>/dev/null; then
    # Add InfluxData repo
    curl -s https://repos.influxdata.com/influxdata-archive.key | gpg --dearmor -o /usr/share/keyrings/influxdb-archive-keyring.gpg
    echo "deb [signed-by=/usr/share/keyrings/influxdb-archive-keyring.gpg] https://repos.influxdata.com/debian stable main" \
        > /etc/apt/sources.list.d/influxdb.list
    apt-get update -qq
    apt-get install -y -qq influxdb2
    echo "InfluxDB installed."
else
    echo "InfluxDB already installed: $(influxd version 2>&1 | head -1)"
fi

# Enable and start InfluxDB
systemctl enable influxdb
systemctl start influxdb

# Wait for InfluxDB to be ready
echo -n "Waiting for InfluxDB to start..."
for i in $(seq 1 30); do
    if curl -s http://localhost:8086/health | grep -q '"status":"pass"'; then
        echo " ready."
        break
    fi
    if [[ $i -eq 30 ]]; then
        echo " TIMEOUT — check: journalctl -u influxdb"
        exit 1
    fi
    sleep 1
    echo -n "."
done

# ── 2. Initial InfluxDB setup ───────────────────────────────────────────────
echo "--- Configuring InfluxDB ---"
# Check if already set up
HEALTH=$(curl -s http://localhost:8086/api/v2/setup)
if echo "$HEALTH" | grep -q '"allowed":true'; then
    echo "Running initial setup (org=${INFLUX_ORG}, bucket=${INFLUX_BUCKET}, retention=${INFLUX_RETENTION})..."
    SETUP_RESULT=$(influx setup \
        --org "${INFLUX_ORG}" \
        --bucket "${INFLUX_BUCKET}" \
        --username "${INFLUX_USER}" \
        --password "${INFLUX_PASSWORD}" \
        --retention "${INFLUX_RETENTION}" \
        --force 2>&1)
    echo "${SETUP_RESULT}"
else
    echo "InfluxDB already configured."
fi

# Get the API token
echo "Retrieving API token..."
INFLUX_TOKEN=$(influx auth list --json 2>/dev/null | python3 -c "
import sys, json
auths = json.load(sys.stdin)
for a in auths:
    if 'all access' in a.get('description','').lower() or a.get('permissions'):
        print(a['token'])
        break
" 2>/dev/null || echo "")

if [[ -z "$INFLUX_TOKEN" ]]; then
    echo "WARNING: Could not auto-detect token. Run 'influx auth list' to find it."
else
    echo "Token retrieved successfully."
fi

# ── 3. Install Grafana OSS ──────────────────────────────────────────────────
echo ""
echo "--- Installing Grafana OSS ---"
if ! command -v grafana-server &>/dev/null; then
    # Add Grafana repo
    curl -s https://apt.grafana.com/gpg.key | gpg --dearmor -o /usr/share/keyrings/grafana-archive-keyring.gpg
    echo "deb [signed-by=/usr/share/keyrings/grafana-archive-keyring.gpg] https://apt.grafana.com stable main" \
        > /etc/apt/sources.list.d/grafana.list
    apt-get update -qq
    apt-get install -y -qq grafana
    echo "Grafana installed."
else
    echo "Grafana already installed."
fi

# Enable and start Grafana
systemctl enable grafana-server
systemctl start grafana-server

echo -n "Waiting for Grafana to start..."
for i in $(seq 1 30); do
    if curl -s http://localhost:3000/api/health | grep -q '"database":"ok"'; then
        echo " ready."
        break
    fi
    if [[ $i -eq 30 ]]; then
        echo " TIMEOUT — check: journalctl -u grafana-server"
        exit 1
    fi
    sleep 1
    echo -n "."
done

# ── 4. Provision Grafana datasource for InfluxDB ────────────────────────────
echo "--- Provisioning Grafana datasource ---"
GRAFANA_DS_DIR="/etc/grafana/provisioning/datasources"
mkdir -p "${GRAFANA_DS_DIR}"

cat > "${GRAFANA_DS_DIR}/influxdb-ekayak.yaml" <<DSEOF
apiVersion: 1
datasources:
  - name: InfluxDB-eKayak
    type: influxdb
    access: proxy
    url: http://localhost:8086
    jsonData:
      version: Flux
      organization: ${INFLUX_ORG}
      defaultBucket: ${INFLUX_BUCKET}
    secureJsonData:
      token: ${INFLUX_TOKEN}
    isDefault: true
    editable: true
DSEOF

# Restart Grafana to pick up datasource
systemctl restart grafana-server
echo "Grafana datasource provisioned."

# ── 5. Save config for Python code to use ───────────────────────────────────
echo "--- Saving config for Python application ---"
mkdir -p "$(dirname "${CONFIG_FILE}")"
cat > "${CONFIG_FILE}" <<CFEOF
{
    "url": "http://localhost:8086",
    "org": "${INFLUX_ORG}",
    "bucket": "${INFLUX_BUCKET}",
    "token": "${INFLUX_TOKEN}",
    "retention": "${INFLUX_RETENTION}"
}
CFEOF
chmod 644 "${CONFIG_FILE}"
echo "Config saved to ${CONFIG_FILE}"

# ── 6. Summary ──────────────────────────────────────────────────────────────
PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "==========================================="
echo "  Setup complete!"
echo "==========================================="
echo ""
echo "  InfluxDB:  http://${PI_IP}:8086"
echo "    User: ${INFLUX_USER}"
echo "    Pass: ${INFLUX_PASSWORD}"
echo "    Org:  ${INFLUX_ORG}"
echo "    Bucket: ${INFLUX_BUCKET} (retention: ${INFLUX_RETENTION})"
echo ""
echo "  Grafana:   http://${PI_IP}:3000"
echo "    Default login: admin / admin"
echo "    (Grafana will prompt you to change the password)"
echo ""
echo "  Datasource 'InfluxDB-eKayak' auto-provisioned."
echo ""
echo "  Python config: ${CONFIG_FILE}"
echo "==========================================="
