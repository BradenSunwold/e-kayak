#!/usr/bin/env bash
# Sync Grafana dashboard JSON files from the git repo to the Grafana provisioning directory.
# Run this after editing any dashboard JSON in grafana/dashboards/.
#
# Usage: sudo bash grafana/sync_dashboards.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/dashboards"
DEST="/var/lib/grafana/dashboards/ekayak"

# --delete removes dashboards from the provisioning dir that no longer exist
# in the repo (e.g. the retired stroke_detector dashboard).
rsync -av --delete --include='*.json' --exclude='*' "${SRC}/" "${DEST}/"
chown -R grafana:grafana "${DEST}"
systemctl restart grafana-server

echo "Dashboards synced to ${DEST} and Grafana restarted."
