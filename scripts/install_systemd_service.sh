#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

SERVICE_NAME="dc-ocpp"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
RUNNER="${REPO_DIR}/scripts/dc_ocpp_run.sh"
CONFIG="${REPO_DIR}/configs/charger.json"
USER_NAME="${SUDO_USER:-$(whoami)}"

if [[ ! -x "${RUNNER}" ]]; then
  echo "Runner script not found or not executable: ${RUNNER}" >&2
  exit 1
fi

cat <<SERVICE_EOF | sudo tee "${SERVICE_PATH}" > /dev/null
[Unit]
Description=Joulepoint DC OCPP
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER_NAME}
WorkingDirectory=${REPO_DIR}
Environment=DC_OCPP_CONFIG=${CONFIG}
ExecStart=${RUNNER}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICE_EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}.service"

echo "Installed and started ${SERVICE_NAME}.service"
