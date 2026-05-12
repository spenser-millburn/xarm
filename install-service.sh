#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="xarm"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
PYTHON_BIN="$(command -v python3)"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ $EUID -ne 0 ]]; then
    echo "Re-running with sudo..."
    exec sudo -E "$0" "$@"
fi

echo "Installing ${SERVICE_NAME}.service"
echo "  WorkingDirectory: ${INSTALL_DIR}"
echo "  User:             ${RUN_USER}"
echo "  Python:           ${PYTHON_BIN}"

cat > "${UNIT_PATH}" <<EOF
[Unit]
Description=xArm6 FastAPI Control Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${INSTALL_DIR}
ExecStart=${PYTHON_BIN} ${INSTALL_DIR}/main.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
systemctl restart "${SERVICE_NAME}.service"

echo ""
echo "Installed. Useful commands:"
echo "  systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"
echo "  sudo systemctl restart ${SERVICE_NAME}"
