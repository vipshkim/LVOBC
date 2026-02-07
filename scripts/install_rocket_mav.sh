#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="${TOOLS_DIR:-$HOME/Tools}"
USER_CFG_DIR="${USER_CFG_DIR:-$HOME/.config/rocket-mav}"
USER_SYSTEMD_DIR="${USER_SYSTEMD_DIR:-$HOME/.config/systemd/user}"

INSTALL_SYSTEM_CONFIG=0
INSTALL_ROUTER_SAMPLE=0
ENABLE_BATTD_SERVICE=0
FORCE_OVERWRITE=0

usage() {
  cat <<'EOF'
Usage: install_rocket_mav.sh [options]

Build and install runtime files for rocket MAVLink tools.

Options:
  --tools-dir <path>          Install binaries to this directory (default: ~/Tools)
  --user-config-dir <path>    Install user config to this directory (default: ~/.config/rocket-mav)
  --install-system-config     Install ports.env to /etc/rocket-mav/ports.env
  --install-router-sample     Install router sample to /etc/mavlink-router/main.conf.rocket.sample
  --enable-battd-service      Create/enable user systemd service for mav_battd
  -f, --force                 Overwrite existing config files
  -h, --help                  Show this help
EOF
}

log() { printf '[INFO] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*" >&2; }

copy_with_guard() {
  local src="$1"
  local dst="$2"

  if [[ -e "$dst" && "$FORCE_OVERWRITE" -ne 1 ]]; then
    warn "Skip existing: $dst (use --force to overwrite)"
    return
  fi
  install -D -m 0644 "$src" "$dst"
  log "Installed: $dst"
}

run_as_root_install() {
  local src="$1"
  local dst="$2"

  if [[ $EUID -eq 0 ]]; then
    install -D -m 0644 "$src" "$dst"
  else
    sudo install -D -m 0644 "$src" "$dst"
  fi
  log "Installed (system): $dst"
}

build_binary() {
  local src="$1"
  local out="$2"
  local extra_flags=("${@:3}")

  gcc "$src" \
    -I "$ROOT_DIR/mavlink_lib/common" \
    -I "$ROOT_DIR/rocket_mav_common" \
    "${extra_flags[@]}" \
    -o "$out"
  log "Built: $out"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tools-dir)
      TOOLS_DIR="$2"; shift 2 ;;
    --user-config-dir)
      USER_CFG_DIR="$2"; shift 2 ;;
    --install-system-config)
      INSTALL_SYSTEM_CONFIG=1; shift ;;
    --install-router-sample)
      INSTALL_ROUTER_SAMPLE=1; shift ;;
    --enable-battd-service)
      ENABLE_BATTD_SERVICE=1; shift ;;
    -f|--force)
      FORCE_OVERWRITE=1; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      warn "Unknown option: $1"
      usage
      exit 1 ;;
  esac
done

mkdir -p "$TOOLS_DIR" "$USER_CFG_DIR"

log "Building binaries..."
build_binary "$ROOT_DIR/scan/scan_main.c" "$TOOLS_DIR/scan" -lm
build_binary "$ROOT_DIR/servo_test/servo_test.c" "$TOOLS_DIR/servo_test" -lm
build_binary "$ROOT_DIR/motor_init/motor_init.c" "$TOOLS_DIR/motor_init" -lm
build_binary "$ROOT_DIR/mav_batt/mav_battd.c" "$TOOLS_DIR/mav_battd"
build_binary "$ROOT_DIR/mav_batt/mav_batt.c" "$TOOLS_DIR/mav_batt"

log "Installing user config..."
copy_with_guard "$ROOT_DIR/config/ports.env.sample" "$USER_CFG_DIR/ports.env"

if [[ "$INSTALL_SYSTEM_CONFIG" -eq 1 ]]; then
  run_as_root_install "$ROOT_DIR/config/ports.env.sample" "/etc/rocket-mav/ports.env"
fi

if [[ "$INSTALL_ROUTER_SAMPLE" -eq 1 ]]; then
  run_as_root_install "$ROOT_DIR/config/mavlink-router.main.conf.sample" "/etc/mavlink-router/main.conf.rocket.sample"
fi

if [[ "$ENABLE_BATTD_SERVICE" -eq 1 ]]; then
  mkdir -p "$USER_SYSTEMD_DIR"
  cat >"$USER_SYSTEMD_DIR/mav_battd.service" <<EOF
[Unit]
Description=Rocket MAV battery monitor daemon
After=network-online.target

[Service]
Type=simple
ExecStart=$TOOLS_DIR/mav_battd
Restart=always
RestartSec=1

[Install]
WantedBy=default.target
EOF
  log "Installed user service: $USER_SYSTEMD_DIR/mav_battd.service"
  if command -v systemctl >/dev/null 2>&1; then
    systemctl --user daemon-reload || true
    systemctl --user enable --now mav_battd.service || true
    log "Requested enable/start for user service: mav_battd.service"
  else
    warn "systemctl not found, skipped service enable"
  fi
fi

cat <<EOF

Install completed.
- Binaries: $TOOLS_DIR
- User config: $USER_CFG_DIR/ports.env

Quick checks:
  $TOOLS_DIR/scan --help
  $TOOLS_DIR/servo_test --help
  $TOOLS_DIR/motor_init --help

EOF
