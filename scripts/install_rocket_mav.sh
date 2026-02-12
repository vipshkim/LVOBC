#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="${TOOLS_DIR:-$HOME/Tools}"
DAEMON_DIR="${DAEMON_DIR:-$HOME/.local/libexec/rocket-mav}"
USER_CFG_DIR="${USER_CFG_DIR:-$HOME/.config/rocket-mav}"
USER_SYSTEMD_DIR="${USER_SYSTEMD_DIR:-$HOME/.config/systemd/user}"
DAEMON_APP_NAMES=("monitoringd" "stream_odometry" "stream_commander")

INSTALL_SYSTEM_CONFIG=0
INSTALL_ROUTER_SAMPLE=0
ENABLE_MONITORING_SERVICE=0
ENABLE_STREAM_SERVICES=0
FORCE_OVERWRITE=0
ADD_TOOLS_PATH=0
SHELL_RC_FILE="${SHELL_RC_FILE:-$HOME/.bashrc}"

usage() {
  cat <<'EOF'
Usage: install_rocket_mav.sh [options]

Build and install runtime files for rocket MAVLink tools.

Options:
  --tools-dir <path>          Install binaries to this directory (default: ~/Tools)
  --daemon-dir <path>         Install daemon binaries to this directory (default: ~/.local/libexec/rocket-mav)
  --user-config-dir <path>    Install user config to this directory (default: ~/.config/rocket-mav)
  --install-system-config     Install ports.env + rate.env + servo.env to /etc/rocket-mav/
  --install-router-sample     Install router sample to /etc/mavlink-router/main.conf.rocket.sample
  --enable-monitoring-service Create/enable user systemd service for monitoringd
  --enable-stream-services    Create/enable user systemd services for stream_odometry and stream_commander
  --add-tools-path            Add tools directory to PATH in shell rc (default: ~/.bashrc)
  --shell-rc <path>           Shell rc file path used with --add-tools-path
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

ensure_tools_path_in_shell_rc() {
  local tools_dir="$1"
  local rc_file="$2"
  local start_mark="# >>> rocket-mav tools path >>>"
  local end_mark="# <<< rocket-mav tools path <<<"

  mkdir -p "$(dirname "$rc_file")"
  touch "$rc_file"

  if grep -Fq "$start_mark" "$rc_file"; then
    # Replace existing managed block so tools-dir changes are reflected.
    awk -v s="$start_mark" -v e="$end_mark" '
      BEGIN { skip = 0 }
      $0 == s { skip = 1; next }
      $0 == e { skip = 0; next }
      !skip { print }
    ' "$rc_file" > "${rc_file}.tmp"
    mv "${rc_file}.tmp" "$rc_file"
  fi

  {
    printf '\n%s\n' "$start_mark"
    printf 'if [ -d "%s" ]; then\n' "$tools_dir"
    printf '  case ":$PATH:" in\n'
    printf '    *:"%s":*) ;;\n' "$tools_dir"
    printf '    *) export PATH="%s:$PATH" ;;\n' "$tools_dir"
    printf '  esac\n'
    printf 'fi\n'
    printf '%s\n' "$end_mark"
  } >> "$rc_file"

  log "Updated PATH in shell rc: $rc_file"
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
    --daemon-dir)
      DAEMON_DIR="$2"; shift 2 ;;
    --user-config-dir)
      USER_CFG_DIR="$2"; shift 2 ;;
    --install-system-config)
      INSTALL_SYSTEM_CONFIG=1; shift ;;
    --install-router-sample)
      INSTALL_ROUTER_SAMPLE=1; shift ;;
    --enable-monitoring-service)
      ENABLE_MONITORING_SERVICE=1; shift ;;
    --enable-stream-services)
      ENABLE_STREAM_SERVICES=1; shift ;;
    --add-tools-path)
      ADD_TOOLS_PATH=1; shift ;;
    --shell-rc)
      SHELL_RC_FILE="$2"; shift 2 ;;
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

mkdir -p "$TOOLS_DIR" "$USER_CFG_DIR" "$DAEMON_DIR"

log "Building binaries (Makefile)..."
make -C "$ROOT_DIR" install TOOLS_DIR="$TOOLS_DIR"

log "Installing daemon binaries..."
for daemon in "${DAEMON_APP_NAMES[@]}"; do
  src="$ROOT_DIR/build/bin/$daemon"
  if [[ -x "$src" ]]; then
    install -m 0755 "$src" "$DAEMON_DIR/"
    log "Installed daemon: $DAEMON_DIR/$daemon"
  else
    warn "Missing daemon binary: $src"
  fi
done

log "Installing user config..."
copy_with_guard "$ROOT_DIR/config/ports.env.sample" "$USER_CFG_DIR/ports.env"
copy_with_guard "$ROOT_DIR/config/rate.env.sample" "$USER_CFG_DIR/rate.env"
copy_with_guard "$ROOT_DIR/config/servo.env.sample" "$USER_CFG_DIR/servo.env"

if [[ "$INSTALL_SYSTEM_CONFIG" -eq 1 ]]; then
  run_as_root_install "$ROOT_DIR/config/ports.env.sample" "/etc/rocket-mav/ports.env"
  run_as_root_install "$ROOT_DIR/config/rate.env.sample" "/etc/rocket-mav/rate.env"
  run_as_root_install "$ROOT_DIR/config/servo.env.sample" "/etc/rocket-mav/servo.env"
fi

if [[ "$INSTALL_ROUTER_SAMPLE" -eq 1 ]]; then
  run_as_root_install "$ROOT_DIR/config/mavlink-router.main.conf.sample" "/etc/mavlink-router/main.conf.rocket.sample"
fi

if [[ "$ENABLE_MONITORING_SERVICE" -eq 1 ]]; then
  mkdir -p "$USER_SYSTEMD_DIR"
  cat >"$USER_SYSTEMD_DIR/monitoringd.service" <<EOF
[Unit]
Description=Rocket MAV monitoring daemon
After=network-online.target

[Service]
Type=simple
ExecStart=$DAEMON_DIR/monitoringd
Restart=always
RestartSec=1

[Install]
WantedBy=default.target
EOF
  log "Installed user service: $USER_SYSTEMD_DIR/monitoringd.service"
fi

if [[ "$ENABLE_STREAM_SERVICES" -eq 1 ]]; then
  mkdir -p "$USER_SYSTEMD_DIR"
  cat >"$USER_SYSTEMD_DIR/stream_odometry.service" <<EOF
[Unit]
Description=Rocket MAV odometry stream daemon
After=network-online.target

[Service]
Type=simple
ExecStart=$DAEMON_DIR/stream_odometry
Restart=always
RestartSec=1

[Install]
WantedBy=default.target
EOF
  log "Installed user service: $USER_SYSTEMD_DIR/stream_odometry.service"

  cat >"$USER_SYSTEMD_DIR/stream_commander.service" <<EOF
[Unit]
Description=Rocket MAV commander stream daemon
After=network-online.target

[Service]
Type=simple
ExecStart=$DAEMON_DIR/stream_commander
Restart=always
RestartSec=1

[Install]
WantedBy=default.target
EOF
  log "Installed user service: $USER_SYSTEMD_DIR/stream_commander.service"
fi

if [[ "$ENABLE_MONITORING_SERVICE" -eq 1 || "$ENABLE_STREAM_SERVICES" -eq 1 ]]; then
  if command -v systemctl >/dev/null 2>&1; then
    systemctl --user daemon-reload || true
    if [[ "$ENABLE_MONITORING_SERVICE" -eq 1 ]]; then
      systemctl --user enable --now monitoringd.service || true
      log "Requested enable/start for user service: monitoringd.service"
    fi
    if [[ "$ENABLE_STREAM_SERVICES" -eq 1 ]]; then
      systemctl --user enable --now stream_odometry.service || true
      systemctl --user enable --now stream_commander.service || true
      log "Requested enable/start for user services: stream_odometry.service, stream_commander.service"
    fi
  else
    warn "systemctl not found, skipped service enable"
  fi
fi

if [[ "$ADD_TOOLS_PATH" -eq 1 ]]; then
  ensure_tools_path_in_shell_rc "$TOOLS_DIR" "$SHELL_RC_FILE"
fi

cat <<EOF

Install completed.
- Binaries: $TOOLS_DIR
- User config: $USER_CFG_DIR/ports.env
- User config: $USER_CFG_DIR/rate.env
- User config: $USER_CFG_DIR/servo.env
$(if [[ "$ADD_TOOLS_PATH" -eq 1 ]]; then printf -- "- PATH updated in: %s\n" "$SHELL_RC_FILE"; fi)

Quick checks:
  $TOOLS_DIR/scan --help
  $TOOLS_DIR/monitoring --help
  $TOOLS_DIR/monitoringd --help
  $TOOLS_DIR/servo_test --help
  $TOOLS_DIR/motor_init --help
  $TOOLS_DIR/stream_odometry --help
  $TOOLS_DIR/stream_commander --help
  $TOOLS_DIR/stream_odometry_test --help
  $TOOLS_DIR/stream_commander_test --help

EOF
