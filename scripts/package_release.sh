#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIST_DIR="$ROOT_DIR/dist"
STAMP="$(date +%Y%m%d_%H%M%S)"
PKG_NAME="rocket-mav-${STAMP}"
PKG_DIR="$DIST_DIR/$PKG_NAME"
ARCHIVE="$DIST_DIR/${PKG_NAME}.tar.gz"

mkdir -p "$PKG_DIR"

copy_item() {
  local src="$1"
  local dst="$2"
  if [[ -d "$src" ]]; then
    cp -a "$src" "$dst"
  else
    cp -a "$src" "$dst"
  fi
}

copy_item "$ROOT_DIR/README.md" "$PKG_DIR/"
copy_item "$ROOT_DIR/GIT_GUIDE.md" "$PKG_DIR/"
copy_item "$ROOT_DIR/config" "$PKG_DIR/"
copy_item "$ROOT_DIR/scripts" "$PKG_DIR/"
copy_item "$ROOT_DIR/monitoring" "$PKG_DIR/"
copy_item "$ROOT_DIR/scan" "$PKG_DIR/"
copy_item "$ROOT_DIR/servo_test" "$PKG_DIR/"
copy_item "$ROOT_DIR/motor_init" "$PKG_DIR/"
copy_item "$ROOT_DIR/rocket_mav_common" "$PKG_DIR/"

tar -C "$DIST_DIR" -czf "$ARCHIVE" "$PKG_NAME"
(cd "$DIST_DIR" && sha256sum "$(basename "$ARCHIVE")" > "$(basename "$ARCHIVE").sha256")

cat <<EOF
Release package created:
- $ARCHIVE
- $ARCHIVE.sha256
EOF
