#!/bin/bash
# Install dual_pa into Klipper or Kalico
#
#   bash install.sh              install (or update) and restart Klipper
#   bash install.sh --uninstall  remove it
#
# Optional environment: KLIPPER_DIR (default ~/klipper, or ~/kalico if that
# is what is installed), KLIPPER_SERVICE (default klipper)
set -e

SRCDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE="${KLIPPER_SERVICE:-klipper}"

if [ -z "$KLIPPER_DIR" ]; then
    KLIPPER_DIR="$HOME/klipper"
    [ ! -d "$KLIPPER_DIR/klippy" ] && [ -d "$HOME/kalico/klippy" ] \
        && KLIPPER_DIR="$HOME/kalico"
fi
if [ ! -d "$KLIPPER_DIR/klippy/chelper" ]; then
    echo "Klipper not found in $KLIPPER_DIR (set KLIPPER_DIR)" >&2
    exit 1
fi

# Kalico loads third party modules from klippy/plugins
if [ -d "$KLIPPER_DIR/klippy/plugins" ]; then
    DEST="$KLIPPER_DIR/klippy/plugins"
    REL="klippy/plugins/dual_pa.py"
else
    DEST="$KLIPPER_DIR/klippy/extras"
    REL="klippy/extras/dual_pa.py"
fi
EXCLUDE="$KLIPPER_DIR/.git/info/exclude"

restart() {
    if systemctl list-units --full --all 2>/dev/null \
            | grep -q "^ *$SERVICE.service"; then
        echo "Restarting $SERVICE"
        sudo systemctl restart "$SERVICE"
    else
        echo "Restart Klipper to load the change"
    fi
}

if [ "$1" = "--uninstall" ]; then
    rm -f "$DEST/dual_pa.py"
    [ -f "$EXCLUDE" ] && sed -i "\|^$REL\$|d" "$EXCLUDE"
    rm -f "$SRCDIR/libdual_pa.so"
    echo "dual_pa removed. Delete the [dual_pa] section from printer.cfg."
    restart
    exit 0
fi

# Build the C part now, so a missing compiler shows up here and not at
# Klipper start (Klipper rebuilds it when dual_pa.c or Klipper's headers
# change)
echo "Building libdual_pa.so"
gcc -Wall -O2 -shared -fPIC -I"$KLIPPER_DIR/klippy/chelper" \
    -o "$SRCDIR/libdual_pa.so" "$SRCDIR/dual_pa.c"

ln -sf "$SRCDIR/dual_pa.py" "$DEST/dual_pa.py"
if [ -f "$EXCLUDE" ] && ! grep -qx "$REL" "$EXCLUDE"; then
    echo "$REL" >> "$EXCLUDE"
fi
echo "Linked $DEST/dual_pa.py"
restart
