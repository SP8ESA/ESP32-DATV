#!/usr/bin/env bash
set -euo pipefail
nadajnik_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$nadajnik_root"
if [[ -x "$nadajnik_root/.venv/bin/python" ]] && "$nadajnik_root/.venv/bin/python" -c 'import PyQt5, serial, numpy' 2>/dev/null; then
    exec "$nadajnik_root/.venv/bin/python" -u -B "$nadajnik_root/host/tx_gui.py" "$@"
fi
exec /usr/bin/python3 -u -B "$nadajnik_root/host/tx_gui.py" "$@"
