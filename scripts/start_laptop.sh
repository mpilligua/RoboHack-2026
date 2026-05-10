#!/usr/bin/env bash
# Start the 4 services that run ON THE LAPTOP for the demo:
#   1. voice_server.py --demo (canned-response Flask server, port 5050)
#   2. ngrok http 5050 (public URL so the phone can reach voice_server
#      from outside the local wifi)
#   3. demo_screen.py (four-panel state display)
#   4. demo.py (presenter REPL — connects to the real robot and calls
#      motion handlers on each scene)
#
# Run this on the Mac AFTER scripts/start_robot.sh has been started on the
# robot. Each service opens in its own Terminal.app window.

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AGENT_DIR="${REPO_ROOT}/agent"
VENV_ACTIVATE="${AGENT_DIR}/.venv/bin/activate"

if [ ! -f "$VENV_ACTIVATE" ]; then
    echo "venv not found at $VENV_ACTIVATE" >&2
    echo "run: cd ${AGENT_DIR} && python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt" >&2
    exit 1
fi

if ! command -v ngrok >/dev/null 2>&1; then
    echo "ngrok not found in PATH. Install from https://ngrok.com/download or run voice_server only." >&2
    SKIP_NGROK=1
fi

osascript_open() {
    # Open a new Terminal.app window and run the given shell snippet inside it.
    local title="$1" cmd="$2"
    osascript <<EOF
tell application "Terminal"
    activate
    set newWin to do script "${cmd//\"/\\\"}"
    set custom title of front window to "${title}"
end tell
EOF
}

echo "[laptop] starting voice_server.py --demo on port 5050..."
osascript_open "voice_server" \
    "cd '${AGENT_DIR}' && source '${VENV_ACTIVATE}' && python voice_server.py --demo"

# Give Flask a moment to bind before ngrok tries to forward.
sleep 3

if [ -z "$SKIP_NGROK" ]; then
    echo "[laptop] starting ngrok http 5050..."
    osascript_open "ngrok" "ngrok http 5050"
else
    echo "[laptop] skipping ngrok (not installed). Phone must use http://<laptop-lan-ip>:5050 instead."
fi

echo "[laptop] starting demo_screen (four-panel display)..."
osascript_open "demo_screen" \
    "cd '${AGENT_DIR}' && source '${VENV_ACTIVATE}' && python demo_screen.py"

# demo_screen polls a state file; give it a moment to come up before demo.py
# starts writing.
sleep 1

echo "[laptop] starting demo (presenter REPL, real robot)..."
osascript_open "demo" \
    "cd '${AGENT_DIR}' && source '${VENV_ACTIVATE}' && python demo.py"

echo
echo "[laptop] all windows launched."
echo "[laptop] open the ngrok window to find the public https://...ngrok-free.app URL"
echo "[laptop] open that URL on the phone to start the demo."
