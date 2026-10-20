#!/usr/bin/env bash
# Start the station-free swarm sim viewer (//swarm/sim:viewer) in the
# background so the "sim" named service (overlay.json) is up at every container
# start: http://sim.$CLAUDE_SERVICE_INSTANCE.claude.localhost/
#
# Idempotent and non-fatal: skips if something already listens on the port,
# never blocks the session on the (possibly cold, multi-minute) Bazel build.
# Knobs (forward from the host via overlay.json "env" if wanted):
#   SWARM_SIM_AUTOSTART=0   don't start it
#   SWARM_SIM_ARGS="..."    viewer flags (default: 5 widgets, 3 surveyed anchors)
# Log: /tmp/swarm-sim.log. Restart by hand: bash .claude-container-overlay/startup.sh

[ "${SWARM_SIM_AUTOSTART:-1}" = "0" ] && exit 0

PORT=8181
WS=/workspace
LOG=/tmp/swarm-sim.log
SCRIPT=/tmp/swarm-sim-viewer.sh
PIDFILE=/tmp/swarm-sim.pid
ARGS="${SWARM_SIM_ARGS:---widgets 5 --gps 0 --surveyed 3}"

if (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then
    echo "swarm-sim: already listening on :$PORT"
    exit 0
fi
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "swarm-sim: already starting (pid $(cat "$PIDFILE"), log: $LOG)"
    exit 0
fi
if ! command -v bazelisk >/dev/null 2>&1; then
    echo "swarm-sim: bazelisk not on PATH; not starting the sim viewer" >&2
    exit 0
fi

# --script_path builds, then hands back a runner script instead of exec'ing
# under `bazel run` — so the long-lived viewer doesn't hold the Bazel server
# lock and interactive builds stay unblocked.
nohup bash -c "
    cd '$WS' &&
    bazelisk run --script_path='$SCRIPT' //swarm/sim:viewer &&
    exec '$SCRIPT' --http-port $PORT $ARGS
" >"$LOG" 2>&1 </dev/null &
echo $! >"$PIDFILE"
echo "swarm-sim: building + starting viewer on :$PORT in background (log: $LOG)"
