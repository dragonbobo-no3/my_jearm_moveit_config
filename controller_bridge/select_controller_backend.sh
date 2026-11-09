#!/usr/bin/env bash
# Read-only startup selection; never starts/stops/enables a controller or master.
# RobotMain_v2 passes its fixed EtherCAT CLI path; optional argv is for isolated tests.
set -Eeuo pipefail
[[ $# -le 1 ]] || { echo 'Expected at most an EtherCAT CLI path' >&2; exit 2; }
ecat=${1:-/usr/local/bin/ethercat}
emit_backend() {
    # Optional commissioning pin: still detect normally, but refuse if hardware
    # appears before a simulation-only acceptance start. Never overrides detection.
    if [[ -n "${MK2_EXPECT_BACKEND:-}" && "$1" != "$MK2_EXPECT_BACKEND" ]]; then
        printf 'Detected %s but commissioning requires %s; nothing started\n' "$1" "$MK2_EXPECT_BACKEND" >&2
        return 1
    fi
    printf '%s\n' "$1"
}
[[ -x "$ecat" ]] || { echo 'Cannot determine backend: EtherCAT CLI unavailable; refusing automatic fallback' >&2; exit 1; }
if ! slaves=$(timeout 5 "$ecat" slaves 2>&1); then
    printf 'Cannot determine backend: slave detection failed; refusing automatic fallback\n%s\n' "$slaves" >&2
    exit 1
fi
if [[ -z "${slaves//[[:space:]]/}" ]]; then
    emit_backend sim
    exit 0
fi
count=$(printf '%s\n' "$slaves" | awk 'NF {n++} END {print n+0}')
[[ "$count" -eq 7 ]] || { printf 'Unexpected slave count %s; refusing automatic startup\n' "$count" >&2; exit 1; }
for axis in {0..6}; do
    expression="^[[:space:]]*$axis[[:space:]]+0:$axis[[:space:]]+(INIT|PREOP|SAFEOP|OP|BOOT)[[:space:]]+\\+[[:space:]]+JE DS402 Motor[[:space:]]*$"
    [[ $(printf '%s\n' "$slaves" | grep -Ec "$expression" || true) -eq 1 ]] || {
        printf 'Unexpected slave layout/identity/status at axis %s; refusing automatic startup\n' "$axis" >&2
        exit 1
    }
done
# All further exact SDK/ENI/vendor identity/disabled checks remain in the ORIGINAL
# real startup/handoff scripts. This is selection only, NOT a safety certificate.
emit_backend real