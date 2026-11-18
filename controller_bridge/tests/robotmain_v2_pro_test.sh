#!/usr/bin/env bash
# No SDK or actual controller invocation; verify the intentionally closed gate.
set -Eeuo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out=$(mktemp -d /tmp/robotmain-v2-pro-test.XXXXXXXX)
printf 'Retained launcher test artifacts: %s\n' "$out"
check_exit() {
    local expected=$1 name=$2 rc=0
    shift 2
    /bin/bash "$root/RobotMain_v2_pro" "$@" >"$out/$name.log" 2>&1 || rc=$?
    [[ "$rc" == "$expected" ]] || { cat "$out/$name.log" >&2; exit 1; }
}
check_exit 0 help --help
check_exit 78 default
for action in --check --check-sim --auto --start --real --sim; do
    name=${action#--}
    check_exit 78 "$name" "$action"
    grep -Fxq 'STATUS=NOT_READY' "$out/$name.log"
    grep -Fxq 'NO_CONTROLLER_ACTION=1' "$out/$name.log"
    if [[ "$action" != --check* ]]; then
        grep -q '^START_REFUSED:' "$out/$name.log"
    fi
done
check_exit 2 bad-option --enable
check_exit 2 extra-argument --real --force
# Build/test actions must not guess an SDK include path or fall back to startup.
for action in --test-plugin --test-plugin-sandbox --test-handover-sandbox --build-sim-plugin --test-plugin-sdk-offline; do
    rc=0
    env -u JE2_SDK_INCLUDE /bin/bash "$root/RobotMain_v2_pro" "$action" \
        >"$out/${action#--}-missing-sdk.log" 2>&1 || rc=$?
    if (( EUID == 0 )); then [[ "$rc" == 77 ]]; else [[ "$rc" == 64 ]]; fi
done
(
    unset LD_PRELOAD LD_AUDIT LD_LIBRARY_PATH
    ulimit -Sr 0
    if (( EUID == 0 )); then
        check_exit 77 real-sdk-no-rt --test-plugin-sdk
        check_exit 77 writer-sdk-no-rt --test-writer-exit-virtual
        check_exit 77 handover-sdk-no-rt --test-handover-virtual
    else
        check_exit 78 real-sdk-no-rt --test-plugin-sdk
        check_exit 78 writer-sdk-no-rt --test-writer-exit-virtual
        check_exit 78 handover-sdk-no-rt --test-handover-virtual
        grep -Fq 'Realtime soft ceiling must be >=38' "$out/real-sdk-no-rt.log"
        grep -Fq 'Realtime soft ceiling must be >=38' "$out/writer-sdk-no-rt.log"
        grep -Fq 'Realtime soft ceiling must be >=38' "$out/handover-sdk-no-rt.log"
    fi
)
# Ensure filenames/current directory cannot redirect source/test lookup.
pushd /tmp >/dev/null
check_exit 78 foreign-directory --real
popd >/dev/null
printf '%s\n' 'PASS: RobotMain_v2_pro help/default/check/start/invalid/cwd refusal checks; no SDK or deployment.'