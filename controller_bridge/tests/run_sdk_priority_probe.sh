#!/usr/bin/env bash
# Standalone SDK probe, never the existing RobotMain/ROS launch chain.
# No sudo, controller stop or permissions/limits escalation in this runner.
# --init remains no-motion; comparison modes affect only the isolated SDK robot.
set -Eeuo pipefail
[[ $# == 1 && ( "$1" == --check || "$1" == --init || "$1" == --compare ||
    "$1" == --hold-jog || "$1" == --end-jog || "$1" == --stop-end-jog ||
    "$1" == --protocol-compare || "$1" == --button-jog || "$1" == --button-end-jog ||
    "$1" == --command-jog || "$1" == --command-end-jog ) ]] || {
    echo 'Usage: bash run_sdk_priority_probe.sh --check|--init|--compare|--protocol-compare|<individual-case>' >&2; exit 2;
}
(( EUID != 0 )) || { echo 'Drop to the normal test user before running this script' >&2; exit 77; }
rt_soft=$(ulimit -Sr)
rt_hard=$(ulimit -Hr)
printf 'Probe entry uid=%d rtprio_soft=%s rtprio_hard=%s\n' "$EUID" "$rt_soft" "$rt_hard"
if [[ "$1" != --check && "$rt_soft" != unlimited && "$rt_soft" -lt 38 ]]; then
    echo 'Realtime ceiling was not inherited (need >=38); SDK not loaded' >&2
    exit 78
fi
if [[ "$1" == --compare || "$1" == --protocol-compare ]]; then
    overall=0
    cases=(--hold-jog --end-jog --stop-end-jog)
    if [[ "$1" == --protocol-compare ]]; then
        cases=(--button-jog --button-end-jog --command-jog --command-end-jog)
    fi
    for mode in "${cases[@]}"; do
        bash "$0" "$mode" || overall=$?
    done
    exit "$overall"
fi
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sim=/home/robot/Work/system/robot_config_new_left_sim
real=/home/robot/Work/system/robot_config_new_left
sdk=/lib/libRobotControl.so
expected=40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1
for program in bwrap cc sha256sum timeout readlink; do command -v "$program" >/dev/null; done
[[ -r "$sdk" && -r "$sim/license" && -r "$sim/DeviceConfig.yaml" ]]
[[ $(sha256sum "$sdk" | cut -d ' ' -f 1) == "$expected" ]] || {
    echo 'SDK fingerprint mismatch; nothing loaded' >&2; exit 77;
}
baseline=$(sha256sum "$real/GlobalConfig.yaml" "$real/DeviceConfig.yaml" "$real/BusConfig.yaml")
stage=$(mktemp -d /tmp/mk2-sdk-priority.XXXXXXXX)
printf 'Probe artifacts (retained): %s\n' "$stage"
mkdir "$stage/profile"
for item in DeviceConfig.yaml BusConfig.yaml ExternalDeviceConfig.yaml eni7_sim.xml license data app internal; do
    cp -a -- "$sim/$item" "$stage/profile/"
done
cp -- "$here/sdk_probe_GlobalConfig.yaml" "$stage/profile/GlobalConfig.yaml"
cc -std=c11 -Wall -Wextra -Werror -pedantic \
    -I/home/robot/Work/code/hyy_controller/include \
    "$here/sdk_priority_probe.c" -ldl -o "$stage/sdk_priority_probe"
rc=0
timeout --signal=TERM --kill-after=3s 30s \
    bwrap --unshare-all --die-with-parent --new-session --cap-drop ALL \
    --ro-bind / / --dev /dev --proc /proc \
    --tmpfs /dev/shm --tmpfs /tmp --tmpfs /run \
    --tmpfs "$real" --bind "$stage" /tmp/mk2-probe --chdir /tmp/mk2-probe --clearenv \
    --setenv PATH /usr/bin:/bin --setenv HOME /tmp/mk2-probe \
    --setenv MK2_HOST_MNT "$(readlink /proc/self/ns/mnt)" \
    --setenv MK2_HOST_IPC "$(readlink /proc/self/ns/ipc)" \
    --setenv MK2_HOST_NET "$(readlink /proc/self/ns/net)" \
    --setenv MK2_HOST_PID "$(readlink /proc/self/ns/pid)" \
    --setenv MK2_HOST_USER "$(readlink /proc/self/ns/user)" \
    /tmp/mk2-probe/sdk_priority_probe "$1" >"$stage/probe.log" 2>&1 || rc=$?
cat "$stage/probe.log"
[[ $(sha256sum "$real/GlobalConfig.yaml" "$real/DeviceConfig.yaml" "$real/BusConfig.yaml") == "$baseline" ]] || {
    echo 'Physical configuration baseline changed; investigate, do not start anything' >&2; exit 79;
}
printf 'Probe exit=%d; physical configuration fingerprints unchanged\n' "$rc"
exit "$rc"