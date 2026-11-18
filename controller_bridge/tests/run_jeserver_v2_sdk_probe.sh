#!/usr/bin/env bash
# Opt-in standalone REAL SDK lifecycle probe, never a controller launcher.
# No sudo, installs, limit escalation, controller stop/restart or host writes.
# Vendor declarations: JE2_SDK_INCLUDE=/absolute/include, or the controller's
# fixed /home/robot/Work/code/hyy_controller/include default. Missing json-c dev
# files: JE2_TEST_JSON_INCLUDE=/absolute/include-root and
# JE2_TEST_JSON_LIBRARY=/absolute/libjson-c.so.5 (FULL headers, not a shim).
# Invoke with bash; artifacts are retained even on failure.
set -Eeuo pipefail
if [[ $# != 1 || ( $1 != --check && $1 != --load && $1 != --writer-exit-virtual && $1 != --handover-virtual && $1 != --check-handover-virtual && $1 != --teach-virtual && $1 != --check-teach-virtual && $1 != --teach-external-stop-virtual && $1 != --teach-overlap-stop-virtual ) ]]; then
    echo 'Usage: bash run_jeserver_v2_sdk_probe.sh --check|--load|--writer-exit-virtual|--check-handover-virtual|--handover-virtual|--check-teach-virtual|--teach-virtual|--teach-external-stop-virtual|--teach-overlap-stop-virtual' >&2
    exit 64
fi
(( EUID != 0 )) || { echo 'Refusing root; run as the normal test user.' >&2; exit 77; }
while IFS= read -r name; do
    case "$name" in
        LD_PRELOAD|LD_AUDIT|LD_LIBRARY_PATH|MK2_HOST_*|JE2_HOST_*|JE2_EXPECTED_PLUGIN_SHA256|JE2_*NAMESPACE*|JE2_*_NS)
            echo "Refusing inherited injection/namespace override: $name" >&2; exit 64 ;;
    esac
done < <(compgen -e)
rt_soft=$(ulimit -Sr)
printf 'Probe entry uid=%d rtprio_soft=%s rtprio_hard=%s\n' "$EUID" "$rt_soft" "$(ulimit -Hr)"
if [[ $1 != --check && $1 != --check-handover-virtual && $1 != --check-teach-virtual && $rt_soft != unlimited ]] && (( rt_soft < 38 )); then
    echo 'Realtime soft ceiling must be >=38; SDK not loaded (no escalation attempted).' >&2
    exit 78
fi
mode=$1
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
root=$(cd -- "$here/.." && pwd -P)
probe_source=$here/jeserver_v2_sdk_probe.c
[[ -r $probe_source && ! -L $probe_source ]] || {
    echo 'BLOCKED: tests/jeserver_v2_sdk_probe.c is not ready; no SDK loaded.' >&2; exit 78;
}
for program in bwrap cc sha256sum timeout readlink readelf nm mktemp mkdir cp find \
               sort cmp cat sed grep cut pgrep stat dirname chmod; do
    command -v "$program" >/dev/null || { echo "Missing command: $program" >&2; exit 78; }
done
sim=/home/robot/Work/system/robot_config_new_left_sim
real=/home/robot/Work/system/robot_config_new_left
sdk=/lib/libRobotControl.so
technology=/lib/libHYYRobotTechnology.so
sdk_sha=40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1
# The companion C guard owns the full technology pin. Require exactly one
# distinct full literal with the independently recorded prefix; never accept
# an environment pin or learn a pin from the currently installed library.
# This also fails closed while the companion loader is still being written.
technology_sha=$(LC_ALL=C grep -oE 'cb854[0-9a-f]{59}' "$probe_source" | sort -u) || {
    echo 'BLOCKED: companion C guard must contain the full pinned technology SHA256.' >&2; exit 78;
}
[[ $technology_sha =~ ^cb854[0-9a-f]{59}$ ]] || {
    echo 'Ambiguous/missing companion technology pin; SDK not loaded.' >&2; exit 78;
}
for pair in "$sdk:$sdk_sha" "$technology:$technology_sha"; do
    file=${pair%:*}; expected=${pair##*:}
    [[ -r $file && $(sha256sum -- "$file" | cut -d ' ' -f 1) == "$expected" ]] || {
        echo "Pinned SDK mismatch: $file; nothing loaded." >&2; exit 77;
    }
done
vendor=${JE2_SDK_INCLUDE-/home/robot/Work/code/hyy_controller/include}
[[ $vendor == /* && -r $vendor/HYYRobotInterface.h &&
   -r $vendor/Base/RobotStruct.h && -r $vendor/Move/MovePlan.h ]] || {
    echo 'Missing absolute vendor include tree (JE2_SDK_INCLUDE or fixed controller default).' >&2; exit 78;
}
json_cflags=(); json_libs=()
if [[ -v JE2_TEST_JSON_INCLUDE || -v JE2_TEST_JSON_LIBRARY ]]; then
    [[ ${JE2_TEST_JSON_INCLUDE:-} == /* && ${JE2_TEST_JSON_LIBRARY:-} == /* &&
       -r ${JE2_TEST_JSON_LIBRARY:-} ]] || {
        echo 'JSON overrides require absolute include root AND library file.' >&2; exit 78;
    }
    # The old 810-byte offline shim is NOT sufficient for a real SDK probe.
    for header in json.h json_object.h json_tokener.h json_types.h json_util.h; do
        [[ -r $JE2_TEST_JSON_INCLUDE/json-c/$header ]] || {
            echo "Full json-c headers required; missing $header. No packages installed." >&2; exit 78;
        }
    done
    json_cflags=(-I"$JE2_TEST_JSON_INCLUDE")
    json_libs=("$JE2_TEST_JSON_LIBRARY")
elif command -v pkg-config >/dev/null && pkg-config --exists json-c; then
    read -r -a json_cflags <<< "$(pkg-config --cflags json-c)"
    read -r -a json_libs <<< "$(pkg-config --libs json-c)"
else
    echo 'Full json-c development files missing; use existing staged JE2_TEST_JSON_INCLUDE/LIBRARY. No install.' >&2
    exit 78
fi
other_cflags=(); other_libs=(-lzmq -lcrypto)
if command -v pkg-config >/dev/null && pkg-config --exists libzmq openssl; then
    read -r -a other_cflags <<< "$(pkg-config --cflags libzmq openssl)"
    read -r -a other_libs <<< "$(pkg-config --libs libzmq openssl)"
else
    [[ -r /usr/include/zmq.h && -r /usr/include/openssl/evp.h ]] || {
        echo 'Missing ZeroMQ/OpenSSL development headers; no packages installed.' >&2; exit 78;
    }
fi
for file in jeserver_v2_plugin.c mk2_control_plugin.c jeserver_v2_runtime.inc \
            jeserver_v2.c jeserver_v2.h jeserver_v2_transport.c \
            jeserver_v2_transport.h mk2_control_core.c mk2_control_core.h; do
    [[ -f $root/$file && ! -L $root/$file ]] || { echo "Missing source: $file" >&2; exit 78; }
done
[[ -d $sim && ! -L $sim && $(readlink -f -- "$sim") == "$sim" &&
   -f $sim/mk2_control_bridge.json && ! -L $sim/mk2_control_bridge.json ]] || {
    echo 'Simulation source profile/policy missing or symlinked; refusing.' >&2; exit 77;
}
items=(DeviceConfig.yaml BusConfig.yaml ExternalDeviceConfig.yaml eni7_sim.xml license data app internal)
for item in "${items[@]}"; do
    [[ -e $sim/$item && ! -L $sim/$item ]] || { echo "Missing/symlinked simulation resource: $item" >&2; exit 77; }
    # Reject nested escape links and devices/sockets/FIFOs, rather than cp -a
    # silently preserving a path back into the real/source profile.
    invalid=$(find "$sim/$item" ! -type d ! -type f -print -quit)
    [[ -z $invalid ]] || { echo "Unsafe simulation resource: $invalid" >&2; exit 77; }
done
out=$(mktemp -d /tmp/jeserver-v2-sdk.XXXXXXXX)
chmod 700 "$out"
printf 'Artifacts (retained): %s\n' "$out"
# Before/after snapshots are observational only. A change is reported, NEVER
# repaired by stopping/restarting processes or restoring someone else's files.
snapshot() {
    local dest=$1 profile file status
    : > "$dest.processes"
    for name in RobotMain RobotMain_v2 RobotMain_v2_pro IgHEtherCATMast; do
        status=0
        pgrep -a -x "$name" >> "$dest.processes" || status=$?
        (( status <= 1 )) || return "$status"
    done
    LC_ALL=C sort -o "$dest.processes" "$dest.processes"
    : > "$dest.sha256"
    for profile in "$real" "$sim"; do
        for file in GlobalConfig.yaml DeviceConfig.yaml BusConfig.yaml ExternalDeviceConfig.yaml \
                    mk2_control_bridge.json GlobalConfig.sha256 launch_new_left.sh; do
            if [[ -e $profile/$file ]]; then
                sha256sum -- "$profile/$file" >> "$dest.sha256" || return
            else printf 'ABSENT %s\n' "$profile/$file" >> "$dest.sha256"; fi
        done
    done
    for file in /home/robot/Work/system/RobotMain /home/robot/Work/system/RobotMain_v2 \
                /home/robot/Work/system/launch_new_left.sh /home/robot/Work/system/launch_simulation.sh \
                "$sdk" "$technology"; do
        if [[ -e $file ]]; then sha256sum -- "$file" >> "$dest.sha256" || return
        else printf 'ABSENT %s\n' "$file" >> "$dest.sha256"; fi
    done
}
snapshot "$out/before"
finish() {
    local result=$?
    trap - EXIT
    if ! snapshot "$out/after"; then
        echo 'After-snapshot failed; investigate manually, no corrective action taken.' >&2
        result=79
    elif ! cmp -s "$out/before.sha256" "$out/after.sha256" ||
         ! cmp -s "$out/before.processes" "$out/after.processes"; then
        echo 'Host process/config/launcher snapshot CHANGED; no corrective action taken.' >&2
        result=79
    else echo 'Host process/config/launcher snapshots unchanged.'; fi
    printf 'Probe exit=%d; retained artifacts: %s\n' "$result" "$out"
    exit "$result"
}
trap finish EXIT
mkdir "$out/profile" "$out/empty-profile-mask"
for item in "${items[@]}"; do
    # BusConfig is generated exclusively AFTER the probe has been compiled.
    # Never carry installed-profile absolute ENI references into the sandbox.
    if [[ $item != BusConfig.yaml ]]; then cp -a -- "$sim/$item" "$out/profile/"; fi
done
invalid=$(find "$out/profile" ! -type d ! -type f -print -quit)
[[ -z $invalid ]] || { echo 'Copied profile contains unsafe entries.' >&2; exit 77; }
cp -- "$here/jeserver_v2_sdk_GlobalConfig.yaml" "$out/profile/GlobalConfig.yaml"
strict=(-std=c11 -Wall -Wextra -Werror -pedantic -pthread -isystem "$vendor"
        -I"$root" "${json_cflags[@]}" "${other_cflags[@]}")
libs=("${json_libs[@]}" "${other_libs[@]}" -ldl -lm -pthread)
plugin_flags=(); probe_flags=(); writer_sources=()
policy_mode=--prepare-policy
if [[ $mode == --handover-virtual || $mode == --check-handover-virtual ]]; then
    plugin_flags=(-DJE2_SIM_WRITER_EXPERIMENT=1)
    probe_flags=(-DJE2_SDK_HANDOVER_PROBE=1)
    writer_sources=("$root/jeserver_v2_task_watch.c")
    policy_mode=--prepare-handover-policy
    echo 'EXPLICIT PRIVATE VIRTUAL HANDOVER: commands enabled ONLY in staged immutable simulation policy; no production launch.'
fi
if [[ $mode == --teach-virtual || $mode == --check-teach-virtual || $mode == --teach-external-stop-virtual || $mode == --teach-overlap-stop-virtual ]]; then
    probe_flags=(-DJE2_SDK_TEACH_PROBE=1)
    cc "${strict[@]}" -fsyntax-only "$here/jeserver_v2_sdk_teach_abi.c"
    echo 'JE2_TEACH_ABI_CHECK=PASS vendor-header signatures checked; no SDK linked or loaded.'
    echo 'EXPLICIT PRIVATE NATIVE TEACH SEMANTICS: virtual enable/jog/stop; SDK may internally recover; no plugin loaded, bridge policy remains locked; NOT handover certification.'
fi
cc "${strict[@]}" "${plugin_flags[@]}" -fPIC -shared "$root/jeserver_v2_plugin.c" "$root/jeserver_v2.c" \
    "$root/jeserver_v2_transport.c" "$root/mk2_control_core.c" \
    "${writer_sources[@]}" \
    "${libs[@]}" -o "$out/libJEServer_v2_sim.so"
cc "${strict[@]}" "${probe_flags[@]}" "$probe_source" "$root/mk2_control_core.c" "$root/jeserver_v2_task_watch.c" \
    "${libs[@]}" -o "$out/jeserver_v2_sdk_probe"
for binary in libJEServer_v2_sim.so jeserver_v2_sdk_probe; do
    LC_ALL=C readelf -d "$out/$binary" > "$out/$binary.dynamic.txt"
    grep -q '(NEEDED)' "$out/$binary.dynamic.txt"
    sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p' "$out/$binary.dynamic.txt" > "$out/$binary.needed"
    [[ -s $out/$binary.needed ]] || { echo 'Cannot parse dynamic dependencies.' >&2; exit 77; }
    while IFS= read -r dependency; do
        case "$dependency" in
            libjson-c.so.*|libzmq.so.*|libssl.so.*|libcrypto.so.*|libm.so.*|libc.so.*|libpthread.so.*|libdl.so.*|ld-linux*.so.*) ;;
            *) echo "Unexpected dependency (vendor SDK must NOT be linked): $dependency" >&2; exit 77 ;;
        esac
    done < "$out/$binary.needed"
    nm -D "$out/$binary" > "$out/$binary.symbols.txt"
    forbidden='\b(RobotPower|RobotPoweroff|ClearRobotError|RobotClearError|RobotStopRecover|robot_teach_enable|RobotStopRun)(@.*)?$'
    if [[ ${#plugin_flags[@]} == 1 && $binary == libJEServer_v2_sim.so ]]; then
        forbidden='\b(RobotPower|RobotPoweroff|ClearRobotError|RobotClearError|RobotStopRecover|robot_teach_enable)(@.*)?$'
    fi
    if grep -Eq "$forbidden" "$out/$binary.symbols.txt"; then
        echo 'Forbidden enable/recover/power/handover import.' >&2; exit 77
    fi
done
plugin_sha=$(sha256sum "$out/libJEServer_v2_sim.so" | cut -d ' ' -f 1)
(cd "$out" && sha256sum libJEServer_v2_sim.so jeserver_v2_sdk_probe > SHA256SUMS)
# This host invocation is deliberately policy-only: the C implementation MUST
# not load either SDK here. It validates the original strict schema first,
# hashes the staged DeviceConfig, and creates the output O_EXCL (no fallback).
"$out/jeserver_v2_sdk_probe" --prepare-bus "$sim/BusConfig.yaml" "$out/profile"
"$out/jeserver_v2_sdk_probe" "$policy_mode" "$sim/mk2_control_bridge.json" "$out/profile"
[[ -f $out/profile/mk2_control_bridge.json && ! -L $out/profile/mk2_control_bridge.json ]]
chmod a-w "$out/jeserver_v2_sdk_probe" "$out/libJEServer_v2_sim.so" \
    "$out/profile/GlobalConfig.yaml" "$out/profile/DeviceConfig.yaml" \
    "$out/profile/BusConfig.yaml" "$out/profile/mk2_control_bridge.json"
args=(--unshare-all --die-with-parent --new-session --cap-drop ALL
      --ro-bind / / --dev /dev --proc /proc
      --tmpfs /dev/shm --tmpfs /tmp --tmpfs /run --tmpfs /sys
      --bind "$out" /tmp/mk2-pro --chdir /tmp/mk2-pro --clearenv
      --setenv PATH /usr/bin:/bin --setenv HOME /tmp/mk2-pro
      --setenv JE2_EXPECTED_PLUGIN_SHA256 "$plugin_sha")
for namespace in mnt ipc net pid user; do
    marker="MK2_HOST_${namespace^^}"
    args+=(--setenv "$marker" "$(readlink "/proc/self/ns/$namespace")")
done
# SDK app/data writes affect only our private copy. Every guard input and both
# executable artifacts have independent RO mounts (chmod alone is not a guard).
for file in jeserver_v2_sdk_probe libJEServer_v2_sim.so \
            profile/GlobalConfig.yaml profile/DeviceConfig.yaml profile/BusConfig.yaml \
            profile/ExternalDeviceConfig.yaml profile/mk2_control_bridge.json \
            profile/eni7_sim.xml profile/license profile/internal; do
    args+=(--ro-bind "$out/$file" "/tmp/mk2-pro/$file")
done
for profile in "$real" "$sim"; do
    if [[ -e $profile ]]; then args+=(--ro-bind "$out/empty-profile-mask" "$profile"); fi
done
result=0
timeout --signal=TERM --kill-after=3s 30s bwrap "${args[@]}" \
    /tmp/mk2-pro/jeserver_v2_sdk_probe "$mode" --path /tmp/mk2-pro/profile \
    > "$out/probe.log" 2>&1 || result=$?
cat "$out/probe.log"
exit "$result"