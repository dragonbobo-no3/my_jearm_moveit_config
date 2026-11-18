#!/usr/bin/env bash
# Separate opt-in FULL worker test. Vendor headers ONLY, never a real SDK/remote.
set -Eeuo pipefail
if [[ $# -gt 1 || ( $# == 1 && $1 != --writer-experiment && $1 != --session-experiment ) ]]; then
    echo 'Usage: JE2_SDK_INCLUDE=/absolute/vendor/include bash run_jeserver_v2_sandbox_test.sh [--writer-experiment|--session-experiment]' >&2; exit 64
fi
plugin_flags=(); loader_flags=(); writer_sources=(); client_sources=()
if [[ ${JE2_SDK_INCLUDE:-} != /* || ! -f ${JE2_SDK_INCLUDE:-}/HYYRobotInterface.h ||
      ! -f ${JE2_SDK_INCLUDE:-}/Base/RobotStruct.h || ! -f ${JE2_SDK_INCLUDE:-}/Move/MovePlan.h ]]; then
    echo 'JE2_SDK_INCLUDE must explicitly name absolute vendor declarations' >&2; exit 64
fi
for name in LD_PRELOAD LD_AUDIT LD_LIBRARY_PATH MK2_HOST_MNT MK2_HOST_IPC MK2_HOST_NET MK2_HOST_PID MK2_HOST_USER; do
    if [[ -v $name ]]; then echo "Refusing inherited injection/namespace override: $name" >&2; exit 64; fi
done
command -v bwrap >/dev/null
command -v timeout >/dev/null
pkg-config --exists json-c libzmq openssl
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out=$(mktemp -d /tmp/jeserver-v2-sandbox.XXXXXXXX)
printf 'Artifacts (retained): %s\n' "$out"
read -r -a cflags <<< "$(pkg-config --cflags json-c libzmq openssl)"
read -r -a libs <<< "$(pkg-config --libs json-c libzmq openssl)"
strict=(-std=c11 -Wall -Wextra -Werror -pedantic -pthread -isystem "$JE2_SDK_INCLUDE" "${cflags[@]}")
cc=${CC:-cc}
if [[ ${1:-} == --writer-experiment || ${1:-} == --session-experiment ]]; then
    plugin_flags=(-DJE2_SIM_WRITER_EXPERIMENT=1)
    loader_flags=(-DJE2_FAKE_WRITER_TASKS=1)
    writer_sources=("$root/jeserver_v2_task_watch.c")
fi
if [[ ${1:-} == --session-experiment ]]; then
    plugin_flags+=(-DJE2_SIM_SESSION_EXPERIMENT=1)
    loader_flags+=(-DJE2_FAKE_SESSION=1)
    client_sources=("$root/je2_session_client.c")
fi
# Same real plugin sources and DSO for every fresh load/unload case. Default
# build has no observer macro; only the named opt-in uses the experiment build.
"$cc" "${strict[@]}" "${plugin_flags[@]}" -fPIC -shared "$root/jeserver_v2_plugin.c" \
    "$root/jeserver_v2.c" "$root/jeserver_v2_transport.c" "$root/mk2_control_core.c" \
    "${writer_sources[@]}" "${libs[@]}" -lm -o "$out/libJEServer_v2_sim.so"
"$cc" "${strict[@]}" "${loader_flags[@]}" -Wl,--export-dynamic "$root/tests/jeserver_v2_sandbox_test.c" \
    "${client_sources[@]}" "${libs[@]}" -lm -ldl -o "$out/loader"
for binary in libJEServer_v2_sim.so loader; do
    LC_ALL=C readelf -d "$out/$binary" > "$out/$binary.dynamic.txt"
    grep -q '(NEEDED)' "$out/$binary.dynamic.txt"
    while IFS= read -r dependency; do
        case "$dependency" in
            libjson-c.so.*|libzmq.so.*|libssl.so.*|libcrypto.so.*|libm.so.*|libc.so.*|libpthread.so.*|libdl.so.*|ld-linux*.so.*) ;;
            *) echo "Unexpected dependency: $binary: $dependency" >&2; exit 1 ;;
        esac
    done < <(sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p' "$out/$binary.dynamic.txt")
    nm -D "$out/$binary" > "$out/$binary.symbols.txt"
    if grep -Eq '\b(RobotPower|RobotPoweroff|ClearRobotError|RobotStopRecover)(@.*)?$' "$out/$binary.symbols.txt"; then
        echo 'Forbidden enable/power/clear/recover SDK symbol' >&2; exit 1
    fi
done
# Nonexistent library proves refusal is BEFORE dlopen, without touching profile.
set +e
"$out/loader" /nonexistent-never-load.so --path /tmp/mk2-pro/profile > "$out/host-refusal.log" 2>&1
result=$?
set -e
[[ $result == 77 ]]
grep -q 'BEFORE dlopen' "$out/host-refusal.log"
cat "$out/host-refusal.log"
args=(--unshare-all --die-with-parent --new-session --cap-drop ALL
    --ro-bind / / --dev /dev --proc /proc --tmpfs /dev/shm --tmpfs /tmp --tmpfs /run
    --ro-bind "$out" /tmp/je2-sandbox-test --clearenv --setenv PATH /usr/bin:/bin)
for namespace in mnt ipc net pid user; do
    marker="MK2_HOST_${namespace^^}"
    args+=(--setenv "$marker" "$(readlink "/proc/self/ns/$namespace")")
done
real_profile=/home/robot/Work/system/robot_config_new_left
# Never create the real path. Hide it ONLY when it already exists on this host.
if [[ -e $real_profile ]]; then
    mkdir "$out/empty-profile-mask"
    args+=(--ro-bind "$out/empty-profile-mask" "$real_profile")
fi
timeout --signal=TERM --kill-after=3 30s bwrap "${args[@]}" \
    /tmp/je2-sandbox-test/loader /tmp/je2-sandbox-test/libJEServer_v2_sim.so \
    --path /tmp/mk2-pro/profile 2>&1 | tee "$out/sandbox.log"
printf 'PASS: standalone real-bwrap lifecycle test, FAKE SDK only; no real SDK linked/loaded, no remote\nArtifacts: %s\n' "$out"