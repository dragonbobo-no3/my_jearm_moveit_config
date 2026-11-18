#!/usr/bin/env bash
# Local, SDK-free tests only. Invoke with bash; retain all artifacts in /tmp.
# Requires actual installed json-c/ZeroMQ/OpenSSL and absolute vendor headers.
# No vendor library, namespace launcher, remote access or privilege changes.
set -Eeuo pipefail
[[ $# == 0 ]] || { echo 'No arguments accepted.' >&2; exit 64; }
for name in LD_PRELOAD LD_AUDIT LD_LIBRARY_PATH; do
    if [[ -v $name ]]; then
        echo "Require clean child environment: unset $name before starting this runner." >&2
        exit 64
    fi
done
[[ ${JE2_SDK_INCLUDE:-} == /* && -r $JE2_SDK_INCLUDE/Base/RobotStruct.h ]] || {
    echo 'Require JE2_SDK_INCLUDE=/absolute/vendor/include (Base/RobotStruct.h).' >&2
    exit 64
}
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
root=$(cd -- "$here/.." && pwd -P)
for program in cc pkg-config readelf nm mktemp mkdir sha256sum grep sed bash; do
    command -v "$program" >/dev/null || { echo "Missing local command: $program" >&2; exit 78; }
done
pkg-config --exists json-c libzmq openssl || {
    echo 'Require existing full json-c, ZeroMQ and OpenSSL development files; no installation attempted.' >&2
    exit 78
}
read -r -a cflags <<< "$(pkg-config --cflags json-c libzmq openssl)"
read -r -a libs <<< "$(pkg-config --libs json-c libzmq openssl)"
out=$(mktemp -d /tmp/jeserver-v2-sdk-offline.XXXXXXXX)
stage=$(mktemp -d /tmp/jeserver-v2-sdk.offline.XXXXXXXX)
printf 'Retained build/log artifacts: %s\nRetained policy fixtures: %s\n' "$out" "$stage"
trap 'result=$?; printf "Offline exit=%d; artifacts=%s fixtures=%s\n" "$result" "$out" "$stage"' EXIT
strict=(-std=c11 -Wall -Wextra -Werror -pedantic -pthread
        -isystem "$JE2_SDK_INCLUDE" -I"$root" "${cflags[@]}")
link=("${libs[@]}" -ldl -lm -pthread)
sources=("$here/jeserver_v2_sdk_probe.c" "$root/mk2_control_core.c" "$root/jeserver_v2_task_watch.c")
# Snapshot exactly the inputs this runner consumes; never edit production files.
sha256sum "${sources[@]}" "$root/mk2_control_core.h" "$root/jeserver_v2_task_watch.h" \
    "$here/jeserver_v2_sdk_writer_probe.inc" \
    "$here/jeserver_v2_sdk_handover_probe.inc" "$here/jeserver_v2_sdk_handover_test.c" \
    "$here/jeserver_v2_sdk_teach_probe.inc" "$here/jeserver_v2_sdk_teach_test.c" \
    "$here/run_jeserver_v2_sdk_probe.sh" "$here/jeserver_v2_sdk_runner_preflight_test.sh" \
    "$root/mk2_control_bridge.example.json" "$JE2_SDK_INCLUDE/Base/RobotStruct.h" > "$out/inputs.sha256"
for variant in ordinary asan-ubsan; do
    build=$out/$variant
    fixtures=$stage/$variant
    mkdir "$build" "$fixtures"
    instrumentation=()
    if [[ $variant == asan-ubsan ]]; then
        instrumentation=(-g -O1 -fsanitize=address,undefined -fno-sanitize-recover=all
                         -fno-omit-frame-pointer -fno-pie -no-pie)
    fi
    cc "${strict[@]}" "${instrumentation[@]}" "${sources[@]}" "${link[@]}" -o "$build/probe"
    cc "${strict[@]}" "${instrumentation[@]}" "$here/jeserver_v2_sdk_probe_test.c" \
        "$root/mk2_control_core.c" "$root/jeserver_v2_task_watch.c" "${link[@]}" -o "$build/test"
    cc "${strict[@]}" "${instrumentation[@]}" -DJE2_SDK_HANDOVER_PROBE=1 "${sources[@]}" "${link[@]}" -o "$build/handover-probe"
    cc "${strict[@]}" "${instrumentation[@]}" "$here/jeserver_v2_sdk_handover_test.c" \
        "$root/mk2_control_core.c" "$root/jeserver_v2_task_watch.c" "${link[@]}" -o "$build/handover-test"
    cc "${strict[@]}" "${instrumentation[@]}" -DJE2_SDK_TEACH_PROBE=1 "${sources[@]}" "${link[@]}" -o "$build/teach-probe"
    cc "${strict[@]}" "${instrumentation[@]}" "$here/jeserver_v2_sdk_teach_test.c" \
        "$root/mk2_control_core.c" "$root/jeserver_v2_task_watch.c" "${link[@]}" -o "$build/teach-test"
    for executable in probe test handover-probe handover-test teach-probe teach-test; do
        LC_ALL=C readelf -d "$build/$executable" > "$build/$executable.dynamic.txt"
        sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p' \
            "$build/$executable.dynamic.txt" > "$build/$executable.needed"
        [[ -s $build/$executable.needed ]]
        while IFS= read -r dependency; do
            case "$dependency" in
                libjson-c.so.*|libzmq.so.*|libssl.so.*|libcrypto.so.*|libm.so.*|libc.so.*|libpthread.so.*|libdl.so.*|ld-linux*.so.*) ;;
                libasan.so.*|libubsan.so.*)
                    [[ $variant == asan-ubsan ]] || exit 77 ;;
                *) echo "Unexpected direct dependency: $dependency; no SDK allowed." >&2; exit 77 ;;
            esac
        done < "$build/$executable.needed"
        LC_ALL=C nm -D "$build/$executable" > "$build/$executable.symbols.txt"
        if grep -Eq '\b(commandLineParser1|system_initialize|RobotPower|RobotPoweroff|ClearRobotError|RobotClearError|RobotStopRecover|robot_teach_enable|RobotStopRun)(@.*)?$' \
                "$build/$executable.symbols.txt"; then
            echo 'Unexpected vendor SDK import.' >&2; exit 77
        fi
    done
    # Instrument the actual executable and included-source helper tests alike.
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
        "$build/test" "$build/probe" "$root/mk2_control_bridge.example.json" "$build" "$fixtures" \
        > "$build/results.log" 2>&1 || { cat "$build/results.log" >&2; exit 1; }
    cat "$build/results.log"
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
        "$build/handover-test" "$root/mk2_control_bridge.example.json" "$fixtures" \
        > "$build/handover-results.log" 2>&1 || { cat "$build/handover-results.log" >&2; exit 1; }
    cat "$build/handover-results.log"
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
        "$build/teach-test" > "$build/teach-results.log" 2>&1 || { cat "$build/teach-results.log" >&2; exit 1; }
    tail -n 1 "$build/teach-results.log"
    for mode in --teach-virtual --check-teach-virtual --teach-external-stop-virtual --teach-overlap-stop-virtual; do
        status=0
        "$build/probe" "$mode" --path /tmp/mk2-pro/profile > "$build/mode-reject.log" 2>&1 || status=$?
        [[ $status == 77 ]] && grep -Fq 'unknown mode/profile' "$build/mode-reject.log"
    done
done
bash "$here/jeserver_v2_sdk_runner_preflight_test.sh" > "$out/preflight.log" 2>&1 || {
    cat "$out/preflight.log" >&2; exit 1;
}
cat "$out/preflight.log"
sha256sum --check "$out/inputs.sha256"
echo 'PASS: ordinary + ASan/UBSan offline tests and existing runner preflight; no real SDK linked/loaded.'