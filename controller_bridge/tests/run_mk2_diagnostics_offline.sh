#!/usr/bin/env bash
# Standalone C diagnostics regression. No ROS/CMake or real SDK linking/loading.
set -euo pipefail
if [[ $# != 0 || ${JE2_SDK_INCLUDE:-} != /* || ! -f ${JE2_SDK_INCLUDE:-}/HYYRobotInterface.h ]]; then
    echo 'Usage: JE2_SDK_INCLUDE=/absolute/vendor/include bash run_mk2_diagnostics_offline.sh' >&2; exit 64
fi
if [[ -n ${LD_PRELOAD:-} || -n ${LD_AUDIT:-} || -n ${LD_LIBRARY_PATH:-} ]]; then
    echo 'Refusing injected/preloaded/library search overrides' >&2; exit 64
fi
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out=$(mktemp -d /tmp/mk2-diagnostics-offline.XXXXXXXX)
printf 'Artifacts (retained): %s\n' "$out"
read -r -a flags <<< "$(pkg-config --cflags json-c libzmq openssl)"
strict=(-std=c11 -Wall -Wextra -Werror -pthread -I"$root" -isystem "$JE2_SDK_INCLUDE" "${flags[@]}")
for test in state_error servo_observation state_error_json forced_stop_rearm; do
    for mode in normal sanitized; do
        extra=()
        if [[ $mode == sanitized ]]; then
            extra=(-g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined -fno-pie -no-pie)
        fi
        binary="$out/${test}_${mode}"
        /usr/bin/cc "${strict[@]}" "${extra[@]}" -ffunction-sections -fdata-sections \
          "$root/tests/${test}_test.c" "$root/mk2_control_core.c" \
          -Wl,--gc-sections -ljson-c -lm -o "$binary"
        readelf -d "$binary" > "$binary.dynamic.txt"
        while IFS= read -r dependency; do
            case "$dependency" in
                libjson-c.so.*|libm.so.*|libc.so.*|libpthread.so.*|libasan.so.*|libubsan.so.*|ld-linux*.so.*) ;;
                *) echo "Unexpected dependency: $dependency" >&2; exit 1 ;;
            esac
        done < <(sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p' "$binary.dynamic.txt")
        ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
          "$binary" | tee "$binary.log"
    done
done
printf 'PASS: pre-stop first failure, servo observations and actual JSON adapter; normal + ASan/UBSan; no SDK loaded.\n'