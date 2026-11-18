#!/usr/bin/env bash
# Standalone tests only: vendor declarations, fake SDK definitions, host TCP.
# No ROS/CMake, remote commands, namespaces, controller or real SDK loading.
set -euo pipefail
if [[ $# != 0 ]]; then echo 'Usage: JE2_SDK_INCLUDE=/absolute/vendor/include bash run_jeserver_v2_plugin_tests.sh' >&2; exit 64; fi
if [[ ${JE2_SDK_INCLUDE:-} != /* || ! -f ${JE2_SDK_INCLUDE:-}/HYYRobotInterface.h ||
      ! -f ${JE2_SDK_INCLUDE:-}/Base/RobotStruct.h ||
      ! -f ${JE2_SDK_INCLUDE:-}/Move/MovePlan.h ]]; then
    echo 'JE2_SDK_INCLUDE must explicitly name an absolute vendor SDK include directory' >&2; exit 64
fi
if [[ -n ${LD_PRELOAD:-} || -n ${LD_AUDIT:-} || -n ${LD_LIBRARY_PATH:-} ]]; then
    echo 'Refusing injected/preloaded/library-search overrides for fake-SDK tests' >&2; exit 64
fi
for name in MK2_HOST_MNT MK2_HOST_IPC MK2_HOST_NET MK2_HOST_PID MK2_HOST_USER; do
    if [[ -v $name ]]; then echo "Refusing namespace marker $name; host guard must remain intact" >&2; exit 64; fi
done
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out=$(mktemp -d /tmp/jeserver-v2-plugin-tests.XXXXXXXX)
printf 'Artifacts (retained): %s\n' "$out"
pkg-config --exists json-c libzmq openssl
read -r -a cflags <<< "$(pkg-config --cflags json-c libzmq openssl)"
read -r -a libs <<< "$(pkg-config --libs json-c libzmq openssl)"
cc=${CC:-cc}
strict=(-std=c11 -Wall -Wextra -Werror -pedantic -pthread -isystem "$JE2_SDK_INCLUDE" "${cflags[@]}")
sources=("$root/jeserver_v2.c" "$root/jeserver_v2_transport.c" "$root/mk2_control_core.c")
"$cc" "${strict[@]}" -fPIC -shared "$root/jeserver_v2_plugin.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/libJEServer_v2_sim.so"
"$cc" "${strict[@]}" -Wl,--export-dynamic "$root/tests/jeserver_v2_plugin_test.c" \
    "$root/mk2_control_core.c" "${libs[@]}" -lm -ldl -o "$out/loader"
"$cc" "${strict[@]}" "$root/tests/jeserver_v2_runtime_test.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/runtime"
"$cc" "${strict[@]}" "$root/tests/jeserver_v2_transport_test.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/transport"
"$cc" "${strict[@]}" -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined -fno-pie -no-pie \
    "$root/tests/jeserver_v2_runtime_test.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/runtime_sanitized"
"$cc" "${strict[@]}" -DJE2_SIM_SESSION_EXPERIMENT=1 -fPIC -shared \
    "$root/jeserver_v2_plugin.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/libJEServer_v2_session.so"
"$cc" "${strict[@]}" -DJE2_SIM_SESSION_EXPERIMENT=1 \
    "$root/tests/jeserver_v2_runtime_test.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/session"
"$cc" "${strict[@]}" -DJE2_SIM_SESSION_EXPERIMENT=1 -g -O1 -fno-omit-frame-pointer \
    -fsanitize=address,undefined -fno-pie -no-pie \
    "$root/tests/jeserver_v2_runtime_test.c" "${sources[@]}" "${libs[@]}" -lm -o "$out/session_sanitized"
# Compile original branches independently, without linking/loading vendor SDK.
"$cc" "${strict[@]}" -DMK2_SIMULATION_ONLY=1 -c "$root/mk2_control_plugin.c" -o "$out/original_sim.o"
"$cc" "${strict[@]}" -DMK2_PHYSICAL_CONTROL=1 -c "$root/mk2_control_plugin.c" -o "$out/original_physical.o"
# Explicit direct dependency allowlist: no vendor .so can sneak into a build.
for binary in libJEServer_v2_sim.so libJEServer_v2_session.so loader runtime runtime_sanitized transport session session_sanitized; do
    LC_ALL=C readelf -d "$out/$binary" > "$out/$binary.dynamic.txt"
    grep -q '(NEEDED)' "$out/$binary.dynamic.txt"
    while IFS= read -r dependency; do
        case "$dependency" in
            libjson-c.so.*|libzmq.so.*|libssl.so.*|libcrypto.so.*|libm.so.*|libc.so.*|libpthread.so.*|libdl.so.*|libasan.so.*|libubsan.so.*|ld-linux*.so.*) ;;
            *) echo "Unexpected dependency: $binary: $dependency" >&2; exit 1 ;;
        esac
    done < <(sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p' "$out/$binary.dynamic.txt")
done
"$out/loader" "$out/libJEServer_v2_sim.so" wrongprofile --path /not-a-robot-profile > "$out/wrongprofile.log" 2>&1
"$out/loader" "$out/libJEServer_v2_sim.so" unisolated --path /tmp/mk2-pro/profile > "$out/unisolated.log" 2>&1
if grep -Eq 'Invalid/missing commissioning policy|DeviceConfig hash differs' "$out/unisolated.log"; then
    echo 'Policy/hash rejection masked the isolation test' >&2; exit 1
fi
cat "$out/wrongprofile.log" "$out/unisolated.log" | grep '^PASS:'
"$out/runtime" | tee "$out/runtime.log"
"$out/transport" | tee "$out/transport.log"
"$out/session" | tee "$out/session.log"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "$out/session_sanitized" | tee "$out/session_sanitized.log"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "$out/runtime_sanitized" | tee "$out/runtime_sanitized.log"
printf 'PASS: strict shared plugin + fake export loader + runtime + ASan/UBSan + original sim/physical object builds\nArtifacts: %s\n' "$out"