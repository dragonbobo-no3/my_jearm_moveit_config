#!/usr/bin/env bash
# Standalone SDK-free tests, NOT ROS/CMake targets; never deploy or contact robot.
set -Eeuo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out=$(mktemp -d /tmp/jeserver-v2-offline.XXXXXXXX)
printf 'Retained offline test artifacts: %s\n' "$out"
flags=(-std=c11 -Wall -Wextra -Werror -pedantic -I"$root")
# These overrides affect SDK-free test compilation ONLY. They cannot enable
# RobotMain_v2_pro controller start or stand in for deployment verification.
if [[ -n "${JE2_TEST_JSON_INCLUDE:-}" || -n "${JE2_TEST_JSON_LIBRARY:-}" ]]; then
    [[ "${JE2_TEST_JSON_INCLUDE:-}" == /* && "${JE2_TEST_JSON_LIBRARY:-}" == /* &&
        -r "${JE2_TEST_JSON_INCLUDE:-}/json-c/json.h" && -r "${JE2_TEST_JSON_LIBRARY:-}" ]] || {
        echo 'Offline JSON overrides require an absolute include root and library file.' >&2; exit 78;
    }
    json_flags=(-I"$JE2_TEST_JSON_INCLUDE" "$JE2_TEST_JSON_LIBRARY")
elif command -v pkg-config >/dev/null && pkg-config --exists json-c; then
    read -r -a json_flags <<< "$(pkg-config --cflags --libs json-c)"
else
    echo 'Offline tests need json-c development headers/library; no system package was installed.' >&2
    echo 'For existing staged dependencies set JE2_TEST_JSON_INCLUDE and JE2_TEST_JSON_LIBRARY (tests only).' >&2
    exit 78
fi
cc "${flags[@]}" "$root/jeserver_v2.c" "$root/mk2_control_core.c" \
    "$root/tests/jeserver_v2_test.c" "${json_flags[@]}" -lm -o "$out/jeserver_v2_test"
"$out/jeserver_v2_test"
cc "${flags[@]}" -fsanitize=address,undefined -fno-omit-frame-pointer -g \
    "$root/jeserver_v2.c" "$root/mk2_control_core.c" "$root/tests/jeserver_v2_test.c" \
    "${json_flags[@]}" -lm -o "$out/jeserver_v2_sanitized"
ASAN_OPTIONS=detect_leaks=1 "$out/jeserver_v2_sanitized"
for test in state_error servo_observation activation_recover; do
    cc "${flags[@]}" "$root/mk2_control_core.c" "$root/tests/${test}_test.c" \
        "${json_flags[@]}" -lm -o "$out/${test}_test"
    "$out/${test}_test"
done
printf 'All offline checks passed; no SDK, network, plugin install or robot operation.\n'