#!/usr/bin/env bash
# Standalone C/Linux tests: no SDK, network, CMake, or robot access.
set -Eeuo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
out=$(mktemp -d /tmp/jeserver-v2-task-watch.XXXXXXXX)
printf 'Retained task evidence tests: %s\n' "$out"
flags=(-std=c11 -Wall -Wextra -Werror -pedantic -pthread -I"$root")
sources=("$root/jeserver_v2_task_watch.c" "$root/tests/jeserver_v2_task_watch_test.c")
cc "${flags[@]}" "${sources[@]}" -o "$out/test"
"$out/test"
cc "${flags[@]}" -fsanitize=address,undefined -fno-omit-frame-pointer -g \
    "${sources[@]}" -o "$out/test-sanitized"
ASAN_OPTIONS=detect_leaks=1 "$out/test-sanitized"