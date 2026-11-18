#!/usr/bin/env bash
# Independent C protocol component tests, NOT a ROS/CMake build or test target.
set -Eeuo pipefail
[[ $# == 0 ]] || exit 64
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
out=$(mktemp -d /tmp/je2-session-client.XXXXXXXX)
printf 'Client component test artifacts: %s\n' "$out"
read -r -a flags <<< "$(pkg-config --cflags --libs json-c)"
cc -std=c11 -Wall -Wextra -Werror -pedantic "$root/je2_session_client.c" \
    "$root/tests/je2_session_client_test.c" "${flags[@]}" -lm -o "$out/client_test"
"$out/client_test"
cc -std=c11 -Wall -Wextra -Werror -pedantic -g -fno-omit-frame-pointer \
    -fsanitize=address,undefined -fno-pie -no-pie "$root/je2_session_client.c" \
    "$root/tests/je2_session_client_test.c" "${flags[@]}" -lm -o "$out/client_sanitized"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 "$out/client_sanitized"