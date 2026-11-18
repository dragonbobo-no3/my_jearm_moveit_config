#!/usr/bin/env bash
# SDK-free negative preflight tests. No compiler, bwrap, remote or SDK calls.
set -Eeuo pipefail
[[ $# == 0 ]] || { echo 'No arguments accepted.' >&2; exit 64; }
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
runner=$here/run_jeserver_v2_sdk_probe.sh
out=$(mktemp -d /tmp/jeserver-v2-sdk-preflight.XXXXXXXX)
trap 'rm -rf -- "$out"' EXIT
bash -n "$runner"
bash -n "${BASH_SOURCE[0]}"
printf '%s\n' '---' 'version:' '  major: 12' '  minor: 2' '  build: 1' \
    'serial: JE' 'description: Isolated JEServer_v2 SDK lifecycle probe' \
    'app_path: /tmp/mk2-pro/profile/app' 'simulation: true' > "$out/expected.yaml"
printf '%s' 'plugin_list: []' >> "$out/expected.yaml"
if ! cmp -s "$out/expected.yaml" "$here/jeserver_v2_sdk_GlobalConfig.yaml"; then
    # The companion guard allows exactly one optional final LF, nothing else.
    printf '\n' >> "$out/expected.yaml"
    cmp "$out/expected.yaml" "$here/jeserver_v2_sdk_GlobalConfig.yaml"
fi
expect() {
    local expected=$1 text=$2 result=0
    shift 2
    "$@" > "$out/log" 2>&1 || result=$?
    if [[ $result != "$expected" ]] || ! grep -Fq -- "$text" "$out/log"; then
        cat "$out/log" >&2
        printf 'FAIL: expected exit=%s text=%s, got exit=%s\n' "$expected" "$text" "$result" >&2
        exit 1
    fi
}
expect 64 Usage: bash "$runner"
expect 64 Usage: bash "$runner" --init
expect 64 Usage: bash "$runner" --load --path /tmp/mk2-pro/profile
expect 64 Usage: bash "$runner" --prepare-policy
if (( EUID == 0 )); then
    expect 77 'Refusing root' bash "$runner" --check
    expect 77 'Refusing root' bash "$runner" --load
    expect 77 'Refusing root' bash "$runner" --writer-exit-virtual
    expect 77 'Refusing root' bash "$runner" --handover-virtual
    expect 77 'Refusing root' bash "$runner" --check-handover-virtual
    expect 77 'Refusing root' bash "$runner" --teach-virtual
    expect 77 'Refusing root' bash "$runner" --check-teach-virtual
    expect 77 'Refusing root' bash "$runner" --teach-external-stop-virtual
    expect 77 'Refusing root' bash "$runner" --teach-overlap-stop-virtual
    echo 'PASS: syntax/arguments/root refusal; normal-uid injection and RT cases skipped as root.'
    exit 0
fi
# Empty LD variables are safe for the shell loader but still forbidden by the
# runner; test presence, not merely nonempty values. Namespace markers likewise.
for name in LD_PRELOAD LD_AUDIT LD_LIBRARY_PATH MK2_HOST_MNT MK2_HOST_IPC \
            MK2_HOST_NET MK2_HOST_PID MK2_HOST_USER JE2_HOST_MNT \
            JE2_EXPECTED_PLUGIN_SHA256; do
    expect 64 'Refusing inherited injection/namespace override:' \
        env -u LD_PRELOAD -u LD_AUDIT -u LD_LIBRARY_PATH \
            -u MK2_HOST_MNT -u MK2_HOST_IPC -u MK2_HOST_NET -u MK2_HOST_PID \
            -u MK2_HOST_USER -u JE2_EXPECTED_PLUGIN_SHA256 "$name=" bash "$runner" --check
    expect 64 'Refusing inherited injection/namespace override:' \
        env -u LD_PRELOAD -u LD_AUDIT -u LD_LIBRARY_PATH \
            -u MK2_HOST_MNT -u MK2_HOST_IPC -u MK2_HOST_NET -u MK2_HOST_PID \
            -u MK2_HOST_USER -u JE2_EXPECTED_PLUGIN_SHA256 "$name=" bash "$runner" --check-handover-virtual
    expect 64 'Refusing inherited injection/namespace override:' \
        env -u LD_PRELOAD -u LD_AUDIT -u LD_LIBRARY_PATH \
            -u MK2_HOST_MNT -u MK2_HOST_IPC -u MK2_HOST_NET -u MK2_HOST_PID \
            -u MK2_HOST_USER -u JE2_EXPECTED_PLUGIN_SHA256 "$name=" bash "$runner" --check-teach-virtual
done
# Lower only the child soft limit. Never raise a limit or call sudo/prlimit.
(
    unset LD_PRELOAD LD_AUDIT LD_LIBRARY_PATH MK2_HOST_MNT MK2_HOST_IPC \
        MK2_HOST_NET MK2_HOST_PID MK2_HOST_USER JE2_HOST_MNT JE2_EXPECTED_PLUGIN_SHA256
    ulimit -Sr 0
    expect 78 'Realtime soft ceiling must be >=38' bash "$runner" --load
    expect 78 'Realtime soft ceiling must be >=38' bash "$runner" --writer-exit-virtual
    expect 78 'Realtime soft ceiling must be >=38' bash "$runner" --handover-virtual
    expect 78 'Realtime soft ceiling must be >=38' bash "$runner" --teach-virtual
    expect 78 'Realtime soft ceiling must be >=38' bash "$runner" --teach-external-stop-virtual
    expect 78 'Realtime soft ceiling must be >=38' bash "$runner" --teach-overlap-stop-virtual
    # A separate empty source tree makes this deterministic even if the main
    # agent finishes the real C file concurrently. --check with RT=0 must pass
    # the RT gate and stop at missing C, before dependencies/staging/SDK work.
    mkdir "$out/tests"
    cp -- "$runner" "$out/tests/run_jeserver_v2_sdk_probe.sh"
    expect 78 'BLOCKED: tests/jeserver_v2_sdk_probe.c is not ready' \
        bash "$out/tests/run_jeserver_v2_sdk_probe.sh" --check
    expect 78 'BLOCKED: tests/jeserver_v2_sdk_probe.c is not ready' \
        bash "$out/tests/run_jeserver_v2_sdk_probe.sh" --check-handover-virtual
    expect 78 'BLOCKED: tests/jeserver_v2_sdk_probe.c is not ready' \
        bash "$out/tests/run_jeserver_v2_sdk_probe.sh" --check-teach-virtual
)
echo 'PASS: syntax/arguments/injection/namespace/root-or-normal-uid/RT preflight; no SDK loaded.'