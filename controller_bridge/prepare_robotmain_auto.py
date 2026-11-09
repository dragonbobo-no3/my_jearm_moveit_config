#!/usr/bin/env python3
"""Prepare a minimally changed RobotMain_v2 copy; never start it or modify the input.

Existing real restart/identity/disabled checks and simulation namespace isolation
remain byte-for-byte unchanged outside the exact entrypoint/usage substitutions.
The caller backs up, reviews and installs the output separately.
"""
from pathlib import Path
import sys


def prepare(source):
    changes = [
        ("'Usage: sudo ./RobotMain_v2 [--real|--check|--sim|--check-sim]'",
         "'Usage: sudo ./RobotMain_v2 [--auto|--real|--check|--check-sim]'"),
        ("'Default: check -> new-left EtherCAT master -> original RobotMain with new-left profile.'",
         "'Default: detect slaves once; zero -> isolated SDK simulation, expected seven -> original guarded real startup.'"),
        ("'--sim: explicit isolated SDK simulation; no EtherCAT master or real startup zero.'",
         "'No runtime fallback. --sim remains a compatibility alias; normally no mode flag is needed.'"),
        ('case "${1:---start}" in', 'case "${1:---auto}" in'),
        ('--start|--real|--check|--sim|--check-sim) action=${1:---start} ;;',
         '--auto|--start|--real|--check|--sim|--check-sim) action=${1:---auto} ;;'),
        ("(( EUID == 0 )) || fail 'Run on the controller with sudo.'\n",
         "(( EUID == 0 )) || fail 'Run on the controller with sudo.'\n"
         'if [[ "$action" == --auto ]]; then\n'
         '    [[ -r "$PROFILE/select_controller_backend.sh" ]] || fail "Missing read-only backend selector."\n'
         '    backend=$(bash "$PROFILE/select_controller_backend.sh" "$EC") || fail "Backend detection failed; nothing started."\n'
         '    case "$backend" in sim) action=--sim ;; real) action=--real ;; *) fail "Invalid backend selection." ;; esac\n'
         '    echo "AUTO BACKEND=$backend; selection only, all original startup guards remain active."\n'
         'fi\n'),
    ]
    for old, new in changes:
        if source.count(old) != 1:
            raise ValueError('Launcher differs from reviewed version; refusing substitution: ' + old)
        source = source.replace(old, new, 1)
    return source


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit('Expected original and separate output launcher paths')
    original, output = map(Path, sys.argv[1:])
    if original.resolve() == output.resolve() or output.exists():
        raise SystemExit('Output must be new and separate from the original')
    output.write_text(prepare(original.read_text()))