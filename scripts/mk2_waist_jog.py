#!/usr/bin/env python3
"""Keyboard jog for the MK2 waist (PC side) with live feedback.

Needs mk2_waist_server on the controller started with --enable-control
(it SUB-connects to this PC's --cmd-bind). Keys:
  e        enable (holds the current position, verifies CAN byte order)
  <- / ->  tap: jog -step / +step; hold the key: move continuously, release: stop
  up/down  step size
  space    hold (stop at the current command)
  d        disable (stop, then release once stationary)
  z        set the current position as waist zero (confirm with y)
  q / Esc  disable and quit
The server holds and then releases by itself if this script stops sending.
"""

import argparse
import curses
import json
import locale
import math
import time

import zmq

FEEDBACK_PREFIX = b'WaistFeedback '
STEPS = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0]
HEARTBEAT_S = 0.1
ACTION_REPEAT_S = 0.3
REPEAT_GAP_S = 0.15     # same arrow again this fast -> autorepeat, key is held
RUN_RELEASE_S = 0.2     # no autorepeat for this long -> key released
RUN_REFRESH_S = 0.1     # server holds if run is not refreshed for 0.5 s
ZERO_CONFIRM_S = 5.0    # z must be confirmed with y within this time


class Link:
    def __init__(self, args):
        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 500)
        self.pub.bind(args.cmd_bind)
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt(zmq.SUBSCRIBE, FEEDBACK_PREFIX)
        self.sub.connect(f'tcp://{args.robot_ip}:{args.feedback_port}')
        self.key = args.key
        self.seq = int(time.time() * 1000)  # unique across restarts
        self.msg = {'seq': self.seq, 'op': 'ping'}
        self.msg_time = 0.0
        self.last_send = 0.0
        self.feedback = None
        self.feedback_time = 0.0

    def command(self, op, delta_deg=None, **fields):
        self.seq += 1
        self.msg = {'seq': self.seq, 'op': op, **fields}
        if delta_deg is not None:
            self.msg['delta_deg'] = delta_deg
        self.msg_time = time.monotonic()
        self._send()

    def _send(self):
        self.pub.send_string('WaistCmd ' + json.dumps(self.msg))
        self.last_send = time.monotonic()

    def heartbeat(self):
        now = time.monotonic()
        # Repeat an action briefly (PUB/SUB may drop the first frames), then
        # switch to a fresh 'ping' so a restarted server never replays it.
        if self.msg['op'] != 'ping' and now - self.msg_time > ACTION_REPEAT_S:
            self.seq += 1
            self.msg = {'seq': self.seq, 'op': 'ping'}
        if now - self.last_send >= HEARTBEAT_S:
            self._send()

    def poll(self):
        try:
            while True:
                raw = self.sub.recv(zmq.NOBLOCK)
                payload = json.loads(raw[len(FEEDBACK_PREFIX):])
                waist = payload.get(self.key) if isinstance(payload, dict) else None
                if isinstance(waist, dict) and waist.get('feedback_valid') is True:
                    self.feedback = waist
                    self.feedback_time = time.monotonic()
        except zmq.Again:
            pass
        except ValueError:
            pass

    def age(self):
        return time.monotonic() - self.feedback_time if self.feedback else math.inf

    def close(self):
        self.pub.close()
        self.sub.close()
        self.ctx.term()


def fmt(value, spec='+8.3f'):
    return format(value, spec) if isinstance(value, (int, float)) else '   -    '


def draw(scr, link, args, step, events, zero_pending=False):
    scr.erase()
    fb = link.feedback or {}
    ctl = fb.get('control') or {}
    age = link.age()
    stale = age > 0.5
    row = 0

    def put(text, attr=0):
        nonlocal row
        try:
            scr.addstr(row, 0, text[: max(1, curses.COLS - 1)], attr)
        except curses.error:
            pass
        row += 1

    put(f'MK2 waist jog   feedback tcp://{args.robot_ip}:{args.feedback_port}   cmd {args.cmd_bind}',
        curses.A_BOLD)
    put('')
    if stale:
        put(f'  FEEDBACK STALE ({age:.1f} s) - is mk2_waist_server running / waist powered?', curses.A_REVERSE)
    put(f'  position   {fmt(fb.get("position_deg"))} deg     raw {fb.get("position_bit", "-")}  '
        f'zero {fb.get("zero_bit", "-")}', curses.A_BOLD)
    vel = fb.get('velocity')
    put(f'  velocity   {fmt(math.degrees(vel) if isinstance(vel, (int, float)) else None)} deg/s   '
        f'motor {fmt(fb.get("motor_rpm"), "+7.1f")} rpm')
    put(f'  fault 0x{fb.get("fault", 0):04x}   mode {fb.get("control_mode", "-")}   '
        f'state {fb.get("motor_state", "-")}   Udc {fmt(fb.get("udc"), ".1f")} V   Iq {fmt(fb.get("iq"), "+.3f")} A')
    put(f'  temp motor/mos/mcu {fb.get("temp_motor", "-")}/{fb.get("temp_mos", "-")}/{fb.get("temp_mcu", "-")} C'
        f'   pos err {fb.get("position_err_bit", "-")} bit   age {age * 1000:.0f} ms' if not stale else '')
    put('')
    if not ctl.get('available'):
        put('  control: NOT AVAILABLE (server is read-only; restart it with --enable-control)', curses.A_REVERSE)
    else:
        state = ctl.get('state', '?')
        put(f'  control    {state.upper():10s}  limit +-{ctl.get("limit_deg", 0):.1f} deg  '
            f'max step {ctl.get("max_step_deg", 0):.1f} deg  '
            f'{"ZEROING" if ctl.get("zeroing") else "RUNNING" if ctl.get("running") else "MOVING" if ctl.get("moving") else ""}',
            curses.A_BOLD)
        if state != 'disabled':
            put(f'  target     {fmt(ctl.get("goal_deg"))} deg   planner {fmt(ctl.get("command_deg"))} deg')
        if ctl.get('error'):
            put(f'  last error: {ctl["error"]}', curses.A_REVERSE)
        elif ctl.get('note'):
            put(f'  note: {ctl["note"]}')
    put('')
    put(f'  step {step:.1f} deg    [e] enable  [<-/->] jog -/+  [up/down] step  [space] hold  '
        f'[d] disable  [z] set zero  [q] quit')
    if zero_pending:
        put(f'  SET ZERO at {fmt(fb.get("position_deg"))} deg?  press [y] to confirm, any other key cancels',
            curses.A_REVERSE)
    put('')
    for line in events[-8:]:
        put('  ' + line)
    scr.refresh()


def control_state(link):
    fb = link.feedback or {}
    return (fb.get('control') or {}).get('state')


def shutdown(scr, link, args, step, events):
    if control_state(link) not in (None, 'disabled'):
        link.command('disable')
        events.append('quit: disabling ...')
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and control_state(link) != 'disabled':
            link.poll()
            link.heartbeat()
            draw(scr, link, args, step, events)
            time.sleep(0.05)


def run(scr, args, link):
    curses.curs_set(0)
    scr.nodelay(True)
    scr.keypad(True)
    step_i = STEPS.index(args.step) if args.step in STEPS else STEPS.index(1.0)
    events = []
    last_error = None
    sign = -1 if args.invert else 1
    held_dir = 0           # continuous run in progress (+-1)
    last_dir, last_arrow_t, last_run_t = 0, 0.0, 0.0
    zero_t = None          # time z was pressed, waiting for y
    try:
        while True:
            link.poll()
            link.heartbeat()
            ctl = (link.feedback or {}).get('control') or {}
            if ctl.get('error') and ctl.get('error') != last_error:
                events.append(f'{time.strftime("%H:%M:%S")} server: {ctl["error"]}')
            last_error = ctl.get('error')
            step = STEPS[step_i]
            now = time.monotonic()
            if held_dir and now - last_arrow_t > RUN_RELEASE_S:
                link.command('hold')
                held_dir = 0
                pos = (link.feedback or {}).get('position_deg')
                events.append(f'{time.strftime("%H:%M:%S")} released -> stop'
                              + (f'  (pos {pos:+.3f} deg)' if isinstance(pos, (int, float)) else ''))
            if zero_t is not None and now - zero_t > ZERO_CONFIRM_S:
                zero_t = None
                events.append(f'{time.strftime("%H:%M:%S")} set zero cancelled (no confirmation)')
            draw(scr, link, args, step, events, zero_t is not None)

            key = scr.getch()
            if key == -1:
                time.sleep(0.02)
                continue
            pos = (link.feedback or {}).get('position_deg')
            stamp = time.strftime('%H:%M:%S')
            pos_s = f'{pos:+.3f}' if isinstance(pos, (int, float)) else '?'
            if zero_t is not None:
                zero_t = None
                if key in (ord('y'), ord('Y')):
                    if link.age() > 0.2:
                        events.append(f'{stamp} set zero ignored: no fresh feedback')
                    elif ctl.get('moving') or held_dir:
                        events.append(f'{stamp} set zero ignored: waist is moving')
                    else:
                        link.command('set_zero')
                        events.append(f'{stamp} set zero  (pos {pos_s} deg) - keep the waist still')
                    continue
                events.append(f'{stamp} set zero cancelled')
                if key not in (ord('q'), ord('Q'), 27, ord(' '), ord('d'), ord('D')):
                    continue  # the cancelling key does nothing else (no accidental jog)
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('z'), ord('Z')):
                if not ctl.get('available'):
                    events.append(f'{stamp} set zero ignored: server has no control path')
                else:
                    zero_t = time.monotonic()
            elif key in (ord('e'), ord('E')):
                if link.age() > 0.2:
                    events.append(f'{stamp} enable ignored: no fresh feedback')
                else:
                    link.command('enable')
                    events.append(f'{stamp} enable  (pos {pos_s} deg)')
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                d = (1 if key == curses.KEY_RIGHT else -1) * sign
                now = time.monotonic()
                if ctl.get('state') != 'active':
                    events.append(f'{stamp} jog ignored: control is {ctl.get("state", "unavailable")} (press e)')
                elif held_dir == d:
                    if now - last_run_t >= RUN_REFRESH_S:
                        link.command('run', dir=d)
                        last_run_t = now
                elif held_dir:
                    link.command('hold')
                    held_dir = 0
                elif last_dir == d and now - last_arrow_t < REPEAT_GAP_S:
                    held_dir = d
                    link.command('run', dir=d)
                    last_run_t = now
                    events.append(f'{stamp} key held: run {"+" if d > 0 else "-"}  (pos {pos_s} deg)')
                elif ctl.get('moving'):
                    pass  # one step at a time; the server ignores jogs while moving
                else:
                    delta = step * d
                    link.command('jog', delta)
                    events.append(f'{stamp} jog {delta:+.1f} deg  (pos {pos_s} deg)')
                last_dir, last_arrow_t = d, now
            elif key == curses.KEY_UP:
                step_i = min(step_i + 1, len(STEPS) - 1)
            elif key == curses.KEY_DOWN:
                step_i = max(step_i - 1, 0)
            elif key == ord(' '):
                link.command('hold')
                events.append(f'{stamp} hold  (pos {pos_s} deg)')
            elif key in (ord('d'), ord('D')):
                link.command('disable')
                events.append(f'{stamp} disable  (pos {pos_s} deg)')
    except KeyboardInterrupt:
        pass
    shutdown(scr, link, args, STEPS[step_i], events)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot-ip', default='192.168.0.99')
    parser.add_argument('--feedback-port', type=int, default=8005)
    parser.add_argument('--cmd-bind', default='tcp://*:8006',
                        help='PUB bind the controller connects to (server --cmd-endpoint)')
    parser.add_argument('--key', default='Waist')
    parser.add_argument('--step', type=float, default=1.0, choices=STEPS, help='initial step (deg)')
    parser.add_argument('--invert', action='store_true', help='swap the left/right key direction')
    args = parser.parse_args()

    locale.setlocale(locale.LC_ALL, '')
    link = Link(args)
    try:
        curses.wrapper(run, args, link)
    finally:
        fb = link.feedback or {}
        ctl = fb.get('control') or {}
        print(f'waist position {fb.get("position_deg", float("nan")):+.3f} deg, '
              f'control {ctl.get("state", "unavailable")}'
              + (f', last error: {ctl["error"]}' if ctl.get('error') else ''))
        link.close()


if __name__ == '__main__':
    main()
