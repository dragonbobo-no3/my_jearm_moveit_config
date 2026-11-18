"""Pure, fail-closed ownership policy. No sockets, ROS or SDK calls."""

import math


class OwnershipError(ValueError):
    pass


class Ownership:
    def __init__(self, backend, device_sha256, bounds, timeout=0.15):
        if backend not in ('physical', 'sdk_simulation') or len(bounds) != 7:
            raise ValueError('Invalid ownership configuration')
        self.backend, self.sha, self.bounds = backend, device_sha256, bounds
        self.timeout = timeout
        self.state, self.reason = 'UNAVAILABLE', 'Waiting for native state'
        self.sequence, self.received = None, float('-inf')
        self.bridge, self.robot = {}, {}
        self.stable_since, self.anchor = None, None
        self.generation, self.request_sequence = 0, None
        self.standby_confirmed = False
        self.release_ready_sequence = None

    def confirm_standby(self):
        self.standby_confirmed = True
        if self.state == 'UNAVAILABLE' and self.sequence is not None:
            self.state, self.reason = 'PENDANT', 'Hardware and trajectory controller confirmed inactive'

    def fault(self, reason):
        self.state, self.reason = 'FAULT', reason

    def feed(self, frame, now):
        if self.state == 'FAULT':
            return  # A new frame never resets a fault.
        try:
            b, r = frame['Bridge'], frame['Robot0']
            physical = self.backend == 'physical'
            if (b['mode'] != 'control_guarded' or b['backend'] != self.backend or
                    b['simulation_only'] is not (not physical) or
                    b['simulation_isolated'] is not (not physical)):
                raise OwnershipError('Native backend/isolation mismatch')
            if physical and (b.get('configuration_verified') is not True or
                    b.get('profile') != '/home/robot/Work/system/robot_config_new_left' or
                    b.get('device_config_sha256') != self.sha):
                raise OwnershipError('Physical configuration fingerprint mismatch')
            seq, q = b['sequence'], r['Joint']
            if type(seq) is not int or seq < 0 or len(q) != 7 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in q):
                raise OwnershipError('Incomplete/nonfinite native state')
            if self.sequence is not None and seq < self.sequence:
                raise OwnershipError('Native session/sequence changed')
            if seq == self.sequence:
                return  # Duplicates never extend freshness or settling time.
            if self.sequence is not None and now - self.received > self.timeout:
                raise OwnershipError('Native feedback gap; a late frame cannot renew ownership')
            if any(not low <= v <= high for v, (low, high) in zip(q, self.bounds)):
                raise OwnershipError('Native position outside configured bounds')
            if b['fault'] != 'none' or type(b['sdk_error']) is not int or b['sdk_error'] != 0:
                raise OwnershipError('Native fault: ' + str(b.get('fault')))
            allowed = {'ready'} if self.state in ('UNAVAILABLE', 'PENDANT') else {
                'ready', 'waiting_first_joint', 'active', 'releasing'}
            if b['control_state'] not in allowed:
                raise OwnershipError('Unexpected native control owner/phase')
            if self.state == 'MOVEIT' and b['control_state'] != 'active':
                raise OwnershipError('MoveIt ownership lost')
            if self.state in ('ACQUIRING', 'MOVEIT', 'RELEASING'):
                if not self._healthy(r):
                    raise OwnershipError('SDK health/power lost during ownership transition/control')
                if b.get('ownership_handover_supported') is not True or b.get('clean_release_supported') is not True:
                    raise OwnershipError('Native handover capability lost')
            if self.state == 'MOVEIT' and b.get('commands_enabled') is not True:
                raise OwnershipError('Native command permission lost')
            self.sequence, self.received = seq, now
            self.bridge, self.robot = b, r
            if self.state == 'RELEASING' and b['control_state'] == 'ready' and self.release_ready_sequence is None:
                # READY from the capability-gated native bridge is emitted only
                # after StopRun + its own stationary check + successful ServoEnd.
                self.release_ready_sequence = seq
                self.stable_since, self.anchor = None, None
            velocity = r.get('JointVelocity')
            stationary = self._healthy(r) and r['MoveState'] == 0
            if velocity is not None:
                stationary = stationary and len(velocity) == 7 and all(
                    type(v) in (int, float) and math.isfinite(v) and abs(v) <= 0.01
                    for v in velocity)
            if not stationary:
                self.anchor, self.stable_since = None, None
            elif self.anchor is None or any(abs(a - v) > 0.001 for a, v in zip(self.anchor, q)):
                self.anchor, self.stable_since = list(q), now
            if self.state == 'UNAVAILABLE' and self.standby_confirmed:
                self.state, self.reason = 'PENDANT', 'Hardware and trajectory controller confirmed inactive'
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.fault(str(error))

    @staticmethod
    def _healthy(r):
        return all(type(r.get(k)) is int and r[k] == v for k, v in (
            ('PowerState', 1), ('DriverError', 0), ('MoveError', 0))) and (
            type(r.get('MoveState')) is int and r['MoveState'] >= 0 and r['MoveState'] != 7)

    def fresh(self, now):
        return self.sequence is not None and 0 <= now - self.received <= self.timeout

    def settled(self, now):
        return self.fresh(now) and self.stable_since is not None and now - self.stable_since >= 0.25

    def tick(self, now):
        if self.state not in ('UNAVAILABLE', 'FAULT') and not self.fresh(now):
            self.fault('Native feedback missing/stale')

    def request(self, moveit, now, goal_active=False):
        expected = 'PENDANT' if moveit else 'MOVEIT'
        if self.state != expected:
            raise OwnershipError('Expected owner ' + expected + ', observed ' + self.state)
        if not self.settled(now):
            raise OwnershipError('Require fresh, healthy, idle and settled axes for 250 ms')
        if moveit and (self.bridge.get('ownership_handover_supported') is not True or
                self.bridge.get('clean_release_supported') is not True or
                self.bridge.get('commands_enabled') is not True):
            raise OwnershipError('Native firmware has no verified ownership-handover capability')
        if not moveit and goal_active:
            raise OwnershipError('Cancel/finish the active trajectory before requesting pendant ownership')
        self.generation += 1
        self.request_sequence = self.sequence
        self.release_ready_sequence = None
        self.stable_since, self.anchor = None, None
        self.state = 'ACQUIRING' if moveit else 'RELEASING'
        self.reason = 'Transition requested; do not move from the other control entrance'

    def finish(self, now):
        if not self.fresh(now) or self.sequence <= self.request_sequence:
            return False
        phase = self.bridge['control_state']
        if self.state == 'ACQUIRING' and phase == 'active':
            self.state, self.reason = 'MOVEIT', 'PC owns servo; pendant motion prohibited'
            return True
        if self.state == 'RELEASING' and phase == 'ready' and self.settled(now):
            self.state, self.reason = 'PENDANT', 'Native release and fresh stationary feedback confirmed'
            return True
        return False