#!/usr/bin/env python3
"""Receive-only ZMQ state bridge/preflight. Never opens a command socket."""

import argparse
import json
import math
import time

import zmq


def parse_state(frames, robot_key='Robot0'):
    """Accept only complete, finite seven-axis feedback, never fill with zeros."""
    message = b' '.join(frames)
    prefix = b'State '
    if not message.startswith(prefix):
        raise ValueError('Not a State message')
    state = json.loads(message[len(prefix):])
    if not isinstance(state, dict):
        raise ValueError('State payload must be a JSON object')
    robot = state[robot_key]
    if not isinstance(robot, dict):
        raise ValueError(f'{robot_key} must be a JSON object')

    def array(lower, upper, required=False):
        value = robot.get(lower, robot.get(upper))
        if value is None and not required:
            return []
        if not isinstance(value, list) or len(value) != 7:
            raise ValueError(f'{lower} must contain exactly seven axes')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in value):
            raise ValueError(f'{lower} contains a nonfinite or nonnumeric value')
        return [float(v) for v in value]

    return (array('joint', 'Joint', True), array('jointVelocity', 'JointVelocity'),
            array('jointSensorTorque', 'JointSensorTorque'))


def command_bridge_available(frames):
    """Require explicit capability; state feedback alone cannot prove control."""
    message = b' '.join(frames)
    if not message.startswith(b'State '):
        return False
    state = json.loads(message[6:])
    metadata = state.get('Bridge') if isinstance(state, dict) else None
    return (isinstance(metadata, dict) and metadata.get('commands_enabled') is True
            and metadata.get('mode') != 'feedback_only')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.0.99')
    parser.add_argument('--state-port', type=int, default=8000)
    parser.add_argument('--robot-key', default='Robot0', choices=['Robot0'])
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--startup-timeout', type=float,
                        help='Initial connection deadline; subsequent valid-state timeout remains --timeout')
    parser.add_argument('--limits-file', help='Optional hardware_mk2.yaml for in-bounds preflight')
    parser.add_argument('--probe-only', action='store_true')
    parser.add_argument('--require-command-bridge', action='store_true',
                        help='Refuse driver activation unless State explicitly advertises command support')
    args, ros_args = parser.parse_known_args(argv)
    if not 1 <= args.state_port <= 65535 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Port must be 1..65535 and timeout must be finite and positive')
    if args.startup_timeout is not None and (not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0):
        parser.error('Startup timeout must be finite and positive')
    bounds = None
    if args.limits_file:
        import yaml
        with open(args.limits_file, encoding='utf-8') as stream:
            bounds = yaml.safe_load(stream)['axes']
        if [axis['name'] for axis in bounds] != [f'joint{i}' for i in range(1, 8)]:
            parser.error('Limits must specify joint1..joint7 in SDK order')

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.LINGER, 0)
    subscriber.setsockopt(zmq.RCVHWM, 10)
    subscriber.setsockopt(zmq.SUBSCRIBE, b'State ')
    endpoint = f'tcp://{args.robot_ip}:{args.state_port}'
    subscriber.connect(endpoint)
    node = None
    ros = None
    publisher = None
    try:
        if not args.probe_only:
            import rclpy
            from sensor_msgs.msg import JointState
            ros = rclpy
            ros.init(args=ros_args)
            node = ros.create_node('mk2_state_bridge')
            publisher = node.create_publisher(JointState, 'joint_states', 10)
        print(f'Read-only SUB {endpoint}; no Switch/Joint/Gripper commands', flush=True)
        deadline = time.monotonic() + (args.startup_timeout if args.startup_timeout is not None else args.timeout)
        last_warning = 0.0
        connected = False
        while ros is None or ros.ok():
            if time.monotonic() >= deadline:
                print(f'No fresh valid seven-axis feedback from {endpoint}; stopping. '
                        'Check the active controller profile/State publisher (physical or SDK simulation). '
                        'No zero state is published.', flush=True)
                return 2
            if node is not None:
                ros.spin_once(node, timeout_sec=0)
            if not subscriber.poll(50):
                continue
            try:
                frames = subscriber.recv_multipart()
                positions, velocities, efforts = parse_state(frames, args.robot_key)
            except (ValueError, KeyError, TypeError, UnicodeError) as error:
                if time.monotonic() - last_warning > 1:
                    print(f'Rejected state: {error}', flush=True)
                    last_warning = time.monotonic()
                continue
            deadline = time.monotonic() + args.timeout
            if bounds is not None and any(not axis['smin'] <= q <= axis['smax']
                                          for axis, q in zip(bounds, positions)):
                print('Feedback is outside the configured SDK joint bounds; refusing activation.', flush=True)
                return 3
            if args.require_command_bridge and not command_bridge_available(frames):
                print('Controller bridge does not explicitly advertise command support; '
                      'refusing write-driver activation. Feedback-only mode remains available.', flush=True)
                return 4
            if not connected:
                print(f'Valid {args.robot_key} feedback (SDK radians): {positions}', flush=True)
                connected = True
                if not args.probe_only:
                    print('MK2_FEEDBACK_READY', flush=True)
            if args.probe_only:
                return 0
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = [f'joint{i}' for i in range(1, 8)]
            message.position, message.velocity, message.effort = positions, velocities, efforts
            publisher.publish(message)
        return 0
    except KeyboardInterrupt:
        # A cancelled probe must not be mistaken for a successful preflight.
        return 2 if args.probe_only else 0
    finally:
        subscriber.close()
        context.term()
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())