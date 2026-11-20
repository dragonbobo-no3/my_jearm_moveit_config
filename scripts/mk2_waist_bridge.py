#!/usr/bin/env python3
"""Receive-only waist feedback bridge: ZMQ WaistFeedback -> /joint_states (waist_joint).

Subscribes to mk2_waist_server on the controller (CAN can1, read-only) and
publishes only the waist joint. Never sends any command. When feedback is
stale nothing is published, so MoveIt/RViz see the waist state go stale
instead of a guessed value.
"""

import argparse
import json
import math
import time

import zmq

PREFIX = b'WaistFeedback '


def parse_feedback(message, key='Waist'):
    if not message.startswith(PREFIX):
        raise ValueError('Not a WaistFeedback message')
    payload = json.loads(message[len(PREFIX):])
    waist = payload[key] if isinstance(payload, dict) else None
    if not isinstance(waist, dict) or waist.get('feedback_valid') is not True:
        raise ValueError('Waist feedback missing or invalid')
    values = []
    for field in ('position', 'velocity'):
        value = waist.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{field} is not a finite number')
        values.append(float(value))
    return values[0], values[1], waist


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.0.99')
    parser.add_argument('--port', type=int, default=8005)
    parser.add_argument('--key', default='Waist')
    parser.add_argument('--joint-name', default='waist_joint')
    parser.add_argument('--sign', type=float, default=1.0, choices=[1.0, -1.0],
                        help='URDF waist angle = sign * protocol angle')
    parser.add_argument('--timeout', type=float, default=0.5,
                        help='Warn when no valid feedback for this many seconds')
    args, ros_args = parser.parse_known_args(argv)
    if not 1 <= args.port <= 65535 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Port must be 1..65535 and timeout must be finite and positive')

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.LINGER, 0)
    subscriber.setsockopt(zmq.RCVHWM, 10)
    subscriber.setsockopt(zmq.SUBSCRIBE, PREFIX)
    endpoint = f'tcp://{args.robot_ip}:{args.port}'
    subscriber.connect(endpoint)

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mk2_waist_bridge')
    publisher = node.create_publisher(JointState, 'joint_states', 10)
    print(f'Waist read-only SUB {endpoint} -> /joint_states[{args.joint_name}] sign={args.sign:+.0f}; '
          'no commands are sent', flush=True)
    last_valid = None
    stale_reported = False
    last_warning = 0.0
    last_fault = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            if last_valid is not None and now - last_valid > args.timeout and not stale_reported:
                print(f'WARNING: no waist feedback for >{args.timeout:.1f} s; waist state is stale', flush=True)
                stale_reported = True
            if not subscriber.poll(50):
                continue
            try:
                position, velocity, waist = parse_feedback(subscriber.recv(), args.key)
            except (ValueError, KeyError, TypeError, UnicodeError) as error:
                if now - last_warning > 1:
                    print(f'Rejected waist feedback: {error}', flush=True)
                    last_warning = now
                continue
            if last_valid is None or stale_reported:
                print(f'Waist feedback: {math.degrees(position):.3f} deg (raw {waist.get("position_bit")} bit, '
                      f'zero {waist.get("zero_bit")} bit), fault={waist.get("fault")} '
                      f'mode={waist.get("control_mode")} udc={waist.get("udc")} V', flush=True)
            if waist.get('fault') != last_fault:
                if waist.get('fault'):
                    print(f'WARNING: waist fault 0x{int(waist["fault"]):04x}', flush=True)
                last_fault = waist.get('fault')
            last_valid = now
            stale_reported = False
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = [args.joint_name]
            message.position = [args.sign * position]
            message.velocity = [args.sign * velocity]
            publisher.publish(message)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        subscriber.close()
        context.term()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
