#!/usr/bin/env python3
"""Explicit cooperative ownership supervisor; never opens a native command PUB."""

import json
import time

import rclpy
import yaml
import zmq
from action_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import (ListControllers, ListHardwareComponents,
                                         SetHardwareComponentState, SwitchController)
from lifecycle_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from mk2_ownership import Ownership, OwnershipError


class ControlOwner(Node):
    def __init__(self):
        super().__init__('mk2_control_owner')
        for name, value in (('robot_ip', '192.168.0.99'), ('state_port', 8000),
                            ('backend', 'sdk_simulation'), ('limits_file', '')):
            self.declare_parameter(name, value)
        with open(self.get_parameter('limits_file').value, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        self.policy = Ownership(self.get_parameter('backend').value, config['source']['sha256'],
                                [(x['smin'], x['smax']) for x in config['axes']])
        self.zmq_context = zmq.Context()
        self.sub = self.zmq_context.socket(zmq.SUB)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.setsockopt(zmq.RCVHWM, 8)
        self.sub.setsockopt(zmq.SUBSCRIBE, b'State ')
        self.sub.connect('tcp://{}:{}'.format(self.get_parameter('robot_ip').value,
                                             self.get_parameter('state_port').value))
        self.hardware = self.create_client(SetHardwareComponentState,
                                            '/controller_manager/mk2_set_hardware_component_state')
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_hardware = self.create_client(ListHardwareComponents,
                            '/controller_manager/list_hardware_components')
        self.list_controllers = self.create_client(ListControllers,
                               '/controller_manager/list_controllers')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/mk2/control_owner/state', qos)
        self.service = self.create_service(SetBool, '/mk2/control_owner/set_moveit', self.request)
        self.goal_active = False
        self.goal_sub = self.create_subscription(GoalStatusArray,
            '/jearm_controller/follow_joint_trajectory/_action/status', self.status, qos)
        self.steps = [('check_hardware', None), ('check_controllers', None)]
        self.future, self.deadline = None, None
        self.pending_kind = None
        self.started = time.monotonic()
        self.timer = self.create_timer(0.02, self.poll)
        self.get_logger().info('Owner starts UNAVAILABLE/read-only; PENDANT requires inactive checks. '
                       'SetBool true requests MoveIt; '
                               'false requests pendant. A service ACK is NOT a completed handover.')

    def status(self, message):
        self.goal_active = any(x.status in (1, 2, 3) for x in message.status_list)

    def request(self, request, response):
        try:
            if self.steps or self.future is not None:
                raise OwnershipError('A handover is already in progress')
            if not self.hardware.service_is_ready() or not self.switch.service_is_ready():
                raise OwnershipError('Controller manager services not ready')
            self.policy.request(request.data, time.monotonic(), self.goal_active)
            self.steps = [('hardware', State.PRIMARY_STATE_ACTIVE), ('switch', True)] if request.data else [
                ('switch', False), ('hardware', State.PRIMARY_STATE_INACTIVE)]
            self.deadline = time.monotonic() + 8.0
            response.success = True
            response.message = 'Request accepted, NOT completed; wait for /mk2/control_owner/state '
            response.message += 'MOVEIT' if request.data else 'PENDANT'
        except OwnershipError as error:
            response.success, response.message = False, str(error)
        return response

    def publish(self):
        message = String()
        message.data = json.dumps({'owner': self.policy.state, 'reason': self.policy.reason,
            'generation': self.policy.generation, 'native_sequence': self.policy.sequence,
            'fresh': self.policy.fresh(time.monotonic())}, separators=(',', ':'))
        self.publisher.publish(message)

    def poll(self):
        now = time.monotonic()
        for _ in range(32):  # Bounded work; never starve service responses.
            if not self.sub.poll(0):
                break
            try:
                self.policy.feed(json.loads(self.sub.recv()[6:]), time.monotonic())
            except (ValueError, UnicodeError) as error:
                self.policy.fault('Malformed native feedback: ' + str(error))
        self.policy.tick(time.monotonic())
        if self.policy.state == 'UNAVAILABLE' and now - self.started > 15.0:
            self.policy.fault('Native feedback startup timeout')
        if self.deadline is not None and now > self.deadline:
            self.policy.fault('Handover timeout; ownership is NOT transferred')
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if result is None:
                    raise OwnershipError('Controller manager returned no response')
                if self.pending_kind not in ('check_hardware', 'check_controllers') and not result.ok:
                    raise OwnershipError('Controller/hardware transition rejected')
                if self.pending_kind == 'check_hardware':
                    states = {x.name: x.state.id for x in result.component}
                    if states.get('lh_mk2_system') != State.PRIMARY_STATE_INACTIVE:
                        raise OwnershipError('Shared startup requires inactive hardware')
                elif self.pending_kind == 'check_controllers':
                    states = {x.name: x.state for x in result.controller}
                    if states.get('jearm_controller') != 'inactive':
                        raise OwnershipError('Shared startup requires an inactive trajectory controller')
                    self.policy.confirm_standby()
                if hasattr(result, 'state') and result.state.id != self.target_state:
                    raise OwnershipError('Hardware returned the wrong lifecycle state')
                self.future = None
            except Exception as error:
                self.policy.fault(str(error))
        if self.policy.state != 'FAULT' and self.future is None and self.steps:
            kind, target = self.steps[0]
            client = {'check_hardware': self.list_hardware, 'check_controllers': self.list_controllers,
                      'hardware': self.hardware, 'switch': self.switch}[kind]
            if not client.service_is_ready():
                self.publish()
                return
            self.steps.pop(0)
            self.pending_kind = kind
            if kind == 'check_hardware':
                self.future = self.list_hardware.call_async(ListHardwareComponents.Request())
            elif kind == 'check_controllers':
                self.future = self.list_controllers.call_async(ListControllers.Request())
            elif kind == 'hardware':
                req = SetHardwareComponentState.Request()
                req.name, req.target_state.id = 'lh_mk2_system', target
                self.target_state = target
                self.future = self.hardware.call_async(req)
            else:
                req = SwitchController.Request()
                req.activate_controllers = ['jearm_controller'] if target else []
                req.deactivate_controllers = [] if target else ['jearm_controller']
                req.strictness = SwitchController.Request.STRICT
                req.timeout.sec = 3
                self.future = self.switch.call_async(req)
        if self.future is None and not self.steps and self.deadline is not None:
            if self.policy.finish(time.monotonic()):
                self.deadline = None
        self.publish()
        if self.policy.state == 'FAULT':
            # Process exit is supervised by launch, which stops the write driver.
            # Never label a failed/ambiguous transition as PENDANT or auto-recover.
            raise RuntimeError('Ownership fault: ' + self.policy.reason)

    def destroy_node(self):
        self.sub.close()
        self.zmq_context.term()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ControlOwner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()