"""Pure configuration helpers for SDK-coordinate MK2 hardware descriptions."""

import xml.etree.ElementTree as ET


class FeedbackReadyGate:
    """Consume fragmented process stdout; accept one exact valid-feedback marker."""

    def __init__(self):
        self.buffer = b''
        self.started = False

    def feed(self, text):
        if self.started:
            return False
        self.buffer = (self.buffer + text)[-8192:]
        lines = self.buffer.split(b'\n')
        self.buffer = lines[-1]
        if b'MK2_FEEDBACK_READY' not in [line.strip() for line in lines[:-1]]:
            return False
        self.started = True
        return True


WAIST_SOURCE_JOINT = 'Joint1'
WAIST_JOINT = 'waist_joint'

# SDK simulation only: gen-1 planning limits (my_jearm_moveit_config
# joint_limits.yaml), further capped by MK2 hardware vmax/amax. They stay
# inside the sim driver caps below, which stay inside the sim native policy
# (3.8 / 25) and the sim-only native build caps (4.0 / 40).
SIM_PLANNING_VELOCITY = {'joint1': 2.0, 'joint2': 2.0, 'joint3': 2.5, 'joint4': 2.5,
                         'joint5': 2.5, 'joint6': 3.0, 'joint7': 3.0}
SIM_PLANNING_ACCELERATION = 10.0
SIM_DRIVER_MAX_VELOCITY = 3.5
SIM_DRIVER_MAX_ACCELERATION = 20.0
# Physical (2026-11-19): slightly below sim. Chain: planning (below) < driver
# 2.4 / 12 (je_software physical_max_*) < native physical policy 2.7 / 15 /
# following 0.25 < native physical build caps 3.0 / 20 / 0.3.
PHYSICAL_PLANNING_VELOCITY = {'joint1': 1.5, 'joint2': 1.5, 'joint3': 1.8, 'joint4': 1.8,
                              'joint5': 1.8, 'joint6': 2.0, 'joint7': 2.0}
PHYSICAL_PLANNING_ACCELERATION = 6.0
PHYSICAL_DRIVER_MAX_VELOCITY = 2.4
PHYSICAL_DRIVER_MAX_ACCELERATION = 12.0
# Planning/IK bounds sit this far inside the driver/native hard bounds (rad):
# JTC spline sampling can overshoot a waypoint lying exactly on a bound, and
# RobotMain stops ~0.03 rad short of smin/smax (observed: joint6 commanded
# -1.4678, settled at -1.4481 with smin -1.4778).
PLANNING_POSITION_MARGIN = 0.05


def adapt_dual_model(dual_urdf, dual_srdf, waist=False):
    """Left arm of the dual waist model as the 7-DOF MK2 description.

    joint11..joint17 are renamed joint1..joint7 (the driver, SDK feedback
    bridge and controllers use these names). The right arm becomes fixed at
    its zero pose; only the left arm is actuated. Joint axes and origins are
    unchanged; the left chain matches LH_MK2.urdf exactly.

    waist=False: the waist stays fixed at zero.
    waist=True: Joint1 becomes the revolute 'waist_joint', driven only by the
    separate CAN waist feedback (mk2_waist_bridge.py). It is NOT part of the
    jearm planning group (chain starts at Link1).
    """
    robot = ET.fromstring(dual_urdf)
    for control in robot.findall('ros2_control'):
        robot.remove(control)
    left = {f'joint1{i}': f'joint{i}' for i in range(1, 8)}
    for joint in robot.findall('joint'):
        name = joint.get('name')
        if name in left:
            joint.set('name', left[name])
        elif waist and name == WAIST_SOURCE_JOINT:
            joint.set('name', WAIST_JOINT)
            joint.set('type', 'revolute')
        elif joint.get('type') != 'fixed':
            joint.set('type', 'fixed')
    expected = ([WAIST_JOINT] if waist else []) + list(left.values())
    if [j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'] != expected:
        raise ValueError(f'Dual model must have exactly {expected} movable')
    if waist:
        waist_joint = robot.find(f"joint[@name='{WAIST_JOINT}']")
        if (waist_joint.find('parent').get('link') != 'base_link'
                or waist_joint.find('child').get('link') != 'Link1'
                or waist_joint.find('limit') is None):
            raise ValueError('Waist joint must be base_link -> Link1 with limits')

    source = ET.fromstring(dual_srdf)
    semantic = ET.Element('robot', name=robot.get('name'))
    ET.SubElement(semantic, 'virtual_joint', name='world_joint', type='fixed',
                  parent_frame='world', child_link='base_link')
    ET.SubElement(ET.SubElement(semantic, 'group', name='jearm'), 'chain',
                  base_link='Link1' if waist else 'base_link', tip_link='Left_Link7')
    ET.SubElement(ET.SubElement(semantic, 'group', name='jearm_ee'), 'link', name='Left_Link7')
    zero = ET.SubElement(semantic, 'group_state', name='zero', group='jearm')
    for name in left.values():
        ET.SubElement(zero, 'joint', name=name, value='0')
    for rule in source.findall('disable_collisions'):
        semantic.append(rule)
    return ET.tostring(robot, encoding='unicode'), ET.tostring(semantic, encoding='unicode')


def build_hardware_description(urdf, srdf, hardware, robot_ip, state_port, command_port,
                               control_backend='simulation', confirm_physical_control=False,
                               shared_ownership=False, flange_parent='Link7'):
    """Preserve the teach-pendant URDF geometry and joint direction conventions.

    User verified the source model's motion against the physical arm. Do not
    replace its axes/origins with an assumed +Z MDH chain (notably J4/J6).
    SDK positions pass through unchanged; encoder calibration is not reapplied.
    Hardware bounds, interfaces and a separate fixed flange are the only additions.
    """
    if control_backend not in ('simulation', 'physical'):
        raise ValueError('control_backend must be simulation or physical')
    if control_backend == 'physical' and not confirm_physical_control:
        raise ValueError('Physical control requires separate explicit confirmation')
    axes = hardware['axes']
    if [axis['name'] for axis in axes] != [f'joint{i}' for i in range(1, 8)]:
        raise ValueError('MK2 requires exactly joint1..joint7 in SDK axis order')
    if [axis['id'] for axis in axes] != list(range(7)):
        raise ValueError('MK2 requires Robot0 SDK slots 0..6')
    robot = ET.fromstring(urdf)
    semantic = ET.fromstring(srdf)
    planning = {'default_velocity_scaling_factor': 1,
                'default_acceleration_scaling_factor': 1, 'joint_limits': {}}
    for axis in axes:
        joint = robot.find(f"joint[@name='{axis['name']}']")
        if joint is None:
            raise ValueError(f"Missing model joint {axis['name']}")
        mdh = axis['mdh']
        if mdh['flip'] or mdh['theta'] or mdh['offset']:
            raise ValueError('Nonzero MDH theta/offset or flip requires explicit model adaptation')
        if not axis['smin'] < axis['smax'] or axis['vmax'] <= 0 or axis['amax'] <= 0:
            raise ValueError(f"Invalid hardware bounds for {axis['name']}")
        # Preserve axis, origin and link frames exactly, including the original
        # negative-Z J4/J6 axes. MDH mathematical agreement alone is not evidence
        # that a rewritten visual model matches the real/teach-pendant motion.
        min_position = axis['smin'] + PLANNING_POSITION_MARGIN
        max_position = axis['smax'] - PLANNING_POSITION_MARGIN
        bounds = joint.find('limit')
        bounds.set('lower', str(min_position))
        bounds.set('upper', str(max_position))
        bounds.set('velocity', str(axis['vmax']))
        # Source CAD effort is retained: motor torque_max is NOT an identified
        # joint-output effort limit and must not be copied as such.
        # Keep timed plans inside the driver envelope, leaving headroom for
        # sampling jitter and bounded cancel/hold transitions.
        if control_backend == 'simulation':
            max_velocity = min(SIM_PLANNING_VELOCITY[axis['name']], axis['vmax'])
            max_acceleration = min(SIM_PLANNING_ACCELERATION, axis['amax'])
        else:
            max_velocity = min(PHYSICAL_PLANNING_VELOCITY[axis['name']], axis['vmax'])
            max_acceleration = min(PHYSICAL_PLANNING_ACCELERATION, axis['amax'])
        planning['joint_limits'][axis['name']] = {
            'has_position_limits': True,
            'min_position': min_position, 'max_position': max_position,
            'has_velocity_limits': True, 'max_velocity': max_velocity,
            'has_acceleration_limits': True, 'max_acceleration': max_acceleration,
        }

    ET.SubElement(robot, 'link', name='sdk_flange')
    flange = ET.SubElement(robot, 'joint', name='sdk_flange_fixed', type='fixed')
    ET.SubElement(flange, 'parent', link=flange_parent)
    ET.SubElement(flange, 'child', link='sdk_flange')
    extension = axes[-1]['mdh']['d'] - hardware['cad_link7_distance']
    ET.SubElement(flange, 'origin', xyz=f'0 0 {extension:.17g}', rpy='0 0 0')
    semantic.find("group[@name='jearm']/chain").set('tip_link', 'sdk_flange')
    semantic.find("group[@name='jearm_ee']/link").set('name', 'sdk_flange')

    control = ET.SubElement(robot, 'ros2_control', name='lh_mk2_system', type='system')
    driver = ET.SubElement(control, 'hardware')
    ET.SubElement(driver, 'plugin').text = 'je_software/JeZmqSystemHardware'
    driver_params = {'robot_ip': robot_ip, 'sub_port': state_port,
                     'pub_port': command_port,
                     'mk2_gripper_port': int(command_port) + 1,
                     'mk2_gripper_feedback_port': int(command_port) + 2,
                     'command_dt': hardware['transport']['command_dt'],
                     'mk2_guarded_control': 'true',
                     'mk2_shared_ownership': str(shared_ownership).lower(),
                     'mk2_expected_backend': ('physical' if control_backend == 'physical'
                                              else 'sdk_simulation'),
                     'mk2_device_config_sha256': hardware['source']['sha256'],
                     'mk2_confirm_physical_control': str(confirm_physical_control).lower(),
                     'mk2_feedback_timeout': '0.15'}
    if control_backend == 'simulation':
        driver_params['mk2_max_velocity'] = SIM_DRIVER_MAX_VELOCITY
        driver_params['mk2_max_acceleration'] = SIM_DRIVER_MAX_ACCELERATION
    else:
        driver_params['mk2_max_velocity'] = PHYSICAL_DRIVER_MAX_VELOCITY
        driver_params['mk2_max_acceleration'] = PHYSICAL_DRIVER_MAX_ACCELERATION
    for name, value in driver_params.items():
        ET.SubElement(driver, 'param', name=name).text = str(value)
    for axis in axes:
        joint = ET.SubElement(control, 'joint', name=axis['name'])
        command = ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(command, 'param', name='min').text = str(axis['smin'])
        ET.SubElement(command, 'param', name='max').text = str(axis['smax'])
        # JTC supplies sampled velocity (including explicit zero on cancel).
        # It is internal metadata, NOT an additional field on native Joint wire.
        ET.SubElement(joint, 'command_interface', name='velocity')
        for interface in ('position', 'velocity', 'effort'):
            ET.SubElement(joint, 'state_interface', name=interface)
    return ET.tostring(robot, encoding='unicode'), ET.tostring(semantic, encoding='unicode'), planning