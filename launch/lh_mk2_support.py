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


def build_hardware_description(urdf, srdf, hardware, robot_ip, state_port, command_port):
    """Preserve the teach-pendant URDF geometry and joint direction conventions.

    User verified the source model's motion against the physical arm. Do not
    replace its axes/origins with an assumed +Z MDH chain (notably J4/J6).
    SDK positions pass through unchanged; encoder calibration is not reapplied.
    Hardware bounds, interfaces and a separate fixed flange are the only additions.
    """
    axes = hardware['axes']
    if [axis['name'] for axis in axes] != [f'joint{i}' for i in range(1, 8)]:
        raise ValueError('MK2 requires exactly joint1..joint7 in SDK axis order')
    if [axis['id'] for axis in axes] != list(range(7)):
        raise ValueError('MK2 requires Robot0 SDK slots 0..6')
    robot = ET.fromstring(urdf)
    semantic = ET.fromstring(srdf)
    planning = {'default_velocity_scaling_factor': 0.1,
                'default_acceleration_scaling_factor': 0.1, 'joint_limits': {}}
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
        bounds = joint.find('limit')
        bounds.set('lower', str(axis['smin']))
        bounds.set('upper', str(axis['smax']))
        bounds.set('velocity', str(axis['vmax']))
        # Source CAD effort is retained: motor torque_max is NOT an identified
        # joint-output effort limit and must not be copied as such.
        planning['joint_limits'][axis['name']] = {
            'has_position_limits': True,
            'min_position': axis['smin'], 'max_position': axis['smax'],
            'has_velocity_limits': True, 'max_velocity': min(0.3, axis['vmax']),
            'has_acceleration_limits': True, 'max_acceleration': min(0.5, axis['amax']),
        }

    ET.SubElement(robot, 'link', name='sdk_flange')
    flange = ET.SubElement(robot, 'joint', name='sdk_flange_fixed', type='fixed')
    ET.SubElement(flange, 'parent', link='Link7')
    ET.SubElement(flange, 'child', link='sdk_flange')
    extension = axes[-1]['mdh']['d'] - hardware['cad_link7_distance']
    ET.SubElement(flange, 'origin', xyz=f'0 0 {extension:.17g}', rpy='0 0 0')
    semantic.find("group[@name='jearm']/chain").set('tip_link', 'sdk_flange')
    semantic.find("group[@name='jearm_ee']/link").set('name', 'sdk_flange')

    control = ET.SubElement(robot, 'ros2_control', name='lh_mk2_system', type='system')
    driver = ET.SubElement(control, 'hardware')
    ET.SubElement(driver, 'plugin').text = 'je_software/JeZmqSystemHardware'
    for name, value in {'robot_ip': robot_ip, 'sub_port': state_port,
                        'pub_port': command_port,
                        'command_dt': hardware['transport']['command_dt']}.items():
        ET.SubElement(driver, 'param', name=name).text = str(value)
    for axis in axes:
        joint = ET.SubElement(control, 'joint', name=axis['name'])
        command = ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(command, 'param', name='min').text = str(axis['smin'])
        ET.SubElement(command, 'param', name='max').text = str(axis['smax'])
        for interface in ('position', 'velocity', 'effort'):
            ET.SubElement(joint, 'state_interface', name=interface)
    return ET.tostring(robot, encoding='unicode'), ET.tostring(semantic, encoding='unicode'), planning