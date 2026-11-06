"""Static MK2 resource checks; no ROS nodes or hardware connections are needed."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class Mk2ConfigTests(unittest.TestCase):
    def setUp(self):
        self.robot = ET.parse(ROOT / 'config/LH_MK2.urdf').getroot()
        self.semantic = ET.parse(ROOT / 'config/LH_MK2.srdf').getroot()

    def test_seven_joint_chain(self):
        joints = self.robot.findall('joint')
        self.assertEqual([j.get('name') for j in joints], [f'joint{i}' for i in range(1, 8)])
        for i, joint in enumerate(joints, 1):
            self.assertEqual(joint.find('parent').get('link'), 'base_link' if i == 1 else f'Link{i-1}')
            self.assertEqual(joint.find('child').get('link'), f'Link{i}')
        self.assertEqual(self.robot.get('name'), self.semantic.get('name'))
        self.assertEqual(
            self.semantic.find("group[@name='jearm']/chain").attrib,
            {'base_link': 'base_link', 'tip_link': 'Link7'},
        )

    def test_packaged_meshes(self):
        meshes = self.robot.findall('.//mesh')
        self.assertEqual(len(meshes), 16)
        for mesh in meshes:
            uri = mesh.get('filename')
            prefix = 'package://my_jearm_moveit_config/'
            self.assertTrue(uri.startswith(prefix))
            mesh_file = ROOT / uri.removeprefix(prefix)
            self.assertTrue(mesh_file.is_file(), str(mesh_file))
            self.assertEqual(mesh_file.parent, ROOT / 'config/meshes/lh_mk2')

    def test_only_adjacent_collision_exclusions(self):
        adjacent = {frozenset((j.find('parent').get('link'), j.find('child').get('link')))
                    for j in self.robot.findall('joint')}
        disabled = {frozenset((pair.get('link1'), pair.get('link2')))
                    for pair in self.semantic.findall('disable_collisions')}
        self.assertEqual(disabled, adjacent)

    def test_dedicated_limits(self):
        limits = yaml.safe_load((ROOT / 'config/joint_limits_mk2.yaml').read_text())['joint_limits']
        self.assertEqual(set(limits), {j.get('name') for j in self.robot.findall('joint')})
        for joint in self.robot.findall('joint'):
            bounds = joint.find('limit')
            planning = limits[joint.get('name')]
            self.assertEqual(planning['min_position'], float(bounds.get('lower')))
            self.assertEqual(planning['max_position'], float(bounds.get('upper')))
            self.assertLessEqual(planning['max_velocity'], float(bounds.get('velocity')))
            self.assertGreater(planning['max_acceleration'], 0)

    def test_controller_joint_mapping(self):
        config = yaml.safe_load((ROOT / 'config/moveit_controllers_single.yaml').read_text())
        controller = config['moveit_simple_controller_manager']['jearm_controller']
        self.assertEqual(controller['joints'], [f'joint{i}' for i in range(1, 8)])

    def test_no_hardware_plugin(self):
        self.assertIsNone(self.robot.find('ros2_control'))
        self.assertFalse(self.robot.findall('.//plugin'))

    def test_ompl_group_config(self):
        config = yaml.safe_load((ROOT / 'config/ompl_planning_mk2.yaml').read_text())
        self.assertEqual(config['planning_pipelines'], ['ompl'])
        for planner in config['ompl']['jearm']['planner_configs']:
            self.assertIn(planner, config['ompl']['planner_configs'])


if __name__ == '__main__':
    unittest.main()