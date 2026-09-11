from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]

class RepositoryStructureTest(unittest.TestCase):
    def test_locked_top_level_layout(self):
        required = [
            'package.xml', 'README.md', 'include/px4_offboard_controllers',
            'src/core', 'src/controllers', 'src/px4', 'src/state_sources', 'src/ros2',
            'vehicles/f450', 'third_party', 'research/control_boundary_comparison',
        ]
        missing = [p for p in required if not (ROOT / p).exists()]
        self.assertEqual([], missing, f'missing required paths: {missing}')
        self.assertFalse((ROOT / 'simulation/ros2/package.xml').exists(),
                         'nested simulation/ros2 ament package is forbidden')

if __name__ == '__main__':
    unittest.main()
