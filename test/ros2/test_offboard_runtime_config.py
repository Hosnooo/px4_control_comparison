from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class OffboardRuntimeConfig(unittest.TestCase):
    def test_controller_is_explicit(self):
        launch = (ROOT / "launch/offboard_controller.launch.py").read_text()
        config = (ROOT / "config/controllers/controllers.yaml").read_text()
        self.assertIn('DeclareLaunchArgument("controller")', launch)
        self.assertNotIn("default_value", launch)
        self.assertIn('controller: ""', config)

    def test_runtime_rejects_unknown_selection(self):
        source = (ROOT / "src/ros2/offboard_controller_node.cpp").read_text()
        self.assertIn("supportedController", source)
        self.assertIn("return 2", source)


if __name__ == "__main__":
    unittest.main()
