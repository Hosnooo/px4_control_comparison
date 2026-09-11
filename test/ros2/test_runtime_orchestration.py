from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class RuntimeOrchestrationTest(unittest.TestCase):
    def test_selected_controller_drives_native_offboard_heartbeat(self):
        source = (ROOT / "src/ros2/offboard_controller_node.cpp").read_text(encoding="utf-8")
        self.assertIn("controlLevelFor(*controller_kind)", source)
        self.assertIn("create_wall_timer", source)
        self.assertIn("publishControlMode", source)
        self.assertIn("RCL_SYSTEM_TIME", source)
        self.assertNotIn("node->get_clock()->now()", source)
        docs = (ROOT / "docs/px4_interface.md").read_text(encoding="utf-8")
        self.assertIn("system time", docs.lower())
        self.assertIn("uxrce-dds", docs.lower())

    def test_vehicle_commands_are_explicit_typed_actions(self):
        header = (ROOT / "src/ros2/runtime.hpp").read_text(encoding="utf-8")
        source = (ROOT / "src/ros2/px4_command_publisher.cpp").read_text(encoding="utf-8")
        self.assertIn("publishArmCommand", header)
        self.assertIn("publishOffboardModeCommand", header)
        self.assertIn("VEHICLE_CMD_COMPONENT_ARM_DISARM", source)
        self.assertIn("VEHICLE_CMD_DO_SET_MODE", source)
        self.assertIn("ARMING_ACTION_ARM", source)
        self.assertIn("from_external", source)


if __name__ == "__main__":
    unittest.main()
