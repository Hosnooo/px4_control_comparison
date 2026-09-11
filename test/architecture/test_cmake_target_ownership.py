from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]


class CMakeTargetOwnershipTest(unittest.TestCase):
    def test_ros2_boundary_has_one_library_owner(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        match = re.search(
            r"add_library\(px4_offboard_ros2\s+(.*?)\)", cmake, re.DOTALL
        )
        self.assertIsNotNone(match, "src/ros2 PX4 boundary needs px4_offboard_ros2 owner")
        owned = match.group(1)
        for source in (
            "src/ros2/px4_message_adapter.cpp",
            "src/ros2/px4_state_input.cpp",
            "src/ros2/px4_command_publisher.cpp",
        ):
            self.assertIn(source, owned)
            self.assertEqual(1, cmake.count(source), f"{source} must have one logical owner")

        executable = re.search(
            r"add_executable\(offboard_controller\s+(.*?)\)", cmake, re.DOTALL
        )
        self.assertIsNotNone(executable)
        self.assertIn("src/ros2/offboard_controller_node.cpp", executable.group(1))
        self.assertNotIn("px4_state_input.cpp", executable.group(1))
        self.assertNotIn("px4_command_publisher.cpp", executable.group(1))
        self.assertIn("px4_offboard_ros2", cmake)

    def test_architecture_doc_names_current_ros2_target(self):
        architecture = (ROOT / "docs/architecture.md").read_text(encoding="utf-8")
        self.assertNotIn("px4_offboard_ros2_messages", architecture)
        self.assertIn("`px4_offboard_ros2`", architecture)

    def test_gazebo_support_does_not_force_shared_linkage(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertNotIn("add_library(px4_offboard_gazebo_runtime SHARED", cmake)


if __name__ == "__main__":
    unittest.main()
