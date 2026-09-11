from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class GazeboCMakeContractTest(unittest.TestCase):
    def test_native_gazebo_relay_is_built_and_installed(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        package = (ROOT / "package.xml").read_text(encoding="utf-8")

        for dependency in (
            "find_package(gz_msgs_vendor REQUIRED)",
            "find_package(gz-msgs REQUIRED)",
            "find_package(gz_transport_vendor REQUIRED)",
            "find_package(gz-transport REQUIRED)",
        ):
            self.assertIn(dependency, cmake)

        self.assertIn("add_library(px4_offboard_gazebo_runtime", cmake)
        self.assertIn("src/ros2/gazebo_pose_relay.cpp", cmake)
        self.assertIn("src/ros2/gazebo_position_ros.cpp", cmake)
        self.assertIn("add_executable(gazebo_pose_relay", cmake)
        self.assertIn("src/ros2/gazebo_pose_relay_main.cpp", cmake)
        self.assertIn("gz-msgs::core", cmake)
        self.assertIn("gz-transport::core", cmake)
        self.assertIn("gazebo_pose_relay", cmake)

        self.assertIn("<depend>gz_msgs_vendor</depend>", package)
        self.assertIn("<depend>gz_transport_vendor</depend>", package)
        self.assertIn("<exec_depend>ros_gz_bridge</exec_depend>", package)


if __name__ == "__main__":
    unittest.main()
