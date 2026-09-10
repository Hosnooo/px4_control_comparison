import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
ROS_PACKAGE = ROOT / "simulation" / "ros2"


class GazeboRuntimeConfigTest(unittest.TestCase):
    def test_package_declares_runtime_dependencies(self):
        root = ET.parse(ROS_PACKAGE / "package.xml").getroot()
        deps = {element.text for element in root if element.tag in {"depend", "exec_depend"}}
        for required in {
            "geometry_msgs",
            "gz_msgs_vendor",
            "gz_transport_vendor",
            "launch",
            "launch_ros",
            "ros_gz_bridge",
            "rosgraph_msgs",
        }:
            self.assertIn(required, deps)

    def test_launch_is_gz_to_ros_and_identity_preserving(self):
        text = (ROS_PACKAGE / "launch" / "gazebo_direct_position.launch.py").read_text()
        self.assertIn("geometry_msgs/msg/TransformStamped[gz.msgs.Pose", text)
        self.assertIn("rosgraph_msgs/msg/Clock[gz.msgs.Clock", text)
        self.assertNotIn("PoseArray", text)
        self.assertNotIn("@gz.msgs.Pose", text)
        self.assertIn("/clock", text)

    def test_runtime_does_not_add_or_reference_an_f450_sdf(self):
        for path in ROS_PACKAGE.rglob("*"):
            if path.is_file():
                self.assertNotEqual(path.suffix, ".sdf")


if __name__ == "__main__":
    unittest.main()
