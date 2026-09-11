from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class Ros2BoundaryImplementationTest(unittest.TestCase):
    def test_state_input_constructs_native_px4_subscriptions(self):
        text = (ROOT / "src/ros2/px4_state_input.cpp").read_text(encoding="utf-8")
        self.assertIn("rclcpp::SensorDataQoS", text)
        self.assertGreaterEqual(text.count("create_subscription"), 4)
        self.assertIn("requirements.position", text)
        self.assertIn("requirements.velocity", text)
        self.assertIn("requirements.attitude", text)
        self.assertIn("requirements.body_rate", text)

    def test_command_publisher_constructs_native_px4_publishers(self):
        text = (ROOT / "src/ros2/px4_command_publisher.cpp").read_text(encoding="utf-8")
        self.assertGreaterEqual(text.count("create_publisher"), 7)
        for topic in [
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
            "/fmu/in/vehicle_attitude_setpoint",
            "/fmu/in/vehicle_rates_setpoint",
            "/fmu/in/vehicle_thrust_setpoint",
            "/fmu/in/vehicle_torque_setpoint",
            "/fmu/in/vehicle_command",
        ]:
            self.assertIn(topic, text)
        self.assertIn("toPx4OffboardControlMode", text)
        self.assertIn("toPx4TrajectorySetpoint", text)
        self.assertIn("toPx4VehicleAttitudeSetpoint", text)
        self.assertIn("toPx4VehicleRatesSetpoint", text)
        self.assertIn("toPx4VehicleTorqueSetpoint", text)
        self.assertIn("toPx4VehicleThrustSetpoint", text)


if __name__ == "__main__":
    unittest.main()
