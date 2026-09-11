from pathlib import Path
import re, unittest
ROOT=Path(__file__).resolve().parents[2]
class NativePx4Topics(unittest.TestCase):
    def test_runtime_uses_only_native_px4_topics(self):
        paths=[ROOT/'src/ros2/offboard_controller_node.cpp',ROOT/'src/ros2/px4_state_input.cpp',ROOT/'src/ros2/px4_command_publisher.cpp']
        text='\n'.join(p.read_text() for p in paths)
        self.assertNotIn('/custom/',text)
        required={
            '/fmu/in/offboard_control_mode','/fmu/in/trajectory_setpoint','/fmu/in/vehicle_attitude_setpoint',
            '/fmu/in/vehicle_rates_setpoint','/fmu/in/vehicle_thrust_setpoint','/fmu/in/vehicle_torque_setpoint',
            '/fmu/in/vehicle_command','/fmu/out/vehicle_local_position','/fmu/out/vehicle_attitude',
            '/fmu/out/vehicle_angular_velocity','/fmu/out/vehicle_status'
        }
        seen=set(re.findall(r'"(/fmu/(?:in|out)/[a-z0-9_]+)"',text))
        self.assertTrue(required<=seen, required-seen)
    def test_all_public_modes_are_named(self):
        text=(ROOT/'src/ros2/offboard_controller_node.cpp').read_text()
        for mode in ('px4_position','px4_velocity','geometric_acceleration','geometric_attitude','geometric_rate','px4_attitude_rate_mirror','lee_wrench'):
            self.assertIn(mode,text)
if __name__=='__main__':unittest.main()
