from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[2]
class GazeboConfig(unittest.TestCase):
 def test_identity_is_required(self):
  text=(ROOT/'launch/gazebo_direct_position.launch.py').read_text()
  self.assertIn('DeclareLaunchArgument("world_name")',text)
  self.assertIn('DeclareLaunchArgument("model_name")',text)
  self.assertNotIn('default_value="f450_0"',text)
if __name__=='__main__':unittest.main()
