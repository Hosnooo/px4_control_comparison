from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]


class Ros2StubCompileTest(unittest.TestCase):
    def test_native_boundary_sources_compile_against_audited_stubs(self):
        compiler = "c++"
        for source in [
            ROOT / "src/ros2/px4_state_input.cpp",
            ROOT / "src/ros2/px4_command_publisher.cpp",
            ROOT / "src/ros2/offboard_controller_node.cpp",
        ]:
            completed = subprocess.run(
                [
                    compiler,
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Wpedantic",
                    "-Werror",
                    "-fsyntax-only",
                    f"-I{ROOT / 'test/stubs'}",
                    f"-I{ROOT / 'include'}",
                    f"-I{ROOT / 'src/ros2'}",
                    str(source),
                ],
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
