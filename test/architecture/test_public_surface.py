from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
PUBLIC_RUNTIME = [ROOT / "README.md", ROOT / "include", ROOT / "src", ROOT / "launch", ROOT / "config"]


class PublicSurfaceTest(unittest.TestCase):
    def test_no_stale_runtime_names_or_custom_topics(self):
        offenders = []
        for root in PUBLIC_RUNTIME:
            paths = [root] if root.is_file() else root.rglob("*")
            for path in paths:
                if not path.is_file():
                    continue
                text = path.read_text(errors="ignore")
                for needle in ("px4_control_comparison", "/custom/"):
                    if needle in text:
                        offenders.append(f"{path.relative_to(ROOT)}:{needle}")
        self.assertEqual([], offenders)

    def test_ros_gz_bridge_is_runtime_dependency_not_submodule(self):
        modules = (ROOT / ".gitmodules").read_text()
        package = (ROOT / "package.xml").read_text()
        self.assertNotIn("ros_gz_bridge", modules)
        self.assertIn("ros_gz_bridge", package)

    def test_no_hidden_f450_gazebo_default(self):
        launch = (ROOT / "launch/gazebo_direct_position.launch.py").read_text()
        self.assertNotIn("f450_0", launch)


if __name__ == "__main__":
    unittest.main()
