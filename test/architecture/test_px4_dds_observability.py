from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "third_party" / "patches" / "px4_vehicle_angular_velocity_dds.patch"


class Px4DdsObservabilityTest(unittest.TestCase):
    def test_patch_only_enables_vehicle_angular_velocity_publication(self):
        self.assertTrue(PATCH.is_file(), "required PX4 DDS observability patch is missing")
        lines = PATCH.read_text(encoding="utf-8").splitlines()
        changed = [
            line
            for line in lines
            if line.startswith(("+", "-")) and not line.startswith(("+++", "---"))
        ]
        self.assertEqual(
            changed,
            [
                "-  # - topic: /fmu/out/vehicle_angular_velocity",
                "-  #   type: px4_msgs::msg::VehicleAngularVelocity",
                "+  - topic: /fmu/out/vehicle_angular_velocity",
                "+    type: px4_msgs::msg::VehicleAngularVelocity",
            ],
        )

    def test_dependency_docs_preserve_pin_and_explain_patch(self):
        docs = (ROOT / "docs" / "dependencies.md").read_text(encoding="utf-8")
        self.assertIn("c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc", docs)
        self.assertIn("px4_vehicle_angular_velocity_dds.patch", docs)
        self.assertIn("observability", docs.lower())
        self.assertIn("gitlink", docs.lower())

        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("px4_vehicle_angular_velocity_dds.patch", readme)
        self.assertIn("git apply", readme)


if __name__ == "__main__":
    unittest.main()
