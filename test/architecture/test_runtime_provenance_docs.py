from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class RuntimeProvenanceDocsTest(unittest.TestCase):
    def test_f450_esc_range_distinguishes_px4_output_from_gazebo_limit(self):
        sources = (ROOT / "docs/sources.md").read_text(encoding="utf-8")
        for text in ("4022_gz_f450", "150", "1000", "1032", "GZMixingInterfaceESC"):
            self.assertIn(text, sources)
        self.assertIn("hard limit", sources.lower())

    def test_mirror_pending_inputs_are_named(self):
        validation = (ROOT / "docs/validation.md").read_text(encoding="utf-8")
        for text in ("ControlAllocatorStatus", "VehicleLandDetected", "BatteryStatus"):
            self.assertIn(text, validation)


if __name__ == "__main__":
    unittest.main()
