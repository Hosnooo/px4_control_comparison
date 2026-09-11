from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


def _flatten(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from _flatten(item)
        else:
            yield item


class PythonTestDiscoveryTest(unittest.TestCase):
    def test_documented_discovery_reaches_calibration_and_ros2_suites(self):
        suite = unittest.TestLoader().discover(str(ROOT / "test"), pattern="test_*.py")
        test_ids = {case.id() for case in _flatten(suite)}

        self.assertTrue(any("calibration.test_fit_wrench_calibration" in test_id for test_id in test_ids))
        self.assertTrue(any("ros2.test_runtime_orchestration" in test_id for test_id in test_ids))


if __name__ == "__main__":
    unittest.main()
