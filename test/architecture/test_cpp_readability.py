from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_ROOTS = [
    ROOT / "include",
    ROOT / "src",
    ROOT / "vehicles",
    ROOT / "experiment" / "calibration" / "include",
    ROOT / "experiment" / "calibration" / "src",
]


class CppReadabilityTest(unittest.TestCase):
    def test_production_cpp_respects_repository_column_limit(self):
        offenders = []
        for root in PRODUCTION_ROOTS:
            for path in root.rglob("*"):
                if path.suffix not in {".cpp", ".hpp"}:
                    continue
                for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                    if len(line) > 100:
                        offenders.append(
                            f"{path.relative_to(ROOT)}:{line_number}:{len(line)}"
                        )
        self.assertEqual([], offenders)


if __name__ == "__main__":
    unittest.main()
