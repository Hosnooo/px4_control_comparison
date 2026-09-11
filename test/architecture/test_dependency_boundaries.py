from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
GENERIC = [ROOT/'include/px4_offboard_controllers/controllers', ROOT/'include/px4_offboard_controllers/px4']

class DependencyBoundaryTest(unittest.TestCase):
    def test_forbidden_generic_dependencies_and_topics(self):
        offenders=[]
        for base in GENERIC:
            if not base.exists():
                offenders.append(f'missing:{base.relative_to(ROOT)}')
                continue
            for path in base.rglob('*'):
                if path.is_file():
                    text=path.read_text(errors='ignore')
                    for needle in ('f450','hardware_wrench_calibration','gz/','gazebo','/custom/'):
                        if needle in text.lower(): offenders.append(f'{path.relative_to(ROOT)}:{needle}')
        self.assertEqual([], offenders)

    def test_no_sibling_cpp_compilation(self):
        offenders=[]
        for cmake in ROOT.rglob('CMakeLists.txt'):
            text=cmake.read_text(errors='ignore')
            if re.search(r'\.\./[^\s)]*\.cpp', text): offenders.append(str(cmake.relative_to(ROOT)))
        self.assertEqual([], offenders)

if __name__=='__main__': unittest.main()
