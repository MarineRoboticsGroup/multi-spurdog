import unittest
import yaml
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from spurdog_acomms_utils.landmark_utils import validate_landmarks, get_landmark_pos, get_landmark_src


class TestLandmarkUtils(unittest.TestCase):
    def setUp(self):
        # load canonical YAML landmarks for a good case
        # Find the package directory named 'spurdog_acomms' by walking parents
        p = Path(__file__).resolve()
        pkg_dir = None
        for parent in p.parents:
            if parent.name == 'spurdog_acomms':
                pkg_dir = parent
                break
        if pkg_dir is None:
            # fallback: known repo path
            pkg_dir = Path('/home/lizgajski2/catkin_ws/src/multi-spurdog/spurdog_acomms')
        yaml_path = pkg_dir / 'launch' / 'basic_modem_config.yaml'
        if not yaml_path.exists():
            raise FileNotFoundError(f'basic_modem_config.yaml not found at {yaml_path}')
        data = yaml.safe_load(open(yaml_path))
        self.good = data.get('landmarks', {})

    def test_validate_good(self):
        # should not raise
        validate_landmarks(self.good)

    def test_validate_bad_missing_pos(self):
        bad = {'L0': {'src': 8}, 'L1': {'src': 9, 'pos': [1, 2]}}  # L1 pos too short
        with self.assertRaises(ValueError):
            validate_landmarks(bad)

    def test_get_pos_and_src(self):
        # ensure helpers return expected types
        for k, v in self.good.items():
            pos = get_landmark_pos(self.good, k)
            src = get_landmark_src(self.good, k)
            self.assertIsNotNone(pos)
            self.assertIsInstance(pos, list)


if __name__ == '__main__':
    unittest.main()
