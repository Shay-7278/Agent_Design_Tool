"""Confirmed sizes must preserve row identity across startup and refill batches."""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from abb_irb4600_gazebo.brick_models import load_brick_models


HALF_ROWS = [9, 17, 26, 34, 43, 51, 60, 68, 78, 87, 97,
             107, 118, 128, 140, 151, 163, 173, 184, 193, 203, 212]


def fingerprint(rows):
    payload = [[float(value) for value in row] for row in rows]
    return hashlib.sha256(json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


class BrickModelTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.model_dir = self.root / 'models'
        for name in ('brick', 'brick_half'):
            folder = self.model_dir / name
            folder.mkdir(parents=True)
            (folder / 'model.sdf').write_text('<sdf/>')
        self.rows = [[index * .2, 0, .0285, 0] for index in range(221)]
        self.placements = self.root / 'placements.json'
        self.placements.write_text(json.dumps(self.rows))
        self.profile = self.root / 'brick_sizes.json'
        self.profile.write_text(json.dumps({'placement_count': 221,
                                           'placement_sha256': fingerprint(self.rows),
                                           'half_rows': HALF_ROWS}))

    def models(self, path=None):
        return load_brick_models(path or self.placements, self.model_dir, self.profile)

    def test_exact_half_rows_keep_all_221_input_rows(self):
        before = self.placements.read_bytes()
        models = self.models()
        self.assertEqual(len(models), 221)
        self.assertEqual([i + 1 for i, model in enumerate(models) if model.parent.name == 'brick_half'], HALF_ROWS)
        self.assertEqual(sum(model.parent.name == 'brick' for model in models), 199)
        self.assertEqual(self.placements.read_bytes(), before)

    def test_renaming_whitespace_and_integer_float_spelling_preserve_mapping(self):
        renamed = self.root / 'renamed_wall.json'
        rows = [[float(value) for value in row] for row in self.rows]
        renamed.write_text(json.dumps(rows, indent=4))
        self.assertEqual(self.models(renamed), self.models())

    def test_different_221_pose_sequence_does_not_inherit_demo_sizes(self):
        self.rows[0][0] += .001
        self.placements.write_text(json.dumps(self.rows))
        (self.model_dir / 'brick_half/model.sdf').unlink()
        self.assertTrue(all(model.parent.name == 'brick' for model in self.models()))

    def test_absolute_size_mapping_crosses_pickup_refill_boundaries(self):
        models = self.models()
        self.assertEqual(models[0].parent.name, 'brick')
        self.assertEqual(models[50].parent.name, 'brick_half')
        self.assertEqual(models[150].parent.name, 'brick_half')
        self.assertEqual(models[202].parent.name, 'brick_half')
        self.assertEqual(models[211].parent.name, 'brick_half')

    def test_invalid_half_row_profiles_are_rejected(self):
        profile = json.loads(self.profile.read_text())
        for rows in ([9, 9], [0], [222], [True], [9.0]):
            with self.subTest(rows=rows):
                profile['half_rows'] = rows
                self.profile.write_text(json.dumps(profile))
                with self.assertRaises(ValueError):
                    self.models()

    def test_missing_half_model_is_detected_for_matching_sequence(self):
        (self.model_dir / 'brick_half/model.sdf').unlink()
        with self.assertRaisesRegex(FileNotFoundError, 'brick_half'):
            self.models()

    def test_installed_profile_contains_the_confirmed_list_and_pose_identity(self):
        config = Path(__file__).resolve().parents[1] / 'config/brick_sizes.json'
        profile = json.loads(config.read_text(encoding='utf-8'))
        self.assertEqual(profile['placement_count'], 221)
        self.assertEqual(profile['half_rows'], HALF_ROWS)
        self.assertEqual(profile['placement_sha256'], '5deb8cddc155e6a97ae608e92f89dbfea6a0ad25737cd9acd24314a06cf9335f')
