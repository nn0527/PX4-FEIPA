"""Ceiling-only baseline tuning: tolerant sampling, preserved tilt/freshness gates."""
from dataclasses import replace
from pathlib import Path
import math
import unittest
import yaml
from px4_wall_ceiling_control.domain.ceiling_motion import CeilingMotionSettings
from px4_wall_ceiling_control.domain.horizontal_equilibrium import HorizontalEquilibrium


class BaselineTuningTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((Path(__file__).parents[1] / 'config/ceiling_attachment.yaml').read_text())
        params = self.config['ceiling_attachment_decision_node']
        self.settings = CeilingMotionSettings.read(lambda key, default: params.get(key.lstrip('~'), default))

    def baseline(self):
        s = self.settings
        return HorizontalEquilibrium(window=s.motion_equilibrium_window_s,
            max_gap=s.motion_equilibrium_max_gap_s, max_spread_deg=s.motion_equilibrium_max_spread_deg)

    @staticmethod
    def q(degrees):
        angle = math.radians(degrees) / 2
        return (0., math.sin(angle), 0., math.cos(angle))

    def test_user_window_and_tolerant_hover_samples(self):
        self.assertEqual(self.settings.motion_equilibrium_window_s, .3)
        self.assertEqual(self.settings.motion_equilibrium_speed_mps, .3)
        b = self.baseline()
        for stamp, tilt in [(10., 1.), (10.23, 2.), (10.46, 3.)]:
            b.observe(stamp, self.q(tilt), True)
        self.assertTrue(b.ready)
        self.assertTrue(b.freeze(10.5))

    def test_at_least_three_frames_and_fresh_feedback_still_required(self):
        b = self.baseline()
        b.observe(10., self.q(1.), True)
        b.observe(10.4, self.q(1.), True)
        self.assertFalse(b.ready)
        b.observe(10.5, self.q(1.), True)
        self.assertTrue(b.ready)
        self.assertFalse(b.freeze(11.1))

    def test_sample_gaps_and_invalid_feedback_reset_baseline(self):
        b = self.baseline()
        for t in (10., 10.2, 10.4): b.observe(t, self.q(1.), True)
        self.assertTrue(b.ready)
        b.observe(11., self.q(1.), True)
        self.assertFalse(b.ready)
        b.observe(11.2, self.q(1.), False)
        self.assertEqual(len(b.samples), 0)

    def test_five_degree_tilt_limit_is_unchanged(self):
        b = self.baseline()
        for t in (10., 10.2, 10.4): b.observe(t, self.q(6.), True)
        self.assertFalse(b.ready)

    def test_excessive_relaxation_is_rejected(self):
        for fields in ({'motion_equilibrium_speed_mps': .31},
                       {'motion_equilibrium_max_gap_s': 1.1},
                       {'motion_equilibrium_max_spread_deg': 5.1}):
            with self.assertRaises(ValueError): replace(self.settings, **fields)


if __name__ == '__main__':
    unittest.main()
