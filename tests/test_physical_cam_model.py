import unittest

from tools.physical_cam_model import (
    Recipe,
    derive_points,
    object_relative_position,
    quintic_peak_return_dynamics,
    shortest_periodic_target,
)


class PhysicalCamModelTests(unittest.TestCase):
    def test_approved_eight_points(self):
        actual = [(p.x, p.y, p.slope) for p in derive_points(Recipe())]
        self.assertEqual(
            actual,
            [
                (0.0, 0.0, 0.0),
                (1.0, -1.0, -0.4),
                (6.0, -3.0, -0.4),
                (10.0, -7.0, -1.6),
                (15.0, -15.0, -1.6),
                (17.0, -16.6, 0.0),
                (27.0, 0.0, 0.0),
                (63.7, 0.0, 0.0),
            ],
        )

    def test_each_cut_changes_object_relative_position_by_three_mm(self):
        points = derive_points(Recipe())
        self.assertAlmostEqual(object_relative_position(points[1]), 0.0)
        self.assertAlmostEqual(object_relative_position(points[2]), -3.0)
        self.assertAlmostEqual(object_relative_position(points[3]), -3.0)
        self.assertAlmostEqual(object_relative_position(points[4]), 0.0)

    def test_two_second_return_dynamics(self):
        peak_velocity, peak_acceleration, peak_jerk = (
            quintic_peak_return_dynamics(distance=16.6, seconds=2.0)
        )
        self.assertAlmostEqual(peak_velocity, 15.5625, places=4)
        self.assertAlmostEqual(peak_acceleration, 23.9600, places=3)
        self.assertAlmostEqual(peak_jerk, 124.5, places=3)

    def test_default_cut_velocities_are_visibly_different(self):
        recipe = Recipe()
        self.assertEqual(recipe.first_cutter_velocity, -2.0)
        self.assertEqual(recipe.second_cutter_velocity, -8.0)

    def test_shortest_periodic_reference_targets(self):
        self.assertAlmostEqual(
            shortest_periodic_target(0.0, -0.6, 63.7), -0.6
        )
        large_target = shortest_periodic_target(1000.0, -0.6, 63.7)
        self.assertAlmostEqual(large_target, 1018.6)
        self.assertLessEqual(abs(large_target - 1000.0), 31.85)
        self.assertAlmostEqual(
            shortest_periodic_target(40.0, 0.0, 63.7), 63.7
        )
        self.assertAlmostEqual(
            shortest_periodic_target(-40.0, 0.0, 63.7), -63.7
        )

    def test_shortest_periodic_reference_rejects_nonpositive_pitch(self):
        with self.assertRaises(ValueError):
            shortest_periodic_target(0.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
