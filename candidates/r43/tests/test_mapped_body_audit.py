"""Actual boundary reporting must not perpetuate the initial overhang allowance."""
from pathlib import Path
import sys
import unittest

from shapely.geometry import Polygon

HERE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(HERE/'tools'), str(HERE/'source')]
from mapped_body_audit import audit_frames
from stack.core.road_boundary import RoadBoundary


class MappedBodyAuditTests(unittest.TestCase):
    def boundary(self, polygon):
        boundary = RoadBoundary.__new__(RoadBoundary)
        boundary.area = polygon
        boundary.invalid, boundary.seams, boundary.shared_borders = [], [], []
        return boundary

    def frames(self, points):
        return [dict(t=float(index), x=x, y=y, yaw=0., v=0., a=0., rot=0., L=1., W=1.)
                for index, (x, y) in enumerate(points)]

    def test_recovered_initial_overhang_cannot_hide_smaller_new_exit(self):
        boundary = self.boundary(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]))
        result = audit_frames(self.frames([(.1, 5), (1, 5), (.4, 5)]), boundary)
        self.assertAlmostEqual(result['initial_exterior_area'], .4)
        self.assertEqual(result['initial_recovery_t'], 1.)
        self.assertEqual(result['first_exterior_growth_t'], 2.)
        self.assertEqual(len(result['episodes']), 2)

    def test_shrinking_initial_overhang_is_separate_from_unrecovered_state(self):
        boundary = self.boundary(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]))
        recovered = audit_frames(self.frames([(.1, 5), (.3, 5), (1, 5)]), boundary)
        self.assertIsNone(recovered['first_exterior_growth_t'])
        self.assertFalse(recovered['initial_exterior_unrecovered'])
        unrecovered = audit_frames(self.frames([(.1, 5), (.3, 5)]), boundary)
        self.assertTrue(unrecovered['initial_exterior_unrecovered'])
        self.assertIsNone(unrecovered['first_exterior_growth_t'])

    def test_hole_crossing_is_actual_exterior_even_inside_map_envelope(self):
        boundary = self.boundary(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)],
            holes=[[(4, 4), (6, 4), (6, 6), (4, 6)]]))
        result = audit_frames(self.frames([(2, 5), (5, 5), (8, 5)]), boundary)
        self.assertEqual(result['first_exterior_growth_t'], 1.)
        self.assertEqual(result['outside_frame_count'], 1)
        self.assertAlmostEqual(result['max_exterior_area'], 1.)


if __name__ == '__main__':
    unittest.main()
