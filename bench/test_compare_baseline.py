"""Contract tests for exact paired baseline comparisons (standard library)."""
import copy
import unittest

from compare_baseline import ComparisonError, compare_documents


def row(name, end="到达", latency=True):
    result = {"name": name, "end": end, "roadtype": "test", "error": None,
              "t_end": 10, "score": {"comfort": 9, "comfort_detail": {"lon_jerk": 0.1}}}
    if latency:
        result["latency"] = {"mean": 5, "p95": 8, "maximum": 10, "passed": True}
    return result


class PairedComparisonTests(unittest.TestCase):
    def test_duplicate_and_mismatched_scenes_are_rejected(self):
        for candidate in ([row("a"), row("a")], [row("b")], []):
            with self.subTest(candidate=candidate):
                with self.assertRaises(ComparisonError):
                    compare_documents({"rows": [row("a")]}, {"rows": candidate})

    def test_missing_timing_cannot_inherit_a_pass(self):
        result = compare_documents({"rows": [row("a")]}, {"rows": [row("a", latency=False)]})
        timing = result["latency"]["candidate"]
        self.assertFalse(timing["complete"])
        self.assertFalse(timing["all_source_rows_report_pass"])
        self.assertEqual(timing["acceptance_status"], "unknown")
        self.assertFalse(result["promotion"]["approved"])

    def test_collision_improvement_cannot_hide_a_new_collision(self):
        before = {"rows": [row("fixed", "碰撞"), row("regressed", "超时")]}
        after = {"rows": [row("fixed"), row("regressed", "碰撞")]}
        result = compare_documents(before, after)
        self.assertEqual(result["total"]["baseline"]["outcomes"]["碰撞"], 1)
        self.assertEqual(result["total"]["candidate"]["outcomes"]["碰撞"], 1)
        self.assertEqual(result["new_collisions"], ["regressed"])
        self.assertEqual(result["gained"], ["fixed"])

    def test_time_pairs_exclude_zero_and_preserve_score_detail(self):
        before = {"rows": [row("a"), row("zero")]}
        after = copy.deepcopy(before)
        after["rows"][0]["t_end"] = 8
        after["rows"][0]["score"]["comfort_detail"]["lon_jerk"] = 0.2
        before["rows"][1]["t_end"] = 0
        result = compare_documents(before, after)
        self.assertEqual(result["common_arrivals"]["median_time_ratio"], 0.8)
        self.assertEqual(result["common_arrivals"]["invalid_time_pairs"][0]["name"], "zero")
        self.assertAlmostEqual(result["scores"]["per_scene"][0]["comfort_delta"]["comfort_detail"]["lon_jerk"], 0.1)

    def test_new_error_is_separate_and_cannot_be_an_arrival(self):
        before = {"rows": [row("a")]}
        after = copy.deepcopy(before)
        after["rows"][0]["error"] = "raised after action"
        result = compare_documents(before, after)
        self.assertEqual(result["new_errors"], ["a"])
        self.assertEqual(result["lost"], ["a"])
        self.assertEqual(result["common_arrivals"]["n"], 0)
        self.assertFalse(result["latency"]["candidate"]["all_source_rows_report_pass"])

    def test_matching_names_do_not_verify_source_or_environment(self):
        document = {"rows": [row("a")]}
        result = compare_documents(document, document)
        self.assertEqual(result["evidence"]["fixed_source_identity"], "unknown")
        self.assertEqual(result["evidence"]["same_inputs"], "unknown")
        self.assertEqual(result["evidence"]["same_environment"], "unknown")
        self.assertFalse(result["promotion"]["approved"])

    def test_conflicting_road_types_are_rejected(self):
        before = {"rows": [row("a")]}
        after = copy.deepcopy(before)
        after["rows"][0]["roadtype"] = "another road"
        with self.assertRaises(ComparisonError):
            compare_documents(before, after)

    def test_malformed_timing_cannot_report_pass(self):
        before = {"rows": [row("a")]}
        after = copy.deepcopy(before)
        after["rows"][0]["latency"]["maximum"] = 2
        timing = compare_documents(before, after)["latency"]["candidate"]
        self.assertFalse(timing["complete"])
        self.assertFalse(timing["all_source_rows_report_pass"])


if __name__ == "__main__":
    unittest.main()
