import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "components", "vl53l5cx"))

from merge_calibrations import merge, parse_log, render  # noqa: E402

HDR = "#! resolution: 8x8\n#! rotation: 180\n#! max_distance_cm: 205\n"


def run(body):
    return parse_log(HDR + body)


class MergeTests(unittest.TestCase):
    def test_parses_esphome_prefixed_lines(self):
        text = "[22:28:42.140][I][vl53l5cx:941]: #! rotation: 180\n[22:28:42.159][I][vl53l5cx:941]: 2>142/8,205/2\n"
        d, z = parse_log(text)
        self.assertEqual(d["rotation"], "180")
        self.assertEqual(z[2], [(142, 8), (205, 2)])

    def test_union_widens_and_respects_min_delta(self):
        d, m, notes, zn = merge([run("1>134/3\n"), run("1>143/19\n")], 5)
        self.assertEqual(len(m[1]), 1)           # 131..137 lies inside 124..162
        self.assertEqual(m[1][0], (143, 19))
        d, m, _, _ = merge([run("5>205/2\n")], 5)
        self.assertEqual(m[5], [(205, 5)])       # widened to the minimum delta

    def test_disjoint_bands_stay_separate(self):
        _, m, _, _ = merge([run("3>100/5\n"), run("3>150/5\n")], 5)
        self.assertEqual(m[3], [(100, 5), (150, 5)])

    def test_more_than_four_bands_are_merged_with_a_note(self):
        body = "7>50/5,100/5,150/5,200/5\n"
        _, m, _, zn = merge([run(body), run("7>250/5\n")], 5)
        self.assertLessEqual(len(m[7]), 4)
        self.assertIn(7, zn)

    def test_rotation_mismatch_is_an_error(self):
        other = parse_log(HDR.replace("180", "90") + "1>100/5\n")
        with self.assertRaises(ValueError):
            merge([run("1>100/5\n"), other], 5)

    def test_output_passes_the_compile_time_parser(self):
        import esphome.config_validation  # noqa: F401  (needs ESPHome installed)
        from environment import parse_environment_text

        runs = [run("0>102/7\n1>134/3\n2>142/8,205/2\n25>187/56\n"), run("0>100/5\n1>143/19,205/11\n25>184/64\n")]
        d, m, notes, zn = merge(runs, 5)
        result = parse_environment_text(render(d, m, notes, zn, ["a", "b"]))
        self.assertTrue(result["configured"])
        self.assertEqual(result["rotation"], 2)


if __name__ == "__main__":
    unittest.main()
