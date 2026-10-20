"""Tests of the environment.conf parser. Needs ESPHome installed (cv.Invalid).

    python -m unittest tests/test_environment_file.py        (from the project root)
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "components", "vl53l5cx"))

import esphome.config_validation as cv  # noqa: E402
from environment import (  # noqa: E402
    TABLE_STRIDE,
    load_environment_file,
    parse_environment_text,
)

HEADER = "#! resolution: 8x8\n#! rotation: 90\n#! max_distance_cm: 400\n"


def parse(text):
    return parse_environment_text(text, "environment.conf")


class ParserTests(unittest.TestCase):
    def assertInvalid(self, text, *fragments):
        with self.assertRaises(cv.Invalid) as ctx:
            parse(text)
        message = str(ctx.exception)
        for fragment in fragments:
            self.assertIn(fragment, message)

    def test_empty_and_commented_files_are_ignored(self):
        for text in ("", "\n\n", "# only a comment\n", "# a\n#! resolution: 8x8\n# 0>150/3\n"):
            result = parse(text)
            self.assertFalse(result["configured"])
            self.assertEqual(result["zones_with_refs"], 0)
            self.assertEqual(sum(result["table"]), 0)

    def test_valid_file(self):
        result = parse("# c\n" + HEADER + "0>150/3,180/8,200/5\n1>95/4,200/6  # trailing\n\n2>130/5\n")
        self.assertTrue(result["configured"])
        self.assertEqual(result["rotation"], 1)  # 90 degrees
        self.assertEqual(result["max_distance_cm"], 400)
        t = result["table"]
        self.assertEqual(len(t), 64 * TABLE_STRIDE)
        self.assertEqual(t[0:9], [3, 150, 180, 200, 0, 3, 8, 5, 0])
        self.assertEqual(t[9:18], [2, 95, 200, 0, 0, 4, 6, 0, 0])
        self.assertEqual(t[18:27], [1, 130, 0, 0, 0, 5, 0, 0, 0])

    def test_explicit_empty_zone_and_degree_sign(self):
        result = parse("#! resolution: 8X8\n#! rotation: 180°\n5>\n6>210/5\n")
        self.assertEqual(result["table"][5 * TABLE_STRIDE], 0)
        self.assertEqual(result["rotation"], 2)
        self.assertEqual(result["zones_with_refs"], 1)

    def test_only_empty_zones_is_not_configured(self):
        self.assertFalse(parse(HEADER + "5>\n")["configured"])

    def test_invalid_syntax(self):
        self.assertInvalid(HEADER + "0 150/3\n", "environment.conf:4", "invalid line")
        self.assertInvalid(HEADER + "0>150\n", ":4", "distance/delta")
        self.assertInvalid(HEADER + "0>abc/3\n", ":4", "invalid reference")
        self.assertInvalid(HEADER + "0>150/3,\n", ":4", "invalid reference")

    def test_zone_errors(self):
        self.assertInvalid(HEADER + "64>150/3\n", "zone 64 does not exist")
        self.assertInvalid(HEADER + "0>150/3\n0>200/3\n", ":5", "already defined on line 4")

    def test_value_errors(self):
        self.assertInvalid(HEADER + "0>0/3\n", "out of range")
        self.assertInvalid(HEADER + "0>401/3\n", "out of range")
        self.assertInvalid(HEADER + "0>150/101\n", "delta 101")

    def test_too_many_references(self):
        self.assertInvalid(HEADER + "0>50/1,100/1,150/1,200/1,250/1\n", "5 references", "at most 4")
        self.assertIn("4 refs ok", "4 refs ok" if parse(HEADER + "0>50/1,100/1,150/1,200/1\n")["configured"] else "")

    def test_duplicates_and_overlaps(self):
        self.assertInvalid(HEADER + "0>150/3,150/5\n", "duplicate reference 150")
        self.assertInvalid(HEADER + "0>150/5,158/5\n", "overlap")
        self.assertInvalid(HEADER + "0>150/5,160/5\n", "touch")  # 150+5 == 160-5
        self.assertTrue(parse(HEADER + "0>150/5,161/5\n")["configured"])  # separated by one centimetre

    def test_directives(self):
        self.assertInvalid("0>150/3\n", "no '#! resolution")
        self.assertInvalid("#! resolution: 8x8\n0>150/3\n", "no '#! rotation")
        self.assertInvalid("#! resolution: 4x4\n#! rotation: 0\n0>150/3\n", "only 8x8")
        self.assertInvalid("#! resolution: 8x8\n#! rotation: 45\n0>150/3\n", "invalid rotation")
        self.assertInvalid("#! resolution: 8x8\n#! rotation: 0\n#! rotation: 90\n0>1/1\n", "twice")
        self.assertInvalid("#! resolution: 8x8\n#! rotation: 0\n#! colour: red\n0>150/3\n", "unknown directive")
        self.assertInvalid("#! resolution: 8x8\n#! rotation: 0\n#! max_distance_cm: 900\n0>150/3\n", "max_distance_cm")

    def test_missing_file_is_an_error(self):
        with self.assertRaises(cv.Invalid) as ctx:
            load_environment_file("/nonexistent/environment.conf")
        self.assertIn("mandatory", str(ctx.exception))

    def test_shipped_template_is_valid_and_ignored(self):
        path = os.path.join(os.path.dirname(__file__), "..", "components", "vl53l5cx", "environment.conf")
        self.assertFalse(load_environment_file(path)["configured"])

    def test_calibration_output_round_trips(self):
        # the format printed by the component must parse unchanged
        report = "#! resolution: 8x8\n#! rotation: 0\n#! max_distance_cm: 400\n#! frames: 60\n"
        report += "# zone 1: observed 150 cm (n=40), 120 cm (n=1, noise)\n0>150/3,210/5\n1>400/2\n2>\n"
        result = parse(report)
        self.assertEqual(result["zones_with_refs"], 2)

    def test_file_on_disk(self):
        with tempfile.NamedTemporaryFile("w", suffix=".conf", delete=False) as fh:
            fh.write(HEADER + "3>123/4\n")
        try:
            self.assertTrue(load_environment_file(fh.name)["configured"])
        finally:
            os.unlink(fh.name)


if __name__ == "__main__":
    unittest.main()
