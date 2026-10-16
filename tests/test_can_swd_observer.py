"""Prevent old ELF layouts from shifting the CAN2 diagnostic offsets."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from observe_can_swd import FORMATS, decode_diagnostics


class CanDiagnosticLayouts(unittest.TestCase):
    def test_old_and_new_two_bus_layouts(self):
        for version, fields in FORMATS.items():
            with self.subTest(version=version):
                data = []
                for bus in range(2):
                    data.extend(version if key == "format_version" else 1000 * bus + index
                                for index, key in enumerate(fields))
                decoded = decode_diagnostics(data, len(data) * 4)
                self.assertEqual(decoded[1]["tick"], 1000)
                self.assertEqual(decoded[0]["format_version"], version)
                self.assertEqual("forward_pending" in decoded[0], version == 3)

    def test_rejects_wrong_elf_and_incomplete_reads(self):
        fields = FORMATS[3]
        data = [0] * (len(fields) * 2)
        with self.assertRaises(RuntimeError):
            decode_diagnostics(data, len(data) * 4)
        data[fields.index("format_version")] = 3
        data[len(fields) + fields.index("format_version")] = 3
        with self.assertRaises(RuntimeError):
            decode_diagnostics(data[:-1], len(data) * 4)
        with self.assertRaises(RuntimeError):
            decode_diagnostics(data, 16)
