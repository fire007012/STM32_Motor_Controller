import unittest

from tools.aa_packer import BuildError, cmd_position, cmd_speed, pack_aa_stream


class PackerTests(unittest.TestCase):
    def test_x_speed(self):
        self.assertEqual(cmd_speed(1, 100, 50, 0), [1, 0xF6, 0, 0, 50, 3, 0xE8, 0, 0x6B])
        self.assertEqual(cmd_speed(1, -2147483648, 0, 1), [1, 0xF6, 1, 0, 0, 0x75, 0x30, 1, 0x6B])

    def test_x_position(self):
        self.assertEqual(cmd_position(1, 3200, 100, 50, 0, 0),
                         [1, 0xFB, 0, 3, 0xE8, 0, 0, 0x0E, 0x10, 0, 0, 0x6B])
        with self.assertRaises(BuildError):
            cmd_position(1, 1, 100, 50, 3, 0)

    def test_emm_explicit(self):
        self.assertEqual(cmd_speed(1, 100, 50, 0, "emm"), [1, 0xF6, 0, 0, 100, 50, 0, 0x6B])
        self.assertEqual(cmd_position(1, 3200, 100, 50, 0, 0, "emm"),
                         [1, 0xFD, 0, 0, 100, 50, 0, 0, 0x0C, 0x80, 0, 0, 0x6B])

    def test_aa_total_length_and_bounds(self):
        self.assertEqual(pack_aa_stream([1, 0x36, 0x6B]), [0xAA, 0, 8, 1, 0x36, 0x6B, 0x6B])
        self.assertEqual(pack_aa_stream([0] * 92)[2], 97)
        with self.assertRaises(BuildError):
            pack_aa_stream([0] * 93)


if __name__ == "__main__":
    unittest.main()
