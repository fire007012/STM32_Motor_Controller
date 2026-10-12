"""Run the actual two-servo driver with fake HAL, no MCU/physical motion."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class ArmServoControl(unittest.TestCase):
    def run_variant(self, substitutions=None):
        compiler = shutil.which('gcc')
        if not compiler:
            self.skipTest('Native GCC required')
        with tempfile.TemporaryDirectory(prefix='arm-servo-test-') as folder:
            directory = Path(folder)
            config = (ROOT / 'Core/Inc/arm_servo_config.h').read_text(encoding='utf-8')
            for name, value in (substitutions or {}).items():
                config, count = re.subn(r'(^#define ' + name + r'\s+)\S+',
                                        lambda m: m.group(1) + str(value), config, flags=re.M)
                self.assertEqual(count, 1)
            (directory / 'arm_servo_config.h').write_text(config, encoding='utf-8')
            shutil.copyfile(ROOT / 'Core/Inc/arm_servo.h', directory / 'arm_servo.h')
            shutil.copyfile(ROOT / 'Core/Src/arm_servo.c', directory / 'arm_servo.c')
            exe = directory / ('check.exe' if __import__('sys').platform == 'win32' else 'check')
            subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-Wno-misleading-indentation', '-include',
                            str(ROOT / 'tests/support/arm_servo_hal_stub.h'),
                            '-I', str(directory), '-I', str(ROOT / 'Core/Inc'),
                            str(directory / 'arm_servo.c'),
                            str(ROOT / 'tests/arm_servo_regression.c'), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_default_180_degree_servos(self):
        self.run_variant()

    def test_270_degree_variant_and_mirrored_second_servo(self):
        self.run_variant({'ARM_SERVO_1_TRAVEL_DEG': '270U',
                          'ARM_SERVO_2_TRAVEL_DEG': '270U',
                          'ARM_SERVO_2_DIRECTION': '-1'})

if __name__ == '__main__':
    unittest.main()
