"""Compile the real CAN/control sources with a fake HAL and run regressions."""
from pathlib import Path
import shutil
import subprocess
import sys


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("gcc")
    if compiler is None:
        print("Native gcc is required (not arm-none-eabi-gcc).", file=sys.stderr)
        return 2
    output = root / "build" / "host-tests"
    output.mkdir(parents=True, exist_ok=True)
    executable = output / ("can_regression.exe" if sys.platform == "win32" else "can_regression")
    sources = ["can_transport", "zdt_can_driver", "motor_control", "can_protocol", "zdt_status", "stm32f4xx_it", "chassis_collision"]
    command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-g",
               '-DCHASSIS_KINEMATICS_HEADER="chassis_test_config.h"',
               "-include", str(root / "tests/support/hal_stub.h"),
               "-I", str(root / "tests/support"), "-I", str(root / "Core/Inc"),
               str(root / "tests/can_regression.c")]
    command += [str(root / "Core/Src" / f"{name}.c") for name in sources]
    command += ["-o", str(executable)]
    subprocess.run(command, cwd=root, check=True)
    subprocess.run([str(executable)], cwd=root, check=True)
    for test_name, config in [("production_chassis", None),
                              ("unconfigured_chassis", "chassis_unconfigured_config.h")]:
        executable = output / (test_name + (".exe" if sys.platform == "win32" else ""))
        variant_command = [arg for arg in command if not arg.startswith('-DCHASSIS_KINEMATICS_HEADER=')]
        if config:
            variant_command.insert(1, f'-DCHASSIS_KINEMATICS_HEADER="{config}"')
        variant_command[variant_command.index(str(root / "tests/can_regression.c"))] = str(root / "tests" / f"{test_name}.c")
        variant_command[-1] = str(executable)
        subprocess.run(variant_command, cwd=root, check=True)
        subprocess.run([str(executable)], cwd=root, check=True)
    subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"],
                   cwd=root, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
