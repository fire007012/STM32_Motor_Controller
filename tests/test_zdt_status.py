"""Host regression test for the actual STM32 status classifier (no board needed)."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class StatusClassifier(unittest.TestCase):
    def test_vendor_status_flags(self):
        compiler = shutil.which('gcc')
        if not compiler:
            self.skipTest('host gcc not available')
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix='zdt-status-') as tmp:
            tmp = Path(tmp)
            # Compile the production source/header unchanged against tiny host stubs.
            shutil.copyfile(root / 'Core/Inc/zdt_status.h', tmp / 'zdt_status.h')
            shutil.copyfile(root / 'Core/Src/zdt_status.c', tmp / 'zdt_status.c')
            (tmp / 'main.h').write_text(
                '#include <stdint.h>\n#include <stddef.h>\n'
                'static uint32_t test_primask;\n'
                'static inline uint32_t __get_PRIMASK(void) { return test_primask; }\n'
                'static inline void __disable_irq(void) { test_primask = 1U; }\n'
                'static inline void __enable_irq(void) { test_primask = 0U; }\n')
            (tmp / 'motor_control.h').write_text('#define MOTOR_COUNT 5U\n')
            (tmp / 'check.c').write_text(r'''#include <assert.h>
#include "zdt_status.h"
#include "motor_control.h"
int main(void) {
    zdt_estop_event_t event;
    unsigned flags;
    zdt_status_init();
    for (flags = 0; flags < 256; ++flags) {
        assert(zdt_status_is_fault((uint8_t)flags) == !!(flags & 0x08));
    }
    assert(zdt_status_classify(0x80) == ZDT_EVENT_CLASS_INFO);
    assert(zdt_status_classify(0x81) == ZDT_EVENT_CLASS_INFO);
    assert(zdt_status_classify(0x83) == ZDT_EVENT_CLASS_INFO);
    assert(zdt_status_classify(0x85) == ZDT_EVENT_CLASS_WARNING);
    assert(zdt_status_classify(0x89) == ZDT_EVENT_CLASS_FAULT);
    zdt_status_update_status(0, 0x83);
    assert(!zdt_status_take_estop_event(&event));
    zdt_status_update_status(0, 0x85);
    assert(!zdt_status_take_estop_event(&event));
    zdt_status_update_speed(1, -100);
    zdt_status_update_status(1, 0x89);
    assert(zdt_status_take_estop_event(&event));
    assert(event.motor_idx == 1 && event.status_flags == 0x89 && event.speed_rpm == -100);
    assert(!zdt_status_take_estop_event(&event));
    zdt_status_update_status(MOTOR_COUNT, 0x89);
    assert(!zdt_status_take_estop_event(&event));
    return 0;
}
''')
            exe = tmp / ('check.exe' if os.name == 'nt' else 'check')
            subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(tmp), str(tmp / 'zdt_status.c'), str(tmp / 'check.c'),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
