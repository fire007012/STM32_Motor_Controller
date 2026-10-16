"""Read STM32 CAN/feedback diagnostics through an existing OpenOCD Tcl server.

Read only: never halts/resets the CPU, sends CAN commands or writes target memory.
Use the exact ELF that was programmed. JSONL retains every sample, including
failures. This observes STM32, not Linux error frames or remote application ACKs.
"""
import argparse
import datetime
import json
from pathlib import Path
import socket
import struct
import subprocess
import time


FIELDS = ("tick mcr btr esr tsr last_error_tick last_error_esr error_events "
          "tx_ok tx_error arbitration_lost tx_timeouts reply_retries reply_overflow "
          "reply_expired reply_pending rx_frames last_rx_id format_version "
          "tx_pending tx_high_water queue_full hal_submit_errors hal_submit_busy "
          "latest_replaced latest_backpressure canceled_frames timeout_discarded "
          "error_warning_events error_passive_events bus_off_events").split()
FIELDS += [f"lec_{n}" for n in range(8)]
FIELDS += ["rx_overruns", "tx_failures", "backoff_rejected"]
COUNTERS = ("tx_ok rx_frames tx_failures queue_full hal_submit_errors hal_submit_busy "
            "tx_error tx_timeouts reply_overflow reply_expired error_events rx_overruns "
            "latest_backpressure latest_replaced").split()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--interval", type=float, default=1)
    parser.add_argument("--port", type=int, default=6666)
    args = parser.parse_args()
    if args.seconds < 0 or args.interval <= 0:
        parser.error("seconds must be nonnegative; interval must be positive")
    listing = subprocess.check_output(["arm-none-eabi-nm", "-S", str(args.elf)], text=True)
    symbols = {}
    for line in listing.splitlines():
        parts = line.split()
        if len(parts) == 4:
            symbols[parts[-1]] = (int(parts[0], 16), int(parts[1], 16))
    base, size = symbols["can_bus_diagnostics"]
    if size != len(FIELDS) * 8:
        raise RuntimeError("ELF diagnostic size differs from this format-v2 reader")

    with socket.create_connection(("127.0.0.1", args.port), timeout=10) as connection:
        def command(text):
            connection.sendall(text.encode() + b"\x1a")
            result = b""
            while not result.endswith(b"\x1a"):
                block = connection.recv(4096)
                if not block:
                    raise RuntimeError("OpenOCD closed its Tcl connection")
                result += block
            return result[:-1].decode().strip()

        def words(address, count):
            result = command(f"read_memory 0x{address:x} 32 {count}")
            values = [int(word, 0) for word in result.split()]
            if len(values) != count:
                raise RuntimeError(f"Unexpected memory read: {result}")
            return values

        def symbol_words(name):
            address, length = symbols[name]
            return words(address, length // 4)

        started = time.monotonic()
        baseline = None
        records = []
        with args.output.open("w", encoding="utf-8") as output:
            while True:
                data = words(base, size // 4)
                tick = words(symbols["uwTick"][0], 1)[0]
                record = {"time": datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
                          "elapsed_seconds": time.monotonic() - started,
                          "tick": tick, "cpu_state": command("stm32f4x.cpu curstate")}
                for bus in range(2):
                    diag = dict(zip(FIELDS, data[bus * len(FIELDS):(bus + 1) * len(FIELDS)]))
                    if diag["format_version"] != 2:
                        raise RuntimeError("Running target does not expose diagnostic format 2")
                    esr = diag["esr"]
                    diag["TEC"], diag["REC"], diag["LEC"] = (esr >> 16) & 255, esr >> 24, (esr >> 4) & 7
                    record[f"can{bus + 1}"] = diag
                feedback = symbol_words("zdt_feedback_diagnostics")
                motor_words = symbol_words("motors")
                motor_bytes = struct.pack("<" + "I" * len(motor_words), *motor_words)
                record["motors"] = []
                for motor in range(4):
                    speed_tick, speed_count, status_tick, status_count = feedback[motor * 4: motor * 4 + 4]
                    target, speed, target_position, position, flags, fault_code, fault = struct.unpack_from(
                        "<iiiiHHB3x", motor_bytes, motor * 24)
                    record["motors"].append({"index": motor, "target_rpm": target, "speed_01rpm": speed,
                        "flags": flags, "fault": fault, "speed_updates": speed_count, "status_updates": status_count,
                        "speed_age_ms": (tick - speed_tick) & 0xFFFFFFFF,
                        "status_age_ms": (tick - status_tick) & 0xFFFFFFFF})
                record["heartbeat_age_ms"] = (tick - symbol_words("ros_last_heartbeat_tick")[0]) & 0xFFFFFFFF
                record["rcc"] = words(0x40023800, 3)
                if baseline is None:
                    baseline = record
                output.write(json.dumps(record) + "\n")
                output.flush()
                records.append(record)
                if record["elapsed_seconds"] >= args.seconds:
                    break
                time.sleep(min(args.interval, args.seconds - record["elapsed_seconds"]))
        summary = {"elapsed_seconds": records[-1]["elapsed_seconds"], "samples": len(records),
                   "limitations": "STM32 SWD only; does not verify host error frames, ROS readiness or remote application receipt."}
        for bus in ("can1", "can2"):
            summary[bus + "_delta"] = {key: (records[-1][bus][key] - baseline[bus][key]) & 0xFFFFFFFF for key in COUNTERS}
            summary[bus + "_final"] = records[-1][bus]
        summary["motor_updates_delta"] = [{"index": motor,
            "speed": (records[-1]["motors"][motor]["speed_updates"] - baseline["motors"][motor]["speed_updates"]) & 0xFFFFFFFF,
            "status": (records[-1]["motors"][motor]["status_updates"] - baseline["motors"][motor]["status_updates"]) & 0xFFFFFFFF
            } for motor in range(4)]
        summary["motors_final"] = records[-1]["motors"]
        summary["heartbeat_age_ms_final"] = records[-1]["heartbeat_age_ms"]
        args.output.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
