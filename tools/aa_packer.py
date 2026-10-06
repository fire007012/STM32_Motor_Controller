#!/usr/bin/env python3
"""
Build ZDT AA multi-motor command payloads for STM32 `motor_send_multi_cmd()`.

Example:
  python tools/aa_packer.py \
    --speed 1 1200 10 0 \
    --position 2 20000 800 12 0 1 \
    --read-pos 3 \
    --out hex
"""

from __future__ import annotations

import argparse
from typing import List

ZDT_CHECK = 0x6B


class BuildError(ValueError):
    pass


def _u8(v: int, name: str) -> int:
    if v < 0 or v > 0xFF:
        raise BuildError(f"{name} out of range: {v}")
    return v


def _u16(v: int, name: str) -> int:
    if v < 0 or v > 0xFFFF:
        raise BuildError(f"{name} out of range: {v}")
    return v


def _i32(v: int, name: str) -> int:
    if v < -2147483648 or v > 2147483647:
        raise BuildError(f"{name} out of range: {v}")
    return v


def cmd_speed(addr: int, rpm: int, accel: int, sync: int) -> List[int]:
    _u8(addr, "addr")
    _u8(accel, "accel")
    _u8(sync, "sync")
    rpm = _i32(rpm, "rpm")

    direction = 1 if rpm < 0 else 0
    abs_rpm = min(abs(rpm), 3000)
    return [
        addr,
        0xF6,
        direction,
        (abs_rpm >> 8) & 0xFF,
        abs_rpm & 0xFF,
        accel,
        1 if sync else 0,
        ZDT_CHECK,
    ]


def cmd_position(addr: int, pulses: int, speed_rpm: int, accel: int, mode: int, sync: int) -> List[int]:
    _u8(addr, "addr")
    _u8(accel, "accel")
    _u8(mode, "mode")
    _u8(sync, "sync")
    pulses = _i32(pulses, "pulses")
    speed_rpm = min(_u16(speed_rpm, "speed_rpm"), 3000)

    direction = 1 if pulses < 0 else 0
    abs_pulse = abs(pulses)

    return [
        addr,
        0xFD,
        direction,
        (speed_rpm >> 8) & 0xFF,
        speed_rpm & 0xFF,
        accel,
        (abs_pulse >> 24) & 0xFF,
        (abs_pulse >> 16) & 0xFF,
        (abs_pulse >> 8) & 0xFF,
        abs_pulse & 0xFF,
        mode,
        1 if sync else 0,
        ZDT_CHECK,
    ]


def cmd_stop(addr: int, sync: int) -> List[int]:
    _u8(addr, "addr")
    _u8(sync, "sync")
    return [addr, 0xFE, 0x98, 1 if sync else 0, ZDT_CHECK]


def cmd_read_pos(addr: int) -> List[int]:
    _u8(addr, "addr")
    return [addr, 0x36, ZDT_CHECK]


def cmd_read_speed(addr: int) -> List[int]:
    _u8(addr, "addr")
    return [addr, 0x35, ZDT_CHECK]


def cmd_read_status(addr: int) -> List[int]:
    _u8(addr, "addr")
    return [addr, 0x3A, ZDT_CHECK]


def pack_aa_stream(stream: List[int]) -> List[int]:
    if not stream:
        raise BuildError("empty command stream")
    if len(stream) > 0xFFFF:
        raise BuildError("command stream too long")

    ln = len(stream)
    return [0xAA, (ln >> 8) & 0xFF, ln & 0xFF] + stream + [ZDT_CHECK]


def fmt_hex(data: List[int]) -> str:
    return " ".join(f"{b:02X}" for b in data)


def fmt_c_array(data: List[int], symbol: str = "aa_payload") -> str:
    items = ", ".join(f"0x{b:02X}" for b in data)
    return f"static const uint8_t {symbol}[] = {{{items}}};"


def build_from_args(args: argparse.Namespace) -> List[int]:
    stream: List[int] = []

    for item in args.speed:
        stream.extend(cmd_speed(item[0], item[1], item[2], item[3]))
    for item in args.position:
        stream.extend(cmd_position(item[0], item[1], item[2], item[3], item[4], item[5]))
    for item in args.stop:
        stream.extend(cmd_stop(item[0], item[1]))
    for item in args.read_pos:
        stream.extend(cmd_read_pos(item[0]))
    for item in args.read_speed:
        stream.extend(cmd_read_speed(item[0]))
    for item in args.read_status:
        stream.extend(cmd_read_status(item[0]))

    if args.wrap_aa:
        return pack_aa_stream(stream)
    return stream


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ZDT AA command stream packer")

    p.add_argument("--speed", nargs=4, action="append", metavar=("ADDR", "RPM", "ACC", "SYNC"), type=int, default=[])
    p.add_argument(
        "--position",
        nargs=6,
        action="append",
        metavar=("ADDR", "PULSE", "SPEED", "ACC", "MODE", "SYNC"),
        type=int,
        default=[],
    )
    p.add_argument("--stop", nargs=2, action="append", metavar=("ADDR", "SYNC"), type=int, default=[])
    p.add_argument("--read-pos", nargs=1, action="append", metavar=("ADDR",), type=int, default=[])
    p.add_argument("--read-speed", nargs=1, action="append", metavar=("ADDR",), type=int, default=[])
    p.add_argument("--read-status", nargs=1, action="append", metavar=("ADDR",), type=int, default=[])

    p.add_argument("--wrap-aa", action="store_true", help="Output full AA payload (0xAA + len + stream + 0x6B)")
    p.add_argument("--out", choices=["hex", "c"], default="hex", help="Output format")

    return p.parse_args()


def main() -> int:
    args = parse_args()

    try:
        data = build_from_args(args)
    except BuildError as ex:
        print(f"error: {ex}")
        return 2

    if args.out == "c":
        print(fmt_c_array(data))
    else:
        print(fmt_hex(data))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
