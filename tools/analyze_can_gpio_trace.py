"""Export the standalone probe's GPIO DMA trace and mark the expected ACK slot.

This is a digital sample trace, not an analog signal-quality measurement. The
ACK position is derived from the known standard data frame and nominal bitrate.
"""
import argparse
import csv
import json
from pathlib import Path
import struct


def bits(value, width):
    return [(value >> i) & 1 for i in range(width - 1, -1, -1)]


def stuffed_frame(identifier, data):
    logical = [0] + bits(identifier, 11) + [0, 0, 0] + bits(len(data), 4)
    logical += [bit for byte in data for bit in bits(byte, 8)]
    crc = 0
    for bit in logical:
        feedback = ((crc >> 14) & 1) ^ bit
        crc = (crc << 1) & 0x7FFF
        if feedback:
            crc ^= 0x4599
    stuffed = []
    previous, run = None, 0
    for bit in logical + bits(crc, 15):
        stuffed.append(bit)
        run = run + 1 if bit == previous else 1
        previous = bit
        if run == 5:
            stuffed.append(1 - bit)
            previous, run = 1 - bit, 1
    return stuffed, crc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--sequence', type=int, required=True)
    parser.add_argument('--sample-rate', type=float, default=1_000_000)
    parser.add_argument('--bitrate', type=float, default=500_000)
    parser.add_argument('--pin-map', choices=('pd0-pd1', 'pa11-pa12'), default='pd0-pd1',
                        help='GPIO mapping; use pa11-pa12 for captures before the CAN1 pin migration')
    args = parser.parse_args()
    raw = args.trace.read_bytes()
    if not raw or len(raw) % 4:
        parser.error('Trace must contain complete little-endian GPIO IDR words')
    words = struct.unpack('<' + 'I' * (len(raw) // 4), raw)
    rx_bit, tx_bit, rx_pin, tx_pin = ((0, 1, 'PD0', 'PD1') if args.pin_map == 'pd0-pd1'
                                     else (11, 12, 'PA11', 'PA12'))
    tx = [(word >> tx_bit) & 1 for word in words]
    rx = [(word >> rx_bit) & 1 for word in words]
    # Assumes capture begins on an idle bus, just before the known probe frame.
    sof = next((i for i in range(1, len(tx)) if tx[i - 1] == 1 and tx[i] == 0), None)
    if sof is None:
        parser.error('No TX falling edge found')
    data = bytes([0xCA, 0x4E, args.sequence & 255, (args.sequence >> 8) & 255,
                  0x55, 0xAA, 0, 0xFF])
    stuffed, crc = stuffed_frame(0x120, data)
    start_us = sof * 1e6 / args.sample_rate
    bit_us = 1e6 / args.bitrate
    ack_start = start_us + (len(stuffed) + 1) * bit_us
    ack_end = ack_start + bit_us
    times = [i * 1e6 / args.sample_rate for i in range(len(words))]
    ack_indices = [i for i, time in enumerate(times) if ack_start <= time < ack_end]
    report = {
        'samples': len(words), 'sample_rate_hz': args.sample_rate,
        'rx_pin': rx_pin, 'tx_pin': tx_pin,
        'assumed_bitrate': args.bitrate, 'assumed_probe_sequence': args.sequence,
        'expected_id': '0x120', 'expected_data': data.hex().upper(),
        'expected_crc15': hex(crc), 'first_tx_falling_edge_us': start_us,
        'expected_ack_start_us': ack_start, 'expected_ack_end_us': ack_end,
        'tx_rx_different_samples': sum(a != b for a, b in zip(tx, rx)),
        'ack_rx_samples': [rx[i] for i in ack_indices],
        'ack_tx_samples': [tx[i] for i in ack_indices],
        'note': 'ACK location assumes idle start and nominal bit timing; this is not a full CAN decoder.',
    }
    args.trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    with args.trace.with_suffix('.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_us', tx_pin + '_TX', rx_pin + '_RX'])
        writer.writerows(zip(times, tx, rx))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(11, 5.8), constrained_layout=True)
    for axis in axes:
        axis.step(times, [value + 2.3 for value in tx], where='post', label=tx_pin + ' TX', color='#1769aa')
        axis.step(times, rx, where='post', label=rx_pin + ' RX', color='#00846b')
        axis.axvspan(ack_start, ack_end, alpha=0.2, color='#d14b4b', label='Expected ACK slot')
        axis.set_yticks([0, 1, 2.3, 3.3], ['RX 0', 'RX 1', 'TX 0', 'TX 1'])
        axis.set_ylim(-0.4, 3.8)
        axis.set_xlabel('Time from capture start (microseconds)')
        axis.grid(axis='x', alpha=0.2)
    axes[0].set_xlim(0, min(times[-1], ack_end + 35))
    axes[0].set_title('STM32 CAN1 GPIO samples: probe ID 0x120, sequence ' + str(args.sequence))
    axes[0].legend(loc='upper right', fontsize=8)
    axes[1].set_xlim(max(0, ack_start - 16), ack_end + 25)
    axes[1].set_title('ACK detail: low RX indicates acknowledgement; high RX is recessive')
    fig.savefig(args.trace.with_suffix('.png'), dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
