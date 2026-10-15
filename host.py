#!/usr/bin/env python3
"""UltraTac development host: CRC-checked envelope acquisition; counts, not calibrated volts."""
import argparse
import csv
import json
import pathlib
import secrets
import struct
import time
import zlib

HEADER = struct.Struct('<4sBBHIIIIHBBIIIII')
MAX_FRAME = 4852
ERRORS = ['OK', 'BUSY', 'ARGUMENT', 'TIMEOUT', 'IO', 'PARITY', 'DEVICE',
          'MISMATCH', 'ADC', 'DISCONNECTED', 'ABORTED', 'OVERFLOW']


def command(op, request, argument=0):
    body = struct.pack('<4sB3xII', b'EFC1', op, request, argument)
    return body + struct.pack('<I', zlib.crc32(body))


def decode(packet):
    if len(packet) < HEADER.size + 4:
        raise ValueError('Truncated frame')
    (magic, version, kind, status, size, request, sequence, rate, count, pulses,
     flags, carrier, offset, missed, reset, failures) = HEADER.unpack_from(packet)
    if magic != b'EFW1' or version != 1 or kind not in (1, 2, 3, 4):
        raise ValueError('Unsupported frame header')
    if size != len(packet) or size != HEADER.size + 2 * count + 4 or size > MAX_FRAME:
        raise ValueError('Invalid frame length')
    if zlib.crc32(packet[:-4]) != struct.unpack_from('<I', packet, size - 4)[0]:
        raise ValueError('Frame CRC failed; capture is not valid')
    if status >= len(ERRORS) or rate not in (1200000, 2400000) or not (1 <= pulses <= 20 or (kind == 3 and pulses == 0 and flags & 16)):
        raise ValueError('Invalid metadata')
    if (kind == 3 and (count != 2400 or status != 0)) or (kind != 3 and count != 0):
        raise ValueError('Invalid payload for frame type')
    samples = struct.unpack_from(f'<{count}H', packet, HEADER.size)
    if any(sample > 4095 for sample in samples):
        raise ValueError('ADC value exceeds 12-bit range')
    return dict(kind=kind, status=ERRORS[status], request=request, sequence=sequence,
                sample_rate_hz=rate, sample_count=count, pulses=pulses,
                baseline=bool(flags & 16), streaming=bool(flags & 2), state=['FAULT', 'IDLE', 'ACQUIRING', 'READY'][(flags >> 2) & 3],
                carrier_hz=carrier, nominal_burst_offset_ns=offset,
                timing_calibrated=not bool(flags & 1), missed_slots=missed,
                reset_flags=hex(reset), failures=failures, samples=samples)


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 12:
            if self.buffer[:4] != b'EFW1':
                raise ValueError('Unexpected data; reconnect to establish a new session')
            size = struct.unpack_from('<I', self.buffer, 8)[0]
            if not 52 <= size <= MAX_FRAME:
                raise ValueError('Invalid packet length')
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            del self.buffer[:size]
            frames.append((packet, decode(packet)))
        return frames


class Link:
    def __init__(self, port):
        try:
            import serial
        except ImportError as exc:
            raise SystemExit('Install pyserial: python -m pip install pyserial') from exc
        self.serial = serial.Serial(port, 115200, timeout=0.05, write_timeout=1)
        self.parser = Parser()
        self.queue = []
        self.request = secrets.randbelow(0xffff0000)

    def send(self, op, arg=0):
        self.request = (self.request + 1) & 0xffffffff
        data = command(op, self.request, arg)
        if self.serial.write(data) != len(data):
            raise IOError('Incomplete command write')
        return self.request

    def next(self, deadline):
        while not self.queue:
            if time.monotonic() >= deadline:
                raise TimeoutError('Firmware response timed out; capture was not retried')
            self.queue.extend(self.parser.feed(self.serial.read(4096)))
        return self.queue.pop(0)

    def ack(self, request, timeout=3):
        deadline = time.monotonic() + timeout
        while True:
            _, frame = self.next(deadline)
            if frame['request'] == request and frame['kind'] in (1, 2, 4):
                if frame['status'] != 'OK':
                    raise RuntimeError(json.dumps({k: v for k, v in frame.items() if k != 'samples'}))
                return frame


def save_csv(path, frames):
    with open(path, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['sequence', 'sample_index', 'nominal_time_us', 'time_origin', 'adc_counts', 'sample_rate_hz', 'pulses', 'missed_slots'])
        for frame in frames:
            for i, sample in enumerate(frame['samples']):
                t = i * 1e6 / frame['sample_rate_hz'] - frame['nominal_burst_offset_ns'] / 1000
                writer.writerow([frame['sequence'], i, t, 'first_sample_trigger' if frame['baseline'] else 'burst', sample, frame['sample_rate_hz'], frame['pulses'], frame['missed_slots']])


def plot(frames):
    import matplotlib.pyplot as plt
    for frame in frames:
        t = [i * 1e6 / frame['sample_rate_hz'] - frame['nominal_burst_offset_ns'] / 1000 for i in range(frame['sample_count'])]
        plt.plot(t, frame['samples'], label=str(frame['sequence']))
    origin = 'first sample trigger' if all(frame['baseline'] for frame in frames) else 'burst'
    plt.xlabel(f'Nominal time from {origin} (us; aperture delay uncalibrated)')
    plt.ylabel('Envelope ADC counts (0..4095)')
    plt.legend(title='Sequence'); plt.tight_layout(); plt.show()


def acquire(args):
    link = Link(args.port)
    frames = []
    last_sequence = None
    try:
        link.ack(link.send(5))  # Explicitly stop a previous session before configuring.
        if args.recover:
            link.ack(link.send(6))
        info = link.ack(link.send(1))
        print(json.dumps({k: v for k, v in info.items() if k != 'samples'}, indent=2))
        if args.info:
            return
        link.ack(link.send(2, args.pulses | ((args.rate == 2400000) << 8)))
        # Exclusive create: never overwrite a previous measurement accidentally.
        with open(args.output, 'xb') as output:
            request = link.send((8 if args.baseline else 3) if args.frames == 1 else 4, 0 if args.frames == 1 else args.period)
            link.ack(request)
            while len(frames) < args.frames:
                packet, frame = link.next(time.monotonic() + max(3, args.period / 1000 + 2))
                if frame['kind'] == 4:
                    raise RuntimeError(f"Capture failed: {frame['status']}")
                if frame['kind'] != 3 or frame['request'] != request:
                    raise ValueError('Unexpected response during acquisition')
                if last_sequence is not None and frame['sequence'] != (last_sequence + 1) & 0xffffffff:
                    raise ValueError('Sequence gap; record is incomplete')
                last_sequence = frame['sequence']
                output.write(packet); output.flush(); frames.append(frame)
                print(f"Frame {frame['sequence']}: {frame['sample_count']} samples; missed slots={frame['missed_slots']}")
    finally:
        try:
            link.ack(link.send(5), timeout=2)
        finally:
            link.serial.close()
    if args.csv:
        save_csv(args.csv, frames)
    if args.plot:
        plot(frames)


def read_file(args):
    parser = Parser()
    records = parser.feed(pathlib.Path(args.input).read_bytes())
    if parser.buffer:
        raise ValueError('Capture file ends with a truncated frame')
    frames = [frame for _, frame in records if frame['kind'] == 3]
    if not frames:
        raise ValueError('No waveform records')
    print(f'{len(frames)} CRC-verified envelope records')
    if args.csv:
        save_csv(args.csv, frames)
    if args.plot:
        plot(frames)


def selftest():
    assert HEADER.size == 48
    samples = tuple((i * 17) & 4095 for i in range(2400))
    header = HEADER.pack(b'EFW1', 1, 3, 0, MAX_FRAME, 7, 11, 2400000, 2400, 5, 1, 1000000, 10083, 0, 0, 0)
    body = header + struct.pack('<2400H', *samples)
    packet = body + struct.pack('<I', zlib.crc32(body))
    for chunk in (1, 7, 64, 511, 4096, MAX_FRAME):
        parser = Parser(); records = []
        for i in range(0, len(packet), chunk):
            records += parser.feed(packet[i:i + chunk])
        assert len(records) == 1 and records[0][1]['samples'] == samples and not parser.buffer
    for boundary in range(len(packet) + 1):
        parser = Parser()
        records = parser.feed(packet[:boundary]) + parser.feed(packet[boundary:])
        assert len(records) == 1 and records[0][1]["samples"] == samples
    assert len(Parser().feed(packet * 2)) == 2
    for offset in (0, 4, 8, 24, 48, len(packet) - 1):
        damaged = bytearray(packet); damaged[offset] ^= 1
        try:
            decode(damaged)
        except ValueError:
            pass
        else:
            raise AssertionError('Corrupt packet accepted')
    assert zlib.crc32(command(3, 42)[:16]) == struct.unpack_from('<I', command(3, 42), 16)[0]
    print('PASS: protocol layout, fragmentation, coalescing, metadata, CRC corruption')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('selftest')
    sub.add_parser('ports')
    read = sub.add_parser('read'); read.add_argument('input')
    acquire_parser = sub.add_parser('capture')
    acquire_parser.add_argument('--port', required=True)
    acquire_parser.add_argument('--pulses', type=int, choices=range(1, 21), default=5)
    acquire_parser.add_argument('--rate', type=int, choices=(1200000, 2400000), default=1200000)
    acquire_parser.add_argument('--frames', type=int, default=1)
    acquire_parser.add_argument('--period', type=int, default=20, help='stream period in ms, 20..10000')
    acquire_parser.add_argument('--output', default='capture.efw')
    acquire_parser.add_argument('--baseline', action='store_true', help='ADC-only capture; keep IO1/IO2 inhibited')
    acquire_parser.add_argument('--info', action='store_true')
    acquire_parser.add_argument('--recover', action='store_true')
    for p in (read, acquire_parser):
        p.add_argument('--csv'); p.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    if args.action == 'selftest':
        selftest()
    elif args.action == 'ports':
        from serial.tools import list_ports
        for port in list_ports.comports():
            print(f'{port.device}: {port.description}; serial={port.serial_number}')
    elif args.action == 'read':
        read_file(args)
    else:
        if args.baseline and args.frames != 1:
            parser.error('--baseline requires --frames 1')
        if not 1 <= args.frames <= 10000 or not 20 <= args.period <= 10000:
            parser.error('frames must be 1..10000 and period must be 20..10000 ms')
        acquire(args)


if __name__ == '__main__':
    main()
