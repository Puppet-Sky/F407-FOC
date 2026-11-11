#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AS5600 realtime plotter — host parses raw I2C register bytes.

Protocol (15 bytes, little-endian):
  AA 55
  seq:u32
  status          # 0x0B
  raw_h, raw_l    # 0x0C..0x0D  (same I2C burst with status/angle)
  ang_h, ang_l    # 0x0E..0x0F
  agc             # 0x1A
  mag_h, mag_l    # 0x1B..0x1C
  chk = XOR(bytes[2:14])

Usage:
  python tools/as5600_plot.py -p COM5 -b 460800
  python tools/as5600_plot.py -p COM5 --jump 20 --log jump.csv
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing pyserial. Run: pip install pyserial")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("Missing matplotlib. Run: pip install matplotlib")
    sys.exit(1)

MAGIC = b"\xAA\x55"
# magic(2) + seq(4) + 8 i2c bytes + chk(1) = 15
FRAME_FMT = "<2sI8sB"
FRAME_SIZE = struct.calcsize(FRAME_FMT)
assert FRAME_SIZE == 15


def checksum(payload: bytes) -> int:
    chk = 0
    for b in payload:
        chk ^= b
    return chk & 0xFF


def u12(hi: int, lo: int) -> int:
    return ((hi & 0x0F) << 8) | lo


def to_deg(counts: int) -> float:
    return counts * 360.0 / 4096.0


def shortest_delta_deg(a: float, b: float) -> float:
    d = b - a
    if d > 180.0:
        d -= 360.0
    elif d < -180.0:
        d += 360.0
    return d


@dataclass
class Sample:
    t: float
    seq: int
    status: int
    raw_h: int
    raw_l: int
    ang_h: int
    ang_l: int
    agc: int
    mag_h: int
    mag_l: int
    raw: int
    angle: int
    mag: int
    raw_deg: float
    ang_deg: float


def parse_i2c_bytes(i2c8: bytes) -> tuple[int, int, int, int, int, int, int, int]:
    status, raw_h, raw_l, ang_h, ang_l, agc, mag_h, mag_l = i2c8
    return status, raw_h, raw_l, ang_h, ang_l, agc, mag_h, mag_l


class As5600Reader:
    def __init__(
        self,
        port: str,
        baud: int,
        period_us: int,
        maxlen: int,
        jump_deg: float,
        log_path: str | None,
    ):
        self.port = port
        self.baud = baud
        self.period_us = period_us
        self.jump_deg = jump_deg
        self.log_path = log_path
        self.ser: serial.Serial | None = None
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.t0 = time.perf_counter()
        self.times: deque[float] = deque(maxlen=maxlen)
        self.raw_degs: deque[float] = deque(maxlen=maxlen)
        self.ang_degs: deque[float] = deque(maxlen=maxlen)
        self.agcs: deque[float] = deque(maxlen=maxlen)
        self.seq_last = None
        self.prev: Sample | None = None
        self.frames = 0
        self.drop = 0
        self.bad_chk = 0
        self.jumps = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._log_fp = None
        self._log_writer = None

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        cmd = f"as5600 start {self.period_us}\r\n".encode("ascii")
        self.ser.write(cmd)
        self.ser.flush()
        print(f"[TX] {cmd!r}")
        print(
            "Host parses I2C bytes: STATUS RAW_H RAW_L ANG_H ANG_L AGC MAG_H MAG_L"
        )
        if self.log_path:
            self._log_fp = open(self.log_path, "w", newline="", encoding="utf-8")
            self._log_writer = csv.writer(self._log_fp)
            self._log_writer.writerow(
                [
                    "t",
                    "seq",
                    "status",
                    "raw_h",
                    "raw_l",
                    "ang_h",
                    "ang_l",
                    "agc",
                    "mag_h",
                    "mag_l",
                    "raw",
                    "angle",
                    "raw_deg",
                    "ang_deg",
                    "d_raw_deg",
                    "note",
                ]
            )
        self.t0 = time.perf_counter()

    def close(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"as5600 stop\r\n")
                self.ser.flush()
                time.sleep(0.05)
            except Exception:
                pass
            self.ser.close()
        if self._log_fp:
            self._log_fp.close()
            self._log_fp = None

    def start_thread(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _report_jump(self, prev: Sample, cur: Sample, d_raw: float) -> None:
        self.jumps += 1
        print(
            f"[JUMP #{self.jumps}] seq {prev.seq}->{cur.seq}  "
            f"RAW {prev.raw_deg:.2f}->{cur.raw_deg:.2f} (d={d_raw:+.2f})  "
            f"ANG {prev.ang_deg:.2f}->{cur.ang_deg:.2f}"
        )
        print(
            f"  prev i2c: STAT={prev.status:02X} RAW={prev.raw_h:02X}{prev.raw_l:02X} "
            f"ANG={prev.ang_h:02X}{prev.ang_l:02X} AGC={prev.agc:02X} "
            f"MAG={prev.mag_h:02X}{prev.mag_l:02X}  "
            f"MD={(prev.status>>5)&1} ML={(prev.status>>4)&1} MH={(prev.status>>3)&1}"
        )
        print(
            f"  curr i2c: STAT={cur.status:02X} RAW={cur.raw_h:02X}{cur.raw_l:02X} "
            f"ANG={cur.ang_h:02X}{cur.ang_l:02X} AGC={cur.agc:02X} "
            f"MAG={cur.mag_h:02X}{cur.mag_l:02X}  "
            f"MD={(cur.status>>5)&1} ML={(cur.status>>4)&1} MH={(cur.status>>3)&1}"
        )
        # Bit-level hint
        if (prev.raw_h & 0x0F) != (cur.raw_h & 0x0F) and abs(d_raw) > self.jump_deg:
            print(
                f"  hint: RAW high-nibble {prev.raw_h&0x0F}->{cur.raw_h&0x0F} "
                f"(often bus tear / magnet / AGC)"
            )
        if ((cur.status >> 4) & 1) or ((cur.status >> 3) & 1):
            print("  hint: ML/MH set — magnet too far/near at this position")
        if abs(cur.raw_deg - cur.ang_deg) > 5.0:
            print(
                f"  hint: RAW vs ANGLE differ by {cur.raw_deg - cur.ang_deg:+.1f} deg "
                "(ZPOS/MPOS scaling?)"
            )

    def _feed(self, data: bytes) -> None:
        self.buf.extend(data)
        while True:
            idx = self.buf.find(MAGIC)
            if idx < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < FRAME_SIZE:
                return

            frame = bytes(self.buf[:FRAME_SIZE])
            del self.buf[:FRAME_SIZE]
            magic, seq, i2c8, chk = struct.unpack(FRAME_FMT, frame)
            if magic != MAGIC:
                continue
            if checksum(frame[2:14]) != chk:
                self.bad_chk += 1
                continue

            if self.seq_last is not None:
                gap = (seq - self.seq_last) & 0xFFFFFFFF
                if gap > 1:
                    self.drop += gap - 1
            self.seq_last = seq

            status, raw_h, raw_l, ang_h, ang_l, agc, mag_h, mag_l = parse_i2c_bytes(i2c8)
            raw = u12(raw_h, raw_l)
            angle = u12(ang_h, ang_l)
            mag = u12(mag_h, mag_l)
            raw_deg = to_deg(raw)
            ang_deg = to_deg(angle)
            t = time.perf_counter() - self.t0

            cur = Sample(
                t=t,
                seq=seq,
                status=status,
                raw_h=raw_h,
                raw_l=raw_l,
                ang_h=ang_h,
                ang_l=ang_l,
                agc=agc,
                mag_h=mag_h,
                mag_l=mag_l,
                raw=raw,
                angle=angle,
                mag=mag,
                raw_deg=raw_deg,
                ang_deg=ang_deg,
            )

            note = ""
            d_raw = 0.0
            if self.prev is not None:
                d_raw = shortest_delta_deg(self.prev.raw_deg, cur.raw_deg)
                if abs(d_raw) >= self.jump_deg:
                    note = "JUMP"
                    self._report_jump(self.prev, cur, d_raw)

            if self._log_writer is not None:
                self._log_writer.writerow(
                    [
                        f"{t:.6f}",
                        seq,
                        f"{status:02X}",
                        f"{raw_h:02X}",
                        f"{raw_l:02X}",
                        f"{ang_h:02X}",
                        f"{ang_l:02X}",
                        f"{agc:02X}",
                        f"{mag_h:02X}",
                        f"{mag_l:02X}",
                        raw,
                        angle,
                        f"{raw_deg:.3f}",
                        f"{ang_deg:.3f}",
                        f"{d_raw:.3f}",
                        note,
                    ]
                )

            with self.lock:
                self.times.append(t)
                self.raw_degs.append(raw_deg)
                self.ang_degs.append(ang_deg)
                self.agcs.append(float(agc))
                self.frames += 1

            self.prev = cur

    def _loop(self) -> None:
        assert self.ser is not None
        while not self._stop.is_set():
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
                if data:
                    self._feed(data)
            except Exception as exc:
                print(f"[ERR] serial read: {exc}")
                break

    def snapshot(self):
        with self.lock:
            return (
                list(self.times),
                list(self.raw_degs),
                list(self.ang_degs),
                list(self.agcs),
                self.frames,
                self.drop,
                self.bad_chk,
                self.jumps,
            )


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="AS5600 plotter (host parses raw I2C bytes)"
    )
    parser.add_argument("-p", "--port", help="serial port, e.g. COM5")
    parser.add_argument("-b", "--baud", type=int, default=460800, help="baudrate")
    parser.add_argument("--period", type=int, default=500, help="sample period us")
    parser.add_argument("--window", type=float, default=5.0, help="plot window seconds")
    parser.add_argument("--list", action="store_true", help="list serial ports")
    parser.add_argument(
        "--jump",
        type=float,
        default=20.0,
        help="print diagnostic when |dRAW| >= this (deg), still plot all",
    )
    parser.add_argument(
        "--log",
        default=None,
        help="csv log path for all samples / jump marks",
    )
    parser.add_argument(
        "--agc",
        action="store_true",
        help="also plot AGC on secondary axis",
    )
    args = parser.parse_args()

    if args.list or not args.port:
        list_serial_ports()
        if not args.port:
            print("\nExample: python tools/as5600_plot.py -p COM5 -b 460800 --jump 20")
            return 0 if args.list else 1

    maxlen = max(1000, int(args.window * (1_000_000 / max(args.period, 1)) * 1.2))
    reader = As5600Reader(
        args.port,
        args.baud,
        args.period,
        maxlen,
        jump_deg=args.jump,
        log_path=args.log,
    )

    try:
        reader.open()
    except serial.SerialException as exc:
        print(f"Open {args.port} failed: {exc}")
        return 1

    reader.start_thread()

    fig, ax = plt.subplots(figsize=(11, 5))
    (line_raw,) = ax.plot([], [], lw=1.2, label="RAW (host parse)")
    (line_ang,) = ax.plot([], [], lw=1.0, alpha=0.75, label="ANGLE (host parse)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("angle (deg, 0~360)")
    ax.set_ylim(-5, 365)
    ax.set_xlim(0, args.window)
    ax.grid(True, alpha=0.3)
    title = ax.set_title("AS5600 raw I2C debug")

    ax2 = None
    line_agc = None
    if args.agc:
        ax2 = ax.twinx()
        (line_agc,) = ax2.plot([], [], lw=0.8, alpha=0.5, color="tab:green", label="AGC")
        ax2.set_ylabel("AGC")
        ax2.set_ylim(0, 255)

    lines = [line_raw, line_ang]
    labels = [ln.get_label() for ln in lines]
    if line_agc is not None:
        lines.append(line_agc)
        labels.append(line_agc.get_label())
    ax.legend(lines, labels, loc="upper right")

    def update(_frame_idx):
        ts, raws, angs, agcs, frames, drop, bad, jumps = reader.snapshot()
        if not ts:
            return (line_raw,)

        t_end = ts[-1]
        t_min = max(0.0, t_end - args.window)
        i0 = 0
        for i, t in enumerate(ts):
            if t >= t_min:
                i0 = i
                break
        ts_v = ts[i0:]
        line_raw.set_data(ts_v, raws[i0:])
        line_ang.set_data(ts_v, angs[i0:])
        artists = [line_raw, line_ang]
        if line_agc is not None:
            line_agc.set_data(ts_v, agcs[i0:])
            artists.append(line_agc)

        ax.set_xlim(t_min, max(t_min + args.window, t_end + 0.05))
        rate = frames / max(ts[-1], 1e-3)
        title.set_text(
            f"AS5600 raw-I2C  frames={frames} ~{rate:.0f}Hz  "
            f"drop={drop} bad={bad} jumps={jumps}"
        )
        return tuple(artists)

    anim = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    print("Plotting... jumps print I2C hex. Close window / Ctrl+C to stop.")
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        _ = anim
    return 0


if __name__ == "__main__":
    sys.exit(main())
