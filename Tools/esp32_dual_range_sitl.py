#!/usr/bin/env python3
"""Feed the ESP32 dual-range UART protocol to PX4 SITL through a pseudo-TTY.

The module can also be imported by higher-level ceiling/wall flight tests.  A
standalone run creates a PTY, starts ``uart_rx`` and ``CeilingReader`` through
the PX4 daemon socket, and streams fixed UP/FRONT values until interrupted.
"""

import argparse
import os
import pty
import socket
import threading
import time
from typing import Optional, Tuple


FRAME_SIZE = 9
INVALID_DISTANCE = 0xFFFF


def crc8_atm(data: bytes) -> int:
    """CRC-8/ATM: poly=0x07, init=0, refin=false, xorout=0."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_px4_frame(up_distance_mm: int, front_distance_mm: int) -> bytes:
    """Build ``AA 55 UP_H UP_L FWD_H FWD_L CRC 0D 0A``."""
    for name, value in (("up_distance_mm", up_distance_mm),
                        ("front_distance_mm", front_distance_mm)):
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"{name} must be in [0, 65535]")

    payload = bytes((
        0xAA,
        0x55,
        up_distance_mm >> 8,
        up_distance_mm & 0xFF,
        front_distance_mm >> 8,
        front_distance_mm & 0xFF,
    ))
    return payload + bytes((crc8_atm(payload), 0x0D, 0x0A))


def px4_command(command: str, socket_path: str = "/tmp/px4-sock-0",
                timeout: float = 5.0, check: bool = True) -> Tuple[int, str]:
    """Execute one PX4 shell command through the POSIX daemon socket."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout)
        client.connect(socket_path)
        client.sendall(command.encode("utf-8") + b"\0")  # final byte: isatty=false

        response = bytearray()
        while True:
            chunk = client.recv(4096)
            if not chunk:
                break
            response.extend(chunk)

    if len(response) < 2 or response[-2] != 0:
        raise RuntimeError(f"malformed PX4 response to {command!r}: {bytes(response)!r}")

    return_code = response[-1]
    output = bytes(response[:-2]).decode("utf-8", errors="replace").strip()
    if check and return_code != 0:
        raise RuntimeError(f"PX4 command failed ({return_code}): {command}\n{output}")
    return return_code, output


class DualRangePty:
    """Pseudo-serial endpoint and optional periodic dual-range publisher."""

    def __init__(self, socket_path: str = "/tmp/px4-sock-0") -> None:
        self.socket_path = socket_path
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_path = os.ttyname(self.slave_fd)
        self._lock = threading.Lock()
        self._up_mm = INVALID_DISTANCE
        self._front_mm = INVALID_DISTANCE
        self._period = 1.0 / 30.0
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start_px4_modules(self, start_wall_perch: bool = False,
                          start_ceiling_controller: bool = True) -> None:
        """Restart uart_rx on this PTY and ensure consumers are running."""
        px4_command("uart_rx stop", self.socket_path, check=False)
        time.sleep(0.1)
        _, output = px4_command(
            f"uart_rx start -d {self.slave_path} -b 115200", self.socket_path)
        if output:
            print(output)

        commands = ["CeilingReader start"]
        if start_ceiling_controller:
            commands.append("ceiling_controller start")

        for command in commands:
            _, output = px4_command(command, self.socket_path, check=False)
            if output:
                print(output)

        if start_wall_perch:
            _, output = px4_command("wall_perch start", self.socket_path, check=False)
            if output:
                print(output)

    def stop_px4_uart(self) -> None:
        px4_command("uart_rx stop", self.socket_path, check=False)

    def set_ranges(self, up_mm: int, front_mm: int) -> None:
        build_px4_frame(up_mm, front_mm)  # validate before updating shared state
        with self._lock:
            self._up_mm = up_mm
            self._front_mm = front_mm

    def write_frame(self, up_mm: int, front_mm: int, *, corrupt_crc: bool = False,
                    garbage: bytes = b"") -> None:
        frame = bytearray(build_px4_frame(up_mm, front_mm))
        if corrupt_crc:
            frame[6] ^= 0x01
        if garbage:
            os.write(self.master_fd, garbage)
        os.write(self.master_fd, frame)

    def start_stream(self, up_mm: int, front_mm: int, rate_hz: float = 30.0) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        if self._running:
            raise RuntimeError("stream already running")

        self.set_ranges(up_mm, front_mm)
        self._period = 1.0 / rate_hz
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    def _stream_loop(self) -> None:
        next_send = time.monotonic()
        while self._running:
            with self._lock:
                up_mm = self._up_mm
                front_mm = self._front_mm
            self.write_frame(up_mm, front_mm)
            next_send += self._period
            time.sleep(max(0.0, next_send - time.monotonic()))

    def stop_stream(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def close(self) -> None:
        self.stop_stream()
        for fd_name in ("master_fd", "slave_fd"):
            fd = getattr(self, fd_name, -1)
            if fd >= 0:
                os.close(fd)
                setattr(self, fd_name, -1)

    def __enter__(self) -> "DualRangePty":
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


def self_test() -> None:
    expected = bytes.fromhex("AA 55 00 64 00 C8 C5 0D 0A")
    actual = build_px4_frame(100, 200)
    if actual != expected or len(actual) != FRAME_SIZE:
        raise AssertionError(f"golden frame mismatch: {actual.hex(' ')}")
    print(f"CRC/frame self-test passed: {actual.hex(' ').upper()}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--up-mm", type=int, default=1000,
                        help="UP millimetres; use 0 or 65535 to invalidate only UP")
    parser.add_argument("--front-mm", type=int, default=2000,
                        help="FRONT millimetres; use 0 or 65535 to invalidate only FRONT")
    parser.add_argument("--rate", type=float, default=30.0, help="frame rate in Hz")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds to run; zero means until Ctrl-C")
    parser.add_argument("--socket", default="/tmp/px4-sock-0", help="PX4 daemon socket")
    parser.add_argument("--start-wall-perch", action="store_true")
    parser.add_argument("--corrupt-once", action="store_true",
                        help="insert one bad-CRC frame before the valid stream")
    parser.add_argument("--garbage-hex", default="",
                        help="insert hexadecimal garbage bytes before the valid stream")
    parser.add_argument("--self-test", action="store_true", help="verify only the golden frame")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    self_test()
    if args.self_test:
        return

    garbage = bytes.fromhex(args.garbage_hex)
    with DualRangePty(args.socket) as link:
        print(f"Pseudo UART: {link.slave_path}")
        link.start_px4_modules(start_wall_perch=args.start_wall_perch)
        if garbage or args.corrupt_once:
            link.write_frame(args.up_mm, args.front_mm,
                             corrupt_crc=args.corrupt_once, garbage=garbage)
        link.start_stream(args.up_mm, args.front_mm, args.rate)
        print(f"Streaming UP={args.up_mm} mm FRONT={args.front_mm} mm at {args.rate:g} Hz")

        try:
            if args.duration > 0:
                time.sleep(args.duration)
            else:
                while True:
                    time.sleep(1.0)
        except KeyboardInterrupt:
            print("Stopping")
        finally:
            link.stop_px4_uart()


if __name__ == "__main__":
    main()
