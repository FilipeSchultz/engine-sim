"""UDP acquisition of the engine-sim indicating stream (docs/indicom_protocol.md).

Runs in its own thread: receives the raw encoder angle + cylinder pressures,
cuts one 720 deg cycle per cylinder around the TDC angle set by the operator
and analyses it. Results are handed to the UI through a queue.
"""

import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from analysis import Geometry, Settings, analyze_cycle

HEADER = struct.Struct("<4sBBHIHH")
MAGIC = b"ESIN"
TYPE_DATA = 1
TYPE_INFO = 2

BUFFER_SAMPLES = 120_000     # several cycles even at low speed and high sample rate


@dataclass
class EngineInfo:
    name: str = ""
    cylinders: int = 0
    bore_mm: float = 0.0
    stroke_mm: float = 0.0
    rod_mm: float = 0.0
    compression_ratio: float = 0.0
    frequency_hz: float = 0.0
    tdc_deg: list = field(default_factory=list)


@dataclass
class Status:
    packets_per_s: float = 0.0
    lost_packets: int = 0
    rpm: float = 0.0
    deg_per_sample: float = 0.0
    receiving: bool = False


class Acquisition(threading.Thread):
    def __init__(self, port: int):
        super().__init__(daemon=True)
        self.port = port
        self.results = queue.Queue(maxsize=5000)

        self._lock = threading.Lock()
        self._geometry = Geometry()
        self._settings = Settings()
        self._tdc = []                  # firing TDC encoder angle per cylinder [deg]
        self._active = []               # cylinders selected for analysis

        self.info = EngineInfo()
        self.status = Status()
        self._running = True
        self._reset_buffers(0)

        # Bound here (caller's thread) so a busy port raises OSError to the UI
        # instead of silently killing the acquisition thread
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        try:
            self._socket.bind(("0.0.0.0", self.port))
        except OSError:
            self._socket.close()
            raise
        self._socket.settimeout(0.2)

    # Configuration (called from the UI thread)

    def configure(self, geometry: Geometry, settings: Settings, tdc: list, active: list):
        with self._lock:
            self._geometry = geometry
            self._settings = settings
            self._active = list(active)
            if list(tdc) != self._tdc:
                self._tdc = list(tdc)
                self._next_center = [None] * len(self._tdc)

    def stop(self):
        self._running = False

    # Thread

    def run(self):
        sock = self._socket
        last_sequence = None
        packets = 0
        window_start = time.monotonic()

        while self._running:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                self.status.receiving = False
                continue

            now = time.monotonic()
            if now - window_start >= 1.0:
                self.status.packets_per_s = packets / (now - window_start)
                packets = 0
                window_start = now

            if len(data) < HEADER.size:
                continue
            magic, version, kind, cylinders, sequence, count, _ = HEADER.unpack_from(data)
            if magic != MAGIC or version != 1:
                continue

            packets += 1
            self.status.receiving = True
            if last_sequence is not None and sequence != (last_sequence + 1) & 0xFFFFFFFF:
                if sequence < last_sequence:
                    self._reset_buffers(cylinders)   # simulator restarted
                else:
                    self.status.lost_packets += sequence - last_sequence - 1
            last_sequence = sequence

            payload = memoryview(data)[HEADER.size:]
            if kind == TYPE_INFO:
                self._parse_info(payload, cylinders)
            elif kind == TYPE_DATA:
                self._on_data(payload, cylinders, count)

        sock.close()

    # Packets

    def _parse_info(self, payload, cylinders):
        values = struct.unpack_from(f"<{5 + cylinders}f", payload)
        offset = 4 * (5 + cylinders)
        name_length = payload[offset]
        name = bytes(payload[offset + 1: offset + 1 + name_length]).decode("utf-8", "replace")
        self.info = EngineInfo(
            name=name, cylinders=cylinders,
            bore_mm=values[0], stroke_mm=values[1], rod_mm=values[2],
            compression_ratio=values[3], frequency_hz=values[4],
            tdc_deg=list(values[5:]))

    def _reset_buffers(self, cylinders):
        # Preallocated buffer (2x the history): packets are written in place and
        # the history is shifted back to the start only when the end is reached
        self._cylinders = cylinders
        self._capacity = 2 * BUFFER_SAMPLES
        self._buffer_angle = np.empty(self._capacity)
        self._buffer_pressure = np.empty((self._capacity, cylinders), dtype=np.float32)
        self._length = 0
        self._last_raw = None
        self._turns = 0.0
        self._next_center = [None] * len(getattr(self, "_tdc", []))

    def _on_data(self, payload, cylinders, count):
        if cylinders != self._cylinders:
            self._reset_buffers(cylinders)

        records = np.frombuffer(payload, dtype="<f4", count=count * (1 + cylinders)).reshape(count, 1 + cylinders)
        raw = records[:, 0].astype(np.float64)

        # Unwrap the 0..720 encoder into a continuous angle
        previous = raw[0] if self._last_raw is None else self._last_raw
        steps = np.diff(np.concatenate([[previous], raw]))
        wraps = np.cumsum(steps < -360.0) * 720.0
        angle = raw + self._turns + wraps
        self._turns += wraps[-1]
        self._last_raw = raw[-1]

        if len(steps) > 1:
            step = float(np.median(np.mod(steps[1:], 720.0)))
            self.status.deg_per_sample = step
            if self.info.frequency_hz > 0:
                self.status.rpm = step * self.info.frequency_hz / 6.0

        if self._length + count > self._capacity:
            keep = BUFFER_SAMPLES
            self._buffer_angle[:keep] = self._buffer_angle[self._length - keep:self._length]
            self._buffer_pressure[:keep] = self._buffer_pressure[self._length - keep:self._length]
            self._length = keep
        self._buffer_angle[self._length:self._length + count] = angle
        self._buffer_pressure[self._length:self._length + count] = records[:, 1:]
        self._length += count

        self._cut_cycles()

    @property
    def _angle(self):
        return self._buffer_angle[:self._length]

    @property
    def _pressure(self):
        return self._buffer_pressure[:self._length]

    def _cut_cycles(self):
        with self._lock:
            tdc = list(self._tdc)
            active = list(self._active)
            geometry = self._geometry
            settings = self._settings
            if len(self._next_center) != len(tdc):
                self._next_center = [None] * len(tdc)
            next_center = self._next_center

        if len(self._angle) < 2 or len(tdc) != self._cylinders:
            return

        first, last = self._angle[0], self._angle[-1]
        for i, tdc_angle in enumerate(tdc):
            center = next_center[i]
            if center is None:
                # First full cycle inside the buffer
                k = np.ceil((first + 360.0 - tdc_angle) / 720.0)
                center = tdc_angle + 720.0 * k

            selected = i < len(active) and active[i]
            while last >= center + 360.0:
                if not selected:
                    center += 720.0
                    continue

                lo = np.searchsorted(self._angle, center - 360.0)
                hi = np.searchsorted(self._angle, center + 360.0)
                theta = self._angle[lo:hi] - center
                # Skip cycles with gaps (lost packets) or incomplete coverage
                if hi - lo > 50 and theta[0] < -355.0 and theta[-1] > 355.0 and np.max(np.diff(theta)) < 10.0:
                    try:
                        result, curves = analyze_cycle(i + 1, theta, self._pressure[lo:hi, i], geometry, settings)
                        self.results.put_nowait((result, curves))
                    except (queue.Full, ValueError, FloatingPointError):
                        pass
                center += 720.0

            next_center[i] = center
