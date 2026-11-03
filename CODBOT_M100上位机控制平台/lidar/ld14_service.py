# -*- coding: utf-8 -*-
import socket
import threading
import time
from dataclasses import dataclass
from typing import Tuple

from .ld14_protocol import FrameStreamParser, LidarPoint
from .ld14_sun_filter import SunNoiseFilter, SunNoiseFilterConfig


@dataclass(frozen=True)
class LidarSnapshot:
    online: bool = False
    points: Tuple[LidarPoint, ...] = ()
    speed_dps: int = 0
    good_frames: int = 0
    bad_packets: int = 0
    last_frame_age_s: float = 999.0
    raw_point_count: int = 0
    filtered_point_count: int = 0


class LidarService:
    """UDP 4212接收服务，兼容原公开API，并在PC端过滤阳光近距噪点。"""

    def __init__(self, port=4212, offline_timeout_s=0.5, sun_filter_config=None):
        self.port = int(port)
        self.offline_timeout_s = float(offline_timeout_s)
        self._lock = threading.Lock()
        self._raw_points = {}
        self._points = {}
        self._speed = 0
        self._good = 0
        self._bad = 0
        self._last = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._sock = None
        cfg = sun_filter_config or SunNoiseFilterConfig()
        self.sun_filter = SunNoiseFilter(cfg)

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="LD14-UDP", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def _run(self):
        parser = FrameStreamParser()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("", self.port))
            sock.settimeout(0.2)
            self._sock = sock
        except OSError:
            with self._lock:
                self._bad += 1
            sock.close()
            return

        while not self._stop.is_set():
            try:
                packet, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            if packet.startswith(b"LD14:"):
                packet = packet[5:]
            frames = parser.feed(packet)
            if not frames:
                with self._lock:
                    self._bad += 1
                continue

            with self._lock:
                for frame in frames:
                    self._speed = frame.speed_dps
                    self._good += 1
                    self._last = time.monotonic()
                    for point in frame.points:
                        self._raw_points[int(round(point.angle_deg)) % 360] = point

                    # 对当前360度快照做空间和时间联合过滤。
                    filtered = self.sun_filter.process_scan(self._raw_points.values())
                    self._points = {int(round(p.angle_deg)) % 360: p for p in filtered}

        try:
            sock.close()
        except OSError:
            pass

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            age = now - self._last if self._last else 999.0
            return LidarSnapshot(
                online=age <= self.offline_timeout_s,
                points=tuple(self._points.values()),
                speed_dps=self._speed,
                good_frames=self._good,
                bad_packets=self._bad,
                last_frame_age_s=age,
                raw_point_count=len(self._raw_points),
                filtered_point_count=len(self._points),
            )

    def raw_snapshot(self):
        """调试接口：返回未过滤点，便于比较开帘、关帘和真实障碍。"""
        now = time.monotonic()
        with self._lock:
            age = now - self._last if self._last else 999.0
            return LidarSnapshot(
                online=age <= self.offline_timeout_s,
                points=tuple(self._raw_points.values()),
                speed_dps=self._speed,
                good_frames=self._good,
                bad_packets=self._bad,
                last_frame_age_s=age,
                raw_point_count=len(self._raw_points),
                filtered_point_count=len(self._points),
            )
