# -*- coding: utf-8 -*-
"""LD14 阳光近距离噪点过滤器。

处理链：
1. 距离和置信度基础门限
2. 近距离点的角度邻域一致性
3. 近距离点的多圈稳定性确认

注意：参数均为建议初值，必须在开帘、关帘和真实近障碍物三种场景下实测。
过滤只发生在PC端，不改变ESP32的原始47字节帧转发。
"""
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Dict, Iterable, List, Tuple

from .ld14_protocol import LidarPoint


@dataclass
class SunNoiseFilterConfig:
    enabled: bool = True
    # 小于该距离的点采用更严格过滤。建议初值，待实测。
    near_range_m: float = 0.60
    # LD14极近距离中的无效区。不要设得过大，避免漏掉真实近障碍。
    absolute_min_distance_m: float = 0.08
    max_distance_m: float = 12.0
    # 普通点和近距离点的最低置信度。建议初值，待实测。
    min_confidence_normal: int = 5
    min_confidence_near: int = 12
    # 角度分箱和空间连续性。
    angle_bin_deg: float = 1.0
    neighbor_radius_bins: int = 2
    min_neighbor_count: int = 3
    neighbor_distance_tolerance_m: float = 0.10
    # 多圈确认。3/5为建议初值。
    history_scans: int = 5
    required_hits: int = 3
    temporal_distance_tolerance_m: float = 0.10
    # 强近障碍物快速通道：置信度高且空间连续时无需等待3圈。
    immediate_confidence: int = 80


class SunNoiseFilter:
    """有状态过滤器。每次process_scan()应输入当前已组装的一圈/快照点。"""

    def __init__(self, config: SunNoiseFilterConfig = None):
        self.cfg = config or SunNoiseFilterConfig()
        self._history: Dict[int, deque] = {}
        self.last_raw_count = 0
        self.last_valid_count = 0
        self.last_rejected_count = 0

    def reset(self):
        self._history.clear()
        self.last_raw_count = 0
        self.last_valid_count = 0
        self.last_rejected_count = 0

    def _bin(self, angle_deg: float) -> int:
        bins = max(1, int(round(360.0 / self.cfg.angle_bin_deg)))
        return int(round((angle_deg % 360.0) / self.cfg.angle_bin_deg)) % bins

    def _base_valid(self, p: LidarPoint) -> bool:
        return (
            self.cfg.absolute_min_distance_m <= p.distance_m <= self.cfg.max_distance_m
            and p.confidence >= self.cfg.min_confidence_normal
        )

    def _spatial_ok(self, p: LidarPoint, by_bin: Dict[int, LidarPoint]) -> Tuple[bool, int]:
        center = self._bin(p.angle_deg)
        bins = max(1, int(round(360.0 / self.cfg.angle_bin_deg)))
        distances = []
        for offset in range(-self.cfg.neighbor_radius_bins, self.cfg.neighbor_radius_bins + 1):
            q = by_bin.get((center + offset) % bins)
            if q is not None and self._base_valid(q):
                if abs(q.distance_m - p.distance_m) <= self.cfg.neighbor_distance_tolerance_m:
                    distances.append(q.distance_m)
        if len(distances) < self.cfg.min_neighbor_count:
            return False, len(distances)
        spread = max(distances) - min(distances)
        return spread <= 2.0 * self.cfg.neighbor_distance_tolerance_m, len(distances)

    def _update_history(self, by_bin: Dict[int, LidarPoint]):
        bins = max(1, int(round(360.0 / self.cfg.angle_bin_deg)))
        for b in range(bins):
            hist = self._history.setdefault(b, deque(maxlen=self.cfg.history_scans))
            p = by_bin.get(b)
            if p is None or not self._base_valid(p):
                hist.append(None)
            else:
                hist.append(float(p.distance_m))

    def _temporal_ok(self, p: LidarPoint) -> bool:
        hist = self._history.get(self._bin(p.angle_deg), ())
        values = [v for v in hist if v is not None]
        if len(values) < self.cfg.required_hits:
            return False
        center = median(values)
        consistent = sum(abs(v - center) <= self.cfg.temporal_distance_tolerance_m for v in values)
        return consistent >= self.cfg.required_hits and abs(p.distance_m - center) <= self.cfg.temporal_distance_tolerance_m

    def process_scan(self, points: Iterable[LidarPoint]) -> Tuple[LidarPoint, ...]:
        pts = tuple(points)
        self.last_raw_count = len(pts)
        if not self.cfg.enabled:
            self.last_valid_count = len(pts)
            self.last_rejected_count = 0
            return pts

        # 同一角度分箱只保留最近点，便于邻域判断。
        by_bin: Dict[int, LidarPoint] = {}
        for p in pts:
            if self._base_valid(p):
                b = self._bin(p.angle_deg)
                old = by_bin.get(b)
                if old is None or p.distance_m < old.distance_m:
                    by_bin[b] = p

        self._update_history(by_bin)
        out: List[LidarPoint] = []
        for p in by_bin.values():
            # 普通距离只做基础门限，避免不必要地损失远处细目标。
            if p.distance_m > self.cfg.near_range_m:
                out.append(p)
                continue

            if p.confidence < self.cfg.min_confidence_near:
                continue

            spatial_ok, neighbor_count = self._spatial_ok(p, by_bin)
            temporal_ok = self._temporal_ok(p)
            immediate_ok = p.confidence >= self.cfg.immediate_confidence and spatial_ok

            if (spatial_ok and temporal_ok) or immediate_ok:
                out.append(p)

        out.sort(key=lambda p: p.angle_deg)
        self.last_valid_count = len(out)
        self.last_rejected_count = self.last_raw_count - self.last_valid_count
        return tuple(out)
