# -*- coding: utf-8 -*-
import math

class LidarCollisionGuard:
    def __init__(self, config): self.cfg=config; self.latched=False; self._hit=0; self._clear=0; self.nearest_m=None
    @staticmethod
    def _signed_angle(a): return ((float(a)+180.0)%360.0)-180.0
    def evaluate(self,snapshot):
        if not self.cfg.collision_stop_enabled:
            self.latched=False; self._hit=self._clear=0; self.nearest_m=None; return False
        if not snapshot.online:
            self.nearest_m=None
            return bool(self.cfg.stop_on_sensor_loss)
        candidates=[p.distance_m for p in snapshot.points if p.distance_m>0 and p.confidence>=self.cfg.min_confidence and abs(self._signed_angle(p.angle_deg))<=self.cfg.front_half_angle_deg]
        self.nearest_m=min(candidates) if candidates else None
        hit=self.nearest_m is not None and self.nearest_m<=self.cfg.stop_distance_m
        clear=self.nearest_m is None or self.nearest_m>=self.cfg.release_distance_m
        self._hit=self._hit+1 if hit else 0; self._clear=self._clear+1 if clear else 0
        if self._hit>=max(1,self.cfg.confirm_frames): self.latched=True
        if self._clear>=max(1,self.cfg.release_frames): self.latched=False
        return self.latched
