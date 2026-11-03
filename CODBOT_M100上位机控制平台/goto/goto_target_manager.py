# -*- coding: utf-8 -*-
from dataclasses import dataclass
import math, time
@dataclass
class TargetUpdate:
    accepted: bool; x: float=None; y: float=None; reason: str=''; committed: bool=False
class GotoTargetManager:
    """连续落点目标门控，不负责速度控制。参数均为建议初值，接入LandingNet前独立验证。"""
    def __init__(self,field_w,field_h,min_confidence=0.5,max_jump_m=0.35,min_update_m=0.01,timeout_s=0.30):
        self.field_w=float(field_w); self.field_h=float(field_h); self.min_confidence=float(min_confidence); self.max_jump_m=float(max_jump_m); self.min_update_m=float(min_update_m); self.timeout_s=float(timeout_s); self.target=None; self.last_t=0.0; self.committed=False
    def update(self,x,y,confidence=1.0,now=None):
        now=time.monotonic() if now is None else float(now); x=float(x); y=float(y)
        if self.committed: return TargetUpdate(False,reason='committed',committed=True)
        if confidence<self.min_confidence: return TargetUpdate(False,reason='low_confidence')
        if not (0<=x<=self.field_w and 0<=y<=self.field_h): return TargetUpdate(False,reason='outside_field')
        if self.target:
            d=math.hypot(x-self.target[0],y-self.target[1])
            if d>self.max_jump_m: return TargetUpdate(False,reason='jump')
            if d<self.min_update_m: return TargetUpdate(False,reason='deadband')
        self.target=(x,y); self.last_t=now; return TargetUpdate(True,x,y,'accepted')
    def commit(self): self.committed=True
    def stale(self,now=None): return not self.last_t or (time.monotonic() if now is None else float(now))-self.last_t>self.timeout_s
