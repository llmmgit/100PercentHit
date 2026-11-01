import time

class BallTracker:
    """统一观测；短时漏检保留最后结果，超时后才失效。"""
    def __init__(self, cfg): self.cfg=cfg; self.reset()
    def reset(self):
        self.confirmed_frames=0; self.last_seen=0.0
        self.filtered_x_error=0.0; self.filtered_y_norm=0.0; self.filtered_size_norm=0.0
        self.valid=False; self.detected=False
    def update(self, ball, frame_width, frame_height, now=None):
        now=time.monotonic() if now is None else now
        timeout=max(0.0,float(self.cfg.get('lost_timeout_ms',180)))
        if ball is None or frame_width<=0 or frame_height<=0:
            self.detected=False
            age=(now-self.last_seen)*1000 if self.last_seen else 1e9
            if age>timeout: self.confirmed_frames=0; self.valid=False
            return self.snapshot(now)
        self.detected=True
        xe=(float(ball['cx'])-frame_width/2.0)/(frame_width/2.0)
        yn=float(ball['cy'])/frame_height
        sn=max(float(ball['w'])/frame_width,float(ball['h'])/frame_height)
        a=max(.01,min(1.0,float(self.cfg.get('error_filter_alpha',.35))))
        if self.confirmed_frames==0:
            self.filtered_x_error=xe; self.filtered_y_norm=yn; self.filtered_size_norm=sn
        else:
            self.filtered_x_error+=a*(xe-self.filtered_x_error)
            self.filtered_y_norm+=a*(yn-self.filtered_y_norm)
            self.filtered_size_norm+=a*(sn-self.filtered_size_norm)
        self.confirmed_frames+=1; self.last_seen=now
        self.valid=self.confirmed_frames>=int(self.cfg.get('acquire_confirm_frames',3))
        return self.snapshot(now)
    def snapshot(self, now=None):
        now=time.monotonic() if now is None else now
        age=(now-self.last_seen)*1000 if self.last_seen else 1e9
        return {'valid':self.valid,'detected':self.detected,'confirmed_frames':self.confirmed_frames,
                'last_seen':self.last_seen,'age_ms':age,'x_error':self.filtered_x_error,
                'y_norm':self.filtered_y_norm,'size_norm':self.filtered_size_norm}
