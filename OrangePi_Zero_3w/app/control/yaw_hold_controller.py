class YawHoldController:
    """BNO085相对航向PD保持。每次ARM由主程序重设yaw_ref。"""
    def __init__(self, cfg):
        self.cfg = cfg

    def reset(self):
        pass

    def update(self, yaw, yaw_ref, yaw_rate, fresh):
        if not self.cfg.get('yaw_hold_enabled', True) or not fresh:
            return 0.0
        error = ((yaw_ref - yaw + 180.0) % 360.0) - 180.0
        if abs(error) <= float(self.cfg.get('yaw_deadband_deg', 1.5)):
            return 0.0
        raw = float(self.cfg.get('yaw_sign', 1.0)) * (
            float(self.cfg.get('yaw_kp_radps_per_deg', 0.025)) * error
            - float(self.cfg.get('yaw_kd_radps_per_dps', 0.006)) * yaw_rate
        )
        limit = float(self.cfg.get('yaw_max_wz_radps', 0.45))
        return max(-limit, min(limit, raw))
