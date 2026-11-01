class OACController:
    """预测式Y轴OAC：接球线位置反馈 + 视线速度前馈 + 短时预测。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.previous_y = None
        self.filtered_y_rate = 0.0
        self.previous_vx = 0.0
        self.last_detected_y = None
        self.last_detected_rate = 0.0

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def ramp(self, target, dt):
        accel = max(0.01, float(self.cfg.get('max_accel_vx_per_s', 20.0)))
        step = accel * dt
        self.previous_vx += self.clamp(target - self.previous_vx, -step, step)
        return self.previous_vx

    def _command_from_observation(self, y, dt, max_vx):
        # 第一帧也用位置误差立即产生动作，不再固定输出0。
        if self.previous_y is None:
            raw_rate = 0.0
            self.filtered_y_rate = 0.0
        else:
            raw_rate = (y - self.previous_y) / dt
            alpha = self.clamp(float(self.cfg.get('rate_filter_alpha', 0.70)), 0.01, 1.0)
            self.filtered_y_rate += alpha * (raw_rate - self.filtered_y_rate)

        catch_y = self.clamp(float(self.cfg.get('catch_y_norm', 0.72)), 0.05, 0.95)
        lookahead = self.clamp(float(self.cfg.get('lookahead_s', 0.18)), 0.0, 0.60)
        predicted_y = self.clamp(y + self.filtered_y_rate * lookahead, 0.0, 1.0)
        position_error = catch_y - predicted_y

        deadband = self.clamp(float(self.cfg.get('position_deadband_norm', 0.03)), 0.0, 0.30)
        upward_rate = max(0.0, -self.filtered_y_rate)
        min_upward_rate = max(0.0, float(self.cfg.get('min_upward_rate', 0.005)))

        # kp是接球线位置反馈；kd在新版中作为向上视线速度前馈。
        kp = max(0.0, float(self.cfg.get('kp', 2.0)))
        rate_ff = max(0.0, float(self.cfg.get('kd', 0.80)))
        effective_error = max(0.0, position_error - deadband)
        target = kp * effective_error + rate_ff * upward_rate

        # 小球明显向上时，即使短时预测已接近接球线，也保留速度前馈。
        if upward_rate >= min_upward_rate:
            target = max(target, rate_ff * upward_rate)

        fast_error = max(deadband, float(self.cfg.get('fast_error_norm', 0.18)))
        fast_vx = self.clamp(float(self.cfg.get('fast_vx', max_vx)), 0.0, max_vx)
        if position_error >= fast_error:
            target = max(target, fast_vx)

        target = self.clamp(target, 0.0, max_vx)
        min_vx = self.clamp(float(self.cfg.get('min_vx', 0.75)), 0.0, max_vx)
        if 0.0 < target < min_vx:
            target = min_vx

        self.previous_y = y
        self.last_detected_y = y
        self.last_detected_rate = self.filtered_y_rate
        return target

    def update(self, observation, dt):
        if not self.cfg.get('enabled', False):
            self.reset()
            return 0.0

        dt = self.clamp(float(dt), 0.005, 0.20)
        valid = bool(observation.get('valid', False))
        detected = bool(observation.get('detected', valid))
        age_ms = float(observation.get('age_ms', 1e9))
        lost_timeout_ms = float(self.cfg.get('lost_timeout_ms', 300))

        if not valid or age_ms > lost_timeout_ms:
            self.reset()
            return 0.0

        sign = float(self.cfg.get('output_sign', 1.0))
        max_vx = max(0.0, float(self.cfg.get('max_vx', 1.40)))

        if not detected:
            min_rate = max(0.0, float(self.cfg.get('min_upward_rate', 0.005)))
            over_top = (
                bool(self.cfg.get('overtop_enabled', True))
                and self.last_detected_y is not None
                and self.last_detected_y <= self.clamp(float(self.cfg.get('overtop_y_norm', 0.18)), 0.0, 1.0)
                and self.last_detected_rate <= -min_rate
                and age_ms <= float(self.cfg.get('overtop_hold_ms', 300))
            )
            if over_top:
                target = self.clamp(float(self.cfg.get('overtop_vx', max_vx)), 0.0, max_vx)
                return self.ramp(target * sign, dt)
            if age_ms <= float(self.cfg.get('dropout_hold_ms', 150)):
                return self.previous_vx
            return self.ramp(0.0, dt)

        y = self.clamp(float(observation.get('y_norm', 0.0)), 0.0, 1.0)
        target = self._command_from_observation(y, dt, max_vx)
        return self.ramp(target * sign, dt)
