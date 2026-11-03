class LateralController:
    """速度优先的图像X轴横移控制器。

    大误差时直接进入高速区，小误差时保留比例控制和最小速度补偿。
    所有速度值均为Mode 6协议速度尺度，不代表已经标定的物理m/s。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.previous_error = 0.0
        self.previous_vy = 0.0
        self.initialized = False

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def update(self, observation, dt):
        if not self.cfg.get("enabled", False) or not observation.get("valid", False):
            self.reset()
            return 0.0

        dt = self.clamp(float(dt), 0.001, 0.20)
        error = float(observation.get("x_error", 0.0))
        deadband = max(0.0, float(self.cfg.get("center_deadband_norm", 0.025)))
        max_vy = max(0.0, float(self.cfg.get("max_vy", 1.20)))
        min_vy = self.clamp(float(self.cfg.get("min_vy", 0.30)), 0.0, max_vy)
        fast_error = max(deadband, float(self.cfg.get("fast_error_norm", 0.18)))
        fast_vy = self.clamp(float(self.cfg.get("fast_vy", max_vy)), 0.0, max_vy)

        if abs(error) <= deadband:
            target = 0.0
        else:
            direction = 1.0 if error > 0.0 else -1.0
            sign = float(self.cfg.get("image_error_to_body_y_sign", 1.0))
            if abs(error) >= fast_error:
                target = sign * direction * fast_vy
            else:
                effective = error - deadband if error > 0.0 else error + deadband
                derivative = 0.0 if not self.initialized else (error - self.previous_error) / dt
                target = sign * (
                    float(self.cfg.get("kp", 1.20)) * effective
                    + float(self.cfg.get("kd", 0.0)) * derivative
                )
                target = self.clamp(target, -max_vy, max_vy)
                if 0.0 < abs(target) < min_vy:
                    target = min_vy if target > 0.0 else -min_vy

        max_accel = max(0.01, float(self.cfg.get("max_accel_vy_per_s", 6.0)))
        max_step = max_accel * dt
        command = self.previous_vy + self.clamp(
            target - self.previous_vy,
            -max_step,
            max_step,
        )

        self.previous_error = error
        self.previous_vy = command
        self.initialized = True
        return command
