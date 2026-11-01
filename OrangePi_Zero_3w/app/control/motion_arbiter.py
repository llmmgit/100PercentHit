class MotionArbiter:
    """合并X、Y和yaw输出，并保留麦轮平移菱形速度包络。"""

    def __init__(self, cfg, lateral, yaw_hold, oac):
        self.cfg = cfg
        self.lateral = lateral
        self.yaw_hold = yaw_hold
        self.oac = oac
        self.last_trace = self._empty_trace()

    @staticmethod
    def _empty_trace():
        return {
            "y_request": 0.0,
            "x_request": 0.0,
            "y_output": 0.0,
            "x_output": 0.0,
            "limited": False,
            "scale": 1.0,
        }

    def reset(self):
        self.lateral.reset()
        self.yaw_hold.reset()
        self.oac.reset()
        self.last_trace = self._empty_trace()

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def _limit_translation_diamond(self, vx, vy):
        motion = self.cfg.get("motion", {})
        limit = max(0.0, float(motion.get("auto_translation_limit", 1.40)))
        if limit <= 0.0:
            return vx, vy, False, 1.0

        total = abs(vx) + abs(vy)
        if total <= limit:
            return vx, vy, False, 1.0

        if bool(motion.get("auto_translation_priority_y", False)):
            output_vx = self._clamp(float(vx), -limit, limit)
            remaining = max(0.0, limit - abs(output_vx))
            output_vy = self._clamp(float(vy), -remaining, remaining)
            scale = abs(output_vx) / abs(vx) if abs(vx) > 1e-9 else 1.0
            return output_vx, output_vy, True, scale

        scale = limit / total
        return vx * scale, vy * scale, True, scale

    def automatic(self, mode, observation, dt, yaw, yaw_ref, yaw_rate, imu_fresh):
        vx = vy = wz = 0.0
        if mode in ("ARMED", "TRACKING", "AUTO_MOVE"):
            wz = self.yaw_hold.update(yaw, yaw_ref, yaw_rate, imu_fresh)
        if mode == "TRACKING":
            x_request = self.lateral.update(observation, dt)
            y_request = self.oac.update(observation, dt)
            vx, vy, limited, scale = self._limit_translation_diamond(y_request, x_request)
            self.last_trace = {
                "y_request": y_request,
                "x_request": x_request,
                "y_output": vx,
                "x_output": vy,
                "limited": limited,
                "scale": scale,
            }
        else:
            self.lateral.reset()
            self.oac.reset()
            self.last_trace = self._empty_trace()
        return vx, vy, wz
