# -*- coding: utf-8 -*-
"""
goto_strategy_basic.py
----------------------
V4.1-test multi-strategy point-to-point controller for CODBOT M100.

Compatibility:
    This file keeps the existing public API used by M100 Control Center:
        from goto_strategy_basic import GotoController, GotoConfig
        controller.start(..., config=GotoConfig(...))
        controller.update(pose, dt)

Main idea:
    You can now switch goto behavior by editing only the CONFIG AREA below,
    without touching the main Control Center program.

Coordinate conventions:
    world x: field left/right direction, increasing to the right
    world y: field bottom/top direction, increasing upward
    yaw    : AprilTag/world math yaw in radians, 0 along +world x, +pi/2 along +world y
    vx     : robot forward/backward command, positive means forward
    vy     : robot left/right command, positive means robot-left

Important current project decision:
    Max WZ default is 1.0 in the main UI.
    Pure rotation should not be multiplied by throttle/speed bar anymore.
    Translation speed and rotation speed are tuned separately.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple


# ============================================================
# CONFIG AREA - edit here for weekend tests
# ============================================================
# Select one strategy for normal Go:
#   "p"                Current baseline P control. Good reference group.
#   "rcpd"             Ramp + Cruise + P/PD-style brake. Recommended default for V4.1.
#                       Goal: soft start to reduce wheel slip, short cruise at Max, then compact brake zone.
#   "max_then_slow"    Existing baseline: far = max speed, near = slow down.
#   "pd_damping"       P control plus visual-speed damping. Good after FPS/pose is stable.
#   "brake_distance"   Uses estimated visual speed and braking-distance model.
#   "bang_bang"        Aggressive full-speed test. Use carefully on a small field.
DEFAULT_GOTO_STRATEGY = "rcpd"

# Keep return/center conservative and accurate unless explicitly testing.
DEFAULT_CENTER_STRATEGY = "p"

# Current baseline P control parameters.
DEFAULT_KP = 0.7
DEFAULT_MAX_CMD = 1.4
DEFAULT_STOP_RADIUS = 0.05

# Strategy: RCPD = Ramp + Cruise + P Brake.
# Problem solved:
#   1) Startup slip: do not jump to Max immediately. Limit command rise rate.
#   2) Too-long slow zone on a 1.2m field: keep brake radius compact.
#   3) End-point oscillation: use proportional brake speed near target.
# Parameter meaning:
#   RCPD_ACCEL_RATE: max command increase per second. At 20Hz, 2.0 -> 0.10 per update.
#   RCPD_SLOW_RADIUS: brake zone radius. For 1.2m field, start with 0.12 to 0.18m.
#   RCPD_MIN_CMD: minimum non-zero command in brake zone to overcome static friction.
#   RCPD_DECEL_RATE: max command decrease per second. Usually larger than accel so braking is not too soft.
# Launch-RCPD defaults for the 1.2m x 1.2m field.
# Startup protection is time-limited, then high Ramp/s is used to reach Max quickly.
RCPD_LAUNCH_TIME = 0.10
RCPD_LAUNCH_SPEED = 0.55
RCPD_ACCEL_RATE = 8.0
RCPD_SLOW_RADIUS = 0.12
RCPD_MIN_CMD = 0.14
RCPD_DECEL_RATE = 10.0

# Strategy: max_then_slow
# Far from target: command MAX_CMD directly toward target.
# Inside SLOW_RADIUS: reduce speed toward MIN_CMD, then stop inside STOP_RADIUS.
# Suggested first tests:
#   SLOW_RADIUS = 0.20 to 0.35 for a 1.2m field
#   MIN_CMD     = 0.12 to 0.20
SLOW_RADIUS = 0.25
MIN_CMD = 0.15
SLOW_CURVE_POWER = 1.0    # 1.0 = linear; 1.5/2.0 = softer near target

# Strategy: pd_damping
# Command in world frame:
#   cmd_world = Kp * position_error - Kd * visual_velocity
# This is often more useful than full PID for vision control.
PD_KP = 1.8
PD_KD = 0.45
PD_MIN_CMD = 0.00         # set 0.10~0.15 only if the car sticks near target

# Strategy: brake_distance
# Estimate stopping distance:
#   brake_distance = radial_speed^2 / (2 * BRAKE_DECEL)
# When remaining distance is near brake_distance + buffer, speed is reduced.
# BRAKE_DECEL is an empirical parameter in command-unit/s^2, not a perfect physics value.
BRAKE_DECEL = 2.5
BRAKE_BUFFER = 0.06
BRAKE_MIN_CMD = 0.12

# Strategy: bang_bang
# Aggressive test. Full-speed until BANG_SLOW_RADIUS, then use near-zone P correction.
# This avoids instantly commanding reverse unless BANG_REVERSE_BRAKE=True.
BANG_SLOW_RADIUS = 0.25
BANG_NEAR_KP = 2.0
BANG_REVERSE_BRAKE = False
BANG_REVERSE_GAIN = 0.35

# Visual velocity estimator.
# Smaller alpha = smoother but slower. Larger alpha = more responsive but noisier.
VEL_FILTER_ALPHA = 0.35
VEL_MAX_DT = 0.25
VEL_MIN_DT = 0.005

# Safety and command shaping.
# If True, ensure a non-zero command is at least min_cmd when outside stop radius.
ENABLE_MIN_CMD_FLOOR = True

# Mecanum translation limit. Keep diamond because project goal prioritizes fastest arrival
# within wheel target envelope rather than circular normalization.
USE_DIAMOND_LIMIT = True


# ============================================================
# Utilities
# ============================================================
def wrap_rad(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


@dataclass
class GotoConfig:
    # Normal Go strategy.
    strategy: str = DEFAULT_GOTO_STRATEGY

    # Baseline settings kept compatible with the main Control Center UI.
    kp: float = DEFAULT_KP
    max_cmd: float = DEFAULT_MAX_CMD
    stop_radius: float = DEFAULT_STOP_RADIUS

    # Sign calibration for Mode6 body-frame commands.
    # Current validation: vx+ = forward, vy+ = left. Keep both +1.
    vx_sign: float = 1.0
    vy_sign: float = 1.0

    # If AprilTag printed orientation is not aligned with car forward, adjust this.
    # Current validation: AprilTag yaw is accurate, so keep 0.
    yaw_offset_rad: float = 0.0

    # Launch-RCPD parameters.
    # brake_kp uses the common kp field from the UI. Recommended first value: 4.0 to 5.0.
    rcpd_launch_time: float = RCPD_LAUNCH_TIME
    rcpd_launch_speed: float = RCPD_LAUNCH_SPEED
    rcpd_accel_rate: float = RCPD_ACCEL_RATE
    rcpd_slow_radius: float = RCPD_SLOW_RADIUS
    rcpd_min_cmd: float = RCPD_MIN_CMD
    rcpd_decel_rate: float = RCPD_DECEL_RATE

    # max_then_slow parameters.
    slow_radius: float = SLOW_RADIUS
    min_cmd: float = MIN_CMD
    slow_curve_power: float = SLOW_CURVE_POWER

    # PD damping parameters.
    pd_kp: float = PD_KP
    pd_kd: float = PD_KD
    pd_min_cmd: float = PD_MIN_CMD

    # Brake distance parameters.
    brake_decel: float = BRAKE_DECEL
    brake_buffer: float = BRAKE_BUFFER
    brake_min_cmd: float = BRAKE_MIN_CMD

    # Bang-bang parameters.
    bang_slow_radius: float = BANG_SLOW_RADIUS
    bang_near_kp: float = BANG_NEAR_KP
    bang_reverse_brake: bool = BANG_REVERSE_BRAKE
    bang_reverse_gain: float = BANG_REVERSE_GAIN

    # Return/center strategy and yaw phase.
    center_strategy: str = DEFAULT_CENTER_STRATEGY
    yaw_kp: float = 1.2
    yaw_max_wz: float = 0.6
    yaw_tolerance_rad: float = math.radians(5.0)
    center_kp: float = 0.55
    center_max_cmd: float = 0.45
    center_stop_radius: float = 0.03
    center_slow_radius: float = 0.25
    center_min_cmd: float = 0.10


@dataclass
class GotoResult:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    done: bool = False
    state: str = "idle"
    error_m: Optional[float] = None
    yaw_error_rad: Optional[float] = None
    debug: Dict[str, Any] = field(default_factory=dict)


class GotoController:
    def __init__(self, *, max_target_raw: float = 1400.0):
        self.max_target_raw = float(max_target_raw)
        self.running = False
        self.mode = "goto"
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = None
        self.state = "idle"
        self.config = GotoConfig()

        # Visual velocity estimate in world frame.
        self.prev_x = None
        self.prev_y = None
        self.prev_t = None
        self.vx_world_f = 0.0
        self.vy_world_f = 0.0

        # Command state for ramp/slew-limited strategies such as RCPD.
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.run_start_t = None

    def is_running(self) -> bool:
        return bool(self.running)

    def start(self, x: float, y: float, *, mode: str = "goto", target_yaw: Optional[float] = None, config: Optional[GotoConfig] = None):
        self.target_x = float(x)
        self.target_y = float(y)
        self.mode = str(mode)
        self.target_yaw = target_yaw
        self.config = config or GotoConfig()
        self.running = True
        self.state = "move"
        self.run_start_t = time.time()
        self._reset_velocity_estimator()
        self._reset_command_state()

    def update_target(self, x: float, y: float, *, target_yaw=None):
        """连续GoTo目标更新：不重置速度估计、Launch计时或命令斜坡状态。"""
        self.target_x = float(x)
        self.target_y = float(y)
        if target_yaw is not None:
            self.target_yaw = target_yaw
        if not self.running:
            self.running = True
            self.state = "move"
            self.run_start_t = time.time()

    def stop(self):
        self.running = False
        self.state = "idle"
        self.run_start_t = None
        self._reset_velocity_estimator()
        self._reset_command_state()

    def _reset_velocity_estimator(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_t = None
        self.vx_world_f = 0.0
        self.vy_world_f = 0.0

    def _reset_command_state(self):
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0

    def _update_velocity_estimator(self, pose, fallback_dt: float = 0.05) -> Tuple[float, float, Dict[str, Any]]:
        """Estimate visual velocity in world frame from consecutive valid poses."""
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        t = pose.get("t", None)
        if t is None:
            # Main app usually provides pose['t']; fallback keeps compatibility.
            t = (self.prev_t + fallback_dt) if self.prev_t is not None else 0.0
        t = float(t)

        debug = {"vel_valid": False, "vx_world": self.vx_world_f, "vy_world": self.vy_world_f, "vel_dt": None}
        if self.prev_x is None or self.prev_y is None or self.prev_t is None:
            self.prev_x, self.prev_y, self.prev_t = x, y, t
            return self.vx_world_f, self.vy_world_f, debug

        dt = t - float(self.prev_t)
        if not (VEL_MIN_DT <= dt <= VEL_MAX_DT):
            self.prev_x, self.prev_y, self.prev_t = x, y, t
            return self.vx_world_f, self.vy_world_f, debug

        vx_raw = (x - float(self.prev_x)) / dt
        vy_raw = (y - float(self.prev_y)) / dt
        a = clamp(VEL_FILTER_ALPHA, 0.0, 1.0)
        self.vx_world_f = (1.0 - a) * self.vx_world_f + a * vx_raw
        self.vy_world_f = (1.0 - a) * self.vy_world_f + a * vy_raw
        self.prev_x, self.prev_y, self.prev_t = x, y, t
        debug.update({
            "vel_valid": True,
            "vx_world_raw": vx_raw,
            "vy_world_raw": vy_raw,
            "vx_world": self.vx_world_f,
            "vy_world": self.vy_world_f,
            "vel_dt": dt,
        })
        return self.vx_world_f, self.vy_world_f, debug

    def _diamond_limit(self, vx: float, vy: float, limit: float):
        """Mecanum diamond envelope for translation only: abs(vx)+abs(vy)<=limit."""
        limit = abs(float(limit))
        if not USE_DIAMOND_LIMIT:
            mag = math.hypot(vx, vy)
            if mag > limit and mag > 1e-9:
                s = limit / mag
                return vx * s, vy * s
            return vx, vy
        load = abs(vx) + abs(vy)
        if load > limit and load > 1e-9:
            s = limit / load
            vx *= s
            vy *= s
        return vx, vy

    def _world_error_to_body(self, dx: float, dy: float, yaw: float, yaw_offset: float = 0.0):
        """Rotate world error vector into robot body frame.

        Returns:
            forward: target component along robot forward direction
            left:    target component along robot-left direction
        """
        y = float(yaw) + float(yaw_offset)
        forward = dx * math.cos(y) + dy * math.sin(y)
        left = -dx * math.sin(y) + dy * math.cos(y)
        return forward, left

    def _world_vector_to_body(self, wx: float, wy: float, yaw: float, yaw_offset: float = 0.0):
        return self._world_error_to_body(wx, wy, yaw, yaw_offset)

    def _apply_min_cmd_floor(self, vx: float, vy: float, min_cmd: float, max_cmd: float):
        """Raise tiny non-zero command to min_cmd while preserving direction."""
        if not ENABLE_MIN_CMD_FLOOR:
            return vx, vy
        min_cmd = max(0.0, float(min_cmd))
        if min_cmd <= 1e-9:
            return vx, vy
        mag = math.hypot(vx, vy)
        if 1e-9 < mag < min_cmd:
            s = min(min_cmd, abs(max_cmd)) / mag
            return vx * s, vy * s
        return vx, vy

    def _command_toward_target_by_speed(self, dx: float, dy: float, yaw: float, speed: float, max_cmd: float, min_cmd: float = 0.0):
        dist = math.hypot(dx, dy)
        if dist <= 1e-9:
            return 0.0, 0.0, {"direction_valid": False}
        ux = dx / dist
        uy = dy / dist
        speed = clamp(speed, 0.0, abs(max_cmd))
        wx = ux * speed
        wy = uy * speed
        forward, left = self._world_vector_to_body(wx, wy, yaw, self.config.yaw_offset_rad)
        vx = self.config.vx_sign * forward
        vy = self.config.vy_sign * left
        vx, vy = self._apply_min_cmd_floor(vx, vy, min_cmd, max_cmd)
        vx, vy = self._diamond_limit(vx, vy, max_cmd)
        return vx, vy, {"ux": ux, "uy": uy, "speed_cmd": speed}

    def _xy_common(self, pose):
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = pose.get("yaw", None)
        dx = self.target_x - x
        dy = self.target_y - y
        dist = math.hypot(dx, dy)
        return x, y, yaw, dx, dy, dist

    def _slew_limit_xy(self, target_vx: float, target_vy: float, dt: float, accel_rate: float, decel_rate: float):
        """Limit command vector change in body-frame command units per second.

        accel_rate limits command growth to reduce startup wheel slip.
        decel_rate is usually larger so the car can slow down near the target.
        """
        try:
            dt = float(dt)
        except Exception:
            dt = 0.05
        if dt <= 0.0 or dt > 0.25:
            dt = 0.05
        accel_rate = max(0.05, abs(float(accel_rate)))
        decel_rate = max(accel_rate, abs(float(decel_rate)))

        def step_axis(cur, target):
            # Growth means absolute command magnitude is increasing in the same direction.
            growing = abs(target) > abs(cur) and (cur == 0.0 or target * cur >= 0.0)
            rate = accel_rate if growing else decel_rate
            delta = clamp(target - cur, -rate * dt, rate * dt)
            return cur + delta

        self.cmd_vx = step_axis(float(self.cmd_vx), float(target_vx))
        self.cmd_vy = step_axis(float(self.cmd_vy), float(target_vy))
        return self.cmd_vx, self.cmd_vy

    def _xy_control_rcpd(self, pose, cfg: GotoConfig, dt: float):
        """Launch-RCPD: short anti-slip launch + fast ramp + compact P brake.

        Phase 1 / Launch:
            For the first cfg.rcpd_launch_time seconds, target speed is capped by
            cfg.rcpd_launch_speed. This gives the wheels a short chance to bite.
        Phase 2 / Fast Cruise:
            After launch, use high cfg.rcpd_accel_rate to reach cfg.max_cmd quickly.
        Phase 3 / Compact P Brake:
            Inside cfg.rcpd_slow_radius, target speed = cfg.kp * remaining distance,
            clipped by cfg.rcpd_min_cmd and cfg.max_cmd.
        """
        x, y, yaw, dx, dy, dist = self._xy_common(pose)
        if dist <= cfg.stop_radius:
            self._reset_command_state()
            return 0.0, 0.0, dist, True, {"dx": dx, "dy": dy, "reason": "reached", "strategy": "rcpd", "zone": "stop"}
        if yaw is None:
            self._reset_command_state()
            return 0.0, 0.0, dist, False, {"dx": dx, "dy": dy, "reason": "no_yaw", "strategy": "rcpd"}

        now = time.time()
        if self.run_start_t is None:
            self.run_start_t = now
        elapsed = max(0.0, now - float(self.run_start_t))

        slow_r = max(float(cfg.rcpd_slow_radius), cfg.stop_radius + 1e-6)
        launch_time = max(0.0, float(cfg.rcpd_launch_time))
        launch_speed = clamp(float(cfg.rcpd_launch_speed), 0.0, cfg.max_cmd)

        if dist <= slow_r:
            # Compact P brake zone. For the 1.2m test field, Kp around 4.0-6.0 is useful.
            brake_speed = float(cfg.kp) * max(0.0, dist - cfg.stop_radius)
            target_speed = clamp(brake_speed, cfg.rcpd_min_cmd, cfg.max_cmd)
            zone = "brake"
        elif elapsed < launch_time:
            target_speed = min(cfg.max_cmd, launch_speed)
            zone = "launch"
        else:
            target_speed = cfg.max_cmd
            zone = "cruise"

        target_vx, target_vy, dbg2 = self._command_toward_target_by_speed(
            dx, dy, float(yaw), target_speed, cfg.max_cmd, min_cmd=0.0
        )
        vx, vy = self._slew_limit_xy(
            target_vx, target_vy, dt,
            accel_rate=cfg.rcpd_accel_rate,
            decel_rate=cfg.rcpd_decel_rate,
        )
        vx, vy = self._diamond_limit(vx, vy, cfg.max_cmd)
        dbg2.update({
            "dx": dx, "dy": dy, "yaw": yaw, "dist": dist,
            "zone": zone, "strategy": "rcpd",
            "elapsed": elapsed,
            "target_speed": target_speed,
            "target_vx": target_vx, "target_vy": target_vy,
            "cmd_vx": vx, "cmd_vy": vy,
            "rcpd_launch_time": cfg.rcpd_launch_time,
            "rcpd_launch_speed": cfg.rcpd_launch_speed,
            "rcpd_accel_rate": cfg.rcpd_accel_rate,
            "rcpd_slow_radius": cfg.rcpd_slow_radius,
            "rcpd_min_cmd": cfg.rcpd_min_cmd,
            "rcpd_decel_rate": cfg.rcpd_decel_rate,
            "rcpd_brake_kp": cfg.kp,
        })
        return vx, vy, dist, False, dbg2

    def _xy_control_p(self, pose, kp: float, max_cmd: float, stop_radius: float):
        x, y, yaw, dx, dy, dist = self._xy_common(pose)
        if dist <= stop_radius:
            return 0.0, 0.0, dist, True, {"dx": dx, "dy": dy, "reason": "reached", "strategy": "p"}
        if yaw is None:
            return 0.0, 0.0, dist, False, {"dx": dx, "dy": dy, "reason": "no_yaw", "strategy": "p"}

        cfg = self.config
        forward, left = self._world_error_to_body(dx, dy, float(yaw), cfg.yaw_offset_rad)
        vx = cfg.vx_sign * kp * forward
        vy = cfg.vy_sign * kp * left
        vx, vy = self._diamond_limit(vx, vy, max_cmd)
        return vx, vy, dist, False, {
            "dx": dx, "dy": dy, "yaw": yaw, "forward": forward, "left": left,
            "vx_sign": cfg.vx_sign, "vy_sign": cfg.vy_sign, "strategy": "p",
        }

    def _xy_control_max_then_slow(self, pose, max_cmd: float, stop_radius: float, slow_radius: float, min_cmd: float, curve_power: float):
        x, y, yaw, dx, dy, dist = self._xy_common(pose)
        if dist <= stop_radius:
            return 0.0, 0.0, dist, True, {"dx": dx, "dy": dy, "reason": "reached", "strategy": "max_then_slow"}
        if yaw is None:
            return 0.0, 0.0, dist, False, {"dx": dx, "dy": dy, "reason": "no_yaw", "strategy": "max_then_slow"}

        slow_radius = max(float(slow_radius), stop_radius + 1e-6)
        if dist >= slow_radius:
            speed = max_cmd
            zone = "max"
        else:
            r = (dist - stop_radius) / max(1e-6, slow_radius - stop_radius)
            r = clamp(r, 0.0, 1.0)
            r = r ** max(0.1, float(curve_power))
            speed = min_cmd + (max_cmd - min_cmd) * r
            zone = "slow"
        vx, vy, dbg2 = self._command_toward_target_by_speed(dx, dy, float(yaw), speed, max_cmd, min_cmd)
        dbg2.update({"dx": dx, "dy": dy, "yaw": yaw, "dist": dist, "zone": zone, "strategy": "max_then_slow"})
        return vx, vy, dist, False, dbg2

    def _xy_control_pd_damping(self, pose, cfg: GotoConfig, vx_world: float, vy_world: float):
        x, y, yaw, dx, dy, dist = self._xy_common(pose)
        if dist <= cfg.stop_radius:
            return 0.0, 0.0, dist, True, {"dx": dx, "dy": dy, "reason": "reached", "strategy": "pd_damping"}
        if yaw is None:
            return 0.0, 0.0, dist, False, {"dx": dx, "dy": dy, "reason": "no_yaw", "strategy": "pd_damping"}

        wx = cfg.pd_kp * dx - cfg.pd_kd * vx_world
        wy = cfg.pd_kp * dy - cfg.pd_kd * vy_world
        forward, left = self._world_vector_to_body(wx, wy, float(yaw), cfg.yaw_offset_rad)
        vx = cfg.vx_sign * forward
        vy = cfg.vy_sign * left
        vx, vy = self._apply_min_cmd_floor(vx, vy, cfg.pd_min_cmd, cfg.max_cmd)
        vx, vy = self._diamond_limit(vx, vy, cfg.max_cmd)
        return vx, vy, dist, False, {
            "dx": dx, "dy": dy, "yaw": yaw, "vx_world": vx_world, "vy_world": vy_world,
            "cmd_world_x": wx, "cmd_world_y": wy, "strategy": "pd_damping",
        }

    def _xy_control_brake_distance(self, pose, cfg: GotoConfig, vx_world: float, vy_world: float):
        x, y, yaw, dx, dy, dist = self._xy_common(pose)
        if dist <= cfg.stop_radius:
            return 0.0, 0.0, dist, True, {"dx": dx, "dy": dy, "reason": "reached", "strategy": "brake_distance"}
        if yaw is None:
            return 0.0, 0.0, dist, False, {"dx": dx, "dy": dy, "reason": "no_yaw", "strategy": "brake_distance"}

        ux = dx / max(dist, 1e-9)
        uy = dy / max(dist, 1e-9)
        # Positive radial_speed means moving toward the target.
        radial_speed = vx_world * ux + vy_world * uy
        brake_distance = max(0.0, radial_speed) ** 2 / max(1e-6, 2.0 * cfg.brake_decel)
        threshold = brake_distance + cfg.brake_buffer + cfg.stop_radius
        if dist > threshold:
            speed = cfg.max_cmd
            zone = "max"
        else:
            # Desired speed that can still stop within remaining distance.
            remain = max(0.0, dist - cfg.stop_radius)
            speed = math.sqrt(max(0.0, 2.0 * cfg.brake_decel * remain))
            speed = clamp(speed, cfg.brake_min_cmd, cfg.max_cmd)
            zone = "brake"
        vx, vy, dbg2 = self._command_toward_target_by_speed(dx, dy, float(yaw), speed, cfg.max_cmd, cfg.brake_min_cmd)
        dbg2.update({
            "dx": dx, "dy": dy, "yaw": yaw, "radial_speed": radial_speed,
            "brake_distance": brake_distance, "threshold": threshold, "zone": zone,
            "strategy": "brake_distance",
        })
        return vx, vy, dist, False, dbg2

    def _xy_control_bang_bang(self, pose, cfg: GotoConfig, vx_world: float, vy_world: float):
        x, y, yaw, dx, dy, dist = self._xy_common(pose)
        if dist <= cfg.stop_radius:
            return 0.0, 0.0, dist, True, {"dx": dx, "dy": dy, "reason": "reached", "strategy": "bang_bang"}
        if yaw is None:
            return 0.0, 0.0, dist, False, {"dx": dx, "dy": dy, "reason": "no_yaw", "strategy": "bang_bang"}

        if dist > cfg.bang_slow_radius:
            speed = cfg.max_cmd
            vx, vy, dbg2 = self._command_toward_target_by_speed(dx, dy, float(yaw), speed, cfg.max_cmd, cfg.min_cmd)
            zone = "full"
        else:
            forward, left = self._world_error_to_body(dx, dy, float(yaw), cfg.yaw_offset_rad)
            vx = cfg.vx_sign * cfg.bang_near_kp * forward
            vy = cfg.vy_sign * cfg.bang_near_kp * left
            # Optional reverse braking against current velocity. Use carefully.
            if cfg.bang_reverse_brake:
                vf, vl = self._world_vector_to_body(vx_world, vy_world, float(yaw), cfg.yaw_offset_rad)
                vx -= cfg.bang_reverse_gain * vf
                vy -= cfg.bang_reverse_gain * vl
            vx, vy = self._diamond_limit(vx, vy, cfg.max_cmd)
            dbg2 = {"zone": "near", "speed_cmd": None}
            zone = "near"
        dbg2.update({"dx": dx, "dy": dy, "yaw": yaw, "dist": dist, "zone": zone, "strategy": "bang_bang"})
        return vx, vy, dist, False, dbg2

    def _xy_control_by_strategy(self, pose, cfg: GotoConfig, *, for_center: bool = False, velocity_debug: Optional[Dict[str, Any]] = None):
        vx_world = 0.0 if velocity_debug is None else float(velocity_debug.get("vx_world", 0.0))
        vy_world = 0.0 if velocity_debug is None else float(velocity_debug.get("vy_world", 0.0))

        if for_center:
            strategy = str(cfg.center_strategy).lower()
            # Center should generally remain conservative.
            if strategy == "max_then_slow":
                return self._xy_control_max_then_slow(
                    pose, cfg.center_max_cmd, cfg.center_stop_radius,
                    cfg.center_slow_radius, cfg.center_min_cmd, cfg.slow_curve_power
                )
            return self._xy_control_p(pose, cfg.center_kp, cfg.center_max_cmd, cfg.center_stop_radius)

        strategy = str(cfg.strategy).lower()
        if strategy in ("rcpd", "ramp_cruise_pd", "ramp_cruise_p", "ramp"):
            return self._xy_control_rcpd(pose, cfg, dt=velocity_debug.get("dt", 0.05) if velocity_debug else 0.05)
        if strategy in ("p", "basic", "kp"):
            return self._xy_control_p(pose, cfg.kp, cfg.max_cmd, cfg.stop_radius)
        if strategy in ("max_then_slow", "slow", "trapezoid"):
            return self._xy_control_max_then_slow(pose, cfg.max_cmd, cfg.stop_radius, cfg.slow_radius, cfg.min_cmd, cfg.slow_curve_power)
        if strategy in ("pd", "pd_damping", "damping"):
            return self._xy_control_pd_damping(pose, cfg, vx_world, vy_world)
        if strategy in ("brake", "brake_distance"):
            return self._xy_control_brake_distance(pose, cfg, vx_world, vy_world)
        if strategy in ("bang", "bang_bang"):
            return self._xy_control_bang_bang(pose, cfg, vx_world, vy_world)

        # Unknown strategy falls back to baseline P safely.
        vx, vy, dist, reached, debug = self._xy_control_p(pose, cfg.kp, cfg.max_cmd, cfg.stop_radius)
        debug["strategy_requested"] = strategy
        debug["strategy_fallback"] = "p"
        return vx, vy, dist, reached, debug

    def update(self, pose, dt: float = 0.05) -> GotoResult:
        if not self.running:
            return GotoResult(state="idle", done=True)
        if not pose or not pose.get("valid", False):
            return GotoResult(state="no_pose", done=False, debug={"reason": "vision pose invalid"})

        cfg = self.config
        vx_world, vy_world, vel_debug = self._update_velocity_estimator(pose, fallback_dt=dt)
        vel_debug["dt"] = dt

        if self.mode == "center":
            if self.state == "move":
                vx, vy, dist, reached, debug = self._xy_control_by_strategy(
                    pose, cfg, for_center=True, velocity_debug=vel_debug
                )
                debug.update(vel_debug)
                if debug.get("reason") == "no_yaw":
                    return GotoResult(vx=0.0, vy=0.0, wz=0.0, done=False, state="center_no_yaw", error_m=dist, debug=debug)
                if not reached:
                    return GotoResult(vx=vx, vy=vy, wz=0.0, done=False, state="center_move", error_m=dist, debug=debug)
                self.state = "yaw"

            yaw = pose.get("yaw", None)
            if yaw is None or self.target_yaw is None:
                self.running = False
                return GotoResult(done=True, state="center_done_no_yaw", error_m=0.0)
            err = wrap_rad(float(self.target_yaw) - (float(yaw) + cfg.yaw_offset_rad))
            if abs(err) <= cfg.yaw_tolerance_rad:
                self.running = False
                return GotoResult(done=True, state="center_done", error_m=0.0, yaw_error_rad=err)
            wz = clamp(cfg.yaw_kp * err, -cfg.yaw_max_wz, cfg.yaw_max_wz)
            return GotoResult(vx=0.0, vy=0.0, wz=wz, done=False, state="center_yaw", error_m=0.0, yaw_error_rad=err)

        vx, vy, dist, reached, debug = self._xy_control_by_strategy(pose, cfg, for_center=False, velocity_debug=vel_debug)
        debug.update(vel_debug)
        if debug.get("reason") == "no_yaw":
            return GotoResult(vx=0.0, vy=0.0, wz=0.0, done=False, state="goto_no_yaw", error_m=dist, debug=debug)
        if reached:
            self.running = False
            self._reset_command_state()
            return GotoResult(done=True, state="goto_done", error_m=dist, debug=debug)
        state_name = "goto_" + str(debug.get("strategy", cfg.strategy))
        return GotoResult(vx=vx, vy=vy, wz=0.0, done=False, state=state_name, error_m=dist, debug=debug)
