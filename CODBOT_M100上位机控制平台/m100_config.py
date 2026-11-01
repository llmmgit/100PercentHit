# -*- coding: utf-8 -*-
"""M100 Control Center V4.1.0 可调配置。

说明：
- 本文件中的雷达过滤和防撞参数均为建议初值，必须通过实车场景测试标定。
- OneDrive中的实际文件名通常为 m100_config.py.txt。
- 本地运行时对应文件名为 m100_config.py。
"""
from dataclasses import dataclass

# 上位机显示和版本记录使用的应用版本号。
APP_VERSION = "4.1.0"


@dataclass
class LidarConfig:
    # 是否启用LD14模块；False时主程序应忽略雷达显示和雷达相关功能。
    enabled: bool = True

    # ESP32向PC发送LD14原始有效帧所使用的UDP监听端口。
    udp_port: int = 4212

    # 超过该时间未收到有效LD14帧时，将雷达状态判定为离线，单位：秒。
    offline_timeout_s: float = 0.50

    # 是否启用雷达保护性停车；建议在过滤参数完成实车验证前保持False。
    collision_stop_enabled: bool = False

    # 前方有效障碍物距离小于等于该值时进入STOP确认逻辑，单位：米。
    stop_distance_m: float = 0.35

    # 已锁存STOP后，最近障碍物距离大于等于该值时进入释放确认逻辑，单位：米。
    # 应大于stop_distance_m，以形成距离滞回并减少停车状态抖动。
    release_distance_m: float = 0.45

    # 防撞判断使用的车体前方半扇区角度，单位：度；45表示检测车头左右各45度。
    front_half_angle_deg: float = 45.0

    # 进入防撞候选集合所需的最低LD14置信度，范围按设备原始1字节数值处理。
    min_confidence: int = 10

    # 连续检测到STOP距离内障碍物达到该次数后锁存停车。
    confirm_frames: int = 2

    # 连续满足释放条件达到该次数后解除雷达停车锁存。
    release_frames: int = 3

    # 雷达离线时是否触发停车；当前LD14为可选传感器，默认False。
    stop_on_sensor_loss: bool = False

    # LD14页面可见时的UI刷新周期，单位：毫秒；数值越小刷新越快、UI负载越高。
    visible_refresh_ms: int = 50

    # LD14页面隐藏时的后台UI刷新周期，单位：毫秒；用于降低无效重绘负载。
    hidden_refresh_ms: int = 250

    # 是否启用PC端LD14阳光近距离噪点过滤。
    sun_filter_enabled: bool = True

    # 小于等于该距离的点采用更严格的阳光噪点过滤，单位：米。
    sun_near_range_m: float = 0.60

    # 小于该距离的点直接视为无效极近距离点，单位：米；不可设置过大，以免漏检贴近车身的真实障碍物。
    sun_absolute_min_distance_m: float = 0.08

    # 普通距离点通过基础检查所需的最低LD14置信度。
    sun_min_confidence_normal: int = 5

    # 近距离点通过阳光过滤所需的最低LD14置信度；提高可增强过滤，但可能漏掉低反射率真实障碍物。
    sun_min_confidence_near: int = 12

    # 空间一致性检查向当前角度左右各扩展的角度分箱数量；当前每个分箱约1度。
    sun_neighbor_radius_bins: int = 2

    # 当前点被视为空间连续目标时，邻域内至少需要的距离相近点数量。
    sun_min_neighbor_count: int = 3

    # 邻域点与当前点允许的最大距离差，单位：米；越小越严格，但可能过滤细小或倾斜目标。
    sun_neighbor_tolerance_m: float = 0.10

    # 每个角度分箱保留的历史扫描数量，用于时间稳定性判断。
    sun_history_scans: int = 5

    # 在历史扫描窗口中，距离一致的有效命中至少达到该次数才视为稳定点。
    sun_required_hits: int = 3

    # 历史距离与历史中位数允许的最大偏差，单位：米；越小越严格。
    sun_temporal_tolerance_m: float = 0.10

    # 达到该高置信度且同时满足空间连续性时，允许近障碍物不等待完整历史确认直接通过。
    sun_immediate_confidence: int = 80


# 全局LD14配置实例，供主程序、雷达服务、UI和防撞模块统一读取。
LIDAR_CONFIG = LidarConfig()
