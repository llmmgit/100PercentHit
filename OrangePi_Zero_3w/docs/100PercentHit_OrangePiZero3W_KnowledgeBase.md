# 100 Percent Hit：Orange Pi Zero 3W 车载视觉伺服完整知识库

**基线日期：2026-11-01**  
**平台：Orange Pi Zero 3W + AR0234 + Allwinner A733 NPU + ESP32 + STM32F103RET6 + CODBOT M100**  
**文档定位：公开部署说明与项目后续工作的当前唯一基线。**

## 1. 项目介绍

100 Percent Hit 是一个车载视觉伺服自动接球项目。

AR0234摄像头安装在麦克纳姆轮小车上，摄像头面向抛球者并以约75°仰角观察空中小球。小车放置在抛球者前方，小球向小车方向抛出。Orange Pi Zero 3W持续检测小球在图像中的位置与运动变化，控制小车横向对准并沿前后方向追赶。小车在小球落地前与小球发生撞击，视为接球成功。

本支线采用车载相对视觉，不依赖外部固定摄像头或世界坐标。系统目标不是先重建完整三维轨迹，而是在不到一秒的有效视野内，以低延迟视觉闭环让小车快速到达小球运动路径。

## 2. 场地、安装与成功标准

- 小车为CODBOT M100麦轮底盘，位于抛球者前方。
- 摄像头朝向抛球者，约75°仰角。
- AR0234物理旋转90°安装，以获得更大的Y轴纵向视野；主程序采集后使用 `cv2.ROTATE_90_CLOCKWISE`恢复显示和控制坐标。
- 车体 `vx` 表示Y轴前后运动，车体 `vy` 表示X轴横移，`wz` 表示旋转。
- 小球在落地前与小车发生撞击，判定为SUCCESS。
- 正式测试应保证抛球路径、人员和小车移动区无障碍，并保留物理/网页STOP和LD14保护。

## 3. 当前状态摘要

- YOLOv5n UINT8 NBG为正式默认检测模型，YOLOv5s可在网页中切换。
- 当前两个正式NB模型针对**约6 cm、快速飞行的红色或橙色泡沫小球**训练。目标尺寸、颜色、材质、相机或场景明显变化时，应重新采集数据并训练、量化和验证模型。
- X轴采用速度优先参数，快速启动已经基本满足当前接球项目要求；允许一定震荡，不以静态精准停中为首要目标。
- Y轴采用基于图像纵向视线的预测式OAC，算法结构基本锁定，仍需继续小步微调参数。

## 4. 系统架构与数据流

```text
AR0234
  -> Vision线程（旋转/翻转）
  -> YOLOv5n/YOLOv5s NPU 或 HSV
  -> BallTracker
  -> X横移控制 + Y预测式OAC + W相对航向保持
  -> ESP32桥接
  -> STM32麦轮运动学、编码器PID和电机
```

- Orange Pi Zero 3W 安装官方ubuntu镜像，负责相机、检测、跟踪、状态机、高层控制、网页、图传、LD14安全雷达、GPIO和状态灯。
- ESP32负责3W/Windows控制入口、STM32串口桥、BNO085、电池ADC与断联STOP。
- STM32负责协议解析、麦轮运动学、编码器闭环和电机执行。
- YOLO C++共享库只负责推理与后处理，不直接发送运动命令。

## 5. 当前软件目录与文件职责

| 文件/目录 | 职责 |
| --- | --- |
| app/main.py | 程序入口；解析 `--config` 与 `--arm-trace`，组织状态转换、ARM/SAFE、网络切换、安全关机、线程启动与清理。 |
| app/core.py | 核心运行线程与共享状态；包含相机采集/检测、串口收发、X/Y/W控制调用、ARMTRACE输出和LD14线程。 |
| app/configuration.py | 只读取 `config/local.yaml`；为网页提供可编辑配置快照，并原子保存回 `local.yaml`、生成 `.bak`。 |
| app/web.py | Flask网页/API；状态、视频流、手动控制、ARM/STOP、网络、雷达旁路和配置应用/保存。 |
| app/hardware.py | GPIO按钮读取、WS2812 SPI驱动与状态灯动画。 |
| app/network.py | 通过 NetworkManager/nmcli选择家庭Wi-Fi或3W热点，并执行安全的异步切换。 |
| app/protocol.py | 生成M100 Mode 6速度帧和STOP帧；协议缩放不等同于精确物理速度。 |
| app/ld14.py | LD14 47字节帧解析、CRC、SDK角度修正、扫描缓存和障碍Guard。 |
| app/yolo_detector.py | Python ctypes封装 `libyolo_ball.so`，把BGR帧传给NPU检测并返回最佳小球框。 |
| app/tracking/ball_tracker.py | 目标确认、X/Y/尺寸一阶滤波、短时漏检保持和观测快照。 |
| app/control/lateral_controller.py | 速度优先的X轴横移控制器，输出车体 `vy`。 |
| app/control/oac_controller.py | 预测式Y轴OAC，包含接球线位置反馈、视线速度前馈、短时外推、漏检保持和顶部续冲。 |
| app/control/yaw_hold_controller.py | 基于BNO085相对航向的PD保持，输出 `wz`。 |
| app/control/motion_arbiter.py | 合并X、Y、W请求，执行麦轮菱形包络，并保存ARMTRACE诊断值。 |
| config/local.yaml | 当前实际运行配置，也是网页“保存”的目标文件。 |
| config/config.yaml | 默认/参考配置；systemd当前明确使用 `config/local.yaml`。 |
| templates/index.html | 手机网页结构。 |
| static/app.js | 网页交互、摇杆、状态刷新、设置页面和API调用。 |
| static/style.css | 手机端页面布局、竖向视频区、设置滑块和状态样式。 |
| systemd/percenthit.service | 开机服务定义，以root运行主程序并加载 `config/local.yaml`。 |
| scripts/setup_network.sh | 创建3W热点的示例脚本；公开仓库中应使用占位SSID和密码。 |
| esp32/M100_ESP32_DualControl_3W_UART_WiFi_V5.ino | ESP32桥接固件；连接3W、STM32、BNO085和电池ADC，保留500 ms断联STOP。公开前必须移除真实Wi-Fi凭据。 |
| tools/ld14_validate.py | LD14独立串口与CRC验证工具。 |
| tools/ld14_room_map.py | LD14房间点云和角度偏移验证工具。 |
| yolo/CMakeLists.txt | 构建独立检测程序和 `libyolo_ball.so`。 |
| yolo/SHA256SUMS | 关键二进制、模型和源码的校验值。 |
| yolo/bin/libyolo_ball.so | Python主程序调用的YOLO NPU共享库。 |
| yolo/bin/yolov5_ball_live_full | 独立摄像头/NPU性能验证程序。 |
| yolo/models/yolov5n_ball_uint8.nb | 当前YOLOv5n UINT8 NBG模型，默认使用。 |
| yolo/models/yolov5s_ball_uint8.nb | 当前YOLOv5s UINT8 NBG模型，可在网页中切换。 |
| yolo/models/v5backup/* | 旧数据集模型备份；README明确标记动态捕捉效果较差，不作为当前模型。 |
| yolo/src/yolo_ball_api.cpp/.h | 共享库C接口、640输入预处理、NPU调用和耗时输出。 |
| yolo/src/yolov5_post_process.cpp/.h | 单类别ball解码、候选框生成、NMS和最佳目标选择。 |
| yolo/src/main.cpp | 独立高帧率检测与性能统计程序。 |
| yolo/libawnn_viplite/* | Allwinner VIPLite/AWNN网络加载、量化转换和缓冲区访问支持代码。 |
| yolo/libawutils/* | JPEG和二进制数据工具代码。 |

## 6. YOLO模型基线

正式模型位于 `yolo/models/`：

- `yolov5n_ball_uint8.nb`：默认模型，资源占用和速度优先，仓库作者实测单帧检测时间约7-8ms。
- `yolov5s_ball_uint8.nb`：可选模型，适合对比识别稳定性，仓库作者实测单帧检测时间约13-16ms。

当前模型只应被视为对现有任务目标有效：约6 cm、快速飞行、红色或橙色泡沫球。更换目标时必须重新训练，不应仅通过修改HSV或面积阈值假设YOLO仍然可靠。

完整训练、四输出ONNX导出、Allwinner官方UINT8量化、NBG导出与部署流程见同目录文档：

[`100PercentHit_YOLOv5_NPU_Training_Quantization_Deployment_Guide_V2.0.md`](./100PercentHit_YOLOv5_NPU_Training_Quantization_Deployment_Guide_V2.0.md)

当前正式流程使用Allwinner官方四步：`pegasus_import.sh`、`pegasus_quantize.sh`、`pegasus_inference.sh`、`pegasus_export_ovx.sh`。主文档不再重复完整转换命令。

## 7. 视觉与BallTracker

- `vision.detector: yolo`时使用NPU模型；`hsv`时使用颜色检测。
- 当前 `yolo_fallback_to_hsv: false`，YOLO异常不会静默切换检测逻辑。
- C++后处理返回候选数量和最高置信度目标，Python当前只控制最佳目标。
- BallTracker输出 `valid`、`detected`、`age_ms`、`x_error`、`y_norm` 和 `size_norm`。
- `acquire_confirm_frames: 1`使首个有效检测即可进入TRACKING。
- `lateral_tracking.lost_timeout_ms: 250`既是BallTracker有效窗口，也用于TRACKING返回ARMED的状态判断。

## 8. X轴控制

### 8.1 当前速度优先基线

X轴通过图像水平误差控制车体 `vy`。当前参数强调小球进入画面后立即横移：死区为0、确认帧为1、D为0，允许一定反向修正和震荡。当前目标是短时间内对准飞行小球，不是让静态目标长期无抖动地保持在画面中心。

当前关键值：`kp=0.35`、`center_deadband_norm=0`、`min_vy=0.20`、`max_vy=0.50`、`fast_error_norm=0.10`、`fast_vy=0.50`、`max_accel_vy_per_s=2.5`。

### 8.2 如需精准X轴跟踪

如果使用者的任务更重视精准居中、低震荡或静态跟踪，可按以下方向逐项调整，每次只改一个参数并保留回退值：

- 增大 `center_deadband_norm`：减少中心附近频繁反向动作。
- 降低 `min_vy`：减小最小横移跳变，但必须高于底盘静摩擦有效值。
- 降低 `fast_vy` 和 `max_vy`：减少高速接近后的过冲。
- 增大 `fast_error_norm`：仅在更大偏差时进入高速区。
- 降低 `kp`：减弱位置误差响应。
- 适度降低 `error_filter_alpha`：增加平滑，但会引入延迟。
- 在视觉稳定后增加少量 `kd`：为快速误差变化增加阻尼。
- 降低 `max_accel_vy_per_s`：减少突发加速和轮胎打滑。
- 将 `acquire_confirm_frames`提高到2至3：降低单帧误检触发风险，但会延迟启动。

精准跟踪与本项目的极速接球目标存在直接取舍。参数不能在未测试的情况下同时承诺“零震荡”和“首帧高速响应”。

## 9. Y轴预测式OAC

当前Y轴不是经典真实仰角OAC，而是**基于图像纵向视线的预测式OAC**：

```text
滤波纵向速度 = 对相邻帧y_norm变化做一阶滤波
预测位置 = 当前y_norm + 滤波纵向速度 × lookahead_s
位置误差 = catch_y_norm - 预测位置
目标速度 = kp × 有效位置误差 + kd × 向上视线速度
```

- `kp`负责判断还需要追多远。
- `kd`在当前实现中是视线速度前馈，负责判断现在需要追多急。
- `lookahead_s`使用滤波后的匀速外推，不是卡尔曼滤波。
- 误差超过 `fast_error_norm`时请求 `fast_vx`。
- 普通漏检在 `dropout_hold_ms`内保持上一指令。
- 小球从顶部区域快速出界时，可按 `overtop_vx`继续冲刺 `overtop_hold_ms`。
- 当前状态机退出TRACKING仍以X侧 `lost_timeout_ms`为准，因此Y侧300 ms超时不能单独延长TRACKING。

## 10. X/Y麦轮速度包络

自动平移保留麦轮菱形包络：

```text
abs(vx) + abs(vy) <= 1.40
```

当前 `auto_translation_priority_y: false`。X/Y请求总和超过1.40时按相同比例缩放，保留请求方向。例如X请求0.50、Y请求1.40，实际约为X 0.368、Y 1.032。该设计让每个方向尽可能使用底盘允许的轮速边界，不追求圆形包络下各方向车体速度相同。

## 11. W航向保持

每次ARM时记录当前yaw为本轮参考方向。BNO085数据新鲜时，PD控制器输出 `wz` 抑制自动追球过程中新产生的偏航。IMU为当前ARM必要条件，但不用于二维位置定位。

## 12. 状态机与安全

| 状态 | 行为 |
| --- | --- |
| BOOT | 初始化，禁止运动 |
| SAFE | 安全待机 |
| MANUAL | 手机摇杆控制，依赖心跳 |
| ARMED | 设置yaw参考并等待目标 |
| TRACKING | 执行X、Y和W闭环 |
| AUTO_MOVE | 保留自动运动状态 |
| SUCCESS | 接球触点触发并STOP |
| BLOCKED | LD14触发并STOP |
| FAULT | 故障禁止运动 |
| SHUTTING_DOWN | 重复STOP后关机 |

安全规则：

- 网页STOP、设置页打开、网络切换、BLOCKED和关机均清零运动并发送STOP。
- 手动心跳超时回SAFE。
- ESP32超过500 ms未收到有效控制命令时重复发送STOP。
- LD14可在网页临时旁路，但切换前先STOP并回SAFE。
- 高速测试前必须验证方向、STOP、串口和传感器状态。

## 13. 配置机制

- 正常服务只读取 `config/local.yaml`。
- 网页“应用”修改当前进程内存配置，不写文件。
- 网页“保存”只更新允许编辑的 `vision`、`lateral_tracking`、`imu`、`oac`分区。
- 保存使用临时文件、`fsync`和 `os.replace`原子替换，并生成 `config/local.yaml.bak`。

## 14. 当前配置参数详细解读

| 参数 | 当前公开值 | 作用与调参影响 |
| --- | --- | --- |
| `server.host` | `0.0.0.0` | Flask监听地址；0.0.0.0允许同网段终端访问。 |
| `server.port` | `8080` | 网页/API端口。 |
| `server.preview_fps` | `24` | MJPEG预览上限；降低可减少CPU和Wi-Fi负载，不改变后台检测频率。 |
| `server.preview_width` | `640` | 预览最大宽度；旋转后的竖屏画面保持比例。 |
| `server.preview_height` | `360` | 保留参数；当前视频代码主要按最大宽度等比例缩放。 |
| `server.jpeg_quality` | `72` | 预览JPEG质量。 |
| `system.shutdown_stop_repeat` | `5` | 关机前重复STOP次数。 |
| `system.shutdown_stop_interval_s` | `0.05` | 重复STOP间隔。 |
| `system.shutdown_led_s` | `1.5` | 关机状态灯显示后执行poweroff的等待时间。 |
| `camera.device` | `/dev/video0` | AR0234 V4L2设备。 |
| `camera.width` | `640` | 物理采集宽度；旋转后显示宽高互换。 |
| `camera.height` | `480` | 物理采集高度。 |
| `camera.fps` | `120` | 请求采集FPS，实际值由设备和处理链决定。 |
| `camera.fourcc` | `MJPG` | 采集格式。 |
| `camera.flip_horizontal` | `True` | 旋转画面后的水平镜像。 |
| `camera.flip_vertical` | `False` | 旋转画面后的垂直镜像。 |
| `vision.detector` | `yolo` | 当前检测器，正式基线为yolo。 |
| `vision.yolo_library` | `yolo/bin/libyolo_ball.so` | NPU共享库路径。 |
| `vision.yolo_model` | `yolo/models/yolov5n_ball_uint8.nb` | 当前NBG模型路径。 |
| `vision.yolo_fallback_to_hsv` | `False` | YOLO异常时是否自动切到HSV；当前false，避免无意改变检测行为。 |
| `vision.h_center` | `18` | HSV色相中心，仅HSV模式使用。 |
| `vision.s_center` | `180` | HSV饱和度中心。 |
| `vision.v_center` | `220` | HSV亮度中心。 |
| `vision.h_range` | `10` | 色相容差。 |
| `vision.s_range` | `70` | 饱和度容差。 |
| `vision.v_range` | `70` | 亮度容差。 |
| `vision.min_area` | `90` | 候选框/轮廓最小面积；YOLO封装也使用面积过滤。 |
| `vision.max_area` | `50000` | 最大面积。 |
| `vision.min_circularity` | `0.55` | HSV最小圆度。 |
| `vision.morphology_kernel` | `5` | HSV开闭运算核。 |
| `vision.sample_radius` | `4` | 网页取色采样半径。 |
| `vision.draw_mask_preview` | `False` | HSV掩膜预览。 |
| `motion.vx_output_sign` | `1.0` | Y前后命令到Mode 6前的方向符号。 |
| `motion.vy_output_sign` | `-1.0` | X横移命令方向符号。 |
| `motion.wz_output_sign` | `-1.0` | 旋转命令方向符号。 |
| `motion.auto_translation_limit` | `1.4` | 麦轮菱形包络上限，满足 abs(vx)+abs(vy)<=1.40。 |
| `motion.auto_translation_priority_y` | `False` | false时X/Y超限同比缩放；true时Y优先占用额度。当前false。 |
| `manual.send_hz` | `50` | 3W串口控制循环目标频率。 |
| `manual.max_translation_mps` | `1.4` | 手动平移协议满量程参考，并非已标定m/s。 |
| `manual.max_rotation_radps` | `1.0` | 手动旋转上限。 |
| `manual.heartbeat_timeout_ms` | `450` | 手动页面心跳超时后回SAFE。 |
| `manual.takeover_joystick_deadband` | `0.08` | 摇杆接管自动状态的幅度阈值。 |
| `manual.takeover_throttle_min` | `0.03` | 接管所需最低油门。 |
| `lateral_tracking.enabled` | `True` | 启用X横移。 |
| `lateral_tracking.kp` | `0.35` | 水平位置误差比例增益。 |
| `lateral_tracking.kd` | `0` | 误差变化阻尼；当前速度优先基线为0。 |
| `lateral_tracking.center_deadband_norm` | `0` | 中心死区；当前0使轻微偏差也动作。 |
| `lateral_tracking.error_filter_alpha` | `0.3` | X/Y观测共用的一阶滤波系数，越大越跟随新测量。 |
| `lateral_tracking.min_vy` | `0.2` | 非零动作的最低X速度。 |
| `lateral_tracking.max_vy` | `0.5` | X请求上限。 |
| `lateral_tracking.fast_error_norm` | `0.1` | 进入X高速区的误差阈值。 |
| `lateral_tracking.fast_vy` | `0.5` | X高速区请求。 |
| `lateral_tracking.max_accel_vy_per_s` | `2.5` | X指令每秒最大变化量。 |
| `lateral_tracking.acquire_confirm_frames` | `1` | 进入有效跟踪所需连续帧数。 |
| `lateral_tracking.lost_timeout_ms` | `250` | BallTracker有效窗口，同时决定TRACKING何时退回ARMED。 |
| `lateral_tracking.image_error_to_body_y_sign` | `1.0` | 图像水平误差到车体横移方向。 |
| `oac.enabled` | `True` | 启用Y预测式OAC。 |
| `oac.kp` | `2.0` | 预测位置与接球线之间的位置反馈增益。 |
| `oac.kd` | `0.8` | 当前实现中为向上视线速度前馈，不是传统微分项。 |
| `oac.catch_y_norm` | `0.72` | 画面纵向接球参考线。 |
| `oac.lookahead_s` | `0.18` | 按滤波视线速度做匀速外推的前视时间。 |
| `oac.position_deadband_norm` | `0.03` | 接球线位置死区。 |
| `oac.fast_error_norm` | `0.11` | 预测位置误差达到该值时直接请求高速区。 |
| `oac.min_vx` | `0.75` | 非零Y动作的最低速度。 |
| `oac.max_vx` | `1.4` | Y请求上限。 |
| `oac.rate_filter_alpha` | `0.7` | 视线速度滤波系数。 |
| `oac.min_upward_rate` | `0.005` | 判定明显向上运动的最小速率。 |
| `oac.fast_vx` | `1.4` | Y高速区速度。 |
| `oac.max_accel_vx_per_s` | `20` | Y指令变化率上限。 |
| `oac.lost_timeout_ms` | `300` | OAC内部允许使用观测的最大年龄；但TRACKING退出仍由X侧lost_timeout决定。 |
| `oac.output_sign` | `1.0` | Y前后方向。 |
| `oac.dropout_hold_ms` | `150` | 普通短时漏检保持上一Y指令。 |
| `oac.overtop_enabled` | `True` | 允许小球从顶部出界后续冲。 |
| `oac.overtop_y_norm` | `0.18` | 顶部区域阈值。 |
| `oac.overtop_hold_ms` | `300` | 顶部续冲时间。 |
| `oac.overtop_vx` | `1.4` | 顶部续冲速度。 |
| `transport.serial_device` | `/dev/ttyS7` | 3W到ESP32 UART7设备。 |
| `transport.baud` | `115200` | 串口波特率。 |
| `transport.reconnect_s` | `1.0` | 串口失败重连间隔。 |
| `imu.timeout_ms` | `500` | IMU新鲜度窗口。 |
| `imu.required_for_arm` | `True` | 无新鲜IMU时拒绝ARM。 |
| `imu.yaw_hold_enabled` | `True` | 自动状态启用相对航向保持。 |
| `imu.yaw_deadband_deg` | `1.5` | 航向死区。 |
| `imu.yaw_kp_radps_per_deg` | `0.025` | 航向P。 |
| `imu.yaw_kd_radps_per_dps` | `0.006` | 航向角速度阻尼。 |
| `imu.yaw_max_wz_radps` | `0.45` | 自动纠偏角速度上限。 |
| `imu.yaw_sign` | `1.0` | 纠偏方向。 |
| `lidar.enabled` | `True` | 启动LD14读取线程。 |
| `lidar.control_enabled` | `True` | 是否参与BLOCKED控制；网页可临时旁路。 |
| `lidar.required` | `False` | 是否为系统启动必需。 |
| `lidar.direct_serial_device` | `/dev/ttyS6` | LD14 UART6。 |
| `lidar.baud` | `115200` | LD14波特率。 |
| `lidar.active_modes` | `['ARMED', 'TRACKING', 'AUTO_MOVE']` | 会触发BLOCKED的状态。 |
| `lidar.sdk_angle_correction` | `True` | 启用SDK几何角度修正。 |
| `lidar.angle_offset_deg` | `174.0` | 雷达角度到车体前方的安装偏移。 |
| `lidar.sector_center_deg` | `0.0` | 保护扇区中心。 |
| `lidar.sector_width_deg` | `90.0` | 保护扇区宽度。 |
| `lidar.ignore_sectors_deg` | `[[120.0, 180.0], [-180.0, -120.0]]` | 忽略扇区。 |
| `lidar.scan_bin_deg` | `1.0` | 扫描缓存角度桶。 |
| `lidar.point_max_age_s` | `0.45` | 点云有效年龄。 |
| `lidar.offline_timeout_s` | `0.6` | 雷达离线判定。 |
| `lidar.min_confidence` | `10` | 最小反射置信度。 |
| `lidar.stop_distance_m` | `0.25` | 停车距离。 |
| `lidar.release_distance_m` | `0.3` | 解除阈值，形成滞回。 |
| `lidar.stop_confirm_frames` | `2` | 连续阻挡确认帧。 |
| `lidar.release_confirm_frames` | `3` | 连续安全确认帧。 |
| `io.enabled` | `True` | 启用GPIO按钮。 |
| `io.arm_line_name` | `PE4` | ARM/SAFE按钮GPIO。 |
| `io.catch_line_name` | `PL4` | 接球触点/测试按钮GPIO。 |
| `io.active_low` | `True` | 低电平有效。 |
| `io.debounce_ms` | `60` | 轮询和消抖周期。 |
| `io.network_switch_hold_s` | `3.0` | 接球按钮长按切换网络。 |
| `io.shutdown_hold_s` | `3.0` | ARM按钮长按关机。 |
| `ws2812.enabled` | `True` | 启用状态灯。 |
| `ws2812.count` | `11` | 总灯数11。 |
| `ws2812.front_count` | `3` | 前三颗灯。 |
| `ws2812.tail_count` | `8` | 尾部8颗。 |
| `ws2812.spi_bus` | `3` | SPI总线。 |
| `ws2812.spi_cs` | `0` | SPI片选设备号。 |
| `ws2812.spi_mode` | `0` | SPI模式。 |
| `ws2812.bits_per_word` | `8` | SPI字长。 |
| `ws2812.spi_speed_hz` | `2400000` | WS2812编码时钟。 |
| `ws2812.no_cs` | `False` | 当前已验证为false。 |
| `ws2812.reset_bytes` | `32` | 帧尾复位低电平长度。 |
| `ws2812.brightness` | `30` | 全局亮度；过高可能影响相机曝光。 |
| `ws2812.refresh_hz` | `8` | 灯效刷新频率。 |
| `ws2812.boot_color` | `[0, 80, 255]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.wifi_color` | `[0, 80, 255]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.ap_color` | `[150, 40, 255]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.safe_color` | `[255, 150, 0]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.manual_color` | `[0, 90, 255]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.armed_color` | `[0, 255, 80]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.tracking_color` | `[0, 220, 255]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.auto_color` | `[0, 255, 40]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.blocked_color` | `[255, 70, 0]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.success_color` | `[255, 255, 255]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.fault_color` | `[255, 0, 0]` | 配置项；以当前代码和实机验证为准。 |
| `ws2812.shutdown_color` | `[255, 190, 120]` | 配置项；以当前代码和实机验证为准。 |
| `network.auto_manage` | `True` | 程序自动管理Wi-Fi/热点。 |
| `network.wifi_connection_name` | `<YOUR_WIFI_CONNECTION_NAME>` | NetworkManager家庭Wi-Fi连接名，公开配置必须使用占位符。 |
| `network.ap_connection_name` | `<YOUR_AP_CONNECTION_NAME>` | 热点连接名。 |
| `network.ap_ssid` | `<YOUR_AP_SSID>` | 热点SSID，公开配置使用占位符。 |
| `network.ap_password` | `<YOUR_AP_PASSWORD>` | 热点密码，禁止提交真实值。 |
| `network.ap_address` | `192.168.x.1/24` | 热点网段。 |
| `network.ready_confirm_s` | `1.0` | 网络成功灯效时间。 |
| `network.error_confirm_s` | `2.0` | 失败提示时间。 |
| `battery.enabled` | `True` | 显示ESP32 ADC电池数据。 |
| `battery.empty_voltage` | `9.6` | 0%电压参考，必须按电池实测校准。 |
| `battery.full_voltage` | `12.45` | 100%电压参考。 |
| `battery.filter_window` | `5` | 中值滤波窗口。 |
| `battery.filter_alpha` | `0.12` | 低通滤波系数。 |
| `battery.low_percent` | `20` | 低电量提示阈值。 |
| `catch.enabled` | `True` | 启用接球触点。 |
| `catch.valid_modes` | `['ARMED', 'TRACKING', 'AUTO_MOVE']` | 允许触发SUCCESS的状态。 |
| `future_servo.enabled` | `False` | 遗留兼容配置；当前实际X/Y控制由lateral_tracking和oac决定。 |

## 15. 硬件接口摘要

- `/dev/video0`：AR0234。
- `/dev/ttyS7`：Zero 3W与ESP32双向UART，115200 8N1；物理Pin 16 TX、Pin 18 RX、Pin 14 GND。
- `/dev/ttyS6`：LD14单向UART，115200；物理Pin 23 RX、Pin 39 GND。
- `/dev/spidev3.0`：WS2812数据，物理Pin 19 MOSI。
- Pin 26 / PE4：ARM/SAFE按钮，低有效。
- Pin 28 / PL4：接球测试/网络切换按钮，低有效。
- Pin 4：Zero 3W主5V输入；Pin 2为附件5V分配点；系统必须共地。
- WS2812当前为11颗：前三颗状态灯加尾部8颗。

## 16. 部署与运行

项目目录示例：

```bash
/home/orangepi/projects/100percenthit
```

安装依赖并启用服务后，systemd使用：

```text
/home/orangepi/projects/100percenthit/.venv/bin/python -m app.main --config config/local.yaml
```

常用命令：

```bash
sudo systemctl status percenthit.service --no-pager
sudo systemctl restart percenthit.service
sudo systemctl stop percenthit.service
sudo journalctl -u percenthit.service -b -n 200 --no-pager
```

## 17. ARMTRACE前台调试

正常 `percenthit.service`不启用ARMTRACE，也不会创建ARMTRACE日志文件。需要诊断一次抛球时，先停止服务，再以前台root权限运行：

```bash
sudo systemctl stop percenthit.service
cd /home/orangepi/projects/100percenthit
sudo .venv/bin/python -m app.main \
  --config config/local.yaml \
  --arm-trace 2>&1 | \
  grep --line-buffered '^\[ARMTRACE\]'
```

输出字段：

- `detected`：当前帧是否检测到球。
- `valid`：BallTracker观测是否仍有效。
- `age_ms`：距最后一次检测的时间。
- `ball=(x_error,y_norm)`：滤波后的图像位置。
- `Xreq/Yreq`：X、Y控制器原始请求。
- `OUT_X/OUT_Y`：菱形包络后的实际输出。
- `limited`：是否触发包络限幅。
- `scale`：同比缩放比例。

测试结束按 `Ctrl+C`，再恢复服务：

```bash
sudo systemctl start percenthit.service
```

## 18. 网络与多客户端注意事项

- 程序启动优先连接配置的Wi-Fi，失败后启用3W热点。
- 手机和电脑同时打开3W页面时，两个视频/状态客户端会增加3W负载和无线发送竞争；实测中关闭电脑页面后手机摇杆恢复顺畅。
- ESP32同时保留Windows UDP入口和3W UART入口，但不得让两个控制源同时主动控制底盘。

## 19. 已知限制与后续工作

1. 当前只使用最高置信度候选，尚未基于历史轨迹完成多目标身份关联。
2. X/Y共享BallTracker滤波和TRACKING超时，Y内部300 ms超时不能独立维持状态。
3. Y轴算法基本锁定，但接球线、前视时间和速度分配仍需微调。

## 20. 当前基线结论

截至本基线，Orange Pi Zero 3W已经形成可运行的车载视觉伺服接球链路。X轴快速响应基本满足任务需要，Y轴预测式OAC已进入实测微调阶段；在照明充足且YOLO连续捕捉小球的情况下，系统已经具备实际命中能力。