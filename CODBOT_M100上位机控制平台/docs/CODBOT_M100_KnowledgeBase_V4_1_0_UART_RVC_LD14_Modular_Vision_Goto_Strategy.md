# CODBOT M100 平台开发知识库 V4.1.0

**项目：百分百命中 / 100 Percent Hit**  
**版本：V4.1.0 Modular + UART-RVC + LD14 + Vision GoTo**  
**更新日期：2026-08-30**  
**文档格式：Markdown**

---

## 0. 文档定位

本文档是《百分百命中》CODBOT M100 平台的当前完整工程知识库，用于在未来恢复项目上下文、复现硬件接线、核对通信协议、排查传感器与控制问题，并继续开发自动接球闭环。

V4.1.0 在 V4.0.9 基础上完成以下关键升级：

1. 保留并整理 V4.0-V4.0.9 已验证的 M100 底盘、STM32、Mode 0 PID、Mode 6、UDP桥、BNO085 UART-RVC、电池ADC、视觉定位、GoTo、RCPD、AprilTag门控与方向走廊知识。
2. 将 M100 Control Center 固化为稳定应用壳，界面升级为四个Tab，并把高变化功能拆分为独立模块。
3. 集成 LD14 激光雷达。LD14 使用 ESP32 硬件 UART0，RX 重映射到 GPIO32，原始有效帧通过独立 UDP 4212 发往PC。
4. 新增 Tab3 LD14 Radar、点云显示、在线状态、雷达保护性停止及可配置安全参数；原日志页后移为Tab4。
5. 明确 LD14 是可选传感器。雷达移除、无信号或功能关闭时，默认不影响M100的基本控制、BNO085、ADC、视觉GoTo和断联STOP。
6. 增加连续GoTo目标管理方向，为LandingNet落点逐步收敛、动态目标更新和自动接球状态机预留接口。

本文档强调三类结论：

- **已验证事实**：当前硬件接线、协议、已经实车运行的功能。
- **工程约束**：后续开发必须保持的接口、安全链路和坐标定义。
- **建议初值/待实测项**：尚未通过完整场景重复验证的参数，不应当作最终标定值。

---

## 1. 项目目标与当前里程碑

### 1.1 最终目标

构建完整自动接球机器人系统：

1. 工业摄像头检测空中飞行小球。
2. 连续跟踪小球并记录轨迹。
3. 图像坐标统一转换到世界坐标。
4. 几何方法或 LandingNet 预测落点。
5. 将预测落点转为M100动态目标点。
6. M100在视觉定位和安全门控下快速到达落点。
7. 完成感知、预测、决策、控制、执行和验证的实时闭环。

### 1.2 V4.1.0当前状态

| 模块 | 状态 | 当前结论 |
|---|---|---|
| M100底盘 | 已完成可用 | 四轮电机、编码器、TB6612和麦轮运动链路可用 |
| STM32固件 | 已完成可用 | 保留Mode 0 PID与Mode 6运动控制，负责底层闭环 |
| ESP32 UDP桥 | 已完成可用 | UDP 4210/4211，UART2连接STM32，500ms断联STOP保留 |
| BNO085 | 已切换并验证 | UART-RVC，GPIO26单向接收，姿态与加速度可用 |
| 电池ADC | 已完成可用 | GPIO34，38700Ω/9660Ω分压，上位机滤波显示 |
| LD14 | 已完成集成并实车验证 | GPIO32硬件UART0接收，UDP 4212，Tab3点云可用于室内盲开辅助 |
| M100 Control Center | V4.1.0 | 四Tab模块化应用壳，LD14与高变化功能外置 |
| 场地定位 | 已完成 | ArUco 0/1/2/3建立并锁定Homography |
| 小车视觉定位 | 已完成 | AprilTag 36h11 ID10逐帧输出x/y/yaw |
| 静态GoTo | 主链路完成 | world error转body velocity后发送Mode 6 |
| GoTo策略 | 可用并持续调参 | P、Max Then Slow、PD、Brake Distance、Bang-Bang、RCPD |
| AprilTag门控/方向走廊 | 已形成方案 | 参数仍需低速到高速逐步验证 |
| 连续GoTo | 已预留模块方向 | 目标更新不能反复重置Launch/速度估计 |
| 自动接球 | 继续开发 | LandingNet输出到动态目标管理器是下一主线 |

### 1.3 已验证的新能力

V4.1.0中，LD14雷达已经能够稳定工作。实车可以仅依赖Tab3雷达点云和人工操控在房间内穿行。这证明以下链路已贯通：

`LD14 UART → ESP32 UART0/GPIO32 → CRC有效帧 → UDP 4212 → PC LidarService → Tab3点云显示`

该结果只证明当前雷达显示与人工辅助驾驶链路可用，不代表自动导航、SLAM或防撞参数已经完成最终标定。

---

## 2. 系统总体架构与分层职责

### 2.1 当前总体架构

```text
工业摄像头 / AR0234
        |
        v
PC: Python / OpenCV / AprilTag / LandingNet / M100 Control Center V4.1.0
        |
        | UDP 4210: Mode 6 / Mode 0 / STOP / PING
        v
ESP32 Bridge
  - UDP <-> STM32 UART2
  - BNO085 UART-RVC / UART1
  - LD14 / UART0 RX remap
  - Battery ADC
  - PC command timeout STOP
        |
        | UART2 115200
        | GPIO17 TX2 -> STM32 RX
        | GPIO16 RX2 <- STM32 TX
        v
STM32F103RET6 / CODBOT控制板
        |
        | Mode协议 + 麦轮运动学 + PID + PWM + Encoder
        v
TB6612 x2 -> 4个520编码器电机 -> M100麦克纳姆轮底盘

ESP32 -> PC:
  UDP 4211: STM:, IMU, ADC, ESPSTAT
  UDP 4212: "LD14:" + 47字节LD14原始有效帧
```

### 2.2 分层责任

| 层级 | 应承担的职责 | 不应承担的职责 |
|---|---|---|
| PC | UI、视觉、LandingNet、目标管理、GoTo、策略、日志、雷达显示、安全仲裁 | 不直接产生电机PWM |
| ESP32 | UDP透明桥、串口转发、BNO085、LD14、ADC、通信断联STOP | 不做视觉定位、路径规划、GoTo和底盘PID |
| STM32 | Mode协议、麦轮运动学、编码器闭环、PID、PWM | 不做相机、LandingNet和雷达点云UI |

### 2.3 核心架构原则

1. PC是高层控制与可视化中心。
2. ESP32保持桥接与传感器汇聚角色，不把高层决策下沉。
3. STM32保持稳定底层执行层，不因LD14或UI调整而频繁修改。
4. 运动输出必须集中经过一个主发送入口，使E-STOP、视觉门控、雷达STOP和通信STOP可以统一仲裁。
5. 传感器接收与UI绘图解耦。隐藏Tab时仍更新数据与安全状态，但不需要高频重画Canvas。

---

## 3. 硬件与当前最终接线

### 3.1 已确认硬件

| 硬件 | 当前信息 | 项目作用 |
|---|---|---|
| 底盘 | CODBOT M100麦克纳姆轮 | 快速平移、横移、旋转 |
| 主控 | STM32F103RET6 | 底层Mode与轮速闭环 |
| 电机 | 520编码器电机 | 四轮Actual反馈 |
| 轮子 | 80mm麦克纳姆轮 | 全向运动 |
| 驱动 | TB6612双H桥x2 | 四轮功率驱动 |
| 无线与传感器桥 | ESP32 | UDP、UART、ADC和断联STOP |
| 姿态 | BNO085 | UART-RVC姿态与加速度 |
| 雷达 | LDROBOT LD14 | 360度二维点云、人工辅助驾驶、防撞输入 |
| 相机 | AR0234 USB工业摄像头 | 场地、小车与小球视觉 |

### 3.2 STM32与ESP32

| STM32 CN3 | ESP32 | 方向 | 说明 |
|---|---|---|---|
| GND | GND | 公共地 | 必须共地 |
| UART_TX / PA9 | GPIO16 / RX2 | STM32 → ESP32 | 反馈帧与PID数据 |
| UART_RX / PA10 | GPIO17 / TX2 | ESP32 → STM32 | Mode 6与Mode 0命令 |

串口：`115200, 8N1, no parity`。

### 3.3 BNO085 UART-RVC

| BNO085 | ESP32/状态 | 说明 |
|---|---|---|
| VCC | 3.3V | 当前优先供电方式 |
| GND | GND | 与ESP32/STM32共地 |
| SDA / UART-RVC TX | GPIO26 / UART1 RX | 单向RVC数据流 |
| SCL | 悬空 | UART-RVC不使用 |
| PS0 | 3.3V | 当前模块RVC模式选择 |
| PS1 | GND | 当前模块RVC模式选择 |
| UART-RVC RX | 不接 | 程序只接收 |

修改PS0/PS1或接线后，应让BNO085与ESP32整套完全断电再上电。

### 3.4 LD14

| LD14 | ESP32 | 说明 |
|---|---|---|
| P5V | 稳定5V | 手册范围4.5V至5.5V |
| GND | GND | 与ESP32、STM32共地 |
| TX | GPIO32 | ESP32硬件UART0 RX，115200 |
| PWM | 悬空或接地 | 使用LD14内部控速，默认约6Hz |

运行期UART分配：

| UART | RX/TX | 用途 |
|---|---|---|
| UART0 | RX=GPIO32，TX=GPIO1 | LD14输入；GPIO1保留USB串口诊断输出 |
| UART1 | RX=GPIO26，TX=GPIO27未接 | BNO085 UART-RVC |
| UART2 | RX=GPIO16，TX=GPIO17 | STM32透明桥 |

注意：UART0 RX重映射后，USB串口监视器仍可接收GPIO1输出，但PC通过USB向程序发送串口输入不再进入运行期UART0 RX。当前程序不依赖USB串口输入，因此无实际影响。

### 3.5 电池分压检测

| 项目 | 当前值/连接 |
|---|---|
| ADC引脚 | GPIO34 |
| 上拉电阻 | 电池+ → 38700Ω → GPIO34节点 |
| 下拉电阻 | GPIO34节点 → 9660Ω → GND |
| 建议滤波 | GPIO34节点 → 100nF → GND |
| ADC配置 | 12位，ADC_11db |
| 换算 | `Vbat = Vadc × (Rtop + Rbottom) / Rbottom` |

这里的“电阻检测”是通过实测电阻值建立分压比例，再由ESP32 ADC读取电压。上位机显示还应保留低通滤波、百分比滞回和与万用表校验。

---

## 4. STM32固件、Mode系统与反馈

### 4.1 STM32关键源文件

| 文件 | 职责 | 项目关注点 |
|---|---|---|
| `main.c` | 系统入口、任务、Mode选择 | 项目主要使用Mode 0和Mode 6 |
| `motor.c` | 麦轮运动学、目标轮速、PID到PWM | X/Y/W到四轮映射 |
| `pid.c` | 速度PID | 当前默认1.25/0.76/0.25 |
| `feedback.c` | FD反馈帧封包 | 车速、轮速、电池、IMU、车型 |
| `comm.c` | Mode 0 KDCOM/PID协议 | C命令帧与0/1/2/3数据帧 |
| `autocontrol.c` | Mode 6串口控制 | 解析`CD 0A 01`并调用运动控制 |
| `bsp_uart.c` | UART、中断与缓存 | PA9/PA10链路 |
| `bsp_encoder.c` | 编码器测速 | 四轮Actual来源 |
| `bsp_adc.c` | 板载ADC | 原板电压反馈来源 |

### 4.2 Mode取舍

| Mode | 原功能 | 本项目处理 |
|---|---|---|
| 0 | 蓝牙/PID相关 | 保留作为PID调试协议 |
| 1 | PS2遥控 | 非主线 |
| 2-5 | 巡线/超声/CCD | 不作为当前入口 |
| 6 | 串口控制 | 视觉闭环、手动控制、GoTo主模式 |

### 4.3 STM32反馈协议

STM32到上位机方向由ESP32以`STM:`前缀透传原始字节。主要反馈：

| 帧 | 长度 | 含义 |
|---|---:|---|
| `FD 0A 02` | 13 | 底盘vx/vy/wz反馈 |
| `FD 09 03` | 12 | 四轮速度反馈 |
| `FD 02 04` | 5 | 电池电压反馈 |
| `FD 0A 05` | 13 | Pitch/Roll/Yaw反馈 |
| `FD 1F 06` | 34 | 原始IMU/四元数反馈 |
| `FD 02 07` | 5 | 底盘型号 |

反馈XOR规则与控制帧一致：从功能位到最后一个数据字节逐字节异或，再与末尾校验字节比较。

### 4.4 四轮物理映射

| 通道 | 物理轮 | UI位置 |
|---|---|---|
| CH1 | LF 左前 | 左上 |
| CH2 | LR 左后 | 左下 |
| CH3 | RF 右前 | 右上 |
| CH4 | RR 右后 | 右下 |

UI绘制顺序通常为LF、RF、LR、RR，但协议通道顺序是CH1、CH2、CH3、CH4。

---

## 5. Mode 6运动控制协议

### 5.1 帧格式

固定13字节：

`CD 0A 01 X_DIR X_H X_L Y_DIR Y_H Y_L W_DIR W_H W_L XOR`

| 字节 | 字段 | 说明 |
|---:|---|---|
| 0 | `0xCD` | PC到STM32帧头 |
| 1 | `0x0A` | 功能位+数据位长度 |
| 2 | `0x01` | 运动控制功能 |
| 3 | X方向 | 0正向，1反向 |
| 4-5 | X速度 | mm/s，高字节在前 |
| 6 | Y方向 | 0左平移，1右平移 |
| 7-8 | Y速度 | mm/s，高字节在前 |
| 9 | W方向 | 0左转，1右转 |
| 10-11 | W值 | 0.01rad/s，高字节在前 |
| 12 | XOR | `XOR(frame[2:12])` |

上位机换算：

- `vx_mps × 1000`
- `vy_mps × 1000`
- `wz_radps × 100`

### 5.2 STOP帧

`CD 0A 01 00 00 00 00 00 00 00 00 00 01`

### 5.3 安全与单位约束

1. Mode 6中的vx/vy是车体坐标，不是世界坐标。
2. `MAX_TARGET=1400`是当前底层单轮100% Target基准，不是标定后的精确1.4m/s。
3. 平移使用麦轮菱形包络：`abs(vx)+abs(vy)`超过上限时同比例缩放。
4. 控制频率当前常用20Hz；提高频率前先确认视觉FPS、网络延迟和主循环负载。
5. 高速测试必须保留E-STOP和500ms断联STOP。

---

## 6. Mode 0 PID协议

### 6.1 命令帧

固定10字节：

| 字节 | 内容 |
|---:|---|
| 0 | ASCII `C` |
| 1 | CMD |
| 2 | RW，0读、1写 |
| 3 | 通道0..3 |
| 4 | 保留，当前为0 |
| 5..8 | little-endian float32 |
| 9 | LF `0x0A` |

### 6.2 命令定义

| CMD | 功能 |
|---:|---|
| 1 | P |
| 2 | I |
| 3 | D |
| 4 | Target |
| 6 | Actual |
| 7 | Start |
| 8 | Stop |

### 6.3 数据帧与已知兼容问题

- 数据帧头ASCII `0/1/2/3`对应四个通道。
- 当前默认PID：`P=1.25, I=0.76, D=0.25`。
- `raw 1400 = 100% Target`。
- 卖家固件的C帧回复中byte2/byte3可能不能可靠表示原请求通道，因此上位机使用`pending_read`映射最后请求的命令/通道。
- Mode 0与Mode 6应通过主程序模式保护避免混发。

---

## 7. ESP32 V4.1.0桥接程序

### 7.1 当前运行文件

当前已验证方向的文件名：

`M100_ESP32_UDP_Bridge_BNO085_UART_RVC_Debug_Fix_LD14_UART0_Diagnostic_v4.ino`

建议后续固化为不含`Diagnostic`字样的正式版本，但在固化前应保留每秒一次的传感器计数诊断开关。

### 7.2 UDP端口

| 端口 | 方向 | 数据 |
|---:|---|---|
| 4210 | PC → ESP32 | Mode 6、Mode 0、STOP、PING |
| 4211 | ESP32 → PC | `STM:`、`IMU,`、`ADC,`、`ESPSTAT,` |
| 4212 | ESP32 → PC | `LD14:` + 47字节原始有效雷达帧 |

必须先由PC向4210发送PING或控制包，让ESP32记录`pcIP`并设置`hasPC=true`，之后4211和4212才能发往正确PC。

### 7.3 断联STOP

- `COMMAND_TIMEOUT_MS = 500`
- 超时后按`STOP_REPEAT_MS = 100`重复向STM32发送STOP。
- LD14无信号不应修改`lastCommandMs`，也不应触发或阻塞这条链路。

### 7.4 传感器统计

建议保留：

- BNO085：`rvc_bytes/rvc_headers/rvc_ok/rvc_bad/bno`
- LD14：`ld_bytes/ld_h54/ld_h542c/ld_ok/ld_bad/lidar`
- 通信：`udp_rx/uart_tx/last_len/wifi_rssi/hasPC/pcIP`

高频逐字节打印会干扰实时性，正式版只保留低频状态行或ESPSTAT遥测。

---

## 8. BNO085 UART-RVC

### 8.1 RVC帧

固定19字节：

`AA AA index YawL YawH PitchL PitchH RollL RollH AccXL AccXH AccYL AccYH AccZL AccZH ReservedL ReservedH Checksum`

换算：

- Yaw/Pitch/Roll：`int16_le / 100.0`度。
- AccX/Y/Z：`int16_le / 1000.0 × 9.80665`，单位m/s²。

### 8.2 checksum Debug Fix

正确范围：

```cpp
uint8_t sum = 0;
for (int i = 2; i < 18; ++i) {
    sum = (uint8_t)(sum + p[i]);
}
return sum == p[18];
```

两个`0xAA`帧头不参与校验。旧版若从`i=0`开始，会出现有字节但`rvc_ok=0`和UI zero pending。

### 8.3 与旧I2C实现的差异

UART-RVC不提供旧UI期待的独立陀螺仪XYZ。为保持遥测字段兼容，当前gx/gy/gz可填0。影响：

- RVC Yaw/Pitch/Roll可用。
- 旧的Gyro yaw积分对比不再是有效数据源。
- GoTo二维位置和航向仍以AprilTag为主，BNO085不能替代视觉世界坐标定位。

### 8.4 V4.0.9/V4.1.0显示映射

Zero后显示映射：

```python
yaw_mag = wrap_deg(yaw_raw - att_zero_yaw)
pitch = wrap_deg(roll_raw - att_zero_roll)
roll = -wrap_deg(pitch_raw - att_zero_pitch)
```

该映射只作用于姿态显示，不应修改AprilTag yaw、world-to-body、GoTo方向或Mode 6定义。

### 8.5 G-Force

RVC加速度包含重力。当前近似水平底盘简化处理：

1. 各轴除以9.81转为G。
2. Z轴扣除符号一致的1g。
3. 计算`gt_raw`。
4. `G_ZERO_DEADBAND=0.06`以内归零。

`0.06G`是建议初值。坡面、抬头或侧倾场景若需准确线性加速度，应使用姿态四元数计算重力投影。

---

## 9. LD14协议、桥接与排障知识

### 9.1 LD14基本接口

- 1.25mm 4PIN：PWM、GND、TX、P5V。
- P5V典型5V。
- TX为3.3V UART输出。
- 单向发送，不需要PC或ESP32向雷达发指令。
- `115200, 8N1, no parity, no flow control`。
- PWM悬空或接地时使用内部控速，默认约6Hz。

### 9.2 数据帧

固定47字节：

| 字段 | 长度 | 说明 |
|---|---:|---|
| Header | 1 | `0x54` |
| VerLen | 1 | `0x2C`，12个点 |
| Speed | 2 | little-endian，度/秒 |
| Start angle | 2 | little-endian，0.01度 |
| 12 points | 36 | 每点2字节距离mm + 1字节强度 |
| End angle | 2 | 0.01度 |
| Timestamp | 2 | ms，最大30000后重计 |
| CRC8 | 1 | 前46字节校验 |

每点角度按起始角与结束角线性插值。跨越360度时必须处理角度回绕。

### 9.3 坐标

LD14使用左手坐标系：

- 传感器正前方为0度。
- 角度顺时针增加。
- Tab3当前以0度向上画为车头方向。
- 安装角度若与车头不一致，应增加可配置的角度偏置，不要在多个模块中分别硬编码。

### 9.4 软件串口失败记录

初版将LD14接GPIO32并使用EspSoftwareSerial。实测出现：

```text
bytes持续快速增加
h54很多
h542c极少
good=0
bad随偶然帧头增加
online=0
```

这说明GPIO有数据，但115200连续流在当前Wi-Fi、BNO085、STM32和ADC并发环境下无法可靠恢复字节。增加缓冲、提高读取预算或修改上层同步均不能解决底层采样错误。

此外，EspSoftwareSerial类不支持HardwareSerial的`setRxBufferSize()`成员，不能调用：

```cpp
LD14Serial.setRxBufferSize(1024);
```

最终方案是放弃软件串口，将LD14切换到硬件UART0 RX并重映射到GPIO32。

### 9.5 UART0方案验证特征

正常日志关系应为：

```text
h54 ≈ h542c
good ≈ h542c
bad远小于good
online=1
```

PC连接后还应为：

```text
hasPC=1
pc=<PC局域网地址>
```

### 9.6 PC端LD14模块

| 模块 | 职责 |
|---|---|
| `lidar/ld14_protocol.py` | 帧同步、CRC、字段解析、角度插值 |
| `lidar/ld14_service.py` | UDP 4212后台接收、快照、在线超时 |
| `lidar/ld14_tab.py` | Tab3极坐标点云和状态显示 |
| `lidar/lidar_collision_guard.py` | 停止区、释放区、置信度、确认帧与锁存 |

数据线程不得直接调用Tkinter控件。Tkinter只能在主线程中读取快照并刷新。

### 9.7 雷达保护性停止

当前设计为可配置功能：

- `collision_stop_enabled=False`作为安全调试默认值。
- 打开后只对配置扇区、距离、置信度和确认帧进行判断。
- STOP距离和RELEASE距离必须分离，形成滞回。
- 雷达离线默认不停车：`stop_on_sensor_loss=False`。
- 若未来启用严格模式，才允许雷达保护已开启且传感器丢失时触发STOP。
- LD14缺失不能成为M100启动条件。

建议初值，均待完整实测：

| 参数 | 建议初值 |
|---|---:|
| STOP距离 | 0.35m |
| RELEASE距离 | 0.45m |
| 前方半角 | 45° |
| 最低强度/置信度 | 10 |
| STOP确认帧 | 2 |
| RELEASE确认帧 | 3 |
| 离线超时 | 0.50s |

### 9.8 当前LD14能力边界

已验证：

- 室内点云显示稳定。
- 实车可以在人工控制下依赖雷达显示辅助穿行。
- 雷达作为可移除模块不破坏主控制链路。

尚未验证为完整功能：

- SLAM建图。
- 自动路径规划。
- 全方向自动避障。
- 高速动态障碍物安全认证。
- 与视觉目标追踪同时运行时的最终防撞参数。

---

## 10. M100 Control Center V4.1.0模块化结构

### 10.1 四Tab布局

| Tab | 内容 |
|---|---|
| Tab1 Control Center | Camera、Map、Position Command、手动控制、状态 |
| Tab2 PID Debug | Mode 0 PID、曲线、四轮状态 |
| Tab3 LD14 Radar | 点云、在线状态、最近距离、防撞配置 |
| Tab4 Logs / Export | 原日志和轨迹导出 |

Tab3视觉要求：保留LD14 viewer的极坐标和点云风格，不使用辉光、不使用扫描线、不提供雷达CSV导出，数据统一为M100普通字体风格。

### 10.2 模块结构

```text
M100_Control_Center_V4_1_0/
├── m100_control_center_v4_1_0.py
├── m100_config.py
├── ui/
│   └── position_command_panel.py
├── goto/
│   ├── goto_strategy_basic.py
│   └── goto_target_manager.py
└── lidar/
    ├── ld14_protocol.py
    ├── ld14_service.py
    ├── ld14_tab.py
    └── lidar_collision_guard.py
```

### 10.3 稳定壳与高变化区

保留在主应用壳：

- `App`生命周期与Notebook。
- SerialHub/UDP 4210和4211。
- STM32解析、Mode 0、Mode 6。
- BNO085、ADC、日志、PID、手柄、键盘和E-STOP。
- 摄像头、ArUco、AprilTag、Homography。
- 唯一运动发送入口与安全仲裁。

拆出：

- LD14协议、服务、Tab和防撞判断。
- Position Command面板。
- GoTo策略模块。
- 连续目标管理器。

### 10.4 初始化顺序问题记录

错误现象：

`_tkinter.tkapp object has no attribute 'lidar_service'`

根因：`build_ui()`创建Tab3时，`self.lidar_service`尚未初始化。

正确顺序：

1. 创建`LidarService`。
2. 创建`LidarCollisionGuard`。
3. 启动LidarService线程。
4. 调用`build_ui()`。
5. `build_tab3()`创建LD14Tab。

关闭时使用`hasattr(self, 'lidar_service')`防御性停止服务。

### 10.5 隐藏Tab策略

- 雷达UDP接收持续运行。
- 防撞判断持续运行。
- 数据模型持续更新。
- Tab3不可见时停止高频Canvas重画。
- 切换到Tab3后立即用最新快照绘制。

---

## 11. 视觉定位与GoTo

### 11.1 场地世界坐标

```python
FIELD_MARKER_WORLD = {
    0: (0.0, 0.0),
    1: (FIELD_WIDTH_M, 0.0),
    2: (FIELD_WIDTH_M, FIELD_HEIGHT_M),
    3: (0.0, FIELD_HEIGHT_M),
}
```

当前场地配置为1.20m × 1.20m。场地、LandingNet、轨迹程序和Control Center必须使用完全相同的世界坐标定义。

### 11.2 Homography

- 四角ArUco 0/1/2/3首次完整识别后计算并锁定H。
- H锁定后停止每帧检测场地ArUco。
- AprilTag ID10保持逐帧检测。
- 相机或场地标记移动后必须重新锁定。

### 11.3 AprilTag

- 类型：36h11。
- ID：10。
- 输出：x、y、yaw、src。
- 标签中心用于车体世界位置。
- 标签上边中点相对中心的世界向量用于估计yaw。

### 11.4 正确控制链路

```text
AprilTag current_x/current_y/current_yaw
        ↓
世界误差 dx/dy
        ↓
策略生成世界坐标期望速度
        ↓
根据AprilTag yaw执行world_to_body
        ↓
车体坐标vx/vy
        ↓
麦轮菱形包络
        ↓
统一安全仲裁
        ↓
Mode 6 → UDP 4210 → ESP32 → STM32
```

### 11.5 螺旋线Bug

早期绕圈、弧线和螺旋靠近目标的根因是把世界误差直接作为Mode 6车体速度发送。以后出现“车头未对准目标时路径弯曲”，第一检查项仍是`world_to_body`。

### 11.6 AprilTag门控与方向走廊

启动自动控制前应检查：

- H已锁定。
- ID10当前有效。
- x/y位于有效场地。
- yaw有效且连续。
- 视觉姿态未过期。
- 世界目标与转换后的车体运动方向合理。
- 标签丢失时显示可短时保留，但不能继续累计运动输出。

走廊阈值必须配置化并先低速实测。

---

## 12. GoTo策略与参数

### 12.1 已保留策略

- `p`
- `max_then_slow`
- `pd_damping`
- `brake_distance`
- `bang_bang`
- `rcpd`

### 12.2 Launch-RCPD

- Launch：短时间限制起步命令，减小打滑。
- Ramp：按斜坡快速提升。
- Cruise：保持速度上限。
- P Brake：进入紧凑SlowR后比例制动。
- Stop：进入停止半径后发零速度。

当前默认/建议基线：

| 参数 | 值 | 说明 |
|---|---:|---|
| Strategy | rcpd | 默认策略 |
| Kp | 5.0 | 当前共用制动增益字段 |
| Max | 1.4 | 协议满量程，不是标定速度 |
| Stop | 0.05m | 完成半径 |
| Hz | 20 | 控制发送频率 |
| LaunchT | 0.10s | 起步保护窗口 |
| Launch | 0.55 | 起步命令上限 |
| Ramp/s | 8.0 | 命令上升速率 |
| SlowR | 0.12m | 紧凑制动区 |
| Min | 0.14 | 制动区最小命令 |

这些参数需在不同电量、地面摩擦、起点、朝向和负载下重复测试。

### 12.3 Return

Return使用更保守的P策略和较低上限，优先回到中心并控制终点精度。

### 12.4 连续GoTo

小球落点随时间逐步精确，动态目标不能简单反复调用`start()`。否则会不断重置：

- Launch计时。
- 视觉速度估计。
- 命令斜坡。
- 运行时间与状态。

`GotoController.update_target()`应只更新目标，不重启控制器。

`GotoTargetManager`负责：

- 世界坐标边界。
- 置信度门控。
- 目标跳变限制。
- 小变化死区。
- 新鲜度与超时。
- Commit锁定。

控制器负责“给定当前目标如何输出速度”，目标管理器负责“是否接受新的预测目标”。

---

## 13. 安全仲裁

建议优先级：

1. 人工E-STOP。
2. PC/ESP32通信断联STOP。
3. LD14保护性STOP。
4. AprilTag/视觉门控STOP。
5. 正常手动或GoTo命令。

所有运动来源最终必须经过同一发送入口。Tab3、雷达线程或策略模块不得直接绕过主程序向STM32发送运动帧。

雷达STOP解除后，不应无条件恢复上一次高速命令。建议后续增加“安全恢复”状态，使GoTo重新评估视觉、目标和当前障碍后再恢复。

---

## 14. 测试SOP

### 14.1 基础通信

1. 检查所有模块共地。
2. 检查GPIO16/17、GPIO26、GPIO32和GPIO34。
3. BNO085模式变化后整套断电重启。
4. 启动ESP32并确认Wi-Fi连接和UDP 4210监听。
5. 启动V4.1.0并点击Connect。
6. 确认Tab1的BNO085、ADC和STM32反馈。
7. 确认ESPSTAT中的`hasPC`对应逻辑已经建立。

### 14.2 LD14

通过标准：

- 雷达旋转。
- `ld_bytes`持续增加。
- `ld_h54`与`ld_h542c`近似同步。
- `ld_ok`持续增加。
- `ld_bad`为0或远小于`ld_ok`。
- `lidar=1`。
- UDP 4212接收到52字节包：5字节`LD14:`加47字节帧。
- Tab3显示ONLINE并出现稳定点云。
- 拔掉LD14后，Tab3离线但M100基本控制继续正常。

### 14.3 BNO085

- `rvc_ok`持续增加。
- `rvc_bad`为0或极少。
- `bno=1`。
- 姿态动作方向正确。
- 水平静止G经死区后接近0。

### 14.4 PID

- Mode 0下读取与写入P/I/D。
- 四轮Target/Actual/Error对应物理轮位。
- 默认1.25/0.76/0.25一致。
- 切回Mode 6前停止PID测试并清理状态。

### 14.5 视觉GoTo

- 固定相机和场地。
- 锁定四角H。
- ID10连续输出x/y/yaw。
- 低速P策略确认方向。
- 再切RCPD，一次只改一个参数。
- 记录时间、误差、Max G、电池、轨迹、过冲和雷达STOP状态。

### 14.6 防撞

1. 架空轮子或最低速开始。
2. 先只显示Warning，不发送STOP。
3. 核对雷达0度与车头方向。
4. 设置较保守STOP/RELEASE距离。
5. 验证障碍进入、锁存、离开和释放。
6. 验证Tab3不可见时仍工作。
7. 验证LD14断开时默认不会阻止M100。
8. 验证E-STOP始终高于雷达状态。

---

## 15. 已解决问题与踩坑档案

| 问题 | 现象 | 处理 |
|---|---|---|
| TB6612损坏 | 电机异常 | 更换驱动恢复 |
| TCP命令堆积 | 停止后仍执行旧命令 | 改UDP透明桥 |
| 串口打印过量 | 看似控制延迟 | 调试输出限频 |
| CH2/CH3错位 | RF/LR显示反 | CH2=LR，CH3=RF |
| PID回复通道不可靠 | byte2/3异常 | pending_read映射 |
| 世界/车体混用 | 弧线、绕圈、螺旋 | world error经yaw转body |
| 斜向轮目标超限 | 单轮超过1400 | 菱形速度包络 |
| BNO085 zero pending | 有字节无有效帧 | checksum改为p[2]..p[17] |
| RVC yaw方向反 | 3D水平旋转相反 | 去旧I2C额外负号 |
| RVC pitch/roll互换 | 抬头显示横滚 | 交换显示轴 |
| RVC静止约1G | 加速度含重力 | 水平场景扣除Z轴1g |
| G静止残余 | 0.04-0.05G | 0.06G死区建议初值 |
| 两PC程序抢UDP端口 | 第二程序不能监听 | LD14独立UDP 4212 |
| SoftwareSerial头文件缺失 | 编译失败 | 安装库或最终改硬件UART0 |
| SoftwareSerial API错误 | 无`setRxBufferSize` | 删除该调用 |
| SoftwareSerial 115200失真 | bytes增但帧头/CRC失败 | LD14改UART0 RX=GPIO32 |
| Tab3服务属性不存在 | tkinter缺`lidar_service` | 服务在build_ui前初始化 |
| Tab3 OFFLINE | 链路任一段未贯通 | 用bytes/header/good/hasPC分段定位 |

---

## 16. 风险、限制与工程约束

### 16.1 控制

- `MAX_TARGET=1400`不是精确物理速度。
- RCPD还需跨电量、地面、方向和负载验证。
- 雷达STOP不是E-STOP替代品。
- 高速避障必须考虑制动距离、网络周期、视觉周期和轮胎打滑。

### 16.2 视觉

- H锁定后相机或角点移动会使映射失效。
- AprilTag yaw仍是world-to-body关键输入。
- 视觉丢失时不能继续累计控制输出。

### 16.3 BNO085

- UART-RVC无独立gyro XYZ。
- 当前G处理只适合近似水平底盘。
- 姿态显示轴修正不能污染视觉坐标控制。

### 16.4 LD14

- 三角测距点需要结合强度、距离与SDK角度修正评估。
- 当前点云显示可以辅助驾驶，但不等于导航定位。
- UART0 RX重映射后，USB输入路径改变。
- 传感器离线策略必须显式配置。
- 防撞参数仍是建议初值。

### 16.5 文件一致性

OneDrive的`.txt`可能是Python源码，运行时必须恢复为`.py`。ESP32 OneDrive备份、实际烧录INO和知识库必须同步，避免旧RVC checksum或SoftwareSerial版本回退。

---

## 17. 未来工作方向

### 17.1 近期固化

1. 将UART0 LD14诊断版固化为正式ESP32版本。
2. 保留可开关的传感器统计，不保留高频打印。
3. 将V4.1.0全部模块备份到OneDrive，Python源码以`.txt`存档并记录对应`.py`名。
4. 增加LD14安装角度偏置和坐标方向配置。
5. 完成雷达保护低速、不同障碍材质和不同方向的测试矩阵。
6. 验证拔除LD14、BNO085短时异常、UDP 4212中断时主程序行为。

### 17.2 连续GoTo测试

在接入主程序前独立测试：

- 固定目标。
- 缓慢漂移目标。
- 逐步收敛目标。
- 高频小幅抖动。
- 突然跳变。
- 目标过期。
- 超出场地。
- 置信度下降。
- 雷达STOP与解除。
- AprilTag短时丢失。

### 17.3 LandingNet接口

建议数据流：

```text
小球检测/轨迹
    ↓
LandingNet落点预测（统一世界坐标）
    ↓
GotoTargetManager
  - 置信度
  - 边界
  - 跳变
  - 新鲜度
  - Commit
    ↓
GotoController.update_target()
    ↓
视觉门控 + 雷达安全仲裁
    ↓
Mode 6
```

### 17.4 自动接球状态机

建议状态：

`Idle → Track Ball → Predict → Commit → Move → Catch/Stop`

异常状态必须覆盖：

- 小球丢失。
- 落点过期。
- AprilTag丢失。
- H失效。
- UDP超时。
- LD14障碍。
- 人工E-STOP。

### 17.5 雷达扩展

- 角度中心修正与原始/修正点对比。
- 车体外形屏蔽区。
- 按当前运动方向动态选择保护扇区。
- 速度相关的动态STOP距离。
- 与视觉目标方向联合安全走廊。
- 记录雷达触发、最小距离和停车响应数据。
- 评估SLAM或局部地图作为独立支线，不直接破坏自动接球主线。

### 17.6 Orange Pi支线

继续评估Orange Pi Zero 3W与AR0234的Linux采集、边缘检测和视觉伺服。保持Windows主线和Orange Pi支线隔离，通过稳定接口共享世界坐标目标，不进行未经验证的大规模合并。

---

## 18. 当前关键配置速查

| 配置 | 当前值/说明 |
|---|---|
| 场地 | 1.20m × 1.20m |
| 四角ArUco | 0,1,2,3 |
| 小车AprilTag | 36h11 ID10 |
| MAX_TARGET | 1400 raw = 单轮100%基准 |
| UI循环 | 5ms建议基线 |
| UDP命令 | 4210 |
| UDP遥测 | 4211 |
| UDP雷达 | 4212 |
| STM32 UART2 | GPIO16/17，115200 |
| BNO085 UART1 | RX GPIO26，115200 |
| LD14 UART0 | RX GPIO32，TX GPIO1，115200 |
| ADC | GPIO34 |
| 分压 | 38700Ω/9660Ω |
| 默认PID | 1.25/0.76/0.25 |
| RVC checksum | p[2]..p[17]累加，p[18]校验 |
| LD14帧 | 47字节，54 2C，12点，CRC8 |
| 通信STOP | 500ms无PC命令 |
| G死区 | 0.06G建议初值 |
| 雷达离线停车 | 默认False |
| 雷达保护 | 可配置，默认调试时关闭 |

---

## 19. 文件索引

| 用途 | OneDrive/资料文件 | 本地或运行文件 |
|---|---|---|
| V4.0.8知识库 | `CODBOT_M100_KnowledgeBase_V4_0_8_Vision_Goto_Strategy.docx` | 参考文档 |
| V4.0.9知识库 | `CODBOT_M100_KnowledgeBase_V4_0_9_UART_RVC_Vision_Goto_Strategy.md` | 旧基线 |
| 周末工作笔记 | `100PercentHit_Weekend_Work_Note_AprilTag_Gating_Corridor_2026-08-27.txt` | 参考文档 |
| LD14手册 | `LD14.md` | 协议与接线参考 |
| STM32源码包 | `STM32_Code.txt` | 91文件源码汇总 |
| V4.1.0主程序 | 建议备份为`m100_control_center_v4_1_0.txt` | `m100_control_center_v4_1_0.py` |
| V4.1.0配置 | 建议备份为`m100_config.txt` | `m100_config.py` |
| Position面板 | 建议备份为`position_command_panel.txt` | `ui/position_command_panel.py` |
| GoTo策略 | `goto_strategy_basic.txt` | `goto/goto_strategy_basic.py` |
| 动态目标 | 建议备份为`goto_target_manager.txt` | `goto/goto_target_manager.py` |
| LD14协议 | 建议备份为`ld14_protocol.txt` | `lidar/ld14_protocol.py` |
| LD14服务 | 建议备份为`ld14_service.txt` | `lidar/ld14_service.py` |
| LD14页面 | 建议备份为`ld14_tab.txt` | `lidar/ld14_tab.py` |
| 雷达保护 | 建议备份为`lidar_collision_guard.txt` | `lidar/lidar_collision_guard.py` |
| ESP32正式候选 | 建议同名`.txt`备份 | `M100_ESP32_UDP_Bridge_BNO085_UART_RVC_Debug_Fix_LD14_UART0_Diagnostic_v4.ino` |
| 本知识库 | `CODBOT_M100_KnowledgeBase_V4_1_0_UART_RVC_LD14_Modular_Vision_Goto_Strategy.md` | 同名Markdown |

---

## 20. 最小恢复上下文

后续新会话可直接提供本文件并说明：

> 请基于《CODBOT M100平台开发知识库V4.1.0》继续开发。当前M100 Control Center已模块化为四个Tab；BNO085使用UART-RVC，GPIO26，checksum为p[2]..p[17]；LD14已通过硬件UART0 RX重映射GPIO32稳定接入，UDP 4212送入Tab3，已经可以辅助人工在室内盲开。STM32仍使用GPIO16/17和Mode 0/Mode 6，电池ADC为GPIO34、38700Ω/9660Ω。视觉使用四角ArUco和AprilTag ID10，GoTo必须保持world-to-body、麦轮菱形包络、E-STOP和500ms断联STOP。下一主线是连续GoTo目标管理、LandingNet落点接入、AprilTag走廊与雷达安全仲裁。

---

**CODBOT M100 Knowledge Base V4.1.0 | 百分百命中 | 2026-08-30**
