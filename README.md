# 百分百命中（100 Percent Hit）

“百分百命中”是一个自动接球机器人实验项目，使用单目摄像头实现小车自动控制接到（或撞击到）抛来的小球。本仓库公开了项目开发过程中形成的三套独立的核心程序：LD14 激光雷达验证工具、CODBOT M100 上位机控制平台，以及基于 Orange Pi Zero 3W 的车载视觉接球程序。

项目涉及激光雷达、工业摄像头、YOLO在嵌入式NPU的量化部署、ESP32、BNO085、STM32 和麦克纳姆轮底盘，目标是逐步建立从视觉感知、目标跟踪到运动控制和安全停车的完整闭环。也是仓库作者自学嵌入式和视觉控制的第一个脑洞项目。

> 本仓库面向学习、研究和工程验证。其理论框架和代码可以在相似项目上复用。但所有速度、距离、增益、阈值及安全参数均应根据实际硬件重新调试和标定。本项目不是经过认证的工业安全系统。

## 1. 硬件清单

### 1.1 底盘与底层控制

| 硬件 | 数量 | 用途 |
|---|---:|---|
| CODBOT M100 麦克纳姆轮底盘 | 1 | 全向移动平台 |
| STM32F103RET6 控制板 | 1 | Mode 协议、麦轮运动学、编码器 PID 和电机执行 |
| 520 编码器电机 | 4 | 四轮驱动及速度反馈 |
| 80 mm 麦克纳姆轮 | 4 | 前后、横向及旋转运动 |
| TB6612 双 H 桥驱动 | 2 | 四路电机功率驱动 |
| ESP32 开发板 | 1 | UART、Wi-Fi/UDP、BNO085、电池 ADC 及断联停车桥接 |
| 电池及分压检测电路 | 1 套 | 小车供电和电池电压采集 |

### 1.2 感知与边缘计算

| 硬件 | 数量 | 用途 |
|---|---:|---|
| Orange Pi Zero 3W | 1 | 车载视觉、网页、状态机和高层运动控制 |
| AR0234 USB 工业摄像头 | 1 | 小球检测及车载相对视觉 |
| LDROBOT LD14 激光雷达 | 1 | 二维点云、环境观察及障碍停车输入 |
| BNO085 姿态传感器 | 1 | UART-RVC 姿态、加速度和相对航向参考 |

### 1.3 操作与状态硬件

| 硬件 | 用途 |
|---|---|
| ARM/SAFE 按钮 | 自动控制使能、安全待机和关机操作 |
| 接球触点或测试按钮 | 接球成功输入和网络切换操作 |
| WS2812 LED 状态灯 | 网络、状态机、故障和运行状态提示 |
| 稳定 5 V 电源、线束和安装支架 | 各控制板、传感器和附件供电与安装 |

### 1.4 STM32 与 CODBOT M100 来源说明

CODBOT M100 底盘、STM32F103RET6 控制板、底层固件和相关通信接口来自 **酷点智能**。本仓库作者基于商家提供的 STM32 接口和控制协议，开发了 ESP32 通信桥、PC 上位机、传感器接入、视觉定位和运动控制程序。

STM32 固件、源码及商家资料以**酷点智能**的许可条款为准。仓库中未包含麦轮小车底盘STM32相关的源代码。

## 2. 开源内容总览

| 部分 | 主要硬件 | 主要作用 |
|---|---|---|
| LD14 独立验证工具 | LD14、ESP32、PC | 验证 LD14 UART 通信、帧解析、角度修正和二维点云显示，不控制底盘运动 |
| CODBOT M100 上位机控制平台 | Windows PC、ESP32、BNO085、LD14、STM32、M100 | 实现手动控制、PID 调试、传感器显示、视觉定位和目标点 GoTo |
| Orange Pi Zero 3W 接球程序 | Zero 3W、AR0234、ESP32、BNO085、LD14、M100 | 实现车载小球检测、跟踪、X/Y 视觉伺服、网页控制、安全状态机和接球管理 |

## 3. 目录结构

```text
100PercentHit/
├── README.md
├── .gitignore
├── LD14激光雷达单独验证程序_ESP32读取/
├── CODBOT_M100上位机控制平台/
└── OrangePi_Zero_3w/
```

## 4. LD14 激光雷达独立验证工具

目录：[`LD14激光雷达单独验证程序_ESP32读取/`](LD14激光雷达单独验证程序_ESP32读取/)

该部分使用 **LD14 + ESP32 + PC**，用于独立验证 LD14 激光雷达的通讯协议、修正参数等。

ESP32 可通过 USB 串口或 Wi-Fi UDP 向 PC 转发雷达数据。PC 端工具用于显示点云、检查链路状态和导出测试数据。该部分不接入 M100 运动控制，也不会产生底盘运动命令。

### 快速运行

```bash
cd LD14激光雷达单独验证程序_ESP32读取
pip install -r requirements.txt
python ld14_viewer_v0_2_1.py
```

烧录 ESP32 程序前，必须修改程序中的 Wi-Fi 名称和密码。建议首次验证优先使用 USB 链路，确认串口、供电、共地和 CRC 正常后再测试 Wi-Fi UDP。

## 5. CODBOT M100 上位机控制平台

目录：[`CODBOT_M100上位机控制平台/`](CODBOT_M100上位机控制平台/)

该部分使用 **Windows PC + ESP32 + BNO085 + LD14 + STM32F103RET6 + CODBOT M100**，是项目前期底盘开发、传感器集成和 Windows 视觉闭环验证平台。（后期已改为车载相对视觉路线）

上位机支持串口和 Wi-Fi UDP 控制，可用于手动驾驶、蓝牙游戏手柄遥控、麦轮底盘PID 调试、ArUco 场地映射、AprilTag 小车定位以及目标点 GoTo。

### 详细文档

- [`CODBOT M100 平台开发知识库 V4.1.0`](CODBOT_M100上位机控制平台/docs/CODBOT_M100_KnowledgeBase_V4_1_0_UART_RVC_LD14_Modular_Vision_Goto_Strategy.md)
- [`CODBOT M100 上位机简要说明`](CODBOT_M100上位机控制平台/README.md)

### 快速运行

```bash
cd CODBOT_M100上位机控制平台
pip install opencv-contrib-python pillow pyserial numpy pygame
python m100_control_center_v4_1_0.py
```

## 6. Orange Pi Zero 3W 车载接球程序

目录：[`OrangePi_Zero_3w/`](OrangePi_Zero_3w/)

该部分使用 **Orange Pi Zero 3W + AR0234摄像头 + ESP32 + LD14激光雷达 + 麦轮小车CODBOT M100**，是项目后期采用的车载相对视觉路线。

- Orange Pi Zero 3W （ubuntu）负责摄像头采集、YOLO/HSV 小球检测、X/Y 视觉伺服、相对航向保持、Flask 手机网页图传、网络切换、LD14 安全判定和统一运动仲裁。
- ESP32 负责 Zero 3W 与 STM32 的串口桥、BNO085、电池 ADC、并于前述PC上位机控制平台兼容。
- STM32 继续负责麦轮运动学、编码器 PID 和电机执行。

### 主要功能

- YOLOv5n/YOLOv5s NPU 检测与 HSV 备用检测。
- 图像 X 误差控制车体横移。
- 图像 Y 方向变化用于前后控制/OAC。
- BNO085 相对航向保持。
- 手机网页图传、手动控制、状态显示和参数配置。
- 家庭 Wi-Fi 与 Zero 3W 热点切换。
- LD14 障碍停车及保护旁路。

### 主要文档

- [`Orange Pi Zero 3W 车载视觉伺服完整知识库`](OrangePi_Zero_3w/docs/100PercentHit_OrangePiZero3W_KnowledgeBase.md)
- [`YOLOv5 训练、Allwinner UINT8 量化与 Zero 3W 部署指南 V2.0`](OrangePi_Zero_3w/docs/100PercentHit_YOLOv5_NPU_Training_Quantization_Deployment_Guide_V2.0.md)
- [`ESP32 桥接说明`](OrangePi_Zero_3w/esp32/README.MD)

### 运行入口

```bash
cd OrangePi_Zero_3w
python -m app.main --config config/local.yaml
```

正式部署通常由 [`percenthit.service`](OrangePi_Zero_3w/systemd/percenthit.service) 启动。设备节点、Device Tree overlay、GPIO、UART、SPI、NetworkManager 和 systemd 配置请参考完整知识库。

## 7. YOLOv5 训练、量化与 Zero 3W 部署

本仓库将 YOLOv5 模型训练、Allwinner UINT8 量化和 Orange Pi Zero 3W 板端部署单独整理为一份指南：

- **重点文档：** [`100PercentHit_YOLOv5_NPU_Training_Quantization_Deployment_Guide_V2.0.md`](OrangePi_Zero_3w/docs/100PercentHit_YOLOv5_NPU_Training_Quantization_Deployment_Guide_V2.0.md)

该流程包括：

1. 在 Windows 环境训练自定义单类别 YOLOv5n 或 YOLOv5s 小球模型。
2. 导出适用于后续量化的传统四输出 ONNX。
3. 在 WSL 和 Docker 中准备 Allwinner NPU 转换环境。
4. 使用 Allwinner 官方 Pegasus 流程完成导入、UINT8 量化、仿真和 NBG 导出。
5. 将 `.nb` 模型部署到 Orange Pi Zero 3W。
6. 通过 VIPLite、C++ 共享库和 Python `ctypes` 接入车载主程序。

### 官方工具来源

量化和板端部署所需的 Docker 镜像、AI SDK、NPU 工具链、系统镜像及配套资料，应根据 Orange Pi Zero 3W 的硬件和系统版本，从以下官方渠道获取：

- **全志（Allwinner）官方网站及其官方开发资源**
- **香橙派（Orange Pi）官方网站及其 Zero 3W 下载页面**

请使用与当前板卡、系统镜像和 NPU 驱动匹配的版本。

## 8. Wi-Fi、IP 和首次配置

### 8.1 ESP32 Wi-Fi

所有 ESP32 程序在烧录前都必须修改为使用者自己的 Wi-Fi 信息：

```cpp
const char* WIFI_SSID = "YOUR_WIFI_NAME_HERE";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD_HERE";
```

若启用静态 IP，还需要根据实际局域网修改：

- `LOCAL_IP`
- `GATEWAY`
- `SUBNET`
- `DNS1`
- PC 地址和 UDP 端口配置

### 8.2 Zero 3W 网络

公开部署或实际使用前，请检查并修改：

- `wifi_connection_name`
- `ap_connection_name`
- `ap_ssid`
- `ap_password`
- `ap_address`
- `scripts/setup_network.sh` 中的热点名称、密码和网段

仓库中的 `192.168.x.x` 地址是局域网地址或部署示例。使用者必须根据自己的路由器、电脑、ESP32 和 Zero 3W 所在网段进行相应修改。

> 仓库中的热点密码属于示例配置。请在部署前替换为自己的安全密码，并避免把真实密码提交到公开仓库。

### 8.3 主要端口

| 端口 | 用途 |
|---:|---|
| 4210 | PC 到 ESP32 的控制命令 |
| 4211 | ESP32 到 PC 的遥测和 STM32 数据 |
| 4212 | M100 上位机路线中的 LD14 数据链路 |
| 8080 | Zero 3W Flask 手机网页 |

不同子项目对端口的使用方式可能不同，请以对应目录的文档和源码为准。

## 9. 安全说明

- 第一次通电和方向测试必须架空车轮。
- 落地测试应从低速开始，并保留可立即断电的手段。
- Zero 3W、ESP32、STM32、传感器及电源必须共地。
- Mode 6 中的速度值是协议换算尺度，不代表已经标定的真实物理速度。
- 麦轮 X/Y 合成控制应保留菱形速度包络，避免单轮目标超限。
- 自动运动前必须确认方向符号、yaw、串口、Wi-Fi、设备节点和传感器数据有效。
- LD14 保护停车不能替代物理 E-STOP，也不构成安全认证。
- 测试区域内应避免人员、宠物和易损物品靠近运动底盘。

## 10. 当前能力边界

本仓库包含已经运行或用于工程验证的程序，但不同功能的成熟度并不相同：

- LD14 独立工具用于协议和点云验证，不是导航或 SLAM 系统。
- M100 上位机已包含视觉定位和静态目标 GoTo，但控制参数仍需按实际底盘调试。
- Zero 3W 已集成 YOLO/HSV、X/Y 控制、状态机和安全链路，但高速自动接球参数需要实际调试。
- 自动控制参数、雷达参数、电池电量映射和摄像头参数都不是通用标定值。
- 多目标关联、误检抑制、掉检鲁棒性和完整接球成功率仍有改进空间。

## 11. 依赖与第三方内容

本项目使用或参考了以下软硬件和开源组件：

- Python、OpenCV、NumPy、Flask、PySerial、PyYAML、Pygame、Pillow
- YOLOv5
- Allwinner A733 NPU、Pegasus、VIPLite 及相关 AI SDK
- Orange Pi Zero 3W 系统和硬件资料
- LDROBOT LD14 通信协议
- ArUco 和 AprilTag
- CODBOT M100、STM32F103RET6 控制板及酷点智能提供的底层接口

仓库中可能包含预编译共享库、可执行文件和 NBG 模型。这些文件的适用平台、来源和许可可能与本仓库原创代码不同。使用、修改或再发布前，请分别确认对应第三方的许可证和分发条件。

## 12. 文档索引

### LD14

- [LD14 核心通信协议与数据解析](LD14激光雷达单独验证程序_ESP32读取/docs/LD14_Core_Communication_and_Parsing_Protocol.md)
- [LD14 Studio V0.2 使用说明](LD14激光雷达单独验证程序_ESP32读取/README_LD14_ESP32_V0_2.md)

### CODBOT M100

- [CODBOT M100 平台开发知识库 V4.1.0](CODBOT_M100上位机控制平台/docs/CODBOT_M100_KnowledgeBase_V4_1_0_UART_RVC_LD14_Modular_Vision_Goto_Strategy.md)
- [M100 Control Center 简要说明](CODBOT_M100上位机控制平台/README.md)

### Orange Pi Zero 3W

- [Orange Pi Zero 3W 车载视觉伺服完整知识库 V3.1](OrangePi_Zero_3w/docs/100PercentHit_OrangePiZero3W_Complete_KnowledgeBase_V3.1_20261006.md)
- [YOLOv5 训练、Allwinner UINT8 量化与 Zero 3W 部署指南 V2.0](OrangePi_Zero_3w/docs/100PercentHit_YOLOv5_NPU_Training_Quantization_Deployment_Guide_V2.0.md)
- [Zero 3W ESP32 桥接说明](OrangePi_Zero_3w/esp32/README.MD)