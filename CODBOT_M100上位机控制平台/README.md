# M100 Control Center V4.1.0 Modular + LD14

## 运行
1. 安装依赖：`pip install opencv-contrib-python pillow pyserial numpy pygame`
2. 运行：`python m100_control_center_v4_1_0.py`

## 文件结构
- `m100_control_center_v4_1_0.py`：稳定应用壳，4个Tab。
- `m100_config.py`：LD14与版本配置。
- `ui/position_command_panel.py`：Tab1 Position Command控件。
- `goto/goto_strategy_basic.py`：现有策略，保持公共API，新增不重置状态的`update_target()`。
- `goto/goto_target_manager.py`：连续落点目标门控，尚未自动接入主循环。
- `lidar/ld14_protocol.py`：LD14协议与CRC。
- `lidar/ld14_service.py`：UDP 4212后台接收。
- `lidar/ld14_tab.py`：Tab3雷达页面，无辉光、无扫描线、无CSV。
- `lidar/lidar_collision_guard.py`：保护停止判断。
- `esp32/*.ino`：UART-RVC Debug Fix基线加可选LD14。

## ESP32接线与库
- STM32保持GPIO16/17。
- BNO085 UART-RVC保持GPIO26。
- LD14 TX接GPIO32，5V供电，共地，PWM悬空或接地。
- Arduino Library Manager安装`EspSoftwareSerial`。
- 固件中`USE_LD14=0`可完全关闭雷达功能。
- LD14数据发往PC UDP 4212；M100原4210/4211保持不变。

## 安全默认值
- `collision_stop_enabled=False`，需在Tab3手动打开。
- LD14失联默认不停车，M100继续工作。
- 停止/释放距离、扇区、确认帧均为建议初值，必须低速实测。

## 已知事项
- 本包基于OneDrive中的V4.0.9主程序和`goto_strategy_basic.py`生成。
- OneDrive未检索到`ld14_viewer_v0_2_1.py`，因此雷达页面按你指定的简化特征实现，而非逐像素复制。
- ESP32文件以可检索到的UART-RVC源码为底，并应用Debug Fix的checksum范围`p[2]..p[17]`。烧录前请与当前本地Debug_Fix.ino做一次diff。
