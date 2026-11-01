# -*- coding: utf-8 -*-
"""
M100 Control Center V4.1.0 Modular + LD14
-----------------------------
V3.6 = V3.4 integrated control center + V3.5 PID diagnostics.

Main upgrades versus V3.4/V3.5:
1. Keeps Vision / Manual / PID / Logs tabs in one program.
2. PID parser can run continuously and does not require switching the whole serial parser off.
3. PID curve buffer increased to 1000 points.
4. PID curve shows Actual / Target / Error.
5. Per-wheel PID status panel: Target / Actual / Error / RX rate / diagnosis.
6. Global PID controls: Apply P/I/D to all four channels.
7. PID CMD RX parsing fixed for seller firmware behavior:
   - Returned C-frame may use byte2=1 and byte3=0, so V3.6 maps read replies by last requested command/channel.
8. Log filters from V3.5: Show TX, Show CMD RX, Show DATA RX, Capture Next CMD.
9. Mode 6 CD 0A 01 motion command kept from V3.4.
10. Base feedback parser kept from V3.4: battery, wheel speed, IMU/G-force, odom.
11. Gamepad support added: left stick drives Tab2 virtual joystick, right stick horizontal controls rotation, BTN5/RB triggers E-STOP/brake.
12. V3.9 gamepad mapping for Beitong Asura 2 Pro driver mode: A0/A1=left stick, A2/A3=right stick, A4=LT, A5=RT.
13. RT axis scales speed from 20% to 100%; D-pad maps to W/A/S/D; gamepad E-STOP visually links to the UI E-STOP button.
14. V3.9.1 updates: BTN5/RB as E-STOP, full-range joystick display, WASD/D-pad visual linkage, Space/E-STOP linkage, percentage speed UI.
15. V3.9.2 updates: physical sticks are independent analog controls; RT speed percentage only applies to D-pad, keyboard WASD, and manual direction buttons.
16. V3.9.4 updates: merged Vision and Manual into one Control Center tab, 3-tab layout, slider/bar based gamepad controls, compact joystick, top camera/map/G-force/wheel layout.
17. V3.9.5 updates: larger top visual area, square map, narrower telemetry column, compact bottom controls, no center E-STOP in arrow pad, smaller virtual joystick.
18. V3.9.6 updates: fixed 1710x990 layout constants, configurable camera resolution/FPS with green overlay, G trace with Max G, reserved 3D attitude panel, go-to stopwatch, wider PID status boxes.
19. V3.9.7 updates: added Serial/WiFi connection transport. WiFi mode connects to ESP32 TCP bridge and keeps existing Mode 6 / Mode 0 protocol unchanged.
20. V3.9.8 updates: PID Target UI changed to -100%..+100% slider with debounced output, PID curve displays percent scale (-110%..+110%) with 600-point right-aligned window, D-pad/WASD release now sends one zero motion frame.
21. V3.9.8.1 updates: Tab1 manual/joystick/D-pad speed scale fixed. 100% manual speed now maps to 1.4 m/s, matching Mode 0 PID Target 1400 = 100%.
22. V3.9.9 updates: Tab1 Wheels changed to 2x2 graphical bars with blue Actual and green Target marker; 3D Attitude compressed. Tab2 PID Curve adds Single CH / 4 Wheels switch; 4 Wheels compares all wheel Target/Actual curves.
23. V3.9.9.1 updates: Fixed physical wheel mapping. STM32 CH2 is LR and CH3 is RF, so UI physical order is LF=CH1, RF=CH3, LR=CH2, RR=CH4.
24. V3.9.9.2 updates: WiFi transport changed from TCP to UDP for ESP32 bridge; added ESP32 IMU/ADC telemetry parsing, pseudo-3D attitude, and battery voltage/percent display.
25. V3.9.9.3 updates: Manual joystick/gamepad maximum chassis speed changed to 1.00 m/s; added mecanum diamond limiting for Mode 6 vx/vy to avoid diagonal wheel over-target; added BNO085 display zeroing on WiFi connection and a manual Zero 085 button.
26. V3.9.9.5 updates: Restored manual full scale to wheel target 1400 using mecanum diamond envelope; enlarged 3D Attitude panel to 180%; car nose points upward at zero yaw; added gyro-z integrated yaw display to compare against magnetic rotation-vector yaw.
27. V3.9.9.6 updates: G Force panel now uses ESP32/BNO085 accelerometer telemetry ax/ay/az directly; keeps red-dot fading trail; bottom text shows real-time G and Max G.
28. V4.0.9 updates: adapt BNO085 UART-RVC axis convention; correct yaw direction, swap pitch/roll mapping, and remove static 1g from the G display.

Install:
    pip install opencv-contrib-python pillow pyserial numpy pygame

Run:
    python m100_control_center_v4_1_0.py
"""

import math
import time
import struct
import csv
import socket
from collections import deque
from goto.goto_strategy_basic import GotoController, GotoConfig
from m100_config import APP_VERSION, LIDAR_CONFIG
from ui.position_command_panel import build_position_command_panel
from lidar.ld14_service import LidarService
from lidar.lidar_collision_guard import LidarCollisionGuard
from lidar.ld14_tab import LD14Tab
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import numpy as np
except Exception:
    np = None

try:
    import cv2
except Exception:
    cv2 = None

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None

try:
    import pygame
except Exception:
    pygame = None

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None

# ============================================================
# REAL FIELD CONFIG
# ============================================================
FIELD_WIDTH_M  = 1.20
FIELD_HEIGHT_M = 1.20

FIELD_MARKER_WORLD = {
    0: (0.0, 0.0),
    1: (FIELD_WIDTH_M, 0.0),
    2: (FIELD_WIDTH_M, FIELD_HEIGHT_M),
    3: (0.0, FIELD_HEIGHT_M),
}

CAR_ARUCO_ID = 10
CAR_APRILTAG_ID = 10
MAX_TARGET = 1400.0
MAX_MANUAL_SPEED_MPS = MAX_TARGET / 1000.0  # Mode 6 full-scale: wheel target 1400 = 100%. Do not treat this as measured m/s.
CAR_LENGTH_M = 0.25
CAR_WIDTH_M = 0.20
PID_HISTORY_LEN = 600
PID_PLOT_LEN = 600
PID_PLOT_LIMIT_PCT = 110.0
PID_DEAD_ACTUAL = 5.0
PID_ACTIVE_TARGET = 50.0
PID_ALARM_SECONDS = 2.0
# ============================================================
# V4.0.9 PERFORMANCE / VISION CONFIG
# ============================================================
# Main UI refresh delay. Important: Tkinter after() delay is added after processing time.
# Old value 33 ms gives about 16-18 FPS when each frame already costs ~22-27 ms.
# Recommended weekend goto test: 5 ms. If UI becomes too busy, try 10 or 15 ms.
UI_LOOP_DELAY_MS = 5

# Camera display resize mode. NEAREST is fastest; BILINEAR is a good visual/FPS balance.
# Options: 'NEAREST', 'BILINEAR', 'BICUBIC', 'LANCZOS'
VIDEO_RESAMPLE_MODE = 'BICUBIC'

# PID drawing switch. For goto testing on Tab1, keep False or draw only when Tab2 is active.
PID_DRAW_ENABLED = True
PID_DRAW_ONLY_WHEN_TAB2_ACTIVE = False

# Field ArUco 0/1/2/3 markers are only needed to lock homography once.
# After field lock, AprilTag ID10 is still detected every frame for car pose.
# If you move the camera or field markers, restart camera/app to relock, or set this False temporarily.
DETECT_ARUCO_UNTIL_FIELD_LOCK_ONLY = True
DETECT_APRILTAG_EVERY_FRAME = True
DRAW_DETECTED_MARKERS = True

CMD_NAMES = {
    1: 'P',
    2: 'I',
    3: 'D',
    4: 'Target',
    6: 'Actual',
    7: 'Start',
    8: 'Stop',
}

PID_HEADS = [ord('C'), ord('0'), ord('1'), ord('2'), ord('3')]

# Physical wheel mapping confirmed on STM32 V1.1:
# CH1 = LF, CH2 = LR, CH3 = RF, CH4 = RR.
# Draw order is physical layout: LF, RF, LR, RR.
WHEEL_DRAW_ORDER = [0, 2, 1, 3]
WHEEL_LABEL_BY_CH = ['LF CH1', 'LR CH2', 'RF CH3', 'RR CH4']
WHEEL_PHYS_LABELS = ['LF', 'RF', 'LR', 'RR']


def target_pct_to_raw(pct):
    """Convert UI percent target to seller firmware raw target value."""
    pct = max(-100.0, min(100.0, float(pct)))
    return MAX_TARGET * pct / 100.0

def raw_to_target_pct(value):
    """Convert raw PID target/actual value to UI percent, based on MAX_TARGET."""
    try:
        return float(value) * 100.0 / MAX_TARGET
    except Exception:
        return None

# ============================================================
# Protocol helpers
# ============================================================

def hexline(data: bytes):
    return ' '.join(f'{b:02X}' for b in data)


def build_motion_cmd(vx_mps=0.0, vy_mps=0.0, wz_radps=0.0):
    """Build Mode 6 motion frame: CD 0A 01 + x + y + wz + xor."""
    vx_mps = max(-2.0, min(2.0, float(vx_mps)))
    vy_mps = max(-2.0, min(2.0, float(vy_mps)))
    wz_radps = max(-10.0, min(10.0, float(wz_radps)))

    frame = [0xCD, 0x0A, 0x01]

    if vx_mps >= 0:
        frame.append(0)
    else:
        frame.append(1)
        vx_mps = -vx_mps
    x = int(round(vx_mps * 1000))
    frame += [(x >> 8) & 0xff, x & 0xff]

    if vy_mps >= 0:
        frame.append(0)
    else:
        frame.append(1)
        vy_mps = -vy_mps
    y = int(round(vy_mps * 1000))
    frame += [(y >> 8) & 0xff, y & 0xff]

    if wz_radps >= 0:
        frame.append(0)
    else:
        frame.append(1)
        wz_radps = -wz_radps
    w = int(round(wz_radps * 100))
    frame += [(w >> 8) & 0xff, w & 0xff]

    chk = 0
    for b in frame[2:12]:
        chk ^= b
    frame.append(chk)
    return bytes(frame)


def build_pid_cmd(cmd_id, rw=0, channel=0, value=0.0):
    frame = bytearray(10)
    frame[0] = ord('C')
    frame[1] = int(cmd_id) & 0xff
    frame[2] = int(rw) & 0xff
    frame[3] = int(channel) & 0xff
    frame[4] = 0
    frame[5:9] = struct.pack('<f', float(value))
    frame[9] = ord('\n')
    return bytes(frame)


def parse_pid_frame(frame: bytes):
    if len(frame) != 10 or frame[-1] != ord('\n'):
        return None
    head = frame[0]
    if head == ord('C'):
        try:
            value = struct.unpack('<f', frame[5:9])[0]
        except Exception:
            value = None
        return {
            'kind': 'cmd',
            'cmd_id': frame[1],
            'cmd_name': CMD_NAMES.get(frame[1], f'CMD{frame[1]}'),
            'rw': frame[2],
            'rw_name': 'Write' if frame[2] == 1 else 'Read' if frame[2] == 0 else f'RW{frame[2]}',
            'channel': frame[3],
            'param3': frame[4],
            'value': value,
            'hex': hexline(frame),
        }
    if head in [ord('0'), ord('1'), ord('2'), ord('3')]:
        ch = head - ord('0')
        actual = struct.unpack('<f', frame[1:5])[0]
        target = struct.unpack('<f', frame[5:9])[0]
        return {
            'kind': 'data',
            'channel': ch,
            'actual': actual,
            'target': target,
            'error': target - actual,
            'hex': hexline(frame),
        }
    return {'kind': 'unknown', 'head': head, 'hex': hexline(frame)}


def xor_check(frame):
    chk = 0
    for b in frame[2:-1]:
        chk ^= b
    return chk == frame[-1]


def signed_word(sign_byte, hi, lo, scale=1.0):
    sign = -1 if sign_byte > 0 else 1
    return sign * ((hi * 256 + lo) / scale)

# ============================================================
# Serial and feedback parsers
# ============================================================

class SerialHub:
    """Unified transport for direct USB serial and ESP32 Wi-Fi UDP bridge.

    Serial mode:
        PC <-> USB serial <-> STM32.

    WiFi mode, V3.9.9.2:
        PC m100_control_center -> UDP 4210 -> ESP32 -> UART2 -> STM32
        PC m100_control_center <- UDP 4211 <- ESP32 <- UART2 <- STM32

    ESP32 packet convention used by the generated bridge:
        - PC sends raw M100/CODBOT bytes to ESP32 UDP_CMD_PORT.
        - ESP32 sends STM32 feedback as b"STM:" + raw_bytes to UDP_TELEM_PORT.
        - ESP32 sends text telemetry lines beginning with "IMU,", "ADC,", "ESPSTAT,".

    Public methods stay compatible with the original SerialHub:
        is_open(), connect(), connect_wifi(), close(), write(), read_available(), clear_input()
    """
    def __init__(self):
        self.ser = None
        self.sock = None
        self.kind = 'serial'
        self.host = None
        self.cmd_port = 4210
        self.local_telem_port = 4211
        self.wifi_rxbuf = bytearray()
        self.telemetry = {
            'imu': {},
            'adc': {},
            'esp': {},
            'last_imu_t': None,
            'last_adc_t': None,
            'last_esp_t': None,
            # Pure gyro-z integration, used to compare with BNO085 magnetic/fused yaw.
            # Unit: degrees, relative to connection/Zero 085 reset.
            'gyro_yaw_deg': 0.0,
            'last_gyro_t': None,
        }

    def is_open(self):
        if self.kind == 'wifi':
            return self.sock is not None
        return self.ser is not None and self.ser.is_open

    def connect(self, port='COM3', baudrate=115200):
        """Serial connection, same behavior as V3.9.9.1."""
        if serial is None:
            raise RuntimeError('pyserial not installed. Run: pip install pyserial')
        self.close()
        self.kind = 'serial'
        self.ser = serial.Serial(port=port, baudrate=int(baudrate), timeout=0.005)
        time.sleep(0.1)

    def connect_wifi(self, host='192.168.3.83', port=4210, timeout=3.0):
        """Wi-Fi UDP connection to ESP32 bridge.

        port is the ESP32 command port, default 4210.
        Local telemetry port is fixed to 4211, matching the ESP32 sketch.
        UDP has no real connect handshake, so we send a small PING and then mark as open.
        """
        self.close()
        self.kind = 'wifi'
        self.host = str(host)
        self.cmd_port = int(port)
        self.local_telem_port = 4211
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(float(timeout))
        s.bind(('', self.local_telem_port))
        s.setblocking(False)
        self.sock = s
        self.wifi_rxbuf.clear()
        self.telemetry['gyro_yaw_deg'] = 0.0
        self.telemetry['last_gyro_t'] = None
        try:
            self.sock.sendto(b'PING', (self.host, self.cmd_port))
        except Exception:
            pass

    def close(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.wifi_rxbuf.clear()

    def write(self, data):
        if not self.is_open():
            raise RuntimeError('Transport not connected')
        if self.kind == 'wifi':
            self.sock.sendto(bytes(data), (self.host, self.cmd_port))
        else:
            self.ser.write(data)

    def _parse_kv_text(self, text):
        out = {}
        parts = text.strip().split(',')
        for p in parts[1:]:
            if '=' not in p:
                continue
            k, v = p.split('=', 1)
            k = k.strip()
            v = v.strip()
            try:
                if any(ch in v for ch in ['.', 'e', 'E']):
                    out[k] = float(v)
                else:
                    out[k] = int(v)
            except Exception:
                out[k] = v
        return out

    def _handle_wifi_packet(self, pkt):
        if not pkt:
            return b''
        if pkt.startswith(b'STM:'):
            return pkt[4:]
        if pkt[0] in PID_HEADS or pkt[0] == 0xFD:
            return pkt
        try:
            text = pkt.decode('utf-8', errors='ignore').strip()
        except Exception:
            return b''
        now = time.time()
        if text.startswith('IMU,'):
            imu = self._parse_kv_text(text)
            self.telemetry['imu'] = imu
            # Integrate calibrated gyro Z as a magnetometer-free relative yaw estimate.
            # ESP32 sends gz in rad/s from SH2_GYROSCOPE_CALIBRATED.
            try:
                gz = float(imu.get('gz', 0.0))
                last_g = self.telemetry.get('last_gyro_t')
                if last_g is not None:
                    dt = now - float(last_g)
                    if 0.0 < dt < 0.2:
                        self.telemetry['gyro_yaw_deg'] = float(self.telemetry.get('gyro_yaw_deg', 0.0)) + math.degrees(gz) * dt
                self.telemetry['last_gyro_t'] = now
            except Exception:
                pass
            self.telemetry['last_imu_t'] = now
        elif text.startswith('ADC,'):
            self.telemetry['adc'] = self._parse_kv_text(text)
            self.telemetry['last_adc_t'] = now
        elif text.startswith('ESPSTAT,') or text.startswith('PONG,'):
            self.telemetry['esp'] = self._parse_kv_text(text)
            self.telemetry['last_esp_t'] = now
        return b''

    def read_available(self):
        if not self.is_open():
            return b''
        if self.kind == 'wifi':
            chunks = []
            while True:
                try:
                    pkt, addr = self.sock.recvfrom(4096)
                    raw = self._handle_wifi_packet(pkt)
                    if raw:
                        chunks.append(raw)
                except BlockingIOError:
                    break
                except socket.timeout:
                    break
                except OSError:
                    self.close()
                    break
            return b''.join(chunks)
        n = self.ser.in_waiting
        if n <= 0:
            return b''
        return self.ser.read(n)

    def clear_input(self):
        if self.kind == 'wifi':
            self.wifi_rxbuf.clear()
            if self.sock is not None:
                while True:
                    try:
                        pkt, addr = self.sock.recvfrom(4096)
                        if not pkt:
                            break
                    except Exception:
                        break
            return
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

class BaseFeedbackParser:
    def __init__(self):
        self.buf = bytearray()
        self.data = {
            'vx':0.0,'vy':0.0,'wz':0.0,
            'v1':None,'v2':None,'v3':None,'v4':None,
            'battery':None,
            'imu_yaw_rel_rad':0.0,'imu_yaw_init_rad':None,
            'gyro_x':None,'gyro_y':None,'gyro_z':None,
            'acc_x':None,'acc_y':None,'acc_z':None,
            'car_type':None,
        }
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_vel_t = None
        self.odom_track = deque(maxlen=2000)

    def reset(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_vel_t = None
        self.odom_track.clear()
        self.data['imu_yaw_init_rad'] = None

    def feed(self, data):
        self.buf.extend(data)
        while True:
            while self.buf and self.buf[0] != 0xFD:
                del self.buf[0]
            if len(self.buf) < 5:
                break
            key = tuple(self.buf[:3])
            lens = {
                (0xFD,0x0A,0x02):13,
                (0xFD,0x09,0x03):12,
                (0xFD,0x02,0x04):5,
                (0xFD,0x0A,0x05):13,
                (0xFD,0x1F,0x06):34,
                (0xFD,0x02,0x07):5,
            }
            if key not in lens:
                del self.buf[0]
                continue
            n = lens[key]
            if len(self.buf) < n:
                break
            frame = bytes(self.buf[:n])
            del self.buf[:n]
            if not xor_check(frame):
                continue
            self.parse_frame(frame)

    def parse_frame(self, f):
        key = tuple(f[:3])
        if key == (0xFD,0x0A,0x02):
            vx = signed_word(f[3],f[4],f[5],1000.0)
            vy = signed_word(f[6],f[7],f[8],1000.0)
            wz = signed_word(f[9],f[10],f[11],100.0)
            if abs(vx) <= 0.01: vx = 0.0
            if abs(vy) <= 0.01: vy = 0.0
            if abs(wz) <= 0.01: wz = 0.0
            self.data.update({'vx':vx,'vy':vy,'wz':wz})
            self.integrate(vx, vy)
        elif key == (0xFD,0x09,0x03):
            self.data.update({
                'v1':(f[3]*256+f[4])/1000.0,
                'v2':(f[5]*256+f[6])/1000.0,
                'v3':(f[7]*256+f[8])/1000.0,
                'v4':(f[9]*256+f[10])/1000.0,
            })
        elif key == (0xFD,0x02,0x04):
            self.data['battery'] = f[3] * 0.1
        elif key == (0xFD,0x0A,0x05):
            yaw = signed_word(f[9],f[10],f[11],100.0) * math.pi / 180.0
            if yaw < 0:
                yaw += 2 * math.pi
            if self.data['imu_yaw_init_rad'] is None:
                self.data['imu_yaw_init_rad'] = yaw
                rel = 0.0
            else:
                rel = -(yaw - self.data['imu_yaw_init_rad'])
                if rel > math.pi: rel -= 2 * math.pi
                if rel < -math.pi: rel += 2 * math.pi
            self.data['imu_yaw_rel_rad'] = rel
            self.odom_yaw = rel
        elif key == (0xFD,0x1F,0x06):
            # Values appear scaled by 1000 in firmware parser. If firmware sends m/s^2, |G| uses /9.81.
            self.data.update({
                'gyro_x':-signed_word(f[3],f[4],f[5],1000.0),
                'gyro_y': signed_word(f[6],f[7],f[8],1000.0),
                'gyro_z': signed_word(f[9],f[10],f[11],1000.0),
                'acc_x': -signed_word(f[12],f[13],f[14],1000.0),
                'acc_y':  signed_word(f[15],f[16],f[17],1000.0),
                'acc_z':  signed_word(f[18],f[19],f[20],1000.0),
            })
        elif key == (0xFD,0x02,0x07):
            self.data['car_type'] = f[3]

    def integrate(self, vx, vy):
        now = time.time()
        if self.last_vel_t is None:
            self.last_vel_t = now
            return
        dt = now - self.last_vel_t
        self.last_vel_t = now
        if dt <= 0 or dt > 0.2:
            dt = 1.0 / 50.0
        yaw = self.odom_yaw
        dx = 1.061 * dt * (vx * math.cos(yaw) - vy * math.sin(yaw))
        dy = 1.077 * dt * (vx * math.sin(yaw) + vy * math.cos(yaw))
        self.odom_x += dx
        self.odom_y += dy
        self.odom_track.append((self.odom_x,self.odom_y))

# ============================================================
# Vision helpers
# ============================================================

class MarkerDetector:
    def __init__(self):
        self.enabled = cv2 is not None and np is not None and hasattr(cv2, 'aruco')
        self.aruco_detector = None
        self.apriltag_detector = None
        if not self.enabled:
            return
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        if hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
            at_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.apriltag_detector = cv2.aruco.ArucoDetector(at_dict, cv2.aruco.DetectorParameters())

    def _detect_with(self, detector, gray):
        if detector is None:
            return {}, [], None
        corners, ids, rejected = detector.detectMarkers(gray)
        out = {}
        if ids is None:
            return out, corners, ids
        ids = ids.flatten()
        for c, mid in zip(corners, ids):
            pts = c.reshape(4,2).astype(np.float32)
            out[int(mid)] = {'corners':pts, 'center':pts.mean(axis=0)}
        return out, corners, ids

    def detect(self, frame, detect_aruco=True, detect_apriltag=True):
        if not self.enabled:
            return {}, {}, [], None, [], None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_map, aruco_corners, aruco_ids = {}, [], None
        april_map, april_corners, april_ids = {}, [], None
        if detect_aruco:
            aruco_map, aruco_corners, aruco_ids = self._detect_with(self.aruco_detector, gray)
        if detect_apriltag and self.apriltag_detector is not None:
            april_map, april_corners, april_ids = self._detect_with(self.apriltag_detector, gray)
        return aruco_map, april_map, aruco_corners, aruco_ids, april_corners, april_ids


def compute_homography(aruco_map):
    if np is None or cv2 is None:
        return None
    img_pts, world_pts = [], []
    for mid, world_xy in FIELD_MARKER_WORLD.items():
        if mid not in aruco_map:
            return None
        img_pts.append(aruco_map[mid]['center'])
        world_pts.append(world_xy)
    return cv2.findHomography(np.array(img_pts,dtype=np.float32), np.array(world_pts,dtype=np.float32))[0]


def img_to_world(H, pt):
    p = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    w = cv2.perspectiveTransform(p,H)[0,0]
    return float(w[0]), float(w[1])


def estimate_yaw_world(H, corners):
    center = corners.mean(axis=0)
    top_mid = (corners[0] + corners[1]) / 2.0
    c = np.array(img_to_world(H, center))
    t = np.array(img_to_world(H, top_mid))
    v = t - c
    if np.linalg.norm(v) < 1e-6:
        return None
    return math.atan2(v[1], v[0])

# ============================================================
# Main application
# ============================================================

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('M100 Control Center V4.1.0 Modular + LD14')
        self.geometry('1710x990')
        self.minsize(1710, 990)
        # self.maxsize(1710, 990)
        # self.resizable(False, False)

        self.mode_var = tk.StringVar(value='Mode 6')
        self.hub = SerialHub()
        self.parser = BaseFeedbackParser()
        self.detector = MarkerDetector()

        self.cap = None
        self.camera_running = False
        # Camera capture request. These values are exposed on the top toolbar.
        # If a camera/driver cannot honor these values, the actual value is read back
        # and shown in the green overlay on the camera view.
        self.cam_width_var = tk.IntVar(value=1280)
        self.cam_height_var = tk.IntVar(value=720)
        self.cam_fps_var = tk.IntVar(value=30)
        self.cam_actual_w = 0
        self.cam_actual_h = 0
        self.cam_actual_fps = 0.0
        self.cam_frame_count = 0
        self.cam_fps_t0 = time.time()
        self.video_display_info = None  # (offset_x, offset_y, scale, frame_w, frame_h)
        self.last_frame = None
        self.last_H = None
        self.car_pose = None
        self.target_xy = None
        self.vision_track = deque(maxlen=2000)
        # G-force trace for the right-side G panel. Points fade out after about 2 seconds.
        self.g_trace = deque(maxlen=160)  # items: (timestamp, gx, gy, g_abs)
        self.max_g_this_run = 0.0
        # Go-to timing. Used to measure how fast the car reaches the selected target.
        self.goto_running = False
        self.goto_start_t = None
        self.goto_elapsed = None
        self.goto_timer_var = tk.StringVar(value='Go Timer: --')
        self.goto_controller = GotoController(max_target_raw=MAX_TARGET)
        self.vision_goto_track = deque(maxlen=3000)
        self.goto_start_pose = None
        self.goto_stop_pose = None
        self.goto_last_result = None
        self.goto_result = {
            'mode': '--', 'state': 'Idle', 'target': None,
            'start': None, 'stop': None, 'error_m': None,
            'elapsed_s': None, 'max_g': None
        }
        self.goto_records = []
        self.goto_record_seq = 0

        # V4.0.3: field homography is locked after the first full ArUco 0/1/2/3 detection.
        # This avoids losing the map when a corner tag is briefly occluded by the car or detection flickers.
        self.field_H = None
        self.field_markers_locked = False
        self.field_marker_img_pts = None
        self.field_lock_time = None

        self.closed_loop_running = False
        self.last_send_t = 0.0
        self.key_down = set()
        self.last_key_send = 0.0
        self.joy_active = False
        self.manual_rotate_dir = 0
        self.joy_vx = 0.0
        self.joy_vy = 0.0
        self.joy_wz = 0.0
        self.joy_disp_x = 0.0
        self.joy_disp_y = 0.0

        # Gamepad state for Beitong Asura 2 Pro with vendor driver.
        # Observed mapping: A0/A1=left stick, A2/A3=right stick, A4=LT, A5=RT.
        # LT/RT range is -1.0 released to +1.0 fully pressed.
        self.gamepad_enabled = tk.BooleanVar(value=True)
        self.gamepad_status_var = tk.StringVar(value='Gamepad: not initialized')
        self.gamepad_live_var = tk.StringVar(value='RX=0.00 RT=-1.00 Speed=20%')
        self.gamepad_deadzone_var = tk.DoubleVar(value=0.12)
        self.gamepad_wz_max_var = tk.DoubleVar(value=1.0)
        self.gamepad_min_speed_pct_var = tk.DoubleVar(value=20.0)
        self.gamepad = None
        self.gamepad_last_send_t = 0.0
        self.gamepad_last_estop_down = False
        self.gamepad_last_nonzero = False
        self.gamepad_key_down = set()
        self.gamepad_speed_scale = 1.00
        self.gamepad_current_speed_pct_var = tk.StringVar(value='Key/D-pad speed: 20%')
        self.key_speed_pct_value = 20.0

        # V4.0.6 battery display filter. ESP32 ADC raw readings can jitter even when
        # the actual divider voltage is stable on a multimeter. Keep raw calculation,
        # but display smoothed voltage and battery percent to avoid UI flicker.
        self.battery_voltage_filtered = None
        self.battery_pct_display = None
        self.battery_last_source = ''

        self.rxbuf = bytearray()
        self.pid_rxbuf = bytearray()
        self.base_rxbuf = bytearray()
        self.pid_hist = [{'a':deque(maxlen=PID_HISTORY_LEN),'t':deque(maxlen=PID_HISTORY_LEN),'e':deque(maxlen=PID_HISTORY_LEN)} for _ in range(4)]
        self.pid_live = [{'actual':None, 'target':None, 'error':None, 'rate':0.0, 'last_data_t':None, 'alarm_start':None} for _ in range(4)]
        self.data_count = [0,0,0,0]
        self.data_rate = [0,0,0,0]
        self.last_rate_t = time.time()
        self.capture_next_cmd = False
        self.last_cmd_rx = None
        self.pending_read = {}  # cmd_id -> channel
        self.pid_target_after_id = None
        self.pid_target_dragging = False
        self.gamepad_dpad_last_nonzero = False
        self.pid_curve_mode_var = tk.StringVar(value='4 Wheels')
        # Connection UI state. Serial keeps the original COM path. WiFi connects to ESP32 UDP bridge.
        self.conn_type_var = tk.StringVar(value='WiFi')
        self.wifi_ip_var = tk.StringVar(value='192.168.3.83')
        self.wifi_port_var = tk.IntVar(value=4210)

        # BNO085 display zero reference.
        # On WiFi connection, the first valid BNO085 frame becomes yaw/pitch/roll = 0.
        self.att_zero_pending = False
        self.att_zero_valid = False
        self.att_zero_yaw = 0.0
        self.att_zero_pitch = 0.0
        self.att_zero_roll = 0.0

        # V4.0.8 Strategy UI: runtime-selectable GoTo strategy and parameters.
        self.goto_strategy_var = tk.StringVar(value='rcpd')
        self.goto_strategy_logic_var = tk.StringVar(value='')
        self.goto_param1_label_var = tk.StringVar(value='P1')
        self.goto_param2_label_var = tk.StringVar(value='P2')
        self.goto_param3_label_var = tk.StringVar(value='P3')
        self.goto_param4_label_var = tk.StringVar(value='P4')
        self.goto_param5_label_var = tk.StringVar(value='P5')
        # V4.0.8 RCPD strategy parameters.
        # RCPD = Ramp + Cruise + P Brake.
        # Common Kp field is used as RCPD brake gain. Recommended first value: 4.0.
        self.rcpd_launch_time_var = tk.DoubleVar(value=0.10) # seconds. Short anti-slip launch window
        self.rcpd_launch_speed_var = tk.DoubleVar(value=0.55)# command cap during launch window
        self.rcpd_accel_rate_var = tk.DoubleVar(value=8.0)   # command units per second; 20Hz -> 0.40/update
        self.rcpd_slow_radius_var = tk.DoubleVar(value=0.12) # compact brake zone for 1.2m field
        self.rcpd_min_cmd_var = tk.DoubleVar(value=0.14)     # minimum brake-zone command
        self.slow_radius_var = tk.DoubleVar(value=0.25)
        self.min_cmd_var = tk.DoubleVar(value=0.15)
        self.slow_curve_power_var = tk.DoubleVar(value=1.0)
        self.pd_kp_var = tk.DoubleVar(value=1.8)
        self.pd_kd_var = tk.DoubleVar(value=0.45)
        self.pd_min_cmd_var = tk.DoubleVar(value=0.00)
        self.brake_decel_var = tk.DoubleVar(value=2.5)
        self.brake_buffer_var = tk.DoubleVar(value=0.06)
        self.brake_min_cmd_var = tk.DoubleVar(value=0.12)
        self.bang_slow_radius_var = tk.DoubleVar(value=0.25)
        self.bang_near_kp_var = tk.DoubleVar(value=2.0)
        self.bang_reverse_gain_var = tk.DoubleVar(value=0.35)

        # V4.1.0: initialize LD14 before build_ui() creates Tab3.
        self.lidar_service = LidarService(
            LIDAR_CONFIG.udp_port,
            LIDAR_CONFIG.offline_timeout_s,
        )
        self.lidar_guard = LidarCollisionGuard(LIDAR_CONFIG)
        self.lidar_service.start()
        self.build_ui()
        self.bind_all('<KeyPress>', self.on_key_press)
        self.bind_all('<KeyRelease>', self.on_key_release)
        self.protocol('WM_DELETE_WINDOW', self.on_close)
        self.after(10, self.serial_loop)
        self.after(UI_LOOP_DELAY_MS, self.ui_loop)

    # ---------------- UI ----------------
    def build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill='x', padx=10, pady=6)
        # ttk.Label(top, text='M100 Control Center V4.0.9 Launch-RCPD Strategy UI Vision Goto', font=('Segoe UI',18,'bold')).pack(side='left')

        ttk.Label(top, text='Conn').pack(side='left', padx=(8,4))
        self.conn_box = ttk.Combobox(top, textvariable=self.conn_type_var, values=['Serial','WiFi'], state='readonly', width=7)
        self.conn_box.pack(side='left')
        ttk.Label(top, text='COM').pack(side='left', padx=(5,4))
        self.port_var = tk.StringVar(value='COM3')
        self.port_box = ttk.Combobox(top,textvariable=self.port_var,width=8)
        self.port_box.pack(side='left')
        ttk.Button(top,text='Refresh',command=self.refresh_ports).pack(side='left',padx=3)
        ttk.Label(top,text='Baud').pack(side='left',padx=(5,4))
        self.baud_var=tk.StringVar(value='115200')
        ttk.Entry(top,textvariable=self.baud_var,width=7).pack(side='left')
        ttk.Label(top,text='IP').pack(side='left',padx=(5,2))
        ttk.Entry(top,textvariable=self.wifi_ip_var,width=12).pack(side='left')
        ttk.Label(top,text='UDP').pack(side='left',padx=(3,2))
        ttk.Entry(top,textvariable=self.wifi_port_var,width=5).pack(side='left')
        self.connect_btn=ttk.Button(top,text='Connect',command=self.toggle_serial)
        self.connect_btn.pack(side='left',padx=2)
        ttk.Button(top,text='Zero 085',command=self.request_attitude_zero).pack(side='left',padx=2)
        ttk.Button(top,text='Clear RX',command=self.clear_rx_buffer).pack(side='left',padx=2)

        ttk.Label(top,text='Camera').pack(side='left',padx=(18,4))
        self.cam_var=tk.IntVar(value=0)
        ttk.Entry(top,textvariable=self.cam_var,width=4).pack(side='left')
        # Camera capture request. Keep these fields small so the toolbar still fits 1710px.
        ttk.Label(top,text='W').pack(side='left',padx=(5,2))
        ttk.Entry(top,textvariable=self.cam_width_var,width=5).pack(side='left')
        ttk.Label(top,text='H').pack(side='left',padx=(3,2))
        ttk.Entry(top,textvariable=self.cam_height_var,width=5).pack(side='left')
        ttk.Label(top,text='FPS').pack(side='left',padx=(3,2))
        ttk.Entry(top,textvariable=self.cam_fps_var,width=4).pack(side='left')
        self.cam_btn=ttk.Button(top,text='Start Camera',command=self.toggle_camera)
        self.cam_btn.pack(side='left',padx=8)

        ttk.Label(top,text='Mode').pack(side='left',padx=(8,4))
        self.mode_box=ttk.Combobox(top,textvariable=self.mode_var,values=['Mode 6','Mode 0'],state='readonly',width=8)
        self.mode_box.pack(side='left')
        # ttk.Button(top,text='STOP',command=self.stop_motion).pack(side='left',padx=10)
        self.estop_btn = tk.Button(top,text='E-STOP',bg='#FF3030',fg='white',activebackground='#CC0000',activeforeground='white',font=('Segoe UI',11,'bold'),width=8,relief='raised',command=self.stop_motion)
        self.estop_btn.pack(side='left', padx=20)

        self.battery_canvas=tk.Canvas(top,width=120,height=34,highlightthickness=0)
        self.battery_canvas.pack(side='right')

        self.status_var=tk.StringVar(value=f'Ready. Field {FIELD_WIDTH_M:.3f}m x {FIELD_HEIGHT_M:.3f}m')
        ttk.Label(self,textvariable=self.status_var,foreground='blue').pack(fill='x',padx=14)

        self.nb=ttk.Notebook(self)
        self.nb.pack(fill='both',expand=True,padx=10,pady=8)
        self.tab1=ttk.Frame(self.nb)
        self.tab2=ttk.Frame(self.nb)
        self.tab3=ttk.Frame(self.nb)
        self.tab4=ttk.Frame(self.nb)
        self.nb.add(self.tab1,text='Tab1 Control Center')
        self.nb.add(self.tab2,text='Tab2 PID Debug')
        self.nb.add(self.tab3,text='Tab3 LD14 Radar')
        self.nb.add(self.tab4,text='Tab4 Logs / Export')
        self.build_tab1()
        self.build_tab2()
        self.build_tab3()
        self.build_tab4()
        self.refresh_ports()
        self.init_gamepad()

    def make_monitor_cluster(self,parent):
        parent.columnconfigure(0,weight=4)
        parent.columnconfigure(1,weight=1)
        parent.rowconfigure(0,weight=1)
        map_box=ttk.LabelFrame(parent,text='Large Vision vs Odom')
        map_box.grid(row=0,column=0,rowspan=2,sticky='nsew',padx=(0,8),pady=(0,8))
        map_canvas=tk.Canvas(map_box,bg='#111111',height=460)
        map_canvas.pack(fill='both',expand=True,padx=6,pady=6)

        side=ttk.Frame(parent)
        side.grid(row=0,column=1,sticky='nsew',pady=(0,8))
        gbox=ttk.LabelFrame(side,text='G Force + |G|')
        gbox.pack(fill='x',pady=(0,8))
        g_canvas=tk.Canvas(gbox,width=190,height=190,bg='#0b0b0b')
        g_canvas.pack(anchor='center',padx=4,pady=4)
        wbox=ttk.LabelFrame(side,text='Wheel Status')
        wbox.pack(fill='both',expand=True)
        wheel_canvas=tk.Canvas(wbox,width=210,height=240,bg='white')
        wheel_canvas.pack(fill='both',expand=True,padx=4,pady=4)

        sbox=ttk.LabelFrame(parent,text='Status Table')
        sbox.grid(row=2,column=0,columnspan=2,sticky='nsew')
        status_canvas=tk.Canvas(sbox,height=185,bg='white')
        status_canvas.pack(fill='both',expand=True,padx=6,pady=6)
        return map_canvas,g_canvas,wheel_canvas,status_canvas

    def build_tab1(self):
        """Fixed-position Control Center layout for a 1710x990 window.

        All important geometry numbers are grouped below. Adjust these constants first
        when tuning the UI. Coordinates are relative to the inside of Tab1, not the
        whole application window.
        """
        # ===== Fixed layout constants, tuned for 1710x990 main window =====
        # Left visual block. Camera width is intentionally equal to Controls + Joystick.
        LEFT_X = 10
        TOP_Y = 8
        LEFT_W = 1000 # 1110
        CAM_H = 620 #650
        BOTTOM_Y = TOP_Y + CAM_H + 10
        BOTTOM_H = 230 #170
        CTRL_W = 780 
        JOY_W = LEFT_W - CTRL_W - 10
        JOY_SIZE = 190 #160

        # Middle map block. MAP_W is fixed so the map remains predictable.
        MAP_X = LEFT_X + LEFT_W + 12
        MAP_Y = TOP_Y
        MAP_W = 500
        MAP_CANVAS_H = 450
        POS_H = 136
        # TIMER_H = 34

        # Right telemetry column. Reserved attitude panel is for future BNO085.
        SIDE_X = MAP_X + MAP_W + 12
        SIDE_Y = TOP_Y
        SIDE_W = 150
        G_SIZE = 120
        ATT_H = 216  # V3.9.9.5: 180% of previous 120px attitude panel
        WHEEL_H = 244  # reduced to keep right telemetry column within the same layout

        # Status table uses the lower-right space under the map/telemetry area.
        STATUS_X = MAP_X
        STATUS_Y = BOTTOM_Y
        STATUS_W = MAP_W + 12 + SIDE_W
        STATUS_H = BOTTOM_H

        # --- Camera view. Use Canvas instead of Label so we can overlay green camera info.
        cam_box = ttk.LabelFrame(self.tab1, text='Camera View')
        cam_box.place(x=LEFT_X, y=TOP_Y, width=LEFT_W, height=CAM_H)
        self.video_label = tk.Canvas(cam_box, bg='black', highlightthickness=0)
        self.video_label.place(x=4, y=4, width=LEFT_W-10, height=CAM_H-28)
        self.video_label.bind('<Button-1>', self.on_camera_click)

        # --- Vision / Odom map, position command, and go-to stopwatch.
        map_box = ttk.LabelFrame(self.tab1, text='Vision / Odom Map')
        map_box.place(x=MAP_X, y=MAP_Y, width=MAP_W, height=MAP_CANVAS_H+32)
        self.map_canvas = tk.Canvas(map_box, bg='#111111', highlightthickness=0)
        self.map_canvas.place(x=6, y=6, width=MAP_W-14, height=MAP_CANVAS_H)
        self.map_canvas.bind('<Button-1>', self.on_map_click)

        pos = ttk.LabelFrame(self.tab1, text='Position Command')
        pos.place(x=MAP_X, y=MAP_Y+MAP_CANVAS_H+42, width=MAP_W, height=POS_H)
        build_position_command_panel(self, pos, FIELD_WIDTH_M, FIELD_HEIGHT_M, MAX_MANUAL_SPEED_MPS)

        # --- Right telemetry column: G force, reserved attitude, wheels.
        gbox = ttk.LabelFrame(self.tab1, text='G Force')
        gbox.place(x=SIDE_X, y=SIDE_Y, width=SIDE_W, height=G_SIZE+28)
        self.g_canvas = tk.Canvas(gbox, bg='#0b0b0b', highlightthickness=0)
        self.g_canvas.place(x=4, y=4, width=SIDE_W-10, height=G_SIZE)

        att_box = ttk.LabelFrame(self.tab1, text='3D Attitude')
        att_box.place(x=SIDE_X, y=SIDE_Y+G_SIZE+34, width=SIDE_W, height=ATT_H)
        self.att_canvas = tk.Canvas(att_box, bg='#111827', highlightthickness=0)
        self.att_canvas.place(x=4, y=4, width=SIDE_W-10, height=ATT_H-28)

        wbox = ttk.LabelFrame(self.tab1, text='Wheels')
        wbox.place(x=SIDE_X, y=SIDE_Y+G_SIZE+34+ATT_H+6, width=SIDE_W, height=WHEEL_H)
        self.wheel_canvas = tk.Canvas(wbox, bg='white', highlightthickness=0)
        self.wheel_canvas.place(x=4, y=4, width=SIDE_W-10, height=WHEEL_H-28)

        # --- Bottom controls. Control width + joystick width matches camera width.
        ctrl = ttk.LabelFrame(self.tab1, text='Controls')
        ctrl.place(x=LEFT_X, y=BOTTOM_Y, width=CTRL_W, height=BOTTOM_H)
        self.jog_speed_var = tk.DoubleVar(value=MAX_MANUAL_SPEED_MPS)
        self.wasd_btns = {}
        pad = ttk.Frame(ctrl)
        pad.place(x=8, y=18, width=150, height=180)
        self.wasd_btns['w'] = tk.Button(pad, text='↑', font=('Segoe UI',12,'bold'), width=3,
                                        command=lambda:self.send_motion(self.jog_speed_var.get()*self.current_speed_scale(),0,0))
        self.wasd_btns['w'].grid(row=0, column=1, padx=2, pady=2)
        self.wasd_btns['a'] = tk.Button(pad, text='←', font=('Segoe UI',12,'bold'), width=3,
                                        command=lambda:self.send_motion(0,self.jog_speed_var.get()*self.current_speed_scale(),0))
        self.wasd_btns['a'].grid(row=1, column=0, padx=2, pady=2)
        self.wasd_btns['d'] = tk.Button(pad, text='→', font=('Segoe UI',12,'bold'), width=3,
                                        command=lambda:self.send_motion(0,-self.jog_speed_var.get()*self.current_speed_scale(),0))
        self.wasd_btns['d'].grid(row=1, column=2, padx=2, pady=2)
        self.wasd_btns['s'] = tk.Button(pad, text='↓', font=('Segoe UI',12,'bold'), width=3,
                                        command=lambda:self.send_motion(-self.jog_speed_var.get()*self.current_speed_scale(),0,0))
        self.wasd_btns['s'].grid(row=2, column=1, padx=2, pady=2)
        self.turn_left_btn = tk.Button(pad, text='↺', font=('Segoe UI',12,'bold'), width=3)
        self.turn_left_btn.grid(row=2, column=0, padx=2, pady=2)
        self.turn_right_btn = tk.Button(pad, text='↻', font=('Segoe UI',12,'bold'), width=3)
        self.turn_right_btn.grid(row=2, column=2, padx=2, pady=2)
        self.turn_left_btn.bind('<ButtonPress-1>', lambda e: self.manual_rotate_press(+1))
        self.turn_left_btn.bind('<ButtonRelease-1>', lambda e: self.manual_rotate_release())
        self.turn_right_btn.bind('<ButtonPress-1>', lambda e: self.manual_rotate_press(-1))
        self.turn_right_btn.bind('<ButtonRelease-1>', lambda e: self.manual_rotate_release())
        self.estop_btn2 = tk.Button(pad, text='E-STOP', bg='#FF3030', fg='white', activebackground='#CC0000',
                                    activeforeground='white', font=('Segoe UI',9,'bold'), width=12,
                                    relief='raised', command=self.stop_motion)
        self.estop_btn2.grid(row=3, column=0, columnspan=3, padx=2, pady=15, sticky='ew')

        # Make on-screen WASD buttons behave like real keys: press = move, release = stop.
        # The Button.command callback fires on mouse release, so disable it here and use bindings instead.
        for key, btn in self.wasd_btns.items():
            btn.configure(command=lambda: None)
            btn.bind('<ButtonPress-1>', lambda e, k=key: self.manual_button_press(k))
            btn.bind('<ButtonRelease-1>', lambda e, k=key: self.manual_button_release(k))

        # Slider area. Min speed is represented by the speed bar itself, so no extra Min% slider.
        settings = ttk.Frame(ctrl)
        settings.place(x=170, y=16, width=CTRL_W-180, height=BOTTOM_H-40)
        self.speed_canvas = tk.Canvas(settings, height=42, bg='white', highlightthickness=1, highlightbackground='#cbd5e1')
        self.speed_canvas.place(x=0, y=0, width=CTRL_W-190, height=42)
        self.speed_canvas.bind('<Button-1>', self.on_speed_bar_drag)
        self.speed_canvas.bind('<B1-Motion>', self.on_speed_bar_drag)

        ttk.Label(settings, text='Dead Zone').place(x=0, y=64)
        self.deadzone_scale = tk.Scale(settings, from_=0.00, to=0.30, orient='horizontal', variable=self.gamepad_deadzone_var,
                                        showvalue=True, resolution=0.01, length=460)
        self.deadzone_scale.place(x=95, y=42, width=470, height=42)

        ttk.Label(settings, text='Max WZ').place(x=0, y=106)
        self.max_wz_scale = tk.Scale(settings, from_=0.0, to=5.0, orient='horizontal', variable=self.gamepad_wz_max_var,
                                     showvalue=True, resolution=0.1, length=460)
        self.max_wz_scale.place(x=95, y=84, width=470, height=42)

        ttk.Checkbutton(settings, text='Gamepad Enable', variable=self.gamepad_enabled).place(x=0, y=144)
        # ttk.Button(settings, text='Refresh', command=self.init_gamepad).place(x=500, y=140, width=75, height=28)
        ttk.Label(settings, textvariable=self.gamepad_live_var, foreground='#0f766e', wraplength=560).place(x=140, y=145)

        joy = ttk.LabelFrame(self.tab1, text='Virtual Joystick')
        joy.place(x=LEFT_X+CTRL_W+10, y=BOTTOM_Y, width=JOY_W, height=BOTTOM_H)
        self.joy_canvas = tk.Canvas(joy, bg='#0b0b0b', highlightthickness=0)
        self.joy_canvas.place(x=8, y=8, width=JOY_SIZE, height=JOY_SIZE)
        self.joy_canvas.bind('<ButtonPress-1>', self.on_joy_down)
        self.joy_canvas.bind('<B1-Motion>', self.on_joy_drag)
        self.joy_canvas.bind('<ButtonRelease-1>', self.on_joy_up)

        status_box = ttk.LabelFrame(self.tab1, text='Status Table')
        status_box.place(x=STATUS_X, y=STATUS_Y, width=STATUS_W, height=STATUS_H)
        self.status_canvas = tk.Canvas(status_box, bg='white', highlightthickness=0)
        self.status_canvas.place(x=6, y=8, width=STATUS_W-16, height=STATUS_H-34)

    def build_tab2(self):
        self.tab2.columnconfigure(0, weight=0)
        self.tab2.columnconfigure(1, weight=1)
        self.tab2.rowconfigure(0, weight=1)

        left=ttk.Frame(self.tab2)
        left.grid(row=0,column=0,sticky='ns',padx=10,pady=10)

        param=ttk.LabelFrame(left,text='Mode 0 PID Parameters')
        param.pack(fill='x',pady=(0,8))
        self.pid_ch=tk.IntVar(value=0)
        ttk.Label(param,text='Channel').grid(row=0,column=0,sticky='w',padx=6,pady=4)
        for i in range(4):
            ttk.Radiobutton(param,text=WHEEL_LABEL_BY_CH[i].replace(' / ', '\n'),variable=self.pid_ch,value=i,command=self.refresh_pid_labels).grid(row=0,column=i+1,padx=4)

        self.pid_vars={name:tk.DoubleVar(value=0.0) for name in ['P','I','D']}
        cmd_ids={'P':1,'I':2,'D':3}
        r=1
        for name,var in self.pid_vars.items():
            ttk.Label(param,text=name).grid(row=r,column=0,pady=4,sticky='w',padx=6)
            ttk.Entry(param,textvariable=var,width=12).grid(row=r,column=1,columnspan=2,padx=4)
            ttk.Button(param,text='Read',command=lambda c=cmd_ids[name], n=name:self.pid_read(c,n)).grid(row=r,column=3,padx=3)
            ttk.Button(param,text='Write',command=lambda c=cmd_ids[name], v=var, n=name:self.pid_write(c,v.get(),n)).grid(row=r,column=4,padx=3)
            r+=1

        self.pid_target_pct_var = tk.DoubleVar(value=0.0)
        self.pid_target_pct_label = tk.StringVar(value='Target: 0%')
        ttk.Label(param,textvariable=self.pid_target_pct_label).grid(row=r,column=0,columnspan=5,sticky='w',padx=6,pady=(8,0))
        r+=1
        self.pid_target_scale = tk.Scale(param, from_=-100, to=100, orient='horizontal',
                                         variable=self.pid_target_pct_var, resolution=10,
                                         showvalue=True, length=270,
                                         command=self.on_pid_target_slider_changed)
        self.pid_target_scale.grid(row=r,column=0,columnspan=5,sticky='ew',padx=6,pady=4)
        self.pid_target_scale.bind('<ButtonPress-1>', self.on_pid_target_slider_press)
        self.pid_target_scale.bind('<ButtonRelease-1>', self.on_pid_target_slider_release)
        r+=1
        ttk.Button(param,text='Read Target',command=lambda:self.pid_read(4,'Target')).grid(row=r,column=0,columnspan=2,sticky='ew',padx=4,pady=4)
        ttk.Button(param,text='Write Target Now',command=self.pid_target_commit_now).grid(row=r,column=2,columnspan=3,sticky='ew',padx=4,pady=4)
        r+=1

        self.pid_actual=tk.StringVar(value='Actual: --')
        self.pid_error=tk.StringVar(value='Error: --')
        ttk.Label(param,textvariable=self.pid_actual).grid(row=r,column=0,columnspan=3,sticky='w',padx=6,pady=6)
        ttk.Button(param,text='Read Actual',command=lambda:self.pid_read(6,'Actual')).grid(row=r,column=3,columnspan=2,sticky='ew',padx=3)
        r+=1
        ttk.Label(param,textvariable=self.pid_error,foreground='red').grid(row=r,column=0,columnspan=5,sticky='w',padx=6,pady=4)
        r+=1
        ttk.Button(param,text='STOP ALL: Target 0%',command=self.pid_stop_all).grid(row=r,column=0,columnspan=5,sticky='ew',padx=4,pady=6)

        global_box=ttk.LabelFrame(left,text='Global PID')
        global_box.pack(fill='x',pady=(0,8))
        self.global_p=tk.DoubleVar(value=1.25)
        self.global_i=tk.DoubleVar(value=0.76)
        self.global_d=tk.DoubleVar(value=0.25)
        for i,(name,var) in enumerate([('P',self.global_p),('I',self.global_i),('D',self.global_d)]):
            ttk.Label(global_box,text=name).grid(row=0,column=i*2,padx=4,pady=4)
            ttk.Entry(global_box,textvariable=var,width=8).grid(row=0,column=i*2+1,padx=4,pady=4)
        ttk.Button(global_box,text='Apply P/I/D To All',command=self.apply_pid_all).grid(row=1,column=0,columnspan=6,sticky='ew',padx=6,pady=4)
        ttk.Button(global_box,text='Read P/I/D All',command=self.read_pid_all).grid(row=2,column=0,columnspan=6,sticky='ew',padx=6,pady=4)

        filters=ttk.LabelFrame(left,text='Log Filter')
        filters.pack(fill='x',pady=(0,8))
        self.show_tx=tk.BooleanVar(value=True)
        self.show_cmd_rx=tk.BooleanVar(value=True)
        self.show_data_rx=tk.BooleanVar(value=False)
        ttk.Checkbutton(filters,text='Show TX',variable=self.show_tx).pack(anchor='w',padx=8,pady=2)
        ttk.Checkbutton(filters,text='Show CMD RX',variable=self.show_cmd_rx).pack(anchor='w',padx=8,pady=2)
        ttk.Checkbutton(filters,text='Show DATA RX',variable=self.show_data_rx).pack(anchor='w',padx=8,pady=2)
        ttk.Button(filters,text='Capture Next CMD',command=self.on_capture_next).pack(fill='x',padx=8,pady=4)
        ttk.Button(filters,text='Clear PID Log',command=lambda:self.pid_text.delete('1.0','end')).pack(fill='x',padx=8,pady=4)

        monitor=ttk.LabelFrame(left,text='Data RX Rate')
        monitor.pack(fill='x')
        self.rate_var=tk.StringVar(value='CH1 --Hz | CH2 --Hz | CH3 --Hz | CH4 --Hz')
        ttk.Label(monitor,textvariable=self.rate_var,wraplength=330).pack(anchor='w',padx=8,pady=7)
        self.last_cmd_var=tk.StringVar(value='Last CMD RX: --')
        ttk.Label(monitor,textvariable=self.last_cmd_var,wraplength=330,foreground='purple').pack(anchor='w',padx=8,pady=(0,7))

        right=ttk.Frame(self.tab2)
        right.grid(row=0,column=1,sticky='nsew',padx=10,pady=10)
        right.rowconfigure(0,weight=0)
        right.rowconfigure(1,weight=4)
        right.rowconfigure(2,weight=1)
        right.columnconfigure(0,weight=1)

        status_box=ttk.LabelFrame(right,text='Four-wheel PID Status')
        status_box.grid(row=0,column=0,sticky='ew',pady=(0,8))
        self.pid_status_canvas=tk.Canvas(status_box,bg='white',height=130)
        self.pid_status_canvas.pack(fill='both',expand=True,padx=4,pady=4)

        curve_box=ttk.LabelFrame(right,text='PID Curve: Actual / Target (%)')
        curve_box.grid(row=1,column=0,sticky='nsew',pady=(0,8))
        curve_toolbar=ttk.Frame(curve_box)
        curve_toolbar.pack(fill='x',padx=6,pady=(4,0))
        ttk.Label(curve_toolbar,text='Curve View').pack(side='left',padx=(0,8))
        ttk.Radiobutton(curve_toolbar,text='Single CH',variable=self.pid_curve_mode_var,value='Single CH').pack(side='left',padx=4)
        ttk.Radiobutton(curve_toolbar,text='4 Wheels',variable=self.pid_curve_mode_var,value='4 Wheels').pack(side='left',padx=4)
        ttk.Label(curve_toolbar,text='Target=green  Actual=blue',foreground='#64748b').pack(side='right')
        self.pid_canvas=tk.Canvas(curve_box,bg='#111111',height=540)
        self.pid_canvas.pack(fill='both',expand=True,padx=6,pady=6)

        log_box=ttk.LabelFrame(right,text='Filtered PID Log')
        log_box.grid(row=2,column=0,sticky='nsew')
        self.pid_text=tk.Text(log_box,height=6,wrap='none')
        self.pid_text.pack(fill='both',expand=True,padx=6,pady=6)
        self.pid_text.tag_configure('tx',foreground='#004C99')
        self.pid_text.tag_configure('cmd',foreground='#7F1D99')
        self.pid_text.tag_configure('data',foreground='#777777')
        self.pid_text.tag_configure('capture',foreground='#FFFFFF',background='#7F1D99')
        self.pid_text.tag_configure('warn',foreground='#B45309')

    def build_tab3(self):
        self.ld14_tab = LD14Tab(self.tab3, self.lidar_service, self.lidar_guard, LIDAR_CONFIG)

    def build_tab4(self):
        """GoTo run recorder and export panel.

        V4.0.3 replaced generic communication logs with per-run GoTo records.
        These records are more useful for tuning the future goto strategy and
        for later integration into the main 100 Percent Hit program.
        """
        top=ttk.Frame(self.tab4)
        top.pack(fill='x',padx=10,pady=8)
        ttk.Button(top,text='Export GoTo CSV',command=self.export_goto_records_csv).pack(side='left',padx=4)
        ttk.Button(top,text='Clear Records',command=self.clear_goto_records).pack(side='left',padx=4)
        ttk.Label(top, text='Records include Go runs only, not Return. Strategy and parameters are recorded for RCPD tuning.', foreground='#475569').pack(side='left',padx=12)

        cols=('id','mode','state','strategy','target_x','target_y','time_s','error_cm','start_x','start_y','start_h','stop_x','stop_y','stop_h','kp','max','stop_r','max_g','params')
        self.goto_tree=ttk.Treeview(self.tab4, columns=cols, show='headings', height=20)
        headings={
            'id':'ID','mode':'Mode','state':'State','strategy':'Strategy','target_x':'Target X','target_y':'Target Y',
            'time_s':'Time s','error_cm':'Err cm','start_x':'Start X','start_y':'Start Y','start_h':'Start H',
            'stop_x':'Stop X','stop_y':'Stop Y','stop_h':'Stop H','kp':'Kp','max':'Max','stop_r':'Stop R','max_g':'Max G','params':'Strategy Params'
        }
        widths={
            'id':45,'mode':65,'state':80,'strategy':95,'target_x':75,'target_y':75,
            'time_s':70,'error_cm':70,'start_x':75,'start_y':75,'start_h':75,
            'stop_x':75,'stop_y':75,'stop_h':75,'kp':55,'max':55,'stop_r':65,'max_g':65,'params':360
        }
        for c in cols:
            self.goto_tree.heading(c, text=headings[c])
            self.goto_tree.column(c, width=widths[c], anchor='center', stretch=False)
        self.goto_tree.pack(fill='both',expand=True,padx=10,pady=(0,6))

        self.log_text=tk.Text(self.tab4, height=7)
        self.log_text.pack(fill='x',padx=10,pady=(0,8))
        self.log('GoTo recorder ready. Communication logs are no longer the focus in V4.0.3.')

    # ---------------- serial ----------------
    def refresh_ports(self):
        ports=[]
        if serial is not None:
            ports=[p.device for p in serial.tools.list_ports.comports()]
        self.port_box['values']=ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set('COM3' if 'COM3' in ports else ports[0])

    def toggle_serial(self):
        """Connect/disconnect either direct serial or ESP32 Wi-Fi bridge.

        Important: WiFi mode is a transparent byte transport. It does not change Mode 6
        motion frames or Mode 0 PID frames. The ESP32 should simply forward bytes between
        UDP and STM32 UART.
        """
        if self.hub.is_open():
            self.stop_motion()
            old_kind = self.hub.kind
            self.hub.close()
            self.connect_btn.config(text='Connect')
            self.status_var.set('Disconnected')
            self.log(f'{old_kind.upper()} disconnected')
        else:
            try:
                t = self.conn_type_var.get()
                if t == 'WiFi':
                    self.hub.connect_wifi(self.wifi_ip_var.get(), int(self.wifi_port_var.get()))
                    self.connect_btn.config(text='Disconnect')
                    self.status_var.set(f'UDP connected ESP32 {self.wifi_ip_var.get()}:{int(self.wifi_port_var.get())}, telemetry local 4211; waiting BNO085 zero')
                    self.clear_rx_buffer()
                    self.request_attitude_zero()
                    self.log(f'UDP connected ESP32 {self.wifi_ip_var.get()}:{int(self.wifi_port_var.get())}, telemetry local 4211; BNO085 zero pending')
                else:
                    self.hub.connect(self.port_var.get(), int(self.baud_var.get()))
                    self.connect_btn.config(text='Disconnect')
                    self.status_var.set(f'Serial connected {self.port_var.get()}')
                    self.clear_rx_buffer()
                    self.log(f'Serial connected {self.port_var.get()}')
            except Exception as e:
                messagebox.showerror('Connection error', str(e))
                self.status_var.set(f'Connection failed: {e}')

    def clear_rx_buffer(self):
        self.rxbuf.clear()
        self.pid_rxbuf.clear()
        self.base_rxbuf.clear()
        self.parser.buf.clear()
        self.hub.clear_input()
        if hasattr(self,'pid_text'):
            self.pid_log('RX buffer cleared\n','warn')

    def serial_loop(self):
        try:
            data=self.hub.read_available()
            if data:
                self.feed_serial(data)
        except Exception as e:
            self.status_var.set(str(e))
        self.after(10,self.serial_loop)

    def feed_serial(self,data:bytes):
        # Unified parser: PID frames and base feedback frames can be processed together.
        self.rxbuf.extend(data)
        while self.rxbuf:
            b=self.rxbuf[0]
            if b in PID_HEADS:
                if len(self.rxbuf) < 10:
                    break
                if self.rxbuf[9] != ord('\n'):
                    del self.rxbuf[0]
                    continue
                frame=bytes(self.rxbuf[:10])
                del self.rxbuf[:10]
                item=parse_pid_frame(frame)
                if item:
                    self.handle_pid_item(item)
                continue
            if b == 0xFD:
                # hand the full pending buffer to base parser. It has its own robust sync.
                before=len(self.rxbuf)
                self.parser.feed(bytes(self.rxbuf))
                self.rxbuf.clear()
                break
            del self.rxbuf[0]

    # ---------------- motion / keyboard / joystick ----------------
    def ensure_mode6(self):
        if self.mode_var.get()!='Mode 6':
            self.status_var.set('Switch to Mode 6 first.')
            return False
        return True

    def ensure_mode0(self):
        if self.mode_var.get()!='Mode 0':
            self.status_var.set('Switch to Mode 0 first.')
            return False
        return True

    def reset_estop_visual(self):
        for btn_name in ['estop_btn', 'estop_btn2']:
            if hasattr(self, btn_name):
                try:
                    getattr(self, btn_name).config(relief='raised', bg='#FF3030')
                except Exception:
                    pass

    def set_estop_visual(self):
        for btn_name in ['estop_btn', 'estop_btn2']:
            if hasattr(self, btn_name):
                try:
                    getattr(self, btn_name).config(relief='sunken', bg='#990000')
                except Exception:
                    pass

    def limit_mode6_xy_diamond(self, vx, vy):
        """Limit Mode 6 chassis vx/vy by a mecanum wheel-safe diamond envelope.

        The physical wheel demand of a mecanum base is roughly proportional to
        abs(vx) + abs(vy). Without this limit, full diagonal joystick can create
        wheel target higher than 1400 raw / 100% range.
        """
        try:
            vx = float(vx)
            vy = float(vy)
        except Exception:
            return 0.0, 0.0
        limit = float(MAX_MANUAL_SPEED_MPS)  # 1.4 protocol unit means 1400 raw wheel target
        load = abs(vx) + abs(vy)
        if load > limit and load > 1e-9:
            scale = limit / load
            vx *= scale
            vy *= scale
        return vx, vy

    def wrap_deg(self, angle):
        """Wrap degree angle to -180..+180."""
        try:
            angle = float(angle)
        except Exception:
            return 0.0
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def request_attitude_zero(self):
        """Use the next valid BNO085 packet as display zero.

        This does not recalibrate BNO085 internally. It only changes the upper
        computer's display reference so the current car pose becomes level and forward.
        """
        self.att_zero_pending = True
        self.att_zero_valid = False
        self.att_zero_yaw = 0.0
        self.att_zero_pitch = 0.0
        self.att_zero_roll = 0.0
        try:
            if hasattr(self, 'hub') and hasattr(self.hub, 'telemetry'):
                self.hub.telemetry['gyro_yaw_deg'] = 0.0
                self.hub.telemetry['last_gyro_t'] = None
        except Exception:
            pass
        try:
            self.max_g_this_run = 0.0
            if hasattr(self, 'g_trace'):
                self.g_trace.clear()
        except Exception:
            pass
        try:
            self.status_var.set('BNO085 zero pending: keep car level and forward; G max reset')
        except Exception:
            pass

    def send_motion(self,vx,vy,wz):
        if not self.ensure_mode6(): return
        # Wheel-safe diamond envelope: full stick in any direction keeps max wheel target at 1400 / 100%.
        # Examples: forward -> 1400/1400/1400/1400; 45deg -> opposite wheels reach 1400.
        vx, vy = self.limit_mode6_xy_diamond(vx, vy)
        if abs(vx) > 1e-5 or abs(vy) > 1e-5 or abs(wz) > 1e-5:
            self.reset_estop_visual()
        try:
            snap = self.lidar_service.snapshot() if LIDAR_CONFIG.enabled else None
            blocked = bool(snap and self.lidar_guard.evaluate(snap))
            if blocked and (abs(vx) > 1e-5 or abs(vy) > 1e-5 or abs(wz) > 1e-5):
                vx = vy = wz = 0.0
                self.status_var.set('LD14 protective STOP: obstacle in configured sector')
            self.hub.write(build_motion_cmd(vx,vy,wz))
        except Exception as e:
            self.status_var.set(str(e))

    def stop_motion(self):
        was_goto_running = getattr(self, 'closed_loop_running', False)
        if was_goto_running:
            try:
                self.goto_controller.stop()
            except Exception:
                pass
            if hasattr(self, 'goto_result'):
                self.goto_result['state'] = 'E-STOP'
                self.goto_result['stop'] = self.car_pose
                if self.goto_start_t is not None:
                    self.goto_result['elapsed_s'] = time.time() - self.goto_start_t
                if self.target_xy and self.car_pose:
                    self.goto_result['error_m'] = math.hypot(self.target_xy[0]-self.car_pose[0], self.target_xy[1]-self.car_pose[1])
                self.goto_result['max_g'] = getattr(self, 'max_g_this_run', 0.0)
                try:
                    self.append_goto_record('E-STOP')
                except Exception:
                    pass
        self.closed_loop_running=False
        self.goto_running=False
        self.joy_active=False
        self.manual_rotate_dir=0
        self.joy_wz=0.0
        self.joy_disp_x=0.0
        self.joy_disp_y=0.0
        self.key_down.clear()
        if hasattr(self, 'gamepad_key_down'):
            self.gamepad_key_down.clear()
        self.set_estop_visual()
        try:
            if self.hub.is_open():
                self.hub.write(build_motion_cmd(0,0,0))
        except Exception:
            pass

        self.status_var.set(
            'E-STOP pressed. Motion stopped.'
        )

    def on_key_press(self, event):
        """键盘按下处理。

        W/A/S/D：前后左右平移
        Q：左转，等同于按住界面左转按钮
        E：右转，等同于按住界面右转按钮
        Space：E-STOP
        """
        k = event.keysym.lower()

        # 空格键保持最高优先级，执行现有E-STOP安全链路
        if k == 'space':
            self.stop_motion()
            return

        # 防止Windows键盘自动重复触发同一个Q/E按下事件
        if k in self.key_down:
            return

        if k in ['w', 'a', 's', 'd']:
            self.key_down.add(k)
            return

        if k == 'q':
            self.key_down.add(k)
            # 与界面左转按钮完全使用同一个执行函数
            self.manual_rotate_press(+1)
            return

        if k == 'e':
            self.key_down.add(k)
            # 与界面右转按钮完全使用同一个执行函数
            self.manual_rotate_press(-1)
            return

    def on_key_release(self, event):
        """键盘释放处理。"""
        k = event.keysym.lower()

        if k in self.key_down:
            self.key_down.remove(k)

        # Q/E任一旋转键释放后，重新判断是否仍有另一个旋转键按住
        if k in ['q', 'e']:
            if 'q' in self.key_down and 'e' not in self.key_down:
                self.manual_rotate_press(+1)
            elif 'e' in self.key_down and 'q' not in self.key_down:
                self.manual_rotate_press(-1)
            else:
                self.manual_rotate_release()
            return

        # 保留原有W/A/S/D释放停车逻辑
        if (
            k in ['w', 'a', 's', 'd', 'space']
            and self.mode_var.get() == 'Mode 6'
            and not any(x in self.key_down for x in ['w', 'a', 's', 'd'])
            and not any(x in self.key_down for x in ['q', 'e'])
        ):
            self.send_motion(0, 0, 0)

    def current_speed_scale(self):
        """Speed scale for keyboard, D-pad, on-screen arrows and turn buttons.

        V4.0.5 fix:
            Previously this returned 1.0 when no gamepad was connected, so WASD always ran at 100%.
            Now it always uses key_speed_pct_value, which is controlled by the speed bar.
            When a gamepad is connected, RT updates the same key_speed_pct_value.
        """
        try:
            pct = float(getattr(self, 'key_speed_pct_value', 20.0))
        except Exception:
            pct = 20.0
        pct = max(0.0, min(100.0, pct))
        return pct / 100.0

    def manual_rotate_press(self, direction):
        """Hold-to-rotate button. direction +1 = left turn, -1 = right turn."""
        self.manual_rotate_dir = 1 if direction > 0 else -1
        wz_max = float(self.gamepad_wz_max_var.get()) if hasattr(self, 'gamepad_wz_max_var') else 1.0
        self.send_motion(0, 0, self.manual_rotate_dir * wz_max * self.current_speed_scale())

    def manual_rotate_release(self):
        self.manual_rotate_dir = 0
        if self.mode_var.get() == 'Mode 6' and not any(x in self.key_down for x in ['w','a','s','d']):
            self.send_motion(0,0,0)

    def manual_button_press(self, key):
        # On-screen arrow buttons share the same press/release semantics as keyboard WASD.
        if key in ['w','a','s','d']:
            self.key_down.add(key)
            sp=(float(self.jog_speed_var.get()) if hasattr(self,'jog_speed_var') else MAX_MANUAL_SPEED_MPS) * self.current_speed_scale()
            vx=vy=0.0
            if key == 'w': vx = sp
            elif key == 's': vx = -sp
            elif key == 'a': vy = sp
            elif key == 'd': vy = -sp
            self.send_motion(vx, vy, 0)

    def manual_button_release(self, key):
        if key in self.key_down:
            self.key_down.remove(key)
        if key in ['w','a','s','d'] and self.mode_var.get()=='Mode 6' and not any(x in self.key_down for x in ['w','a','s','d']):
            self.send_motion(0,0,0)

    def keyboard_step(self):
        if self.mode_var.get()!='Mode 6': return
        now=time.time()
        if now-self.last_key_send < 0.08: return
        if 'space' in self.key_down:
            self.send_motion(0,0,0)
            self.last_key_send=now
            return
        sp=(float(self.jog_speed_var.get()) if hasattr(self,'jog_speed_var') else MAX_MANUAL_SPEED_MPS) * self.current_speed_scale()
        vx=vy=0.0
        if 'w' in self.key_down: vx+=sp
        if 's' in self.key_down: vx-=sp
        if 'a' in self.key_down: vy+=sp
        if 'd' in self.key_down: vy-=sp
        if vx or vy:
            self.send_motion(vx,vy,0)
            self.last_key_send=now
            return
        if getattr(self, 'manual_rotate_dir', 0):
            wz_max = float(self.gamepad_wz_max_var.get()) if hasattr(self, 'gamepad_wz_max_var') else 1.0
            self.send_motion(0,0,self.manual_rotate_dir * wz_max)
            self.last_key_send=now

    def on_joy_down(self,event):
        self.joy_active=True
        self.update_joystick(event.x,event.y)

    def on_joy_drag(self,event):
        self.update_joystick(event.x,event.y)

    def on_joy_up(self,event):
        self.joy_active=False
        self.joy_vx=self.joy_vy=self.joy_wz=0.0
        self.send_motion(0,0,0)

    def update_joystick(self,x,y):
        if not self.ensure_mode6(): return
        w=max(1,self.joy_canvas.winfo_width())
        h=max(1,self.joy_canvas.winfo_height())
        cx=w/2
        cy=h/2
        r=min(w,h)/2-30
        dx=max(-r,min(r,x-cx))
        dy=max(-r,min(r,y-cy))
        mag=math.hypot(dx,dy)
        if mag>r:
            dx=dx/mag*r
            dy=dy/mag*r
        # Virtual joystick drag is an analog input, so it is not limited by RT speed scaling.
        max_sp=float(self.jog_speed_var.get())
        self.joy_disp_x=dx/r
        self.joy_disp_y=dy/r
        self.joy_vy=-self.joy_disp_x*max_sp
        self.joy_vx=-self.joy_disp_y*max_sp
        self.joy_wz=0.0
        self.joy_vx, self.joy_vy = self.limit_mode6_xy_diamond(self.joy_vx, self.joy_vy)
        self.send_motion(self.joy_vx,self.joy_vy,0)


    # ---------------- gamepad ----------------
    def init_gamepad(self):
        """Initialize pygame joystick support without requiring a pygame window."""
        self.gamepad = None
        if pygame is None:
            self.gamepad_status_var.set('Gamepad: pygame not installed. Run: pip install pygame')
            return
        try:
            pygame.init()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count <= 0:
                self.gamepad_status_var.set('Gamepad: not connected')
                return
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            self.gamepad_status_var.set(
                f'Gamepad: {self.gamepad.get_name()} | axes={self.gamepad.get_numaxes()} buttons={self.gamepad.get_numbuttons()} hats={self.gamepad.get_numhats()}'
            )
        except Exception as e:
            self.gamepad = None
            self.gamepad_status_var.set(f'Gamepad init failed: {e}')

    def apply_deadzone(self, v, dz=None):
        try:
            v = float(v)
        except Exception:
            return 0.0
        if dz is None:
            dz = float(self.gamepad_deadzone_var.get())
        dz = max(0.0, min(0.8, float(dz)))
        if abs(v) < dz:
            return 0.0
        # Rescale the remaining range so the output still reaches +/-1.0.
        return math.copysign((abs(v) - dz) / (1.0 - dz), v)

    def gp_axis(self, idx):
        try:
            if self.gamepad is None or idx < 0 or idx >= self.gamepad.get_numaxes():
                return 0.0
            return float(self.gamepad.get_axis(idx))
        except Exception:
            return 0.0

    def gp_button(self, idx):
        try:
            if self.gamepad is None or idx < 0 or idx >= self.gamepad.get_numbuttons():
                return False
            return bool(self.gamepad.get_button(idx))
        except Exception:
            return False

    def gp_hat(self, idx=0):
        try:
            if self.gamepad is None or idx < 0 or idx >= self.gamepad.get_numhats():
                return (0, 0)
            return self.gamepad.get_hat(idx)
        except Exception:
            return (0, 0)

    def rt_speed_scale(self):
        """Map RT axis A5 from -1..+1 to min_percent..100%."""
        rt_raw = self.gp_axis(5)
        rt01 = max(0.0, min(1.0, (rt_raw + 1.0) / 2.0))
        try:
            min_pct = float(self.gamepad_min_speed_pct_var.get())
        except Exception:
            min_pct = 20.0
        min_pct = max(0.0, min(100.0, min_pct))
        pct = min_pct + (100.0 - min_pct) * rt01
        self.key_speed_pct_value = pct
        if hasattr(self, 'gamepad_current_speed_pct_var'):
            self.gamepad_current_speed_pct_var.set(f'Key/D-pad speed: {pct:.0f}%')
        return pct / 100.0

    def on_speed_bar_drag(self, event):
        if not hasattr(self, 'speed_canvas'):
            return
        w = max(80, self.speed_canvas.winfo_width())
        x0, x1 = 10, w - 10
        pct = (max(x0, min(x1, event.x)) - x0) / max(1, (x1 - x0)) * 100.0
        pct = round(pct)
        self.gamepad_min_speed_pct_var.set(pct)
        self.key_speed_pct_value = pct
        if hasattr(self, 'gamepad_current_speed_pct_var'):
            self.gamepad_current_speed_pct_var.set(f'Key/D-pad speed: {pct:.0f}%')

    def draw_speed_bar(self):
        if not hasattr(self, 'speed_canvas'):
            return
        cv = self.speed_canvas
        cv.delete('all')
        w = max(120, cv.winfo_width())
        h = max(40, cv.winfo_height())
        x0, x1 = 12, w - 12
        y0, y1 = 18, 30
        try:
            min_pct = max(0.0, min(100.0, float(self.gamepad_min_speed_pct_var.get())))
        except Exception:
            min_pct = 20.0
        cur_pct = max(0.0, min(100.0, float(getattr(self, 'key_speed_pct_value', min_pct))))
        cx = x0 + (x1 - x0) * cur_pct / 100.0
        mx = x0 + (x1 - x0) * min_pct / 100.0
        cv.create_rectangle(x0, y0, x1, y1, fill='#e5e7eb', outline='#94a3b8')
        cv.create_rectangle(x0, y0, cx, y1, fill='#22c55e', outline='')
        cv.create_line(mx, y0-5, mx, y1+5, fill='#f97316', width=2)
        cv.create_text(x0, 8, text=f'Min {min_pct:.0f}%', anchor='w', fill='#334155', font=('Segoe UI',9,'bold'))
        cv.create_text(x1, 8, text=f'Speed {cur_pct:.0f}%', anchor='e', fill='#166534', font=('Segoe UI',9,'bold'))

    def update_dpad_wasd(self, hat):
        # D-pad directly maps to the existing keyboard W/A/S/D control path.
        # Return True exactly once when D-pad is released from non-zero to zero,
        # so gamepad_step can send a zero motion frame.
        old = set(getattr(self, 'gamepad_key_down', set()))
        for k in old:
            self.key_down.discard(k)
        new_keys = set()
        hx, hy = hat
        if hy > 0: new_keys.add('w')
        if hy < 0: new_keys.add('s')
        if hx < 0: new_keys.add('a')
        if hx > 0: new_keys.add('d')
        self.gamepad_key_down = new_keys
        self.key_down.update(new_keys)
        released_to_zero = bool(old) and not new_keys
        self.gamepad_dpad_last_nonzero = bool(new_keys)
        return released_to_zero

    def gamepad_step(self):
        """Poll gamepad and send Mode 6 motion commands.

        V3.9 Beitong Asura 2 Pro mapping after installing vendor driver:
        - A0/A1: left stick -> vx/vy and Tab2 virtual joystick.
        - A2/A3: right stick -> A2 horizontal rotation wz.
        - A5: RT linear trigger, -1 released to +1 fully pressed, speed 20%..100%.
        - Hat 0: D-pad -> W/A/S/D through existing keyboard_step().
        - BTN5 / RB: E-STOP/brake and UI E-STOP button visual linkage.
        """
        if pygame is None:
            return
        if not hasattr(self, 'gamepad_enabled') or not self.gamepad_enabled.get():
            if hasattr(self, 'gamepad_key_down'):
                for k in self.gamepad_key_down:
                    self.key_down.discard(k)
                self.gamepad_key_down.clear()
            return
        try:
            pygame.event.pump()
        except Exception:
            return
        if self.gamepad is None:
            # Low-rate lazy reconnect.
            now = time.time()
            if now - self.gamepad_last_send_t > 2.0:
                self.gamepad_last_send_t = now
                self.init_gamepad()
            return

        scale = self.rt_speed_scale()
        self.gamepad_speed_scale = scale
        hat = self.gp_hat(0)
        dpad_released_to_zero = self.update_dpad_wasd(hat)

        # BTN5 / RB brake / E-STOP for the current Beitong driver mapping.
        estop_down = self.gp_button(5)
        if estop_down and not self.gamepad_last_estop_down:
            self.stop_motion()
            self.gamepad_status_var.set('Gamepad: BTN5/RB E-STOP sent')
        self.gamepad_last_estop_down = estop_down
        if estop_down:
            return

        lx = self.apply_deadzone(self.gp_axis(0))
        ly = self.apply_deadzone(self.gp_axis(1))
        rx = self.apply_deadzone(self.gp_axis(2))
        ry = self.apply_deadzone(self.gp_axis(3))
        rt_raw = self.gp_axis(5)
        if hasattr(self, 'gamepad_live_var'):
            self.gamepad_live_var.set(
                f'RX={rx:+.2f} RT={rt_raw:+.2f} Speed={scale*100:.0f}% MaxTarget={MAX_TARGET:.0f}'
            )

        if self.mode_var.get() != 'Mode 6' or self.closed_loop_running:
            return

        # Physical sticks are analog controls. Their deflection is already speed magnitude,
        # so RT speed scaling is NOT applied here. RT only affects D-pad / keyboard / buttons.
        sp = float(self.jog_speed_var.get()) if hasattr(self, 'jog_speed_var') else MAX_MANUAL_SPEED_MPS
        wz_max = float(self.gamepad_wz_max_var.get()) if hasattr(self, 'gamepad_wz_max_var') else 2.5

        # Match the existing virtual joystick convention:
        # pushing stick up gives +vx, pushing stick left gives +vy.
        vx = -ly * sp
        vy = -lx * sp
        # V4.0.2: invert right-stick rotation so stick-right means right/CW rotation by user preference.
        wz = -rx * wz_max
        vx, vy = self.limit_mode6_xy_diamond(vx, vy)

        active = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
        self.joy_active = active
        self.joy_vx = vx
        self.joy_vy = vy
        self.joy_wz = wz
        self.joy_disp_x = lx
        self.joy_disp_y = ly

        now = time.time()
        if active:
            if now - self.gamepad_last_send_t >= 0.05:
                self.send_motion(vx, vy, wz)
                self.gamepad_last_send_t = now
                self.gamepad_last_nonzero = True
        else:
            # Send one zero frame when the stick OR D-pad returns to center.
            if self.gamepad_last_nonzero or dpad_released_to_zero:
                self.send_motion(0, 0, 0)
                self.joy_disp_x = 0.0
                self.joy_disp_y = 0.0
                self.gamepad_last_send_t = now
                self.gamepad_last_nonzero = False

    # ---------------- vision ----------------
    def toggle_camera(self):
        if self.camera_running:
            self.camera_running=False
            if self.cap is not None:
                self.cap.release()
            self.cap=None
            self.cam_btn.config(text='Start Camera')
            return
        if cv2 is None:
            messagebox.showerror('Camera error','opencv not installed. Run: pip install opencv-contrib-python')
            return
        idx=int(self.cam_var.get())
        self.cap=cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap=cv2.VideoCapture(idx)
        if not self.cap.isOpened():
            messagebox.showerror('Camera error', f'Cannot open camera {idx}')
            self.cap=None
            return
        # Request camera parameters. The driver may choose the closest supported mode.
        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.cam_width_var.get()))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cam_height_var.get()))
            self.cap.set(cv2.CAP_PROP_FPS, int(self.cam_fps_var.get()))
        except Exception:
            pass
        self.cam_actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        self.cam_actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        self.cam_actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS) or 0.0)
        self.cam_frame_count = 0
        self.cam_fps_t0 = time.time()
        self.camera_running=True
        self.cam_btn.config(text='Stop Camera')

    def _get_var_float(self, var, fallback):
        try:
            return float(var.get())
        except Exception:
            return float(fallback)

    def on_goto_strategy_changed(self, event=None):
        self.update_strategy_param_ui()
        try:
            self.status_var.set(f'GoTo strategy selected: {self.goto_strategy_var.get()}')
        except Exception:
            pass

    def update_strategy_param_ui(self):
        strategy = str(self.goto_strategy_var.get()).lower()
        none_var = tk.StringVar(value='')
        mapping = {
            'rcpd': (
                ('LaunchT', self.rcpd_launch_time_var), ('Launch', self.rcpd_launch_speed_var),
                ('Ramp/s', self.rcpd_accel_rate_var), ('SlowR', self.rcpd_slow_radius_var), ('Min', self.rcpd_min_cmd_var),
                'Launch cap -> fast ramp -> Kp brake'
            ),
            'p': (('Kp', self.kp_var), ('Max', self.max_speed_var), ('Stop', self.stop_radius_var), ('', none_var), ('', none_var), 'P: cmd=Kp*error'),
            'max_then_slow': (('SlowR', self.slow_radius_var), ('Min', self.min_cmd_var), ('Curve', self.slow_curve_power_var), ('', none_var), ('', none_var), 'Far=Max; near slows'),
            'pd_damping': (('PD_Kp', self.pd_kp_var), ('PD_Kd', self.pd_kd_var), ('Min', self.pd_min_cmd_var), ('', none_var), ('', none_var), 'Kp*err - Kd*vel'),
            'brake_distance': (('Decel', self.brake_decel_var), ('Buffer', self.brake_buffer_var), ('Min', self.brake_min_cmd_var), ('', none_var), ('', none_var), 'brake=v^2/(2a)+buf'),
            'bang_bang': (('SlowR', self.bang_slow_radius_var), ('NearKp', self.bang_near_kp_var), ('RevGain', self.bang_reverse_gain_var), ('', none_var), ('', none_var), 'far full; near P/brake'),
        }
        p1,p2,p3,p4,p5,logic = mapping.get(strategy, mapping['rcpd'])
        self.goto_param1_label_var.set(p1[0]); self.goto_param2_label_var.set(p2[0]); self.goto_param3_label_var.set(p3[0])
        self.goto_param4_label_var.set(p4[0]); self.goto_param5_label_var.set(p5[0])
        self.goto_strategy_logic_var.set(logic)
        try:
            self.goto_param1_entry.configure(textvariable=p1[1])
            self.goto_param2_entry.configure(textvariable=p2[1])
            self.goto_param3_entry.configure(textvariable=p3[1])
            self.goto_param4_entry.configure(textvariable=p4[1])
            self.goto_param5_entry.configure(textvariable=p5[1])
        except Exception:
            pass

    def set_target(self,x,y):
        self.target_xy=(float(x),float(y))
        try:
            self.test_x.set(float(x))
            self.test_y.set(float(y))
        except Exception:
            pass
        self.status_var.set(f'Target set x={float(x):.3f}, y={float(y):.3f}')

    def get_strategy_param_summary(self):
        """Compact strategy parameter summary for the GoTo record table and CSV."""
        strategy = str(self.goto_strategy_var.get()).lower() if hasattr(self, 'goto_strategy_var') else '--'
        try:
            kp = float(self.kp_var.get())
            max_cmd = float(self.max_speed_var.get())
            stop_r = float(self.stop_radius_var.get())
        except Exception:
            kp, max_cmd, stop_r = None, None, None
        parts = [f'Kp={self.fmt(kp,2)}', f'Max={self.fmt(max_cmd,2)}', f'Stop={self.fmt(stop_r,3)}']
        if strategy == 'rcpd':
            parts += [
                f'LaunchT={self.fmt(self._get_var_float(self.rcpd_launch_time_var, 0.10),2)}',
                f'Launch={self.fmt(self._get_var_float(self.rcpd_launch_speed_var, 0.55),2)}',
                f'Ramp/s={self.fmt(self._get_var_float(self.rcpd_accel_rate_var, 8.0),2)}',
                f'SlowR={self.fmt(self._get_var_float(self.rcpd_slow_radius_var, 0.12),3)}',
                f'Min={self.fmt(self._get_var_float(self.rcpd_min_cmd_var, 0.14),2)}',
            ]
        elif strategy == 'max_then_slow':
            parts += [
                f'SlowR={self.fmt(self._get_var_float(self.slow_radius_var, 0.25),3)}',
                f'Min={self.fmt(self._get_var_float(self.min_cmd_var, 0.15),2)}',
                f'Curve={self.fmt(self._get_var_float(self.slow_curve_power_var, 1.0),2)}',
            ]
        elif strategy == 'pd_damping':
            parts += [
                f'PD_Kp={self.fmt(self._get_var_float(self.pd_kp_var, 1.8),2)}',
                f'PD_Kd={self.fmt(self._get_var_float(self.pd_kd_var, 0.45),2)}',
                f'Min={self.fmt(self._get_var_float(self.pd_min_cmd_var, 0.0),2)}',
            ]
        elif strategy == 'brake_distance':
            parts += [
                f'Decel={self.fmt(self._get_var_float(self.brake_decel_var, 2.5),2)}',
                f'Buffer={self.fmt(self._get_var_float(self.brake_buffer_var, 0.06),3)}',
                f'Min={self.fmt(self._get_var_float(self.brake_min_cmd_var, 0.12),2)}',
            ]
        elif strategy == 'bang_bang':
            parts += [
                f'SlowR={self.fmt(self._get_var_float(self.bang_slow_radius_var, 0.25),3)}',
                f'NearKp={self.fmt(self._get_var_float(self.bang_near_kp_var, 2.0),2)}',
                f'RevGain={self.fmt(self._get_var_float(self.bang_reverse_gain_var, 0.35),2)}',
            ]
        return '; '.join(parts)

    def get_strategy_config(self, center=False):
        try:
            kp = float(self.kp_var.get())
        except Exception:
            kp = 0.7
        try:
            max_cmd = float(self.max_speed_var.get())
        except Exception:
            max_cmd = MAX_MANUAL_SPEED_MPS
        try:
            stop_radius = float(self.stop_radius_var.get())
        except Exception:
            stop_radius = 0.05
        cfg = GotoConfig(kp=kp, max_cmd=max_cmd, stop_radius=stop_radius)

        strategy = str(self.goto_strategy_var.get()).lower() if hasattr(self, 'goto_strategy_var') else 'p'
        cfg.strategy = strategy
        cfg.rcpd_launch_time = self._get_var_float(self.rcpd_launch_time_var, 0.10)
        cfg.rcpd_launch_speed = self._get_var_float(self.rcpd_launch_speed_var, 0.55)
        cfg.rcpd_accel_rate = self._get_var_float(self.rcpd_accel_rate_var, 8.0)
        cfg.rcpd_slow_radius = self._get_var_float(self.rcpd_slow_radius_var, 0.12)
        cfg.rcpd_min_cmd = self._get_var_float(self.rcpd_min_cmd_var, 0.14)
        cfg.slow_radius = self._get_var_float(self.slow_radius_var, 0.25)
        cfg.min_cmd = self._get_var_float(self.min_cmd_var, 0.15)
        cfg.slow_curve_power = self._get_var_float(self.slow_curve_power_var, 1.0)
        cfg.pd_kp = self._get_var_float(self.pd_kp_var, 1.8)
        cfg.pd_kd = self._get_var_float(self.pd_kd_var, 0.45)
        cfg.pd_min_cmd = self._get_var_float(self.pd_min_cmd_var, 0.0)
        cfg.brake_decel = self._get_var_float(self.brake_decel_var, 2.5)
        cfg.brake_buffer = self._get_var_float(self.brake_buffer_var, 0.06)
        cfg.brake_min_cmd = self._get_var_float(self.brake_min_cmd_var, 0.12)
        cfg.bang_slow_radius = self._get_var_float(self.bang_slow_radius_var, 0.25)
        cfg.bang_near_kp = self._get_var_float(self.bang_near_kp_var, 2.0)
        cfg.bang_reverse_gain = self._get_var_float(self.bang_reverse_gain_var, 0.35)
        if center:
            cfg.center_strategy = 'p'
            cfg.center_kp = min(kp, 0.55)
            cfg.center_max_cmd = min(max_cmd, 0.45)
            cfg.center_stop_radius = min(stop_radius, 0.03)
            cfg.yaw_kp = 1.2
            cfg.yaw_max_wz = 0.6
        return cfg
    def current_pose_dict(self):
        if self.car_pose is None:
            return {'valid': False}
        x, y, yaw, src = self.car_pose
        return {'valid': True, 'x': x, 'y': y, 'yaw': yaw, 'src': src, 't': time.time()}

    def yaw_to_map_heading_deg(self, yaw):
        """Convert world/math yaw to map heading degrees.

        Display convention for V4.0.1:
            map up    = 0 deg
            map right = +90 deg
            map down  = +/-180 deg
            map left  = -90 deg
        """
        if yaw is None:
            return None
        try:
            deg = 90.0 - math.degrees(float(yaw))
        except Exception:
            return None
        while deg > 180.0:
            deg -= 360.0
        while deg < -180.0:
            deg += 360.0
        return deg

    def clear_screen_trace(self):
        """Clear map/camera test traces and task result display without changing connection state."""
        try:
            self.vision_track.clear()
        except Exception:
            pass
        try:
            self.vision_goto_track.clear()
        except Exception:
            pass
        try:
            self.parser.odom_track.clear()
        except Exception:
            pass
        self.target_xy = None
        self.goto_start_pose = None
        self.goto_stop_pose = None
        self.goto_last_result = None
        self.goto_elapsed = None
        self.goto_result = {
            'mode': '--', 'state': 'Idle', 'target': None,
            'start': None, 'stop': None, 'error_m': None,
            'elapsed_s': None, 'max_g': None
        }
        try:
            self.goto_timer_var.set('Go Timer: --')
        except Exception:
            pass
        try:
            self.status_var.set('Screen / tracks cleared.')
        except Exception:
            pass

    def start_goto_target(self, x, y, mode='goto'):
        if not self.ensure_mode6(): return
        if self.car_pose is None:
            messagebox.showwarning('No vision pose','Car marker not detected.')
            return
        self.set_target(x, y)
        self.closed_loop_running = True
        self.goto_running = True
        self.goto_start_t = time.time()
        self.goto_elapsed = None
        self.goto_start_pose = self.car_pose
        self.goto_stop_pose = None
        self.goto_last_result = None
        self.vision_goto_track.clear()
        self.max_g_this_run = 0.0
        if hasattr(self, 'g_trace'):
            self.g_trace.clear()
        self.goto_result = {
            'mode': mode, 'state': 'Running', 'target': (float(x), float(y)),
            'start': self.goto_start_pose, 'stop': None, 'error_m': None,
            'elapsed_s': None, 'max_g': None,
            'strategy': self.goto_strategy_var.get() if hasattr(self, 'goto_strategy_var') else '--',
            'strategy_params': self.get_strategy_param_summary(),
        }
        if hasattr(self, 'goto_timer_var'):
            self.goto_timer_var.set('Go Timer: 0.000 s')
        self.goto_controller.start(float(x), float(y), mode=mode, target_yaw=None, config=self.get_strategy_config(center=False))

    def start_centering(self):
        cx = FIELD_WIDTH_M / 2.0
        cy = FIELD_HEIGHT_M / 2.0
        if not self.ensure_mode6(): return
        if self.car_pose is None:
            messagebox.showwarning('No vision pose','Car marker not detected.')
            return
        self.set_target(cx, cy)
        self.closed_loop_running = True
        self.goto_running = True
        self.goto_start_t = time.time()
        self.goto_elapsed = None
        self.goto_start_pose = self.car_pose
        self.goto_stop_pose = None
        self.goto_last_result = None
        self.vision_goto_track.clear()
        self.max_g_this_run = 0.0
        if hasattr(self, 'g_trace'):
            self.g_trace.clear()
        self.goto_result = {
            'mode': 'center', 'state': 'Running', 'target': (cx, cy),
            'start': self.goto_start_pose, 'stop': None, 'error_m': None,
            'elapsed_s': None, 'max_g': None,
            'strategy': 'center_p',
            'strategy_params': 'center conservative P',
        }
        if hasattr(self, 'goto_timer_var'):
            self.goto_timer_var.set('return: 0.000 s')
        self.goto_controller.start(cx, cy, mode='center', target_yaw=math.pi/2, config=self.get_strategy_config(center=True))

    def finish_goto(self, result, reason='Done'):
        self.send_motion(0,0,0)
        self.closed_loop_running = False
        self.goto_running = False
        self.goto_stop_pose = self.car_pose
        elapsed = None
        if self.goto_start_t is not None:
            elapsed = time.time() - self.goto_start_t
            self.goto_elapsed = elapsed
        error_m = getattr(result, 'error_m', None)
        if error_m is None and self.target_xy and self.car_pose:
            error_m = math.hypot(self.target_xy[0]-self.car_pose[0], self.target_xy[1]-self.car_pose[1])
        self.goto_result.update({
            'state': reason, 'stop': self.goto_stop_pose, 'error_m': error_m,
            'elapsed_s': elapsed, 'max_g': getattr(self, 'max_g_this_run', 0.0)
        })
        if hasattr(self, 'goto_timer_var'):
            if elapsed is not None:
                self.goto_timer_var.set(f'{reason}: {elapsed:.3f} s')
            else:
                self.goto_timer_var.set(reason)
        if error_m is not None:
            self.status_var.set(f'{reason}: error={error_m:.3f} m')
        else:
            self.status_var.set(reason)
        try:
            self.append_goto_record(reason)
        except Exception as e:
            self.log(f'Append GoTo record failed: {e}')

    def get_map_rect(self,canvas):
        w=max(1,canvas.winfo_width())
        h=max(1,canvas.winfo_height())
        margin=10
        aw=max(10,w-margin)
        ah=max(10,h-margin)
        aspect=FIELD_WIDTH_M/FIELD_HEIGHT_M
        mw=aw
        mh=mw/aspect
        if mh>ah:
            mh=ah
            mw=mh*aspect
        return (w-mw)/2, (h-mh)/2, mw, mh

    def on_map_click(self,event):
        ox,oy,mw,mh=self.get_map_rect(event.widget)
        x=(event.x-ox)/mw*FIELD_WIDTH_M
        y=(1-(event.y-oy)/mh)*FIELD_HEIGHT_M
        if 0<=x<=FIELD_WIDTH_M and 0<=y<=FIELD_HEIGHT_M:
            self.start_goto_target(x,y, mode='goto')

    def on_camera_click(self,event):
        if self.last_H is None or self.last_frame is None:
            self.status_var.set('Camera click requires 4 field markers. Use map click.')
            return
        if self.video_display_info is not None:
            ox, oy, scale, fw, fh = self.video_display_info
        else:
            label_w=max(1,self.video_label.winfo_width())
            label_h=max(1,self.video_label.winfo_height())
            fh,fw=self.last_frame.shape[:2]
            scale=min(label_w/fw,label_h/fh)
            ox=(label_w-fw*scale)/2
            oy=(label_h-fh*scale)/2
        ix=(event.x-ox)/scale
        iy=(event.y-oy)/scale
        if 0<=ix<fw and 0<=iy<fh:
            x,y=img_to_world(self.last_H,(ix,iy))
            if 0<=x<=FIELD_WIDTH_M and 0<=y<=FIELD_HEIGHT_M:
                self.start_goto_target(x,y, mode='goto')
            else:
                self.status_var.set('Clicked point outside field.')

    def start_closed_loop(self):
        # Manual coordinate input + Go button.
        self.start_goto_target(float(self.test_x.get()), float(self.test_y.get()), mode='goto')

    def stop_closed_loop(self):
        self.closed_loop_running=False
        self.goto_running = False
        try:
            self.goto_controller.stop()
        except Exception:
            pass
        self.send_motion(0,0,0)

    def closed_loop_step(self):
        if not self.closed_loop_running:
            return
        now=time.time()
        hz=max(1,float(self.send_hz_var.get()))
        if now-self.last_send_t < 1/hz:
            return
        self.last_send_t=now
        pose = self.current_pose_dict()
        dt = 1.0 / hz
        result = self.goto_controller.update(pose, dt)
        self.goto_last_result = result
        if result.state == 'no_pose':
            self.send_motion(0,0,0)
            self.status_var.set('Vision lost: holding stop')
            return
        if result.done:
            self.finish_goto(result, reason='Done')
            return
        self.send_motion(result.vx, result.vy, result.wz)
        if self.goto_running and self.goto_start_t is not None and hasattr(self, 'goto_timer_var'):
            self.goto_timer_var.set(f'Go Timer: {time.time()-self.goto_start_t:.3f} s')
        err = '--' if result.error_m is None else f'{result.error_m:.3f}'
        self.status_var.set(f'{result.state} [{self.goto_strategy_var.get()} {result.debug.get("zone", "")}]: vx={result.vx:.2f}, vy={result.vy:.2f}, wz={result.wz:.2f}, err={err} m')

    def process_frame(self,frame):
        self.last_frame=frame.copy()
        need_aruco = True
        if DETECT_ARUCO_UNTIL_FIELD_LOCK_ONLY and getattr(self, 'field_markers_locked', False):
            need_aruco = False
        aruco_map,april_map,aruco_corners,aruco_ids,april_corners,april_ids=self.detector.detect(
            frame,
            detect_aruco=need_aruco,
            detect_apriltag=DETECT_APRILTAG_EVERY_FRAME,
        )
        if DRAW_DETECTED_MARKERS and cv2 is not None and aruco_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame,aruco_corners,aruco_ids)
        if DRAW_DETECTED_MARKERS and cv2 is not None and april_ids is not None and len(april_corners)>0:
            cv2.aruco.drawDetectedMarkers(frame,april_corners,april_ids,borderColor=(0,128,255))
        # V4.0.3: lock field homography after first successful full corner detection.
        # This avoids losing the coordinate system when a corner marker is occluded or flickers.
        H_new = None
        if not getattr(self, 'field_markers_locked', False):
            H_new=compute_homography(aruco_map)
            if H_new is not None:
                self.field_H = H_new
                self.field_markers_locked = True
                self.field_lock_time = time.time()
                try:
                    self.field_marker_img_pts = [aruco_map[mid]['center'] for mid in FIELD_MARKER_WORLD.keys()]
                except Exception:
                    self.field_marker_img_pts = None
                self.status_var.set('Field homography locked from ArUco 0/1/2/3')
        H = self.field_H if getattr(self, 'field_H', None) is not None else H_new
        self.last_H=H
        if H is not None and cv2 is not None:
            try:
                pts_src = self.field_marker_img_pts
                if pts_src is None:
                    pts_src = [aruco_map[mid]['center'] for mid in FIELD_MARKER_WORLD.keys()]
                pts = np.array(pts_src, dtype=np.int32).reshape((-1,1,2))
                overlay = frame.copy()
                cv2.polylines(overlay, [pts], True, (255, 220, 120), 3)
                cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
            except Exception:
                pass
        car=None
        if H is not None:
            if CAR_APRILTAG_ID in april_map:
                m=april_map[CAR_APRILTAG_ID]
                src='AprilTag ID10'
            elif CAR_ARUCO_ID in aruco_map:
                m=aruco_map[CAR_ARUCO_ID]
                src='ArUco ID10'
            else:
                m=None
                src=''
            if m is not None:
                x,y=img_to_world(H,m['center'])
                yaw=estimate_yaw_world(H,m['corners'])
                car=(x,y,yaw,src)
                self.vision_track.append((x,y))
                if getattr(self, 'closed_loop_running', False):
                    self.vision_goto_track.append((x,y))
        self.car_pose=car
        if Image is not None and ImageTk is not None:
            # Display frame on a Canvas. This uses a real resize, not thumbnail(), so
            # the camera image can fill the large Camera View area while preserving aspect.
            fh, fw = frame.shape[:2]
            label_w=max(1,self.video_label.winfo_width())
            label_h=max(1,self.video_label.winfo_height())
            scale=min(label_w/fw, label_h/fh)
            disp_w=max(1, int(fw*scale))
            disp_h=max(1, int(fh*scale))
            ox=(label_w-disp_w)/2
            oy=(label_h-disp_h)/2
            self.video_display_info=(ox, oy, scale, fw, fh)
            im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            resample_map = {
                'NEAREST': Image.Resampling.NEAREST,
                'BILINEAR': Image.Resampling.BILINEAR,
                'BICUBIC': Image.Resampling.BICUBIC,
                'LANCZOS': Image.Resampling.LANCZOS,
            }
            resample_mode = resample_map.get(str(VIDEO_RESAMPLE_MODE).upper(), Image.Resampling.BILINEAR)
            im=im.resize((disp_w, disp_h), resample_mode)
            tkimg=ImageTk.PhotoImage(im)
            self.video_label.delete('all')
            self.video_label.create_image(ox, oy, image=tkimg, anchor='nw')
            self.video_label.image=tkimg
            # Measured UI/display FPS, shown with requested/actual capture mode.
            self.cam_frame_count += 1
            now=time.time()
            dt=now-self.cam_fps_t0
            if dt >= 1.0:
                self.cam_actual_fps = self.cam_frame_count/dt
                self.cam_frame_count = 0
                self.cam_fps_t0 = now
            aw = self.cam_actual_w or fw
            ah = self.cam_actual_h or fh
            overlay=f'{aw}x{ah}@{self.cam_actual_fps:.1f}fps'
            self.video_label.create_text(8, 8, text=overlay, anchor='nw', fill='#22c55e', font=('Consolas', 10, 'bold'))

    # ---------------- PID ----------------
    def on_capture_next(self):
        self.capture_next_cmd=True
        self.last_cmd_var.set('Last CMD RX: waiting...')
        self.pid_log('Capture Next CMD armed. Now click Read/Write.\n','warn')

    def pid_send(self,cmd,logical_name=''):
        if not self.ensure_mode0(): return
        try:
            self.hub.write(cmd)
            if self.show_tx.get():
                self.pid_log(f'TX {logical_name}\nHEX: {hexline(cmd)}\n','tx')
        except Exception as e:
            self.pid_log(f'TX failed: {e}\n','warn')
            self.status_var.set(str(e))

    def update_pid_target_label(self):
        pct = float(self.pid_target_pct_var.get()) if hasattr(self, 'pid_target_pct_var') else 0.0
        raw = target_pct_to_raw(pct)
        if hasattr(self, 'pid_target_pct_label'):
            self.pid_target_pct_label.set(f'Target: {pct:+.0f}%')

    def on_pid_target_slider_press(self, event=None):
        self.pid_target_dragging = True
        if self.pid_target_after_id is not None:
            try:
                self.after_cancel(self.pid_target_after_id)
            except Exception:
                pass
            self.pid_target_after_id = None

    def on_pid_target_slider_changed(self, value=None):
        self.update_pid_target_label()
        # Debounce output: moving the slider only updates UI. A command is sent
        # after the slider has stopped changing for a short time, or at mouse release.
        if self.pid_target_after_id is not None:
            try:
                self.after_cancel(self.pid_target_after_id)
            except Exception:
                pass
        self.pid_target_after_id = self.after(350, self.pid_target_commit_if_idle)

    def on_pid_target_slider_release(self, event=None):
        self.pid_target_dragging = False
        if self.pid_target_after_id is not None:
            try:
                self.after_cancel(self.pid_target_after_id)
            except Exception:
                pass
            self.pid_target_after_id = None
        self.pid_target_after_id = self.after(120, self.pid_target_commit_now)

    def pid_target_commit_if_idle(self):
        self.pid_target_after_id = None
        if getattr(self, 'pid_target_dragging', False):
            return
        self.pid_target_commit_now()

    def pid_target_commit_now(self):
        if not self.ensure_mode0():
            return
        pct = float(self.pid_target_pct_var.get()) if hasattr(self, 'pid_target_pct_var') else 0.0
        raw = target_pct_to_raw(pct)
        self.pid_write(4, raw, f'Target {pct:+.0f}%')

    def pid_stop_all(self):
        if not self.ensure_mode0():
            return
        # Seller firmware currently appears to only act on CH1 for Target,
        # but send zero to all four channels so this remains safe if firmware is fixed later.
        for ch in range(4):
            frame = build_pid_cmd(4, 1, ch, 0.0)
            try:
                self.hub.write(frame)
                if self.show_tx.get():
                    self.pid_log(f'TX STOP ALL Target CH{ch+1} = 0% / raw 0\nHEX: {hexline(frame)}\n', 'tx')
                time.sleep(0.01)
            except Exception as e:
                self.pid_log(f'STOP ALL failed: {e}\n', 'warn')
                self.status_var.set(str(e))
                return
        self.pid_target_pct_var.set(0.0)
        self.update_pid_target_label()
        self.status_var.set('STOP ALL sent: Target 0% to CH1-CH4')

    def pid_read(self,c,logical_name=''):
        ch=self.pid_ch.get()
        self.pending_read[c]=ch
        self.pid_send(build_pid_cmd(c,0,ch,0),f'Read {logical_name} CH{ch+1}')

    def pid_write(self,c,v,logical_name=''):
        ch=self.pid_ch.get()
        self.pending_read[c]=ch
        self.pid_send(build_pid_cmd(c,1,ch,float(v)),f'Write {logical_name} CH{ch+1} = {float(v):.6g}')

    def apply_pid_all(self):
        if not self.ensure_mode0(): return
        vals=[(1,self.global_p.get(),'P'),(2,self.global_i.get(),'I'),(3,self.global_d.get(),'D')]
        for ch in range(4):
            for cmd,val,name in vals:
                frame=build_pid_cmd(cmd,1,ch,float(val))
                try:
                    self.hub.write(frame)
                    if self.show_tx.get():
                        self.pid_log(f'TX Global Write {name} CH{ch+1} = {float(val):.6g}\nHEX: {hexline(frame)}\n','tx')
                    time.sleep(0.01)
                except Exception as e:
                    self.pid_log(f'Global PID failed: {e}\n','warn')
                    self.status_var.set(str(e))
                    return
        self.status_var.set('Global PID P/I/D sent to CH1-CH4')

    def read_pid_all(self):
        if not self.ensure_mode0(): return
        for ch in range(4):
            for cmd,name in [(1,'P'),(2,'I'),(3,'D')]:
                self.pending_read[cmd]=ch
                frame=build_pid_cmd(cmd,0,ch,0)
                try:
                    self.hub.write(frame)
                    if self.show_tx.get():
                        self.pid_log(f'TX Read {name} CH{ch+1}\nHEX: {hexline(frame)}\n','tx')
                    time.sleep(0.01)
                except Exception as e:
                    self.pid_log(f'Read PID all failed: {e}\n','warn')
                    return

    def handle_pid_item(self,item):
        if item['kind']=='data':
            self.handle_pid_data(item)
        elif item['kind']=='cmd':
            self.handle_pid_cmd(item)
        else:
            self.pid_log(f'RX UNKNOWN: {item["hex"]}\n','warn')

    def handle_pid_data(self,item):
        ch=item['channel']
        if 0<=ch<=3:
            a=item['actual']
            t=item['target']
            e=item['error']
            self.pid_hist[ch]['a'].append(a)
            self.pid_hist[ch]['t'].append(t)
            self.pid_hist[ch]['e'].append(e)
            self.pid_live[ch].update({'actual':a,'target':t,'error':e,'last_data_t':time.time()})
            self.data_count[ch]+=1
            if ch==self.pid_ch.get():
                ap = raw_to_target_pct(a)
                ep = raw_to_target_pct(e)
                self.pid_actual.set(f'Actual: {ap:+.1f}%')
                self.pid_error.set(f'Error: {ep:+.1f}%')
        if self.show_data_rx.get():
            self.pid_log(f'RX DATA CH{ch+1}: actual={item["actual"]:.6g}, target={item["target"]:.6g}, err={item["error"]:.6g}\nHEX: {item["hex"]}\n','data')

    def handle_pid_cmd(self,item):
        raw_ch=item['channel']
        value=item['value']
        cmd_id=item['cmd_id']
        # Seller firmware read reply may use byte3=0 regardless of requested channel.
        mapped_ch=self.pending_read.get(cmd_id, raw_ch)
        if not (0 <= mapped_ch <= 3):
            mapped_ch=raw_ch if 0 <= raw_ch <= 3 else self.pid_ch.get()

        desc=(f'RX CMD {item["rw_name"]} {item["cmd_name"]} '
              f'rawCH{raw_ch+1} mappedCH{mapped_ch+1} param3={item["param3"]}')
        if value is not None:
            desc += f' value={value:.6g}'
        desc += f'\nHEX: {item["hex"]}\n'

        if self.capture_next_cmd:
            self.capture_next_cmd=False
            self.pid_log('*** CAPTURED NEXT CMD RX ***\n'+desc,'capture')
        elif self.show_cmd_rx.get():
            self.pid_log(desc,'cmd')

        self.last_cmd_rx=item
        self.last_cmd_var.set(f'Last CMD RX: {item["cmd_name"]} rawCH{raw_ch+1} mappedCH{mapped_ch+1}, value={value:.6g}' if value is not None else f'Last CMD RX: {item}')

        if cmd_id in [1,2,3] and value is not None and mapped_ch==self.pid_ch.get():
            name=CMD_NAMES.get(cmd_id)
            if name in self.pid_vars:
                self.pid_vars[name].set(value)
        if cmd_id == 4 and value is not None and mapped_ch==self.pid_ch.get():
            if hasattr(self, 'pid_target_pct_var'):
                pct = raw_to_target_pct(value)
                if pct is not None:
                    self.pid_target_pct_var.set(max(-100.0, min(100.0, round(pct / 10.0) * 10.0)))
                    self.update_pid_target_label()

    def update_rate(self):
        now=time.time()
        dt=now-self.last_rate_t
        if dt>=1.0:
            self.data_rate=[c/dt for c in self.data_count]
            for i,r in enumerate(self.data_rate):
                self.pid_live[i]['rate']=r
            self.data_count=[0,0,0,0]
            self.last_rate_t=now
            if hasattr(self,'rate_var'):
                self.rate_var.set(' | '.join([f'CH{i+1} {self.data_rate[i]:.1f}Hz' for i in range(4)]))

    def refresh_pid_labels(self):
        ch=self.pid_ch.get()
        live=self.pid_live[ch]
        a=live['actual']
        t=live['target']
        e=live['error']
        ap = raw_to_target_pct(a) if a is not None else None
        ep = raw_to_target_pct(e) if e is not None else None
        self.pid_actual.set('Actual: --' if ap is None else f'Actual: {ap:+.1f}%')
        self.pid_error.set('Error: --' if ep is None else f'Error: {ep:+.1f}%')

    def pid_log(self,text,tag=None):
        if hasattr(self,'pid_text'):
            self.pid_text.insert('end',text,tag)
            self.pid_text.see('end')

    # ---------------- loops ----------------
    def ui_loop(self):
        if self.camera_running and self.cap is not None:
            ok,frame=self.cap.read()
            if ok:
                self.process_frame(frame)
        self.gamepad_step()
        self.keyboard_step()
        self.closed_loop_step()
        if LIDAR_CONFIG.enabled:
            self.lidar_guard.evaluate(self.lidar_service.snapshot())
            if hasattr(self, 'ld14_tab') and self.nb.select() == str(self.tab3):
                self.ld14_tab.refresh()
        if self.goto_running and self.goto_start_t is not None and hasattr(self, 'goto_timer_var'):
            self.goto_timer_var.set(f'Go Timer: {time.time()-self.goto_start_t:.3f} s')
        self.update_rate()
        self.draw_all()
        self.after(UI_LOOP_DELAY_MS,self.ui_loop)

    # ---------------- draw ----------------
    def draw_all(self):
        self.draw_battery()
        for c in [getattr(self,'map_canvas',None),getattr(self,'map_canvas2',None)]: self.draw_map(c)
        for c in [getattr(self,'g_canvas',None),getattr(self,'g_canvas2',None)]: self.draw_gdot(c)
        for c in [getattr(self,'wheel_canvas',None),getattr(self,'wheel_canvas2',None)]: self.draw_wheels(c)
        if hasattr(self, 'att_canvas'): self.draw_attitude(self.att_canvas)
        for c in [getattr(self,'status_canvas',None),getattr(self,'status_canvas2',None)]: self.draw_status_table(c)
        if hasattr(self,'joy_canvas'): self.draw_joystick()
        self.draw_speed_bar()
        self.update_wasd_visual()
        draw_pid = bool(PID_DRAW_ENABLED)
        if draw_pid and PID_DRAW_ONLY_WHEN_TAB2_ACTIVE:
            try:
                draw_pid = (self.nb.select() == str(self.tab2))
            except Exception:
                draw_pid = False
        if draw_pid and hasattr(self,'pid_canvas'): self.draw_pid_curve()
        if draw_pid and hasattr(self,'pid_status_canvas'): self.draw_pid_status()

    def draw_battery(self):
        """Draw battery voltage and percent with software filtering.

        V4.0.6:
            - raw voltage still comes from STM32 battery feedback or ESP32 ADC divider
            - displayed voltage uses a first-order low-pass filter
            - displayed percent uses additional hysteresis / slow tracking

        This is intended to solve the common symptom where a stable battery divider
        voltage measured by a multimeter still appears as +/-0.1V jitter on ESP32 ADC.
        """
        cv = self.battery_canvas
        cv.delete('all')

        BAT_EMPTY_V = 9.6
        BAT_FULL_V = 12.45
        ADC_TO_BATTERY_ENABLED = True
        BAT_ADC_R_TOP = 38700.0
        BAT_ADC_R_BOTTOM = 9660.0

        # Filter tuning. Smaller alpha = smoother but slower response.
        VOLTAGE_FILTER_ALPHA = 0.08
        PCT_FILTER_ALPHA = 0.05
        PCT_UPDATE_DEADBAND = 0.8      # percent points. Ignore tiny percent changes.
        DISPLAY_DECIMALS = 1           # show 2 decimals so changes are less visually abrupt than rounded 0.1V steps.

        bat_raw = self.parser.data.get('battery')
        source = 'STM32'
        if bat_raw is None and getattr(self.hub, 'kind', '') == 'wifi':
            adc = getattr(self.hub, 'telemetry', {}).get('adc', {})
            mv = adc.get('mv')
            if ADC_TO_BATTERY_ENABLED and mv is not None:
                try:
                    v_adc = float(mv) / 1000.0
                    ratio = (BAT_ADC_R_TOP + BAT_ADC_R_BOTTOM) / BAT_ADC_R_BOTTOM
                    bat_raw = v_adc * ratio
                    source = 'ADC'
                except Exception:
                    bat_raw = None

        if bat_raw is None:
            # No new battery reading. Do not keep stale display forever if connection is absent.
            txt = '--V --%'
            pct = 0.0
            color = '#94a3b8'
            source_txt = ''
        else:
            bat_raw = float(bat_raw)

            # Reset filter when source changes or first valid reading arrives.
            if getattr(self, 'battery_voltage_filtered', None) is None or getattr(self, 'battery_last_source', '') != source:
                self.battery_voltage_filtered = bat_raw
                self.battery_last_source = source
            else:
                self.battery_voltage_filtered = (
                    (1.0 - VOLTAGE_FILTER_ALPHA) * float(self.battery_voltage_filtered)
                    + VOLTAGE_FILTER_ALPHA * bat_raw
                )

            bat = float(self.battery_voltage_filtered)
            pct_raw = max(0.0, min(100.0, (bat - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0))

            # Percent display is deliberately slower than voltage. This prevents visible jumping
            # such as 79% -> 82% -> 80% when ADC only jitters slightly.
            if getattr(self, 'battery_pct_display', None) is None:
                self.battery_pct_display = pct_raw
            else:
                diff = pct_raw - float(self.battery_pct_display)
                if abs(diff) >= PCT_UPDATE_DEADBAND:
                    self.battery_pct_display = float(self.battery_pct_display) + PCT_FILTER_ALPHA * diff

            pct = max(0.0, min(100.0, float(self.battery_pct_display)))
            txt = f'{bat:.{DISPLAY_DECIMALS}f}V {pct:.0f}%'
            color = '#22c55e' if pct >= 50 else '#f59e0b' if pct >= 25 else '#ef4444'
            source_txt = source

        cv.create_rectangle(4, 6, 102, 28, outline='#111827', width=2)
        cv.create_rectangle(104, 12, 112, 22, fill='#111827')
        fill_w = max(0, min(94, pct / 100.0 * 94))
        cv.create_rectangle(8, 10, 8 + fill_w, 24, fill=color, outline='')
        cv.create_text(53, 17, text=txt, fill='#111827', font=('Segoe UI', 9, 'bold'))
        cv.create_text(116, 17, text=source_txt, anchor='w', fill='#64748b', font=('Segoe UI', 7))



    def draw_map(self,cv):
        if cv is None: return
        cv.delete('all')
        ox,oy,mw,mh=self.get_map_rect(cv)
        def px(x,y): return ox+x/FIELD_WIDTH_M*mw, oy+(1-y/FIELD_HEIGHT_M)*mh
        cv.create_rectangle(ox,oy,ox+mw,oy+mh,outline='#bfdbfe',width=2)
        for i in range(7):
            t=i/6
            cv.create_line(ox+t*mw,oy,ox+t*mw,oy+mh,fill='#333')
            cv.create_line(ox,oy+t*mh,ox+mw,oy+t*mh,fill='#333')
        def track(tr,color,width=2):
            pts=[]
            for x,y in tr:
                if -0.2<=x<=FIELD_WIDTH_M+0.2 and -0.2<=y<=FIELD_HEIGHT_M+0.2:
                    pts+=list(px(x,y))
            if len(pts)>=4:
                cv.create_line(*pts,fill=color,width=width)
        # Odom track remains available for reference. Current Go visual track is highlighted.
        track(self.parser.odom_track,'#22c55e',1)
        track(self.vision_track,'#0ea5e9',1)
        track(getattr(self, 'vision_goto_track', []),'#f97316',3)
        if self.target_xy:
            tx,ty=self.target_xy
            x,y=px(tx,ty)
            cv.create_oval(x-8,y-8,x+8,y+8,outline='#ff8c00',width=3)
            cv.create_line(x-12,y,x+12,y,fill='#ff8c00')
            cv.create_line(x,y-12,x,y+12,fill='#ff8c00')
            cv.create_text(x+10,y-14,text=f'T({tx:.2f},{ty:.2f})',fill='#fbbf24',anchor='w',font=('Segoe UI',8,'bold'))
        if self.parser.odom_track:
            x,y=px(self.parser.odom_x,self.parser.odom_y)
            cv.create_oval(x-4,y-4,x+4,y+4,fill='#22c55e')
        if self.car_pose:
            x,y,yaw,src=self.car_pose
            X,Y=px(x,y)
            if yaw is not None:
                L = CAR_LENGTH_M
                W = CAR_WIDTH_M
                # local car coordinates: front is +x, left/right along +y/-y
                corners=[(L/2,W/2),(L/2,-W/2),(-L/2,-W/2),(-L/2,W/2)]
                pts=[]
                for lx,ly in corners:
                    wx=x+lx*math.cos(yaw)-ly*math.sin(yaw)
                    wy=y+lx*math.sin(yaw)+ly*math.cos(yaw)
                    pts.extend(px(wx,wy))
                cv.create_polygon(*pts,fill='#0ea5e9',outline='#bfdbfe',width=2)
                fx=x+(L/2)*math.cos(yaw)
                fy=y+(L/2)*math.sin(yaw)
                FX,FY=px(fx,fy)
                cv.create_line(X,Y,FX,FY,fill='#fde047',width=3,arrow='last')
            else:
                cv.create_oval(X-7,Y-7,X+7,Y+7,fill='#00b7ff')
            cv.create_text(X+8,Y+8,text='CAR',fill='#e0f2fe',anchor='nw',font=('Segoe UI',7,'bold'))
        # Live world-coordinate readout at map lower-left. Heading uses map-up as 0 deg.
        if self.car_pose:
            cx_m, cy_m, cyaw, csrc = self.car_pose
            hdg = self.yaw_to_map_heading_deg(cyaw)
            if hdg is None:
                info = f'CAR x={cx_m:.3f} y={cy_m:.3f} heading=--'
            else:
                info = f'CAR x={cx_m:.3f} y={cy_m:.3f} heading={hdg:+.1f} deg'
            cv.create_rectangle(ox+6, oy+mh-30, ox+360, oy+mh-6, fill='#111827', outline='#334155')
            cv.create_text(ox+12, oy+mh-18, text=info, fill='#bfdbfe', anchor='w', font=('Consolas', 9, 'bold'))
        else:
            cv.create_rectangle(ox+6, oy+mh-30, ox+210, oy+mh-6, fill='#111827', outline='#334155')
            cv.create_text(ox+12, oy+mh-18, text='CAR x=-- y=-- heading=--', fill='#64748b', anchor='w', font=('Consolas', 9, 'bold'))

        cv.create_text(ox+10,oy+16,text=f'Field {FIELD_WIDTH_M:.3f}m x {FIELD_HEIGHT_M:.3f}m | ArUco 0/1/2/3 | AprilTag ID10',fill='#eee',anchor='w')

    def draw_gdot(self,cv):
        """Draw BNO085 acceleration as a G-force dot with fading trail.

        V3.9.9.6 assumes the STM32 board has no IMU chip. The G panel therefore
        uses ESP32/BNO085 UDP telemetry directly:
            IMU,...,ax=...,ay=...,az=...
        where ax/ay/az are in m/s^2 from BNO085 accelerometer report.
        """
        if cv is None: return
        cv.delete('all')
        w=max(80,cv.winfo_width())
        h=max(80,cv.winfo_height())
        cx=w/2
        cy=h/2-8
        r=min(w,h)/2-18
        cv.create_oval(cx-r,cy-r,cx+r,cy+r,outline='#555',width=2)
        cv.create_line(cx-r,cy,cx+r,cy,fill='#333')
        cv.create_line(cx,cy-r,cx,cy+r,fill='#333')

        imu = {}
        if getattr(self.hub, 'kind', '') == 'wifi':
            imu = getattr(self.hub, 'telemetry', {}).get('imu', {}) or {}
        ax = imu.get('ax')
        ay = imu.get('ay')
        az = imu.get('az')
        now=time.time()

        if ax is None or ay is None or az is None:
            cv.create_text(cx,cy,text='Waiting BNO085 ACC',fill='#888',font=('Segoe UI',8))
            cv.create_text(cx,h-10,text=f'G --, Max {getattr(self,"max_g_this_run",0.0):.2f}',fill='#ddd',font=('Segoe UI',8))
            return

        try:
            ax=float(ax)
            ay=float(ay)
            az=float(az)
        except Exception:
            cv.create_text(cx,cy,text='ACC parse error',fill='#f59e0b',font=('Segoe UI',8))
            return

        gx=ax/9.81
        gy=ay/9.81
        gz=az/9.81
        # V4.0.9 UART-RVC：RVC加速度包含静态重力，水平静止时通常约为1g。
        # 小车接球测试保持近似水平，因此仅从Z轴扣除静态1g，恢复原界面“静止G约为0”的含义。
        # 不改变ESP32原始遥测，也不影响姿态、GoTo、PID、轮速或其他参数。
        gz_linear = gz - (1.0 if gz >= 0.0 else -1.0)
        gt_raw=math.sqrt(gx*gx+gy*gy+gz_linear*gz_linear)
        gt=gt_raw
        # 静止噪声死区，当前实测静止残余约0.04至0.05G。
        # 0.06为建议初值，待实测。
        G_ZERO_DEADBAND = 0.06

        if gt_raw <= G_ZERO_DEADBAND:
            # 死区内同时归零实时数值和红点坐标。
            gx = 0.0
            gy = 0.0
            gz_linear = 0.0
            gt = 0.0
        else:
            gt = gt_raw     
        
        self.max_g_this_run=max(getattr(self,'max_g_this_run',0.0), gt)
        if not hasattr(self,'g_trace'):
            self.g_trace=deque(maxlen=160)
        self.g_trace.append((now,gx,gy,gt))
        lim=5.0

        # Draw fading red residual trail. Older points fade and shrink.
        for ts,tx,ty,tg in list(self.g_trace):
            age=now-ts
            if age>2.0:
                continue
            fade=max(0.0,1.0-age/2.0)
            red=int(90+165*fade)
            x=cx+max(-lim,min(lim,ty))/lim*r
            y=cy-max(-lim,min(lim,tx))/lim*r
            radius=2+int(4*fade)
            cv.create_oval(x-radius,y-radius,x+radius,y+radius,fill=f'#{red:02x}2222',outline='')

        x=cx+max(-lim,min(lim,gy))/lim*r
        y=cy-max(-lim,min(lim,gx))/lim*r
        cv.create_oval(x-6,y-6,x+6,y+6,fill='#ef4444',outline='#fecaca',width=2)
        cv.create_text(cx,h-10,text=f'G {gt:.2f}, Max {self.max_g_this_run:.2f}',fill='#ddd',font=('Segoe UI',8,'bold'))

    def draw_attitude(self, cv):
        """Pseudo-3D car attitude display driven by ESP32/BNO085 telemetry.

        V4.0.4 fixes:
            - Roll uses wrap_deg() so 0 <-> 359 deg does not jump.
            - Roll sign is inverted to match the visual left/right roll direction.
            - Roll drawing is constrained to +/-45 deg for stable display.
            - The model is simplified to a flat cuboid with 25:20:7 length/width/height ratio.
        """
        if cv is None:
            return
        cv.delete('all')
        w = max(80, cv.winfo_width())
        h = max(60, cv.winfo_height())
        cx = w / 2
        cy = h / 2 + 10

        imu = {}
        telemetry = getattr(self.hub, 'telemetry', {}) if getattr(self.hub, 'kind', '') == 'wifi' else {}
        if telemetry:
            imu = telemetry.get('imu', {}) or {}

        bno = int(imu.get('bno', 0) or 0) if imu else 0
        yaw_raw = float(imu.get('yaw', 0.0) or 0.0)
        pitch_raw = float(imu.get('pitch', 0.0) or 0.0)
        roll_raw = float(imu.get('roll', 0.0) or 0.0)

        # Capture current BNO085 rotation-vector attitude as display zero.
        if bno and getattr(self, 'att_zero_pending', False):
            self.att_zero_yaw = yaw_raw
            self.att_zero_pitch = pitch_raw
            self.att_zero_roll = roll_raw
            self.att_zero_valid = True
            self.att_zero_pending = False
            try:
                telemetry['gyro_yaw_deg'] = 0.0
                telemetry['last_gyro_t'] = None
            except Exception:
                pass
            try:
                self.status_var.set(f'BNO085 zero set: yaw={yaw_raw:.1f}, pitch={pitch_raw:.1f}, roll={roll_raw:.1f}; gyro yaw reset')
            except Exception:
                pass

        if bno and getattr(self, 'att_zero_valid', False):
            # V4.0.9 UART-RVC显示映射：
            # 1) UART-RVC yaw方向与旧I2C显示映射相反，因此不再额外取负。
            # 2) 当前模块安装方向下，RVC pitch/roll与原3D模型轴互换。
            yaw_mag = self.wrap_deg(yaw_raw - self.att_zero_yaw)
            pitch = self.wrap_deg(roll_raw - self.att_zero_roll)
            roll = -self.wrap_deg(pitch_raw - self.att_zero_pitch)
        elif bno:
            yaw_mag = self.wrap_deg(yaw_raw)
            pitch = self.wrap_deg(roll_raw)
            roll = -self.wrap_deg(pitch_raw)
        else:
            try:
                yaw_mag = math.degrees(self.parser.data.get('imu_yaw_rel_rad', 0.0) or 0.0)
            except Exception:
                yaw_mag = 0.0
            pitch = 0.0
            roll = 0.0

        gyro_yaw = None
        try:
            if telemetry and telemetry.get('last_gyro_t') is not None:
                gyro_yaw = self.wrap_deg(float(telemetry.get('gyro_yaw_deg', 0.0)))
        except Exception:
            gyro_yaw = None

        # Use gyro yaw for the 3D car if available, because it is not corrected by magnetometer.
        yaw = yaw_mag

        cv.create_rectangle(0, 0, w, h, fill='#111827', outline='')
        if bno:
            status = 'BNO085 GYRO' if gyro_yaw is not None else ('BNO085 ZERO' if getattr(self, 'att_zero_valid', False) else 'BNO085 RAW')
            status_color = '#22c55e' if gyro_yaw is not None or getattr(self, 'att_zero_valid', False) else '#facc15'
        else:
            status = 'BNO085 OFF'
            status_color = '#94a3b8'
        if getattr(self, 'att_zero_pending', False):
            status = 'ZERO PENDING'
            status_color = '#fb923c'

        cv.create_text(6, 6, text=status, anchor='nw', fill=status_color, font=('Segoe UI', 8, 'bold'))
        cv.create_text(w - 6, 6, text=f'Y {yaw:+.0f}', anchor='ne', fill='#cbd5e1', font=('Segoe UI', 8, 'bold'))
        mag_text = f'MAG {yaw_mag:+.0f}'
        if gyro_yaw is not None:
            mag_text = f'GYRO {gyro_yaw:+.0f} / MAG {yaw_mag:+.0f}'
        cv.create_text(w / 2, 22, text=mag_text, anchor='n', fill='#93c5fd', font=('Segoe UI', 7))

        # Nose-up convention: yaw=0 should draw forward/up.
        # Original local car nose is +X; subtracting 90deg maps +X to screen-up.
        yaw_r = math.radians(yaw - 90.0)
        pitch_r = math.radians(max(-35, min(35, pitch)))
        roll_for_draw = max(-45, min(45, roll))
        roll_r = math.radians(roll_for_draw)

        # Flat cuboid model, real ratio roughly length 25cm, width 20cm, height 7cm.
        # Local +X is car nose / forward.
        S = 2.2
        L = 25.0 * S
        W = 20.0 * S
        H = 7.0 * S
        pts3 = [
            (-L/2, -W/2, 0), ( L/2, -W/2, 0), ( L/2,  W/2, 0), (-L/2,  W/2, 0),
            (-L/2, -W/2, H), ( L/2, -W/2, H), ( L/2,  W/2, H), (-L/2,  W/2, H),
        ]

        def rot_project(p):
            x, y, z = p
            # Roll around local X axis.
            y, z = y * math.cos(roll_r) - z * math.sin(roll_r), y * math.sin(roll_r) + z * math.cos(roll_r)
            # Pitch around local Y axis.
            x, z = x * math.cos(pitch_r) + z * math.sin(pitch_r), -x * math.sin(pitch_r) + z * math.cos(pitch_r)
            # Yaw around vertical axis.
            x, y = x * math.cos(yaw_r) - y * math.sin(yaw_r), x * math.sin(yaw_r) + y * math.cos(yaw_r)
            return (cx + x, cy + y * 0.72 - z * 0.85)

        pts = [rot_project(p) for p in pts3]
        bottom = pts[:4]
        top = pts[4:]

        # Draw bottom and side edges first, then top face.
        cv.create_polygon(bottom, fill='#1e3a8a', outline='#60a5fa')
        for i in range(4):
            j = (i + 1) % 4
            cv.create_line(bottom[i][0], bottom[i][1], bottom[j][0], bottom[j][1], fill='#3b82f6')
            cv.create_line(top[i][0], top[i][1], top[j][0], top[j][1], fill='#bfdbfe')
            cv.create_line(bottom[i][0], bottom[i][1], top[i][0], top[i][1], fill='#3b82f6')
        cv.create_polygon(top, fill='#2563eb', outline='#bfdbfe')

        # Nose arrow, local +X is forward.
        nose = rot_project((L/2 + 14, 0, H + 2))
        center_top = rot_project((0, 0, H + 2))
        cv.create_line(center_top[0], center_top[1], nose[0], nose[1], fill='#facc15', width=3, arrow='last')
        cv.create_line(8, h - 24, w - 8, h - 24, fill='#334155')
        cv.create_text(cx, h - 12, text=f'R {roll:+.1f}  P {pitch:+.1f}', fill='#cbd5e1', font=('Segoe UI', 8, 'bold'))



    def draw_wheels(self,cv):
        if cv is None: return
        cv.delete('all')
        w=max(120,cv.winfo_width())
        h=max(220,cv.winfo_height())
        labels=WHEEL_PHYS_LABELS
        positions=[(0,0),(1,0),(0,1),(1,1)]
        panel_w=w/2
        panel_h=h/2
        y_limit=110.0
        for draw_idx,(col,row) in enumerate(positions):
            idx=WHEEL_DRAW_ORDER[draw_idx]
            x0=col*panel_w+4
            y0=row*panel_h+4
            x1=(col+1)*panel_w-4
            y1=(row+1)*panel_h-4
            live=self.pid_live[idx]
            diag=self.motor_diag(idx)
            outline={'OK':'#16a34a','WATCH':'#d97706','ALARM':'#dc2626','NO DATA':'#94a3b8'}.get(diag,'#94a3b8')
            cv.create_rectangle(x0,y0,x1,y1,fill='#f8fafc',outline=outline,width=2)

            actual=raw_to_target_pct(live.get('actual')) if live.get('actual') is not None else None
            target=raw_to_target_pct(live.get('target')) if live.get('target') is not None else None
            show_pct=actual if actual is not None else target
            value_text='--' if show_pct is None else f'{show_pct:+.0f}%'
            cv.create_text((x0+x1)/2,y0+13,text=f'{labels[draw_idx]} {value_text}',fill='#0f172a',font=('Segoe UI',8,'bold'))

            bar_top=y0+28
            bar_bottom=y1-18
            bar_cx=(x0+x1)/2
            bar_w=max(14,min(22,(x1-x0)*0.25))
            zero_y=bar_bottom-(bar_bottom-bar_top)*(0+y_limit)/(2*y_limit)
            cv.create_line(x0+8,zero_y,x1-8,zero_y,fill='#94a3b8')
            cv.create_rectangle(bar_cx-bar_w/2,bar_top,bar_cx+bar_w/2,bar_bottom,fill='#e5e7eb',outline='#cbd5e1')

            def pct_to_y(pct):
                pct=max(-y_limit,min(y_limit,float(pct)))
                return bar_bottom-(bar_bottom-bar_top)*(pct+y_limit)/(2*y_limit)

            if actual is not None:
                ay=pct_to_y(actual)
                if actual>=0:
                    cv.create_rectangle(bar_cx-bar_w/2,ay,bar_cx+bar_w/2,zero_y,fill='#38bdf8',outline='')
                else:
                    cv.create_rectangle(bar_cx-bar_w/2,zero_y,bar_cx+bar_w/2,ay,fill='#38bdf8',outline='')
            if target is not None:
                ty=pct_to_y(target)
                cv.create_line(bar_cx-bar_w*0.9,ty,bar_cx+bar_w*0.9,ty,fill='#22c55e',width=3)

            cv.create_text(x0+7,bar_top,text='+100',anchor='w',fill='#94a3b8',font=('Segoe UI',6))
            cv.create_text(x0+7,zero_y,text='0',anchor='w',fill='#64748b',font=('Segoe UI',6))
            cv.create_text(x0+7,bar_bottom,text='-100',anchor='w',fill='#94a3b8',font=('Segoe UI',6))
            cv.create_text((x0+x1)/2,y1-7,text='A blue / T green',fill='#64748b',font=('Segoe UI',6))

    def motor_diag(self,ch):
        live=self.pid_live[ch]
        t=live['target']
        a=live['actual']
        now=time.time()
        if t is None or a is None:
            live['alarm_start']=None
            return 'NO DATA'
        active=abs(t)>=PID_ACTIVE_TARGET and abs(a)<=PID_DEAD_ACTUAL
        if active:
            if live['alarm_start'] is None:
                live['alarm_start']=now
            if now-live['alarm_start']>=PID_ALARM_SECONDS:
                return 'ALARM'
            return 'WATCH'
        live['alarm_start']=None
        return 'OK'

    def draw_status_table(self,cv):
        if cv is None: return
        cv.delete('all')
        headers=['Item','X / Vx','Y / Vy','Yaw / Wz','Error / Time']
        widths=[110,130,130,130,130]
        x=12
        y=12
        row_h=24
        cv.create_rectangle(x,y,x+sum(widths),y+row_h,fill='#e5e7eb',outline='')
        xx=x
        for i,h in enumerate(headers):
            cv.create_text(xx+8,y+12,text=h,anchor='w')
            xx+=widths[i]
        d=self.parser.data
        vision=['Vision','--','--','--','']
        diff=['Diff','--','--','--','--']
        if self.car_pose:
            vx,vy,vyaw,_=self.car_pose
            vision=['Vision',f'{vx:.3f}',f'{vy:.3f}','--' if vyaw is None else f'{math.degrees(vyaw):.1f} deg','']
            dx=vx-self.parser.odom_x
            dy=vy-self.parser.odom_y
            diff=['Diff',f'{dx:.3f}',f'{dy:.3f}','',f'{math.hypot(dx,dy)*100:.1f}cm']
        target=['Target','--','--','','']
        if self.target_xy:
            target=['Target',f'{self.target_xy[0]:.3f}',f'{self.target_xy[1]:.3f}','','']
        gr=getattr(self, 'goto_result', {}) or {}
        err = gr.get('error_m')
        elapsed = gr.get('elapsed_s')
        err_txt = '--' if err is None else f'{err*100:.1f}cm'
        time_txt = '--' if elapsed is None else f'{elapsed:.3f}s'
        mode = gr.get('mode','--')
        state = gr.get('state','Idle')
        task=['Task',str(mode),str(state),'',f'{err_txt} / {time_txt}']
        rows=[
            vision,
            ['Odom',f'{self.parser.odom_x:.3f}',f'{self.parser.odom_y:.3f}',f'{math.degrees(self.parser.odom_yaw):.1f} deg',''],
            diff,
            ['Speed',f'{d["vx"]:.3f}',f'{d["vy"]:.3f}',f'{d["wz"]:.3f}',''],
            target,
            task,
        ]
        y+=row_h
        for row in rows:
            xx=x
            for i,v in enumerate(row):
                cv.create_rectangle(xx,y,xx+widths[i],y+row_h,outline='#e5e7eb',fill='white')
                cv.create_text(xx+8,y+12,text=v,anchor='w')
                xx+=widths[i]
            y+=row_h

    def update_wasd_visual(self):
        if not hasattr(self, 'wasd_btns'):
            return
        active = set(getattr(self, 'key_down', set()))
        for k, btn in self.wasd_btns.items():
            try:
                if k in active:
                    btn.config(relief='sunken', bg='#2563eb', fg='white', activebackground='#1d4ed8')
                else:
                    btn.config(relief='raised', bg='SystemButtonFace', fg='black', activebackground='SystemButtonFace')
            except Exception:
                pass

    def draw_joystick(self):
        cv=self.joy_canvas
        cv.delete('all')
        w=max(100,cv.winfo_width())
        h=max(100,cv.winfo_height())
        cx=w/2
        cy=h/2
        r=min(w,h)/2-35
        cv.create_oval(cx-r,cy-r,cx+r,cy+r,outline='#475569',width=3)
        cv.create_line(cx-r,cy,cx+r,cy,fill='#334155')
        cv.create_line(cx,cy-r,cx,cy+r,fill='#334155')
        if self.joy_active:
            kx=max(-1.0,min(1.0,getattr(self,'joy_disp_x',0.0)))
            ky=max(-1.0,min(1.0,getattr(self,'joy_disp_y',0.0)))
            x=cx+kx*r
            y=cy+ky*r
        else:
            x,y=cx,cy
        cv.create_oval(x-6,y-6,x+6,y+6,fill='#2563eb',outline='#bfdbfe',width=1)
        cv.create_text(8,12,text=f'vx={self.joy_vx:.2f} vy={self.joy_vy:.2f} wz={getattr(self, "joy_wz", 0.0):.2f}',anchor='w',fill='white',font=('Segoe UI',7,'bold'))
        cv.create_text(8,27,text=f'LX={getattr(self, "joy_disp_x", 0.0):+.2f} LY={getattr(self, "joy_disp_y", 0.0):+.2f}',anchor='w',fill='#93c5fd',font=('Segoe UI',7))

    def draw_pid_curve(self):
        cv=self.pid_canvas
        cv.delete('all')
        w=max(100,cv.winfo_width())
        h=max(100,cv.winfo_height())
        mode=self.pid_curve_mode_var.get() if hasattr(self,'pid_curve_mode_var') else 'Single CH'
        if mode == '4 Wheels':
            self.draw_pid_curve_4wheels(cv,w,h)
        else:
            self.draw_pid_curve_single(cv,w,h,self.pid_ch.get())

    def draw_pid_curve_single(self,cv,w,h,ch):
        labels=WHEEL_LABEL_BY_CH
        data=self.pid_hist[ch]
        self.draw_pid_curve_panel(cv, 8, 8, w-8, h-8, data, labels[ch], show_axis=True)

    def draw_pid_curve_4wheels(self,cv,w,h):
        labels=['LF / CH1','RF / CH3','LR / CH2','RR / CH4']
        gap=8
        panel_w=(w-gap*3)/2
        panel_h=(h-gap*3)/2
        for draw_idx,ch in enumerate(WHEEL_DRAW_ORDER):
            col=draw_idx%2
            row=draw_idx//2
            x0=gap+col*(panel_w+gap)
            y0=gap+row*(panel_h+gap)
            x1=x0+panel_w
            y1=y0+panel_h
            self.draw_pid_curve_panel(cv,x0,y0,x1,y1,self.pid_hist[ch],labels[draw_idx],show_axis=False)

    def draw_pid_curve_panel(self,cv,x0,y0,x1,y1,data,title,show_axis=True):
        y_limit = PID_PLOT_LIMIT_PCT
        plot_len = PID_PLOT_LEN
        actual = [raw_to_target_pct(v) for v in list(data['a'])[-plot_len:]]
        target = [raw_to_target_pct(v) for v in list(data['t'])[-plot_len:]]
        actual = [v for v in actual if v is not None]
        target = [v for v in target if v is not None]

        left=x0+(45 if show_axis else 32)
        right=x1-12
        top=y0+24
        bottom=y1-24
        usable_w=max(1,right-left)
        usable_h=max(1,bottom-top)
        cv.create_rectangle(x0,y0,x1,y1,fill='#111111',outline='#334155')
        cv.create_text(x0+8,y0+13,text=title,fill='#dddddd',anchor='w',font=('Segoe UI',8,'bold'))
        cv.create_text(x1-8,y0+13,text='T green / A blue',fill='#94a3b8',anchor='e',font=('Segoe UI',7))
        cv.create_rectangle(left,top,right,bottom,outline='#334155')
        for pct in [-100,-50,0,50,100]:
            y=bottom-usable_h*((pct+y_limit)/(2*y_limit))
            color='#64748b' if pct==0 else '#263244'
            cv.create_line(left,y,right,y,fill=color)
            if show_axis or pct in [-100,0,100]:
                cv.create_text(left-4,y,text=f'{pct:+d}%',fill='#94a3b8',anchor='e',font=('Segoe UI',7))

        def points(series):
            if len(series)<2: return []
            pts=[]
            offset=plot_len-len(series)
            for i,v in enumerate(series):
                v=max(-y_limit,min(y_limit,float(v)))
                x=left+usable_w*(offset+i)/max(1,plot_len-1)
                y=bottom-usable_h*((v+y_limit)/(2*y_limit))
                pts += [x,y]
            return pts
        pt=points(target)
        pa=points(actual)
        if pt:
            cv.create_line(*pt,fill='#22c55e',width=2)
        if pa:
            cv.create_line(*pa,fill='#38bdf8',width=2)
        if not pt and not pa:
            cv.create_text((x0+x1)/2,(y0+y1)/2,text='No PID DATA',fill='#888',font=('Segoe UI',8))
        else:
            # Latest values in the lower right corner.
            latest_a=actual[-1] if actual else None
            latest_t=target[-1] if target else None
            txt=f'T {latest_t:+.0f}%  A {latest_a:+.0f}%' if latest_a is not None and latest_t is not None else ''
            if txt:
                cv.create_text(x1-8,y1-9,text=txt,fill='#cbd5e1',anchor='e',font=('Segoe UI',7,'bold'))

    def draw_pid_status(self):
        cv=self.pid_status_canvas
        cv.delete('all')
        w=max(600,cv.winfo_width())
        h=max(120,cv.winfo_height())
        labels=['LF / CH1','RF / CH3','LR / CH2','RR / CH4']
        gap=8
        left=6
        usable=w-left*2
        box_w=(usable-gap*3)/4
        for draw_idx,i in enumerate(WHEEL_DRAW_ORDER):
            x=left+draw_idx*(box_w+gap)
            y=10
            diag=self.motor_diag(i)
            color={'OK':'#dcfce7','WATCH':'#fef3c7','ALARM':'#fee2e2','NO DATA':'#f1f5f9'}.get(diag,'#f1f5f9')
            outline={'OK':'#16a34a','WATCH':'#d97706','ALARM':'#dc2626','NO DATA':'#94a3b8'}.get(diag,'#94a3b8')
            cv.create_rectangle(x,y,x+box_w,h-10,fill=color,outline=outline,width=2)
            live=self.pid_live[i]
            cv.create_text(x+10,y+16,text=labels[draw_idx],anchor='w',font=('Segoe UI',10,'bold'))
            cv.create_text(x+10,y+40,text=f'Target: {self.fmt_pct(live["target"])}',anchor='w')
            cv.create_text(x+10,y+60,text=f'Actual: {self.fmt_pct(live["actual"])}',anchor='w')
            cv.create_text(x+10,y+80,text=f'Error : {self.fmt_pct(live["error"])}',anchor='w')
            cv.create_text(x+10,y+100,text=f'{diag} / {live["rate"]:.1f} Hz',anchor='w',fill=outline)

    def fmt(self,v):
        return '--' if v is None else f'{v:.1f}'

    def fmt_pct(self,v):
        pct = raw_to_target_pct(v) if v is not None else None
        return '--' if pct is None else f'{pct:+.1f}%'

    def heading_deg_from_pose(self, pose):
        if not pose:
            return None
        try:
            yaw = pose[2]
        except Exception:
            return None
        return self.yaw_to_map_heading_deg(yaw)

    def append_goto_record(self, reason='Done'):
        gr = getattr(self, 'goto_result', {}) or {}
        # V4.0.8 RCPD tuning: record only normal Go runs.
        # Return/center commands are for resetting the car and should not pollute performance data.
        if str(gr.get('mode', '')).lower() == 'center':
            return
        self.goto_record_seq += 1
        start = gr.get('start')
        stop = gr.get('stop')
        target = gr.get('target')
        rec = {
            'id': self.goto_record_seq,
            'mode': gr.get('mode', '--'),
            'state': reason or gr.get('state', '--'),
            'strategy': gr.get('strategy', self.goto_strategy_var.get() if hasattr(self, 'goto_strategy_var') else '--'),
            'strategy_params': gr.get('strategy_params', self.get_strategy_param_summary()),
            'start_x': None if not start else start[0],
            'start_y': None if not start else start[1],
            'start_heading_deg': self.heading_deg_from_pose(start),
            'target_x': None if not target else target[0],
            'target_y': None if not target else target[1],
            'stop_x': None if not stop else stop[0],
            'stop_y': None if not stop else stop[1],
            'stop_heading_deg': self.heading_deg_from_pose(stop),
            'elapsed_s': gr.get('elapsed_s'),
            'error_m': gr.get('error_m'),
            'kp': float(self.kp_var.get()) if hasattr(self, 'kp_var') else None,
            'max_cmd': float(self.max_speed_var.get()) if hasattr(self, 'max_speed_var') else None,
            'stop_radius': float(self.stop_radius_var.get()) if hasattr(self, 'stop_radius_var') else None,
            'max_g': gr.get('max_g'),
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        self.goto_records.append(rec)
        self.refresh_goto_record_table()
        self.log(f"GoTo #{rec['id']} {rec['mode']} {rec['state']} time={self.fmt(rec['elapsed_s'],3)}s error={self.fmt_cm(rec['error_m'])} Kp={self.fmt(rec['kp'],2)}")

    def fmt(self, val, nd=3):
        if val is None:
            return '--'
        try:
            return f'{float(val):.{nd}f}'
        except Exception:
            return '--'

    def fmt_cm(self, val):
        if val is None:
            return '--'
        try:
            return f'{float(val)*100.0:.1f}cm'
        except Exception:
            return '--'

    def refresh_goto_record_table(self):
        if not hasattr(self, 'goto_tree'):
            return
        for iid in self.goto_tree.get_children():
            self.goto_tree.delete(iid)
        for rec in self.goto_records:
            vals=(
                rec['id'], rec['mode'], rec['state'], rec.get('strategy','--'),
                self.fmt(rec['target_x'],3), self.fmt(rec['target_y'],3),
                self.fmt(rec['elapsed_s'],3), '--' if rec['error_m'] is None else f"{rec['error_m']*100.0:.1f}",
                self.fmt(rec['start_x'],3), self.fmt(rec['start_y'],3), self.fmt(rec['start_heading_deg'],1),
                self.fmt(rec['stop_x'],3), self.fmt(rec['stop_y'],3), self.fmt(rec['stop_heading_deg'],1),
                self.fmt(rec['kp'],2), self.fmt(rec['max_cmd'],2), self.fmt(rec['stop_radius'],3), self.fmt(rec['max_g'],2),
                rec.get('strategy_params','')
            )
            self.goto_tree.insert('', 'end', values=vals)

    def clear_goto_records(self):
        self.goto_records.clear()
        self.goto_record_seq = 0
        self.refresh_goto_record_table()
        if hasattr(self, 'log_text'):
            self.log_text.delete('1.0','end')
        self.log('GoTo records cleared.')

    def export_goto_records_csv(self):
        path=filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('CSV','*.csv')])
        if not path:
            return
        fields=['id','timestamp','mode','state','strategy','strategy_params','start_x','start_y','start_heading_deg','target_x','target_y','stop_x','stop_y','stop_heading_deg','elapsed_s','error_m','kp','max_cmd','stop_radius','max_g']
        with open(path,'w',newline='',encoding='utf-8-sig') as f:
            wr=csv.DictWriter(f, fieldnames=fields)
            wr.writeheader()
            for rec in self.goto_records:
                wr.writerow({k: rec.get(k) for k in fields})
        self.status_var.set(f'GoTo records exported: {path}')
        self.log(f'Exported GoTo records: {path}')

    # ---------------- export/log/close ----------------
    def export_csv(self):
        path=filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('CSV','*.csv')])
        if not path: return
        with open(path,'w',newline='',encoding='utf-8') as f:
            wr=csv.writer(f)
            wr.writerow(['type','x','y'])
            for x,y in self.vision_track: wr.writerow(['vision',x,y])
            for x,y in self.parser.odom_track: wr.writerow(['odom',x,y])
        self.log(f'Exported {path}')

    def log(self,msg):
        line=time.strftime('%H:%M:%S')+' '+msg
        if hasattr(self,'log_text'):
            self.log_text.insert('end',line+'\n')
            self.log_text.see('end')

    def on_close(self):
        self.stop_motion()
        if self.cap is not None:
            self.cap.release()
        self.hub.close()
        try:
            if hasattr(self, 'lidar_service'):
                self.lidar_service.stop()
        except Exception:
            pass
        self.destroy()


if __name__ == '__main__':
    App().mainloop()
