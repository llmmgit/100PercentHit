# -*- coding: utf-8 -*-
"""
LD14 Studio V0.2.1
LD14 + ESP32 USB/WiFi UDP 专业二维点云测试工具

运行时文件名: ld14_viewer_v0_2.py
依据官方 ldlidar_sl_sdk:
- 47字节帧、0x54/0x2C同步、CRC8
- 包角跨度有效性检查
- LD14测距中心修正 SlTransform
- 左手坐标系，0度正前方，顺时针增加
"""
import csv
import math
import queue
import socket
import threading
import time
from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None

FRAME_LEN = 47
POINTS_PER_FRAME = 12
MEASURE_FREQ = 2300.0
OFFSET_X_MM = 5.9
OFFSET_Y_MM = -18.975571
CRC_TABLE = (
0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8)

def crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc

def wrap_deg(v):
    return v % 360.0

def ld14_transform_left(raw_angle_deg, distance_mm):
    """等价移植官方 SlTransform 的 LD14 左手坐标分支。"""
    if distance_mm <= 0:
        return wrap_deg(raw_angle_deg), 0.0
    x = distance_mm + OFFSET_X_MM
    y = distance_mm * 0.11923 + OFFSET_Y_MM
    shift = math.degrees(math.atan2(y, x))
    return wrap_deg(raw_angle_deg - shift), shift

@dataclass
class LidarPoint:
    raw_angle: float
    angle: float
    distance: int
    intensity: int
    timestamp: int
    shift: float

class LD14Parser:
    def __init__(self, on_frame):
        self.buf = bytearray()
        self.on_frame = on_frame
        self.good = 0
        self.bad_crc = 0
        self.bad_span = 0
        self.discarded = 0
        self.bytes_rx = 0
        self.last_frame_monotonic = 0.0

    @staticmethod
    def u16(p):
        return p[0] | (p[1] << 8)

    def reset_stats(self):
        self.good = self.bad_crc = self.bad_span = self.discarded = self.bytes_rx = 0

    def feed(self, data):
        self.bytes_rx += len(data)
        self.buf.extend(data)
        while True:
            i = self.buf.find(b'\x54\x2c')
            if i < 0:
                if len(self.buf) > 1:
                    self.discarded += len(self.buf) - 1
                    del self.buf[:-1]
                return
            if i:
                self.discarded += i
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                return
            frame = bytes(self.buf[:FRAME_LEN])
            if crc8(frame[:-1]) != frame[-1]:
                self.bad_crc += 1
                del self.buf[0]
                continue
            del self.buf[:FRAME_LEN]
            decoded = self.decode(frame)
            if decoded is None:
                self.bad_span += 1
                continue
            self.good += 1
            self.last_frame_monotonic = time.monotonic()
            self.on_frame(decoded)

    def decode(self, f):
        speed = self.u16(f[2:4])
        start_raw = self.u16(f[4:6])
        end_raw = self.u16(f[42:44])
        timestamp = self.u16(f[44:46])
        diff_raw = (end_raw + 36000 - start_raw) % 36000
        diff_deg = diff_raw / 100.0
        max_span = speed * POINTS_PER_FRAME / MEASURE_FREQ * 1.5
        if speed <= 0 or diff_deg > max_span:
            return None
        start_deg = start_raw / 100.0
        step = diff_raw / (POINTS_PER_FRAME - 1) / 100.0
        points = []
        for i in range(POINTS_PER_FRAME):
            off = 6 + i * 3
            distance = self.u16(f[off:off+2])
            intensity = f[off+2]
            raw_angle = wrap_deg(start_deg + i * step)
            angle, shift = ld14_transform_left(raw_angle, distance)
            points.append(LidarPoint(raw_angle, angle, distance, intensity, timestamp, shift))
        return {'speed': speed, 'timestamp': timestamp, 'start': start_deg,
                'end': end_raw / 100.0, 'span': diff_deg, 'points': points}

class LD14Studio(tk.Tk):
    BG = '#061018'; PANEL = '#0B1823'; PANEL2 = '#102331'; GRID = '#1C4050'
    TEXT = '#D8EEF3'; MUTED = '#7195A3'; CYAN = '#27E6FF'; GREEN = '#5CFFAA'
    AMBER = '#FFBE55'; RED = '#FF5F6D'; PURPLE = '#B587FF'

    def __init__(self):
        super().__init__()
        self.title('LD14 Studio V0.2.1  |  100 Percent Hit')
        self.geometry('1360x860')
        self.minsize(1100, 720)
        self.configure(bg=self.BG)
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.rx_queue = queue.Queue(maxsize=2048)
        self.parser = LD14Parser(self.on_frame)
        self.running = False
        self.worker = None
        self.ser = None
        self.sock = None
        self.corrected = [None] * 1440
        self.raw = [None] * 1440
        self.last_speed = 0
        self.last_timestamp = 0
        self.last_span = 0.0
        self.last_rx = 0.0
        self.packet_rate = 0.0
        self.byte_rate = 0.0
        self._rate_t = time.monotonic()
        self._rate_frames = 0
        self._rate_bytes = 0
        self.csv_rows = []
        self.sweep_angle = 0.0
        self.full_scans = 0
        self.last_raw_angle = None
        self.nearest_mm = None
        self.avg_intensity = 0.0
        self.visible_points = 0
        self.setup_style()
        self.build_ui()
        self.refresh_ports()
        self.after(10, self.poll)

    def setup_style(self):
        s = ttk.Style(self)
        try: s.theme_use('clam')
        except Exception: pass
        s.configure('.', background=self.BG, foreground=self.TEXT, fieldbackground=self.PANEL2,
                    bordercolor=self.GRID, lightcolor=self.GRID, darkcolor=self.GRID)
        s.configure('TFrame', background=self.BG)
        s.configure('Panel.TFrame', background=self.PANEL)
        s.configure('TLabel', background=self.BG, foreground=self.TEXT, font=('Segoe UI', 10))
        s.configure('Muted.TLabel', foreground=self.MUTED)
        s.configure('Title.TLabel', font=('Segoe UI Semibold', 20), foreground=self.CYAN)
        s.configure('CardValue.TLabel', font=('Consolas', 18, 'bold'), foreground=self.GREEN, background=self.PANEL)
        s.configure('CardName.TLabel', font=('Segoe UI', 9), foreground=self.MUTED, background=self.PANEL)
        s.configure('TButton', background=self.PANEL2, foreground=self.TEXT, padding=(10,7))
        s.map('TButton', background=[('active','#17374A')])
        s.configure('Accent.TButton', background='#087F98', foreground='white')
        s.map('Accent.TButton', background=[('active','#0AA5C4')])
        # Windows下Combobox只读输入区与展开列表需要分别设置颜色
        s.configure(
            'TCombobox',
            fieldbackground=self.PANEL2,
            background=self.PANEL2,
            foreground=self.TEXT,
            arrowcolor=self.CYAN,
            selectbackground=self.PANEL2,
            selectforeground=self.TEXT
        )
        s.map(
            'TCombobox',
            fieldbackground=[
                ('readonly', self.PANEL2),
                ('disabled', self.PANEL),
                ('!disabled', self.PANEL2)
            ],
            background=[
                ('readonly', self.PANEL2),
                ('active', '#17374A'),
                ('!disabled', self.PANEL2)
            ],
            foreground=[
                ('readonly', self.TEXT),
                ('disabled', self.MUTED),
                ('!disabled', self.TEXT)
            ],
            selectbackground=[('readonly', self.PANEL2)],
            selectforeground=[('readonly', self.TEXT)],
            arrowcolor=[('readonly', self.CYAN), ('active', self.GREEN)]
        )
        # 下拉列表实际由Tk Listbox绘制
        self.option_add('*TCombobox*Listbox.background', self.PANEL2)
        self.option_add('*TCombobox*Listbox.foreground', self.TEXT)
        self.option_add('*TCombobox*Listbox.selectBackground', '#087F98')
        self.option_add('*TCombobox*Listbox.selectForeground', '#FFFFFF')
        self.option_add('*TCombobox*Listbox.font', ('Segoe UI', 10))
        # 兼容部分Windows主题
        self.option_add('*Listbox.background', self.PANEL2)
        self.option_add('*Listbox.foreground', self.TEXT)
        self.option_add('*Listbox.selectBackground', '#087F98')
        self.option_add('*Listbox.selectForeground', '#FFFFFF')
        s.configure('TEntry', fieldbackground=self.PANEL2, foreground=self.TEXT, insertcolor=self.CYAN)
        s.configure('TCheckbutton', background=self.BG, foreground=self.TEXT)

    def build_ui(self):
        head = ttk.Frame(self, padding=(14,10)); head.pack(fill='x')
        titlebox = ttk.Frame(head); titlebox.pack(side='left')
        ttk.Label(titlebox, text='LD14 STUDIO V0.2.1', style='Title.TLabel').pack(anchor='w')
        ttk.Label(titlebox, text='ESP32 双链路二维激光雷达实验台  •  官方SDK几何修正', style='Muted.TLabel').pack(anchor='w')
        self.connection_badge = tk.Label(head, text='● OFFLINE', bg=self.PANEL2, fg=self.MUTED,
                                         font=('Consolas',11,'bold'), padx=14, pady=7)
        self.connection_badge.pack(side='right')

        controls = ttk.Frame(self, padding=(14,2,14,8)); controls.pack(fill='x')
        self.mode = tk.StringVar(value='WiFi UDP')
        ttk.Label(controls,text='链路').pack(side='left')
        ttk.Combobox(controls,textvariable=self.mode,values=['USB串口','WiFi UDP'],width=10,state='readonly').pack(side='left',padx=(5,12))
        self.port = tk.StringVar(); self.port_box=ttk.Combobox(controls,textvariable=self.port,width=13)
        self.port_box.pack(side='left'); ttk.Button(controls,text='刷新',command=self.refresh_ports).pack(side='left',padx=(4,12))
        ttk.Label(controls,text='ESP32 IP').pack(side='left'); self.ip=tk.StringVar(value='192.168.3.87')
        ttk.Entry(controls,textvariable=self.ip,width=15).pack(side='left',padx=(5,12))
        self.connect_btn=ttk.Button(controls,text='连接',style='Accent.TButton',command=self.toggle)
        self.connect_btn.pack(side='left'); ttk.Button(controls,text='清空',command=self.clear_scan).pack(side='left',padx=5)
        ttk.Button(controls,text='导出CSV',command=self.export_csv).pack(side='left')

        body=ttk.Frame(self,padding=(14,0,14,14)); body.pack(fill='both',expand=True)
        left=ttk.Frame(body,style='Panel.TFrame',padding=10); left.pack(side='left',fill='both',expand=True)
        right=ttk.Frame(body,style='Panel.TFrame',padding=12,width=280); right.pack(side='right',fill='y',padx=(10,0)); right.pack_propagate(False)
        self.canvas=tk.Canvas(left,bg='#030B11',highlightthickness=1,highlightbackground=self.GRID)
        self.canvas.pack(fill='both',expand=True)

        ttk.Label(right,text='LIVE TELEMETRY',font=('Segoe UI Semibold',12),background=self.PANEL,foreground=self.CYAN).pack(anchor='w',pady=(0,8))
        self.card_vars={}
        cards=[('转速','speed'),('帧率','fps'),('最近距离','nearest'),('有效点','points'),('平均强度','intensity'),('完整圈数','scans')]
        for name,key in cards:
            card=ttk.Frame(right,style='Panel.TFrame',padding=(8,5)); card.pack(fill='x')
            ttk.Label(card,text=name,style='CardName.TLabel').pack(anchor='w')
            v=tk.StringVar(value='--'); self.card_vars[key]=v
            ttk.Label(card,textvariable=v,style='CardValue.TLabel').pack(anchor='w')

        ttk.Separator(right).pack(fill='x',pady=10)
        ttk.Label(right,text='DISPLAY',font=('Segoe UI Semibold',11),background=self.PANEL,foreground=self.CYAN).pack(anchor='w')
        self.display_mode=tk.StringVar(value='修正点云')
        ttk.Combobox(right,textvariable=self.display_mode,values=['修正点云','原始点云','原始+修正'],state='readonly').pack(fill='x',pady=(6,8))
        row=ttk.Frame(right,style='Panel.TFrame'); row.pack(fill='x',pady=3)
        ttk.Label(row,text='显示范围 m',style='CardName.TLabel').pack(side='left'); self.range_m=tk.DoubleVar(value=4.0)
        ttk.Spinbox(row,from_=1.0,to=12.0,increment=0.5,textvariable=self.range_m,width=7).pack(side='right')
        row=ttk.Frame(right,style='Panel.TFrame'); row.pack(fill='x',pady=3)
        ttk.Label(row,text='最小强度',style='CardName.TLabel').pack(side='left'); self.min_intensity=tk.IntVar(value=0)
        ttk.Spinbox(row,from_=0,to=255,textvariable=self.min_intensity,width=7).pack(side='right')
        row=ttk.Frame(right,style='Panel.TFrame'); row.pack(fill='x',pady=3)
        ttk.Label(row,text='保持时间 s',style='CardName.TLabel').pack(side='left'); self.persistence=tk.DoubleVar(value=0.45)
        ttk.Spinbox(row,from_=0.1,to=2.0,increment=0.05,textvariable=self.persistence,width=7).pack(side='right')
        # 热力色映射窗口只影响颜色，不过滤测距点
        self.heat_low=tk.IntVar(value=100)
        self.heat_high=tk.IntVar(value=240)
        row=ttk.Frame(right,style='Panel.TFrame'); row.pack(fill='x',pady=3)
        ttk.Label(row,text='热力下限',style='CardName.TLabel').pack(side='left')
        ttk.Spinbox(row,from_=0,to=254,increment=5,textvariable=self.heat_low,width=7).pack(side='right')
        row=ttk.Frame(right,style='Panel.TFrame'); row.pack(fill='x',pady=3)
        ttk.Label(row,text='热力上限',style='CardName.TLabel').pack(side='left')
        ttk.Spinbox(row,from_=1,to=255,increment=5,textvariable=self.heat_high,width=7).pack(side='right')
        self.glow=tk.BooleanVar(value=False); self.show_sweep=tk.BooleanVar(value=False); self.show_heat=tk.BooleanVar(value=True)
        ttk.Checkbutton(right,text='点云辉光',variable=self.glow).pack(anchor='w',pady=(8,2))
        ttk.Checkbutton(right,text='扫描线',variable=self.show_sweep).pack(anchor='w',pady=2)
        ttk.Checkbutton(right,text='强度热力色',variable=self.show_heat).pack(anchor='w',pady=2)

        ttk.Separator(right).pack(fill='x',pady=10)
        ttk.Label(right,text='LINK HEALTH',font=('Segoe UI Semibold',11),background=self.PANEL,foreground=self.CYAN).pack(anchor='w')
        self.health=tk.StringVar(value='等待连接')
        ttk.Label(right,textvariable=self.health,style='CardName.TLabel',wraplength=245,justify='left').pack(anchor='w',pady=6)

    def refresh_ports(self):
        ports=[] if serial is None else [p.device for p in serial.tools.list_ports.comports()]
        self.port_box['values']=ports
        if ports and not self.port.get(): self.port.set(ports[0])

    def toggle(self):
        self.disconnect() if self.running else self.connect()

    def connect(self):
        try:
            self.running=True
            if self.mode.get()=='USB串口':
                if serial is None: raise RuntimeError('未安装 pyserial')
                if not self.port.get(): raise RuntimeError('请选择ESP32串口')
                self.ser=serial.Serial(self.port.get(),921600,timeout=0.05)
                self.worker=threading.Thread(target=self.serial_loop,daemon=True)
            else:
                self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                self.sock.bind(('',4211)); self.sock.settimeout(0.2)
                self.sock.sendto(b'HELLO',(self.ip.get().strip(),4210))
                self.worker=threading.Thread(target=self.udp_loop,daemon=True)
            self.worker.start(); self.connect_btn.config(text='断开')
            self.connection_badge.config(text='● CONNECTED',fg=self.GREEN)
        except Exception as e:
            self.running=False; messagebox.showerror('连接失败',str(e))

    def disconnect(self):
        self.running=False
        try:
            if self.ser: self.ser.close()
            if self.sock: self.sock.close()
        except Exception: pass
        self.ser=self.sock=None; self.connect_btn.config(text='连接')
        self.connection_badge.config(text='● OFFLINE',fg=self.MUTED)

    def queue_data(self,data):
        try: self.rx_queue.put_nowait(data)
        except queue.Full: pass

    def serial_loop(self):
        while self.running and self.ser:
            try:
                data=self.ser.read(self.ser.in_waiting or 1)
                if data: self.queue_data(data)
            except Exception: break

    def udp_loop(self):
        while self.running and self.sock:
            try:
                data,_=self.sock.recvfrom(4096)
                if data.startswith(b'LD14ESP,'): continue
                self.queue_data(data)
            except socket.timeout:
                try: self.sock.sendto(b'HELLO',(self.ip.get().strip(),4210))
                except Exception: pass
            except Exception: break

    def on_frame(self,frame):
        now=time.monotonic(); self.last_rx=now; self.last_speed=frame['speed']; self.last_timestamp=frame['timestamp']; self.last_span=frame['span']
        self._rate_frames += 1
        ints=[]; nearest=[]
        for p in frame['points']:
            if self.last_raw_angle is not None and p.raw_angle < 20.0 and self.last_raw_angle > 340.0:
                self.full_scans += 1
            self.last_raw_angle=p.raw_angle; self.sweep_angle=p.angle
            if 20 <= p.distance <= 12000:
                idx_raw=int(round(p.raw_angle*4.0))%1440; idx=int(round(p.angle*4.0))%1440
                self.raw[idx_raw]=(p.distance,p.intensity,now,p.raw_angle,p.shift)
                self.corrected[idx]=(p.distance,p.intensity,now,p.raw_angle,p.shift)
                ints.append(p.intensity); nearest.append(p.distance)
                self.csv_rows.append((time.time(),frame['timestamp'],p.raw_angle,p.angle,p.shift,p.distance,p.intensity))
        if ints: self.avg_intensity=sum(ints)/len(ints)
        if nearest: self.nearest_mm=min(nearest)
        if len(self.csv_rows)>300000: del self.csv_rows[:150000]

    def update_rates(self):
        now=time.monotonic(); dt=now-self._rate_t
        if dt>=0.5:
            self.packet_rate=self._rate_frames/dt
            delta_bytes=self.parser.bytes_rx-self._rate_bytes
            self.byte_rate=delta_bytes/dt
            self._rate_frames=0; self._rate_bytes=self.parser.bytes_rx; self._rate_t=now

    def heat_color(self, intensity):
        """把可调强度窗口映射到蓝、青、绿、黄、红完整色带。"""
        low=int(self.heat_low.get())
        high=int(self.heat_high.get())
        if high<=low:
            high=min(255,low+1)
        t=(float(intensity)-low)/float(high-low)
        t=max(0.0,min(1.0,t))
        if t<0.25:
            q=t/0.25
            r=int(30+(25-30)*q)
            g=int(100+(220-100)*q)
            b=255
        elif t<0.50:
            q=(t-0.25)/0.25
            r=int(25+(70-25)*q)
            g=int(220+(255-220)*q)
            b=int(255+(130-255)*q)
        elif t<0.75:
            q=(t-0.50)/0.25
            r=int(70+(255-70)*q)
            g=int(255+(225-255)*q)
            b=int(130+(40-130)*q)
        else:
            q=(t-0.75)/0.25
            r=255
            g=int(225+(65-225)*q)
            b=int(40+(50-40)*q)
        return f'#{r:02x}{g:02x}{b:02x}'

    def draw_point_set(self,items,cx,cy,radius,rmax,now,keep,raw=False):
        count=0
        for idx,item in enumerate(items):
            if not item: continue
            dist,intensity,t,raw_angle,shift=item
            if now-t>keep or dist/1000.0>rmax or intensity<self.min_intensity.get(): continue
            angle=idx/4.0; a=math.radians(angle); rr=(dist/1000.0)/rmax*radius
            x=cx+math.sin(a)*rr; y=cy-math.cos(a)*rr
            if raw: color=self.PURPLE
            elif self.show_heat.get(): color=self.heat_color(intensity)
            else: color=self.CYAN
            age=(now-t)/keep; size=max(1.0,2.6-age*1.4)
            if self.glow.get() and not raw:
                self.canvas.create_oval(x-4,y-4,x+4,y+4,fill='',outline='#123A45',width=1)
            self.canvas.create_oval(x-size,y-size,x+size,y+size,fill=color,outline='')
            count+=1
        return count

    def draw(self):
        c=self.canvas; c.delete('all'); w=max(c.winfo_width(),20); h=max(c.winfo_height(),20)
        cx,cy=w/2,h/2; radius=min(w,h)*0.43; rmax=max(0.5,float(self.range_m.get()))
        c.create_rectangle(0,0,w,h,fill='#030B11',outline='')
        for frac in (0.25,0.5,0.75,1.0):
            r=radius*frac; c.create_oval(cx-r,cy-r,cx+r,cy+r,outline=self.GRID,width=1)
            c.create_text(cx+8,cy-r+8,text=f'{rmax*frac:.1f} m',fill=self.MUTED,anchor='nw',font=('Consolas',9))
        for deg in range(0,360,30):
            a=math.radians(deg); inner=radius-8; outer=radius
            x1=cx+math.sin(a)*inner; y1=cy-math.cos(a)*inner; x2=cx+math.sin(a)*outer; y2=cy-math.cos(a)*outer
            c.create_line(x1,y1,x2,y2,fill=self.GRID)
            tr=radius+18; c.create_text(cx+math.sin(a)*tr,cy-math.cos(a)*tr,text=f'{deg}°',fill=self.MUTED,font=('Consolas',8))
        c.create_line(cx-radius,cy,cx+radius,cy,fill=self.GRID); c.create_line(cx,cy-radius,cx,cy+radius,fill=self.GRID)
        c.create_text(cx,cy-radius-35,text='FRONT',fill=self.CYAN,font=('Segoe UI Semibold',10))
        c.create_text(cx+radius+42,cy,text='RIGHT',fill=self.MUTED,font=('Segoe UI',9))
        if self.show_sweep.get() and self.last_rx:
            a=math.radians(self.sweep_angle); sx=cx+math.sin(a)*radius; sy=cy-math.cos(a)*radius
            c.create_line(cx,cy,sx,sy,fill='#1D8795',width=2)
        now=time.monotonic(); keep=max(0.05,float(self.persistence.get())); mode=self.display_mode.get(); count=0
        if mode in ('原始点云','原始+修正'):
            count += self.draw_point_set(self.raw,cx,cy,radius,rmax,now,keep,raw=True)
        if mode in ('修正点云','原始+修正'):
            count += self.draw_point_set(self.corrected,cx,cy,radius,rmax,now,keep,raw=False)
        self.visible_points=count
        c.create_oval(cx-13,cy-13,cx+13,cy+13,fill='#0D2833',outline=self.CYAN,width=2)
        c.create_polygon(cx,cy-18,cx-7,cy+3,cx+7,cy+3,fill=self.AMBER,outline='')
        c.create_text(14,14,text='LD14 POLAR SCAN  •  LEFT-HAND / CLOCKWISE',fill=self.CYAN,anchor='nw',font=('Consolas',10,'bold'))
        if mode=='原始+修正':
            c.create_text(14,36,text='● 修正/热力   ● 原始紫色',fill=self.TEXT,anchor='nw',font=('Segoe UI',9))

    def update_cards(self):
        age=time.monotonic()-self.last_rx if self.last_rx else 999
        hz=self.last_speed/360.0
        self.card_vars['speed'].set(f'{hz:5.2f} Hz')
        self.card_vars['fps'].set(f'{self.packet_rate:5.1f} pkt/s')
        self.card_vars['nearest'].set('--' if self.nearest_mm is None else f'{self.nearest_mm/1000.0:5.3f} m')
        self.card_vars['points'].set(str(self.visible_points))
        self.card_vars['intensity'].set(f'{self.avg_intensity:5.1f}')
        self.card_vars['scans'].set(str(self.full_scans))
        total=self.parser.good+self.parser.bad_crc+self.parser.bad_span
        quality=100.0*self.parser.good/total if total else 0.0
        self.health.set(f'CRC OK: {self.parser.good}\nCRC错误: {self.parser.bad_crc}\n角跨度拒绝: {self.parser.bad_span}\n丢弃字节: {self.parser.discarded}\n数据率: {self.byte_rate/1024:.1f} KiB/s\n有效率: {quality:.2f}%')
        if age<0.5:
            self.connection_badge.config(text='● LIVE',fg=self.GREEN)
        elif self.running:
            self.connection_badge.config(text='● NO DATA',fg=self.AMBER)

    def poll(self):
        for _ in range(400):
            try: self.parser.feed(self.rx_queue.get_nowait())
            except queue.Empty: break
        self.update_rates(); self.draw(); self.update_cards(); self.after(33,self.poll)

    def clear_scan(self):
        self.corrected=[None]*1440; self.raw=[None]*1440; self.csv_rows.clear(); self.full_scans=0
        self.nearest_mm=None; self.avg_intensity=0.0; self.parser.reset_stats()

    def export_csv(self):
        if not self.csv_rows:
            messagebox.showinfo('导出','当前没有点云数据'); return
        path=filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('CSV','*.csv')],initialfile='ld14_sdk_corrected_points.csv')
        if not path: return
        with open(path,'w',newline='',encoding='utf-8-sig') as f:
            wr=csv.writer(f)
            wr.writerow(['pc_time_s','lidar_timestamp_ms','raw_angle_deg','corrected_angle_deg','shift_deg','distance_mm','intensity'])
            wr.writerows(self.csv_rows)

    def close(self):
        self.disconnect(); self.destroy()

if __name__=='__main__':
    LD14Studio().mainloop()
