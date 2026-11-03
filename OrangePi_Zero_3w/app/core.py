import threading
import time
from collections import deque
from statistics import median
import cv2
import numpy as np
import serial
from .ld14 import Parser, Guard, ScanCache
from .protocol import build_motion_cmd
from .tracking.ball_tracker import BallTracker
from .yolo_detector import YoloBallDetector
from .control.lateral_controller import LateralController
from .control.yaw_hold_controller import YawHoldController
from .control.oac_controller import OACController
from .control.motion_arbiter import MotionArbiter

class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = 'BOOT'
        self.network_mode = ''
        self.network_phase = 'SELECTING'
        self.frame = None
        self.annotated = None
        self.ball = None
        self.ball_observation = {'valid': False, 'age_ms': 1e9, 'x_error': 0.0, 'y_norm': 0.0, 'size_norm': 0.0}
        self.auto_vx = self.auto_vy = self.auto_wz = 0.0
        self.fps = 0.0
        self.camera_ok = False
        self.transport_ok = False
        self.imu_ok = False
        self.jx = self.jy = self.throttle = 0.0
        self.turn = 0
        self.last_control = 0.0
        self.yaw = self.yaw_ref = self.yaw_rate = 0.0
        self.last_yaw_t = 0.0
        self.battery_raw_v = None
        self.battery_v = self.battery_pct = None
        self.lidar_online = False
        # LD14始终可继续接收数据；该开关仅决定雷达是否参与BLOCKED安全控制。
        self.lidar_control_enabled = True
        self.lidar_blocked = False
        self.lidar_nearest = None
        self.lidar_clear = False
        self.lidar_data_age_ms = None
        self.catch = False
        self.error = ''

    def clear_motion(self):
        self.jx = self.jy = self.throttle = 0.0
        self.turn = 0
        self.last_control = 0.0
        self.auto_vx = self.auto_vy = self.auto_wz = 0.0

class Vision(threading.Thread):
    daemon = True
    def __init__(self, cfg, state):
        super().__init__()
        self.cfg = cfg
        self.state = state
        self.stop_evt = threading.Event()
        self.tracker = BallTracker(cfg['lateral_tracking'])
        self.yolo_detector = None
        self.detector_lock = threading.RLock()
        self.loaded_yolo_model = None
        self.detector_mode = str(cfg['vision'].get('detector', 'yolo')).lower()
        if self.detector_mode == 'yolo':
            try:
                self.yolo_detector = YoloBallDetector(
                    cfg['vision']['yolo_library'],
                    cfg['vision']['yolo_model'],
                    cfg['vision'],
                )
                self.loaded_yolo_model = cfg['vision']['yolo_model']
            except Exception as exc:
                if not cfg['vision'].get('yolo_fallback_to_hsv', True):
                    raise
                self.detector_mode = 'hsv'
                with self.state.lock:
                    self.state.error = 'YOLO init failed, using HSV: ' + str(exc)

    def apply_vision_config(self, previous_mode=None, previous_model=None):
        """在SAFE状态下应用检测器及YOLO模型变化。"""
        target_mode = str(self.cfg['vision'].get('detector', 'yolo')).lower()
        target_model = str(self.cfg['vision'].get('yolo_model', ''))
        with self.detector_lock:
            if target_mode == 'hsv':
                self.detector_mode = 'hsv'
                return True, 'HSV已生效'
            if target_mode != 'yolo':
                return False, '无效检测器'
            if self.yolo_detector is not None and self.loaded_yolo_model == target_model:
                self.detector_mode = 'yolo'
                return True, 'YOLO已生效'
            old_model = self.loaded_yolo_model or previous_model
            old_detector = self.yolo_detector
            self.yolo_detector = None
            self.loaded_yolo_model = None
            if old_detector is not None:
                old_detector.close()
            try:
                self.yolo_detector = YoloBallDetector(
                    self.cfg['vision']['yolo_library'],
                    target_model,
                    self.cfg['vision'],
                )
                self.loaded_yolo_model = target_model
                self.detector_mode = 'yolo'
                with self.state.lock:
                    self.state.error = ''
                return True, 'YOLO模型已切换'
            except Exception as exc:
                if old_model:
                    try:
                        self.yolo_detector = YoloBallDetector(
                            self.cfg['vision']['yolo_library'],
                            old_model,
                            self.cfg['vision'],
                        )
                        self.loaded_yolo_model = old_model
                        self.detector_mode = previous_mode or 'yolo'
                    except Exception:
                        self.detector_mode = 'hsv'
                with self.state.lock:
                    self.state.error = 'YOLO model switch failed: ' + str(exc)
                return False, str(exc)

    def reset_tracking(self):
        self.tracker.reset()
        with self.state.lock:
            self.state.ball_observation = self.tracker.snapshot()

    def run(self):
        c = self.cfg['camera']
        while not self.stop_evt.is_set():
            cap = cv2.VideoCapture(c['device'])
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c['height'])
            cap.set(cv2.CAP_PROP_FPS, c['fps'])
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*c['fourcc']))
            started = time.monotonic(); frames = 0
            while cap.isOpened() and not self.stop_evt.is_set():
                ok, frame = cap.read()
                if not ok:
                    break
                # 摄像头物理旋转90°后，把画面转回正常方向。
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

                if c.get('flip_horizontal'):
                    frame = cv2.flip(frame, 1)
                if c.get('flip_vertical'):
                    frame = cv2.flip(frame, 0)
                annotated, ball = self.detect(frame)
                observation = self.tracker.update(ball, frame.shape[1], frame.shape[0])
                frames += 1; now = time.monotonic()
                with self.state.lock:
                    self.state.frame = frame
                    self.state.annotated = annotated
                    self.state.ball = ball
                    self.state.ball_observation = observation
                    self.state.camera_ok = True
                    if self.state.mode == 'ARMED' and observation['valid'] and self.cfg['lateral_tracking'].get('enabled', False):
                        self.state.mode = 'TRACKING'
                    if self.state.mode == 'TRACKING' and observation['age_ms'] > float(self.cfg['lateral_tracking'].get('lost_timeout_ms', 180)):
                        self.state.mode = 'ARMED'
                        self.state.auto_vy = 0.0
                    if now - started >= 1.0:
                        self.state.fps = frames / (now - started)
                        frames, started = 0, now
            cap.release()
            with self.state.lock:
                self.state.camera_ok = False
                if self.state.mode == 'TRACKING':
                    self.state.mode = 'ARMED'
            time.sleep(1)

    def detect(self, frame):
        with self.detector_lock:
            mode = self.detector_mode
            detector = self.yolo_detector
            if mode == 'yolo' and detector is not None:
                try:
                    ball, timing = detector.detect(frame)
                    annotated = frame.copy()
                    if ball:
                        x = int(round(ball['x']))
                        y = int(round(ball['y']))
                        w = int(round(ball['w']))
                        h = int(round(ball['h']))
                        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(annotated, (int(round(ball['cx'])), int(round(ball['cy']))), 4, (0, 255, 255), -1)
                        cv2.putText(annotated, 'ball %.2f' % ball.get('confidence', 0.0),
                                    (x, max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    return annotated, ball
                except Exception as exc:
                    with self.state.lock:
                        self.state.error = 'YOLO detect failed: ' + str(exc)
                    if not self.cfg['vision'].get('yolo_fallback_to_hsv', True):
                        return frame.copy(), None
        return self.detect_hsv(frame)

    def detect_hsv(self, frame):
        v = self.cfg['vision']
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hc=int(v['h_center']);hr=int(v['h_range']);sl=max(0,int(v['s_center'])-int(v['s_range']));sh=min(255,int(v['s_center'])+int(v['s_range']));vl=max(0,int(v['v_center'])-int(v['v_range']));vh=min(255,int(v['v_center'])+int(v['v_range']));lo,hi=hc-hr,hc+hr
        if lo<0: mask=cv2.bitwise_or(cv2.inRange(hsv,np.array([0,sl,vl],np.uint8),np.array([hi,sh,vh],np.uint8)),cv2.inRange(hsv,np.array([180+lo,sl,vl],np.uint8),np.array([179,sh,vh],np.uint8)))
        elif hi>179: mask=cv2.bitwise_or(cv2.inRange(hsv,np.array([lo,sl,vl],np.uint8),np.array([179,sh,vh],np.uint8)),cv2.inRange(hsv,np.array([0,sl,vl],np.uint8),np.array([hi-180,sh,vh],np.uint8)))
        else: mask=cv2.inRange(hsv,np.array([lo,sl,vl],np.uint8),np.array([hi,sh,vh],np.uint8))
        size = max(1, int(v['morphology_kernel']) | 1)
        kernel = np.ones((size,size),np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            area = cv2.contourArea(contour); perimeter = cv2.arcLength(contour,True)
            circularity = 4*np.pi*area/(perimeter*perimeter) if perimeter else 0
            if v['min_area'] <= area <= v['max_area'] and circularity >= v['min_circularity']:
                x,y,width,height = cv2.boundingRect(contour)
                candidate = dict(x=x,y=y,w=width,h=height,cx=x+width/2,cy=y+height/2,area=area,circularity=circularity)
                if best is None or area*circularity > best['area']*best['circularity']:
                    best = candidate
        annotated = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR) if v.get('draw_mask_preview') else frame.copy()
        if best:
            cv2.rectangle(annotated,(best['x'],best['y']),(best['x']+best['w'],best['y']+best['h']),(0,255,0),2)
            cv2.circle(annotated,(int(best['cx']),int(best['cy'])),4,(0,255,255),-1)
        return annotated,best

    def sample(self,xn,yn):
        with self.state.lock:
            frame = None if self.state.frame is None else self.state.frame.copy()
        if frame is None: return None
        h,w = frame.shape[:2]; x=min(w-1,max(0,int(xn*w))); y=min(h-1,max(0,int(yn*h)))
        r=int(self.cfg['vision']['sample_radius']); roi=frame[max(0,y-r):y+r+1,max(0,x-r):x+r+1]
        values=np.median(cv2.cvtColor(roi,cv2.COLOR_BGR2HSV).reshape(-1,3),axis=0).astype(int)
        self.cfg['vision']['h_center'],self.cfg['vision']['s_center'],self.cfg['vision']['v_center']=map(int,values)
        return dict(h=int(values[0]),s=int(values[1]),v=int(values[2]))

class Link(threading.Thread):
    daemon = True
    def __init__(self,cfg,state):
        super().__init__(); self.cfg=cfg; self.state=state; self.stop_evt=threading.Event(); self.serial=None
        lateral=LateralController(cfg['lateral_tracking']); yaw=YawHoldController(cfg['imu']); oac=OACController(cfg['oac'])
        self.arbiter=MotionArbiter(cfg,lateral,yaw,oac); self.last_tick=time.monotonic()
        self.arm_trace = bool(cfg.get('_arm_trace', False))
        self.arm_trace_started = None
        self.battery_samples=deque(maxlen=max(1,int(cfg.get('battery',{}).get('filter_window',5))))
    def reset_automatic(self): self.arbiter.reset()
    def send(self,data):
        try:
            if self.serial and self.serial.is_open: self.serial.write(data)
        except Exception: pass
    def send_stop_repeated(self,count=5,interval=0.05):
        from .protocol import STOP_FRAME
        for _ in range(count): self.send(STOP_FRAME); time.sleep(interval)
    def run(self):
        while not self.stop_evt.is_set():
            try:
                if not self.serial or not self.serial.is_open:
                    self.serial=serial.Serial(self.cfg['transport']['serial_device'],self.cfg['transport']['baud'],timeout=0.01)
                with self.state.lock: self.state.transport_ok=True
                self.receive(); self.tick(); time.sleep(1/max(1,self.cfg['manual']['send_hz']))
            except Exception as exc:
                with self.state.lock: self.state.transport_ok=False; self.state.error=str(exc)
                time.sleep(self.cfg['transport']['reconnect_s'])
    def receive(self):
        data=self.serial.read(4096)
        for line in data.decode('utf-8','ignore').splitlines():
            parts={item.split('=',1)[0]:item.split('=',1)[1] for item in line.split(',')[1:] if '=' in item}; now=time.monotonic()
            with self.state.lock:
                if line.startswith('IMU,') and 'yaw' in parts:
                    new_yaw=float(parts['yaw']); dt=max(.001,now-self.state.last_yaw_t)
                    self.state.yaw_rate=((new_yaw-self.state.yaw+180)%360-180)/dt; self.state.yaw=new_yaw; self.state.last_yaw_t=now; self.state.imu_ok=True
                elif line.startswith('ADC,'):
                    raw=float(parts.get('vbat',0)); b=self.cfg['battery']
                    if raw>0:
                        self.battery_samples.append(raw);med=float(median(self.battery_samples));alpha=float(b.get('filter_alpha',.12));self.state.battery_raw_v=raw;self.state.battery_v=med if self.state.battery_v is None else self.state.battery_v+alpha*(med-self.state.battery_v);self.state.battery_pct=max(0,min(100,100*(self.state.battery_v-b['empty_voltage'])/(b['full_voltage']-b['empty_voltage'])))
    def tick(self):
        now=time.monotonic(); dt=max(.001,min(.2,now-self.last_tick)); self.last_tick=now; manual=self.cfg['manual']
        with self.state.lock:
            mode=self.state.mode; alive=(now-self.state.last_control)*1000<manual['heartbeat_timeout_ms']; vx=vy=wz=0.0
            if mode=='MANUAL' and alive:
                vx=-self.state.jy*manual['max_translation_mps']*self.state.throttle
                vy=self.state.jx*manual['max_translation_mps']*self.state.throttle
                wz=self.state.turn*manual['max_rotation_radps']*self.state.throttle
            elif mode in ('ARMED','TRACKING','AUTO_MOVE'):
                fresh=(now-self.state.last_yaw_t)*1000<self.cfg['imu']['timeout_ms']
                vx,vy,wz=self.arbiter.automatic(mode,self.state.ball_observation,dt,self.state.yaw,self.state.yaw_ref,self.state.yaw_rate,fresh)
            else: self.arbiter.reset()
            if mode=='MANUAL' and not alive:
                self.state.clear_motion(); self.state.mode='SAFE'; vx=vy=wz=0.0
            self.state.auto_vx,self.state.auto_vy,self.state.auto_wz=vx,vy,wz
            m=self.cfg.get('motion',{});ox=vx*float(m.get('vx_output_sign',1));oy=vy*float(m.get('vy_output_sign',1));ow=wz*float(m.get('wz_output_sign',1))
            observation = dict(self.state.ball_observation)
            trace = dict(self.arbiter.last_trace)
        if self.arm_trace and mode in ('ARMED','TRACKING','AUTO_MOVE'):
            if self.arm_trace_started is None:
                self.arm_trace_started = now
            elapsed = now - self.arm_trace_started
            print(
                '[ARMTRACE] '
                f't={elapsed:.3f} mode={mode} '
                f'detected={int(bool(observation.get("detected", False)))} '
                f'valid={int(bool(observation.get("valid", False)))} '
                f'age_ms={float(observation.get("age_ms", 1e9)):.1f} '
                f'ball=({float(observation.get("x_error", 0.0)):.3f},'
                f'{float(observation.get("y_norm", 0.0)):.3f}) '
                f'Xreq={float(trace.get("x_request", 0.0)):.3f} '
                f'Yreq={float(trace.get("y_request", 0.0)):.3f} '
                f'OUT_X={float(trace.get("x_output", 0.0)):.3f} '
                f'OUT_Y={float(trace.get("y_output", 0.0)):.3f} '
                f'limited={int(bool(trace.get("limited", False)))} '
                f'scale={float(trace.get("scale", 1.0)):.3f}',
                flush=True,
            )
        else:
            self.arm_trace_started = None
        self.send(build_motion_cmd(ox,oy,ow))

class Lidar(threading.Thread):
    daemon=True
    def __init__(self,cfg,state,blocked_callback):
        super().__init__();self.cfg=cfg;self.state=state;self.blocked_callback=blocked_callback;self.stop_evt=threading.Event();lc=cfg['lidar'];self.parser=Parser(lc.get('sdk_angle_correction',True));self.guard=Guard(lc);self.cache=ScanCache(lc)
        with self.state.lock:
            self.state.lidar_control_enabled = bool(lc.get('control_enabled', True))
    def run(self):
        if not self.cfg['lidar'].get('enabled',True):return
        lc=self.cfg['lidar'];timeout=float(lc.get('offline_timeout_s',.6))
        while not self.stop_evt.is_set():
            try:
                with serial.Serial(lc['direct_serial_device'],lc['baud'],timeout=.1) as port:
                    last=time.monotonic()
                    while not self.stop_evt.is_set():
                        frames=self.parser.feed(port.read(512));now=time.monotonic()
                        if frames:last=now
                        for pts in frames:
                            self.cache.update(pts,now);blocked=self.guard.evaluate(self.cache.snapshot(now))
                            with self.state.lock:
                                self.state.lidar_online=True;self.state.lidar_nearest=self.guard.nearest;self.state.lidar_clear=self.guard.nearest is None;self.state.lidar_data_age_ms=0
                                control_enabled = self.state.lidar_control_enabled
                                # 旁路状态下保留实时距离，但不保留历史BLOCKED锁存。
                                if not control_enabled:
                                    self.state.lidar_blocked = False
                            if blocked and control_enabled:
                                self.blocked_callback()
                        if not frames:
                            age=now-last
                            with self.state.lock:
                                self.state.lidar_data_age_ms=round(age*1000)
                                if age>timeout:self.state.lidar_online=False;self.state.lidar_nearest=None;self.state.lidar_clear=False
            except Exception:
                with self.state.lock:self.state.lidar_online=False;self.state.lidar_nearest=None;self.state.lidar_clear=False
                time.sleep(1)
