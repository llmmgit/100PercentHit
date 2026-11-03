import copy
import time
import cv2
from flask import Flask, Response, jsonify, render_template, request
from .configuration import runtime_snapshot, save_local

LIMITS = {
    'vision': {
        'h_center': (0,179), 's_center': (0,255), 'v_center': (0,255),
        'h_range': (0,60), 's_range': (0,255), 'v_range': (0,255),
        'min_area': (1,50000), 'max_area': (100,200000), 'min_circularity': (0,1),
        'morphology_kernel': (1,31), 'draw_mask_preview': None,
    },
    'lateral_tracking': {
        'enabled': None, 'kp': (0,1.5), 'kd': (0,0.5), 'center_deadband_norm': (0,0.15),
        'error_filter_alpha': (0.05,1), 'min_vy': (0,1.4), 'max_vy': (0.05,1.4),
        'fast_error_norm': (0.02,1.0), 'fast_vy': (0.05,1.4),
        'max_accel_vy_per_s': (0.1,10), 'acquire_confirm_frames': (1,10),
        'lost_timeout_ms': (50,1000), 'image_error_to_body_y_sign': (-1,1),
    },
    'imu': {
        'yaw_hold_enabled': None, 'required_for_arm': None, 'yaw_kp_radps_per_deg': (0,0.08),
        'yaw_kd_radps_per_dps': (0,0.03), 'yaw_deadband_deg': (0,10),
        'yaw_max_wz_radps': (0.05,1), 'yaw_sign': (-1,1), 'timeout_ms': (100,1500),
    },
    'oac': {
        'enabled': None, 'kp': (0,5), 'kd': (0,3),
        'catch_y_norm': (0.05,0.95), 'lookahead_s': (0,0.60),
        'position_deadband_norm': (0,0.30), 'fast_error_norm': (0.01,0.80),
        'min_vx': (0,1.4), 'max_vx': (0,1.4), 'rate_filter_alpha': (0.01,1),
        'min_upward_rate': (0,1), 'fast_vx': (0,1.4),
        'max_accel_vx_per_s': (0.1,20), 'lost_timeout_ms': (50,1000), 'output_sign': (-1,1),
        'dropout_hold_ms': (0,500), 'overtop_enabled': None, 'overtop_y_norm': (0.02,0.50),
        'overtop_hold_ms': (0,1000), 'overtop_vx': (0,1.4),
    },
}

def validate_update(cfg, payload):
    for section, values in payload.items():
        if section not in LIMITS or not isinstance(values, dict):
            continue
        for key, value in values.items():
            if section == 'vision' and key == 'detector':
                detector = str(value).lower()
                if detector in ('yolo', 'hsv'):
                    cfg['vision']['detector'] = detector
                continue
            if section == 'vision' and key == 'yolo_model':
                model = str(value)
                allowed_models = {
                    'yolo/models/yolov5n_ball_uint8.nb',
                    'yolo/models/yolov5s_ball_uint8.nb',
                }
                if model in allowed_models:
                    cfg['vision']['yolo_model'] = model
                continue
            if key not in LIMITS[section]:
                continue
            limit = LIMITS[section][key]
            if limit is None:
                cfg[section][key] = bool(value)
            else:
                number = float(value)
                number = max(limit[0], min(limit[1], number))
                if key.endswith('_ms') or key in ('h_center','s_center','v_center','h_range','s_range','v_range','min_area','max_area','morphology_kernel','acquire_confirm_frames','dropout_hold_ms','overtop_hold_ms'):
                    number = int(round(number))
                if key in ('image_error_to_body_y_sign','yaw_sign','output_sign'):
                    number = 1.0 if number >= 0 else -1.0
                cfg[section][key] = number

def create(cfg, state, vision, link, config_path, arm_callback, shutdown_callback, network_switch_callback):
    app = Flask(__name__, template_folder='../templates', static_folder='../static')

    def stop_to_safe():
        with state.lock:
            state.clear_motion()
            state.mode = 'SAFE'
            state.catch = False
            state.lidar_blocked = False
        link.reset_automatic()
        vision.reset_tracking()
        link.send_stop_repeated(3)

    @app.get('/')
    def index():
        return render_template('index.html')

    @app.get('/api/status')
    def status():
        with state.lock:
            yaw_rel = ((state.yaw-state.yaw_ref+180)%360)-180
            frame_height = int(state.frame.shape[0]) if state.frame is not None else int(cfg['camera'].get('height',0))
            frame_width = int(state.frame.shape[1]) if state.frame is not None else int(cfg['camera'].get('width',0))
            return jsonify(
                mode=state.mode, network=state.network_mode, camera_ok=state.camera_ok,
                transport_ok=state.transport_ok, imu_ok=state.imu_ok, fps=round(state.fps,1),
                frame_width=frame_width, frame_height=frame_height, preview_fps=int(cfg['server'].get('preview_fps',24)),
                ball=state.ball, observation=state.ball_observation,
                auto=dict(vx=state.auto_vx,vy=state.auto_vy,wz=state.auto_wz),
                yaw_rel=round(yaw_rel,1), battery_raw_v=state.battery_raw_v, battery_v=state.battery_v,
                battery_pct=state.battery_pct, lidar_online=state.lidar_online,
                lidar_control_enabled=state.lidar_control_enabled, lidar_blocked=state.lidar_blocked, lidar_nearest=state.lidar_nearest, lidar_clear=state.lidar_clear, lidar_data_age_ms=state.lidar_data_age_ms,
                active_detector=vision.detector_mode, active_yolo_model=vision.loaded_yolo_model,
                error=state.error,
            )

    @app.post('/api/control')
    def control():
        data=request.get_json(force=True)
        x=max(-1,min(1,float(data.get('x',0))))
        y=max(-1,min(1,float(data.get('y',0))))
        throttle=max(0,min(1,float(data.get('throttle',0))))
        turn=max(-1,min(1,int(data.get('turn',0))))
        manual=cfg['manual']
        takeover=((x*x+y*y)**.5>=manual['takeover_joystick_deadband'] and throttle>=manual['takeover_throttle_min']) or (turn!=0 and throttle>=manual['takeover_throttle_min'])
        with state.lock:
            if state.mode in ('BOOT','FAULT','SHUTTING_DOWN'):
                return jsonify(ok=False,mode=state.mode),409
            if takeover:
                state.mode='MANUAL'; state.lidar_blocked=False; state.catch=False
                link.reset_automatic(); vision.reset_tracking()
            if state.mode=='MANUAL':
                state.jx,state.jy,state.throttle,state.turn=x,y,throttle,turn
                state.last_control=time.monotonic()
        return jsonify(ok=True,mode=state.mode)

    @app.post('/api/stop')
    def stop():
        stop_to_safe()
        return jsonify(ok=True,mode='SAFE')
    @app.post('/api/lidar/toggle')
    def lidar_toggle():
        # 雷达离线时不允许切换，避免把硬件故障误认为主动旁路。
        with state.lock:
            if not state.lidar_online:
                return jsonify(ok=False,message='雷达离线，不能切换保护状态'),409
        # 切换安全传感器参与状态前，先停车并回到SAFE。
        stop_to_safe()
        with state.lock:
            state.lidar_control_enabled = not state.lidar_control_enabled
            state.lidar_blocked = False
            enabled = state.lidar_control_enabled
        return jsonify(ok=True,lidar_control_enabled=enabled,mode='SAFE')

    @app.post('/api/arm')
    def arm():
        ok,message=arm_callback()
        return jsonify(ok=ok,message=message,mode=state.mode),(200 if ok else 409)

    @app.post('/api/shutdown')
    def shutdown():
        with state.lock:
            if state.mode not in ('SAFE','MANUAL'):
                return jsonify(ok=False,message='仅SAFE或MANUAL状态允许网页关机'),409
        shutdown_callback('web')
        return jsonify(ok=True,message='正在安全关机')

    @app.post('/api/network')
    def network():
        target=request.get_json(force=True).get('target')
        if target not in ('wifi','ap'):
            return jsonify(ok=False,message='无效网络模式'),400
        ok,message=network_switch_callback(target)
        return jsonify(ok=ok,message=message),(200 if ok else 409)

    @app.post('/api/settings/open')
    def settings_open():
        with state.lock:
            if state.mode in ('BOOT','FAULT','SHUTTING_DOWN'):
                return jsonify(ok=False,message='当前状态不能打开设置'),409
        stop_to_safe()
        return jsonify(ok=True,settings=runtime_snapshot(cfg))

    @app.get('/api/settings')
    def settings():
        return jsonify(ok=True,settings=runtime_snapshot(cfg))

    @app.post('/api/settings/apply')
    def settings_apply():
        with state.lock:
            if state.mode!='SAFE':
                return jsonify(ok=False,message='仅SAFE状态允许调参'),409
        previous_mode = str(cfg['vision'].get('detector', 'yolo'))
        previous_model = str(cfg['vision'].get('yolo_model', ''))
        validate_update(cfg,request.get_json(force=True))
        ok, message = vision.apply_vision_config(previous_mode, previous_model)
        if not ok:
            cfg['vision']['detector'] = previous_mode
            cfg['vision']['yolo_model'] = previous_model
            return jsonify(ok=False,message='模型切换失败: '+message),500
        link.reset_automatic(); vision.reset_tracking()
        return jsonify(ok=True,settings=runtime_snapshot(cfg),message=message)

    @app.post('/api/settings/save')
    def settings_save():
        with state.lock:
            if state.mode!='SAFE':
                return jsonify(ok=False,message='仅SAFE状态允许保存'),409
        previous_mode = str(cfg['vision'].get('detector', 'yolo'))
        previous_model = str(cfg['vision'].get('yolo_model', ''))
        validate_update(cfg,request.get_json(force=True))
        ok, message = vision.apply_vision_config(previous_mode, previous_model)
        if not ok:
            cfg['vision']['detector'] = previous_mode
            cfg['vision']['yolo_model'] = previous_model
            return jsonify(ok=False,message='模型切换失败: '+message),500
        link.reset_automatic(); vision.reset_tracking(); save_local(cfg)
        return jsonify(ok=True,settings=runtime_snapshot(cfg),message=message)

    @app.post('/api/sample_hsv')
    def sample_hsv():
        data=request.get_json(force=True)
        value=vision.sample(float(data['x']),float(data['y']))
        return jsonify(ok=value is not None,value=value)

    @app.get('/video')
    def video():
        def frames():
            while True:
                with state.lock:
                    frame=None if state.annotated is None else state.annotated.copy()
                if frame is None:
                    time.sleep(.05); continue
                # width=int(cfg['server'].get('preview_width',640))
                # height=int(cfg['server'].get('preview_height',360))
                # frame=cv2.resize(frame,(width,height),interpolation=cv2.INTER_AREA)
                # 保持旋转后画面的原始宽高比，避免竖屏画面被压扁。
                max_width = int(cfg['server'].get('preview_width', 640))
                if frame.shape[1] > max_width:
                    scale = max_width / frame.shape[1]
                    new_width = max_width
                    new_height = int(round(frame.shape[0] * scale))
                    frame = cv2.resize(
                        frame,
                        (new_width, new_height),
                        interpolation=cv2.INTER_AREA,
                    )

                ok,encoded=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),cfg['server']['jpeg_quality']])
                if ok:
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+encoded.tobytes()+b'\r\n'
                time.sleep(1/max(1,cfg['server']['preview_fps']))
        return Response(frames(),mimetype='multipart/x-mixed-replace; boundary=frame')

    app.stop_all=lambda reason='user':stop_to_safe()
    return app
