import argparse
import signal
import subprocess
import threading
import time
from .core import Lidar, Link, State, Vision
from .configuration import load_config
from .hardware import GPIOInputs, StatusLEDService, WS2812SPI
from .network import select_network, switch_network
from .protocol import STOP_FRAME
from .web import create


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/config.yaml")
    parser.add_argument(
        "--arm-trace",
        action="store_true",
        help="仅在SSH前台运行时打印ARM/TRACKING控制诊断，不写日志文件。",
    )
    args = parser.parse_args()
    cfg = load_config(args.config)
    # 仅为本次进程生效，不保存到local.yaml。
    cfg["_arm_trace"] = bool(args.arm_trace)
    state = State()
    holder = {}
    shutdown_lock = threading.Lock()

    def stop_motion():
        with state.lock:
            state.clear_motion()
        if holder.get('link'):
            holder['link'].reset_automatic()
        if holder.get("link"):
            holder["link"].send_stop_repeated(cfg["system"]["shutdown_stop_repeat"], cfg["system"]["shutdown_stop_interval_s"])

    def arm_action():
        with state.lock:
            mode = state.mode
            if mode in ("BOOT", "SHUTTING_DOWN"):
                return False, "当前状态不能ARM"
            if mode == "FAULT":
                if state.error:
                    return False, "故障尚未清除"
                state.mode = "SAFE"
                return True, "已返回SAFE"
            if mode in ("ARMED", "TRACKING", "AUTO_MOVE"):
                state.clear_motion()
                state.mode = "SAFE"
                if holder.get("link"):
                    holder["link"].send(STOP_FRAME)
                return True, "已取消ARM并返回SAFE"
            if mode in ("SUCCESS", "BLOCKED"):
                state.catch = False
                state.lidar_blocked = False
                state.clear_motion()
                state.mode = "SAFE"
                if holder.get("link"):
                    holder["link"].send(STOP_FRAME)
                return True, "已清除本轮状态并返回SAFE"
            fresh_imu = state.imu_ok and (time.monotonic() - state.last_yaw_t) * 1000 < cfg["imu"]["timeout_ms"]
            if cfg["imu"].get("required_for_arm", True) and not fresh_imu:
                return False, "IMU数据未就绪，不能ARM"
            state.clear_motion()
            state.yaw_ref = state.yaw
            state.catch = False
            state.lidar_blocked = False
            state.mode = "ARMED"
            if holder.get("link"):
                holder["link"].reset_automatic()
            if holder.get("vision"):
                holder["vision"].reset_tracking()
        if holder.get("link"):
            holder["link"].send(STOP_FRAME)
        return True, "已进入ARMED，并将当前车头方向设为本轮0度"

    def caught():
        with state.lock:
            if state.mode in cfg["catch"]["valid_modes"]:
                state.clear_motion()
                state.catch = True
                state.mode = "SUCCESS"
        if holder.get("link"):
            holder["link"].send(STOP_FRAME)

    def blocked():
        with state.lock:
            if state.mode in cfg["lidar"]["active_modes"]:
                state.clear_motion()
                state.lidar_blocked = True
                state.mode = "BLOCKED"
            else:
                return
        if holder.get("link"):
            holder["link"].send_stop_repeated(3)

    def safe_shutdown(source="button"):
        if not shutdown_lock.acquire(blocking=False):
            return
        with state.lock:
            state.clear_motion()
            state.mode = "SHUTTING_DOWN"
        stop_motion()
        def worker():
            time.sleep(float(cfg["system"]["shutdown_led_s"]))
            led_service.stop_sync(1.0)
            subprocess.Popen(["systemctl", "poweroff"])
        threading.Thread(target=worker, daemon=True).start()

    def request_network_switch(target=None):
        with state.lock:
            if state.mode not in ("SAFE", "MANUAL"):
                return False, "仅SAFE或MANUAL状态允许切换网络"
            current = state.network_mode
            selected = target or ("ap" if current == "wifi" else "wifi")
            if selected not in ("wifi", "ap"):
                return False, "无效网络模式"
            state.clear_motion()
            state.mode = "SAFE"
            state.network_phase = "NETWORK_SWITCHING"
        stop_motion()

        def done(mode, ok):
            with state.lock:
                state.network_mode = mode
                if ok:
                    state.network_phase = "AP_READY" if mode == "ap" else "WIFI_READY"
                    state.error = ""
                    delay = float(cfg["network"].get("ready_confirm_s", 1.0))
                else:
                    state.network_phase = "NETWORK_ERROR"
                    state.error = "网络切换失败，已尝试恢复原连接"
                    delay = float(cfg["network"].get("error_confirm_s", 2.0))

            def clear_feedback():
                time.sleep(delay)
                with state.lock:
                    state.network_phase = None

            threading.Thread(target=clear_feedback, daemon=True).start()

        switch_network(cfg["network"], selected, done)
        return True, "网络切换已开始，当前页面可能断开"

    gpio = GPIOInputs(
        cfg["io"],
        arm_action,
        lambda: safe_shutdown("button"),
        caught,
        lambda: request_network_switch(),
    )
    led_driver = WS2812SPI(cfg["ws2812"])
    led_service = StatusLEDService(cfg["ws2812"], state, led_driver)
    led_service.start()

    # 启动时不再等待ARM按钮，立即优先连接家庭Wi-Fi，失败时回退到3W热点。
    state.network_mode = select_network(cfg["network"], force_ap=False)
    with state.lock:
        state.network_phase = "AP_READY" if state.network_mode == "ap" else "WIFI_READY" if state.network_mode == "wifi" else "ERROR"
    time.sleep(float(cfg["network"].get("ready_confirm_s", 1.0)))
    with state.lock:
        state.network_phase = None  #网络确认灯效结束，回复按运行状态显示。
        state.mode = "SAFE" if state.network_mode != "error" else "FAULT"
        if state.network_mode == "error":
            state.error = "家庭Wi-Fi和热点均启动失败"

    vision = Vision(cfg, state)
    holder["vision"] = vision
    link = Link(cfg, state)
    holder["link"] = link
    lidar = Lidar(cfg, state, blocked)
    vision.start(); link.start(); lidar.start()
    gpio.runtime_enabled.set(); gpio.start()
    app = create(cfg, state, vision, link, args.config, arm_action, safe_shutdown, request_network_switch)

    def cleanup():
        try: app.stop_all("shutdown")
        except Exception: pass
        gpio.stop_evt.set();vision.stop_evt.set();link.stop_evt.set();lidar.stop_evt.set();led_service.stop_sync(1.0)
    def quit_app(*_args):
        cleanup();raise SystemExit
    signal.signal(signal.SIGTERM, quit_app);signal.signal(signal.SIGINT, quit_app)
    try: app.run(host=cfg["server"]["host"], port=cfg["server"]["port"], threaded=True, use_reloader=False)
    finally: cleanup()


if __name__ == "__main__":
    main()
