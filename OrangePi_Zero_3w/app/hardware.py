import subprocess
import threading
import time

try:
    import spidev
except ImportError:
    spidev = None


class GPIOInputs(threading.Thread):
    """读取ARM和接球输入；ARM短按切换状态、长按关机，接球点短按测试、长按切换网络。"""

    daemon = True

    def __init__(self, cfg, on_arm_short, on_shutdown_long, on_catch_short, on_network_long):
        super().__init__()
        self.cfg = cfg
        self.on_arm_short = on_arm_short
        self.on_shutdown_long = on_shutdown_long
        self.on_catch_short = on_catch_short
        self.on_network_long = on_network_long
        self.stop_evt = threading.Event()
        self.runtime_enabled = threading.Event()
        self.arm_was_active = False
        self.arm_pressed_at = None
        self.arm_long_fired = False
        self.catch_was_active = False
        self.catch_pressed_at = None
        self.catch_long_fired = False

    def read(self, name):
        gpio_map = {
            "PE4": ("gpiochip0", "132"),
            "PL4": ("gpiochip1", "4"),
        }
        try:
            chip, line = gpio_map[name]
            output = subprocess.check_output(
                ["gpioget", "-c", chip, line],
                text=True,
                timeout=0.25,
            ).strip()
            return 1 if output.rsplit("=", 1)[-1] == "active" else 0
        except Exception:
            return 1 if self.cfg.get("active_low", True) else 0

    def active(self, name):
        value = self.read(name)
        return value == 0 if self.cfg.get("active_low", True) else value == 1

    def run(self):
        if not self.cfg.get("enabled", True):
            return

        self.runtime_enabled.wait()
        poll_s = max(0.02, float(self.cfg.get("debounce_ms", 60)) / 1000.0)

        # 启动后必须先检测到ARM键和接球键均已释放，才启用按键功能。
        while not self.stop_evt.is_set():
            arm_released = not self.active(self.cfg["arm_line_name"])
            catch_released = not self.active(self.cfg["catch_line_name"])
            if arm_released and catch_released:
                break
            time.sleep(0.1)

        shutdown_hold = float(self.cfg.get("shutdown_hold_s", 3.0))
        network_hold = float(self.cfg.get("network_switch_hold_s", 3.0))

        while not self.stop_evt.is_set():
            now = time.monotonic()

            arm_active = self.active(self.cfg["arm_line_name"])
            if arm_active and not self.arm_was_active:
                self.arm_pressed_at = now
                self.arm_long_fired = False
            elif arm_active and self.arm_pressed_at is not None and not self.arm_long_fired:
                if now - self.arm_pressed_at >= shutdown_hold:
                    self.arm_long_fired = True
                    self.on_shutdown_long()
            elif not arm_active and self.arm_was_active:
                if not self.arm_long_fired:
                    self.on_arm_short()
                self.arm_pressed_at = None
                self.arm_long_fired = False
            self.arm_was_active = arm_active

            catch_active = self.active(self.cfg["catch_line_name"])
            if catch_active and not self.catch_was_active:
                self.catch_pressed_at = now
                self.catch_long_fired = False
            elif catch_active and self.catch_pressed_at is not None and not self.catch_long_fired:
                if now - self.catch_pressed_at >= network_hold:
                    self.catch_long_fired = True
                    self.on_network_long()
            elif not catch_active and self.catch_was_active:
                if not self.catch_long_fired:
                    self.on_catch_short()
                self.catch_pressed_at = None
                self.catch_long_fired = False
            self.catch_was_active = catch_active

            time.sleep(poll_s)


class WS2812SPI:
    """SPI编码驱动WS2812，显式设置SPI mode、8位字长和约2.4 MHz时钟。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.spi = None
        self.lock = threading.Lock()
        self.last_pixels = None

    def open(self):
        if self.spi is not None:
            return
        if spidev is None:
            raise RuntimeError("未安装spidev Python模块")
        self.spi = spidev.SpiDev()
        self.spi.open(int(self.cfg.get("spi_bus", 3)), int(self.cfg.get("spi_cs", 0)))
        self.spi.mode = int(self.cfg.get("spi_mode", 0))
        self.spi.bits_per_word = int(self.cfg.get("bits_per_word", 8))
        self.spi.max_speed_hz = int(self.cfg.get("spi_speed_hz", 2400000))
        self.spi.no_cs = bool(self.cfg.get("no_cs", False))

    @staticmethod
    def encode_byte(value):
        bits = []
        for bit in range(7, -1, -1):
            bits.extend((1, 1, 0) if value & (1 << bit) else (1, 0, 0))
        output = bytearray()
        accumulator = 0
        count = 0
        for bit in bits:
            accumulator = (accumulator << 1) | bit
            count += 1
            if count == 8:
                output.append(accumulator)
                accumulator = 0
                count = 0
        if count:
            output.append(accumulator << (8 - count))
        return bytes(output)

    def show_pixels(self, pixels, force=False):
        if not self.cfg.get("enabled", False):
            return
        count = int(self.cfg.get("count", 11))
        pixels = list(pixels[:count]) + [(0, 0, 0)] * max(0, count - len(pixels))
        normalized = tuple(
            tuple(max(0, min(255, int(v))) for v in pixel)
            for pixel in pixels
        )
        if not force and normalized == self.last_pixels:
            return

        brightness = max(0, min(255, int(self.cfg.get("brightness", 40)))) / 255.0
        payload = bytearray()
        for red, green, blue in normalized:
            red = int(red * brightness)
            green = int(green * brightness)
            blue = int(blue * brightness)
            payload += self.encode_byte(green)
            payload += self.encode_byte(red)
            payload += self.encode_byte(blue)
        payload += b"\x00" * int(self.cfg.get("reset_bytes", 32))

        try:
            with self.lock:
                self.open()
                self.spi.xfer2(list(payload))
                self.last_pixels = normalized
        except Exception:
            self.close()

    def off(self):
        count = int(self.cfg.get("count", 11))
        self.show_pixels([(0, 0, 0)] * count, force=True)

    def close(self):
        if self.spi is not None:
            try:
                self.spi.close()
            except Exception:
                pass
            self.spi = None


class StatusLEDService(threading.Thread):
    """第1颗为车头灯，第2/3颗为左右状态灯，后8颗为尾部流水灯。"""

    daemon = True

    def __init__(self, cfg, state, driver):
        super().__init__()
        self.cfg = cfg
        self.state = state
        self.driver = driver
        self.stop_evt = threading.Event()
        self.frame = 0

    @staticmethod
    def color(value):
        return tuple(int(v) for v in value)

    def waterfall_tail(self, color):
        count = int(self.cfg.get("tail_count", 8))
        tail = [(0, 0, 0)] * count

        left = count // 2 - 1
        right = count // 2
        steps = []
        while left >= 0 and right < count:
            steps.append((left, right))
            left -= 1
            right += 1

        # 保留两帧熄灭间隔，然后重新从中间向两侧展开。
        steps.extend([(), ()])
        step = self.frame % len(steps)
        for index in steps[step]:
            tail[index] = color
        return tail

    def pixels_for(self, mode, network_phase):
        c = self.cfg
        count = int(c.get("count", 11))
        front_count = int(c.get("front_count", 3))

        if mode == "BOOT":
            blue = self.color(c["boot_color"])
            front = [blue if (self.frame // 5) % 2 == 0 else (0, 0, 0)] * front_count
            return front + self.waterfall_tail(blue)

        if network_phase == "NETWORK_SWITCHING":
            blue = self.color(c["wifi_color"])
            purple = self.color(c["ap_color"])
            color = blue if (self.frame // 2) % 2 == 0 else purple
            return [color] * count

        if network_phase == "NETWORK_ERROR":
            red = self.color(c["fault_color"])
            return [red if self.frame % 2 == 0 else (0, 0, 0)] * count

        if network_phase == "HOTSPOT_SELECTED":
            purple = self.color(c["ap_color"])
            return [purple if self.frame % 2 == 0 else (0, 0, 0)] * count

        if network_phase == "AP_READY":
            return [self.color(c["ap_color"])] * count

        if network_phase == "WIFI_READY":
            return [self.color(c["wifi_color"])] * count

        mapping = {
            "SAFE": "safe_color",
            "MANUAL": "manual_color",
            "ARMED": "armed_color",
            "TRACKING": "tracking_color",
            "AUTO_MOVE": "auto_color",
            "BLOCKED": "blocked_color",
            "SUCCESS": "success_color",
            "FAULT": "fault_color",
            "SHUTTING_DOWN": "shutdown_color",
        }
        color = self.color(c[mapping.get(mode, "fault_color")])

        if mode in {"SAFE", "MANUAL", "ARMED"}:
            # 第1颗车头灯存在已知硬件异常，不作为可靠状态指示。
            # 第2、3颗左右状态灯闪烁；尾部8颗流水灯保持原动画，不随状态灯整体熄灭。
            on = (self.frame // 2) % 2 == 0
            front = [(0, 0, 0)]
            front.extend([color if on else (0, 0, 0)] * max(0, front_count - 1))
            return front + self.waterfall_tail(color)

        if mode in {
            "TRACKING",
            "AUTO_MOVE",
            "BLOCKED",
            "SUCCESS",
            "FAULT",
            "SHUTTING_DOWN",
        }:
            on = (self.frame // (5 if mode == "SHUTTING_DOWN" else 2)) % 2 == 0
            return [color if on else (0, 0, 0)] * count

        return [(0, 0, 0)] * count

    def stop_sync(self, timeout=1.0):
        self.stop_evt.set()
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout)
        self.driver.off()
        self.driver.close()

    def run(self):
        if not self.cfg.get("enabled", False):
            return
        hz = max(5, int(self.cfg.get("refresh_hz", 20)))
        while not self.stop_evt.is_set():
            with self.state.lock:
                mode = self.state.mode
                network_phase = self.state.network_phase
            self.driver.show_pixels(self.pixels_for(mode, network_phase))
            self.frame += 1
            time.sleep(1.0 / hz)
        self.driver.off()
        self.driver.close()
