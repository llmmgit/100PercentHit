import subprocess
import threading


def run(command):
    return subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def connection_active(name):
    result = subprocess.run(["nmcli", "-t", "-f", "NAME", "con", "show", "--active"], capture_output=True, text=True)
    return name in result.stdout.splitlines()


def activate_wifi(cfg):
    run(["nmcli", "con", "down", cfg["ap_connection_name"]])
    return run(["nmcli", "con", "up", cfg["wifi_connection_name"]])


def activate_ap(cfg):
    run(["nmcli", "con", "down", cfg["wifi_connection_name"]])
    return run(["nmcli", "con", "up", cfg["ap_connection_name"]])


def select_network(cfg, force_ap=False):
    if not cfg.get("auto_manage", True):
        return "external"
    if force_ap:
        return "ap" if activate_ap(cfg) else "error"
    if activate_wifi(cfg):
        return "wifi"
    return "ap" if activate_ap(cfg) else "error"


def switch_network(cfg, target, callback):
    """异步切换网络，使HTTP响应有机会先返回。失败时尝试恢复原连接。"""
    def worker():
        previous = "ap" if connection_active(cfg["ap_connection_name"]) else "wifi"
        ok = activate_ap(cfg) if target == "ap" else activate_wifi(cfg)
        if not ok:
            activate_ap(cfg) if previous == "ap" else activate_wifi(cfg)
        callback(target if ok else previous, ok)
    threading.Thread(target=worker, daemon=True).start()
