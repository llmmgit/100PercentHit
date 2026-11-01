def clamp(value, low, high):
    return max(low, min(high, float(value)))


def build_motion_cmd(vx_mps=0.0, vy_mps=0.0, wz_radps=0.0):
    """生成M100 Mode 6速度帧。比例是协议换算，不代表精确物理速度。"""
    vx = clamp(vx_mps, -2, 2)
    vy = clamp(vy_mps, -2, 2)
    wz = clamp(wz_radps, -10, 10)

    def axis(value, scale):
        direction = 0 if value >= 0 else 1
        raw = max(0, min(65535, int(round(abs(value) * scale))))
        return direction, (raw >> 8) & 0xFF, raw & 0xFF

    xd, xh, xl = axis(vx, 1000)
    yd, yh, yl = axis(vy, 1000)
    wd, wh, wl = axis(wz, 100)
    frame = bytearray([0xCD, 0x0A, 0x01, xd, xh, xl, yd, yh, yl, wd, wh, wl, 0])
    checksum = 0
    for byte in frame[2:12]:
        checksum ^= byte
    frame[12] = checksum
    return bytes(frame)


STOP_FRAME = build_motion_cmd()
