import ctypes
from pathlib import Path

import numpy as np


class YoloBallResult(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_int),
        ("confidence", ctypes.c_float),
        ("cx", ctypes.c_float),
        ("cy", ctypes.c_float),
        ("width", ctypes.c_float),
        ("height", ctypes.c_float),
        ("pre_ms", ctypes.c_float),
        ("npu_ms", ctypes.c_float),
        ("post_ms", ctypes.c_float),
    ]


class YoloBallDetector:
    def __init__(self, library_path, model_path, vision_cfg=None):
        self.vision_cfg = vision_cfg if vision_cfg is not None else {}
        self.library_path = str(Path(library_path).resolve())
        self.model_path = str(Path(model_path).resolve())
        self._lib = ctypes.CDLL(self.library_path)

        self._lib.yolo_ball_create.argtypes = [ctypes.c_char_p]
        self._lib.yolo_ball_create.restype = ctypes.c_void_p

        self._lib.yolo_ball_detect.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(YoloBallResult),
        ]
        self._lib.yolo_ball_detect.restype = ctypes.c_int

        self._lib.yolo_ball_destroy.argtypes = [ctypes.c_void_p]
        self._lib.yolo_ball_destroy.restype = None

        self._handle = self._lib.yolo_ball_create(
            self.model_path.encode("utf-8")
        )
        if not self._handle:
            raise RuntimeError("Failed to initialize YOLO model")

    def detect(self, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("Expected BGR image with shape HxWx3")

        image = np.ascontiguousarray(frame, dtype=np.uint8)
        height, width = image.shape[:2]
        result = YoloBallResult()

        rc = self._lib.yolo_ball_detect(
            self._handle,
            image.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
            width,
            height,
            int(image.strides[0]),
            ctypes.byref(result),
        )
        if rc != 0:
            raise RuntimeError(f"YOLO detection failed: {rc}")

        ball = None
        box_area = float(result.width * result.height)
        min_area = max(0.0, float(self.vision_cfg.get("min_area", 0.0)))
        max_area = max(min_area, float(self.vision_cfg.get("max_area", float("inf"))))
        if (result.count > 0 and result.cx >= 0 and result.cy >= 0
                and min_area <= box_area <= max_area):
            x = result.cx - result.width * 0.5
            y = result.cy - result.height * 0.5
            ball = {
                "x": float(x), "y": float(y),
                "w": float(result.width), "h": float(result.height),
                "cx": float(result.cx), "cy": float(result.cy),
                "area": box_area, "confidence": float(result.confidence),
                "count": int(result.count),
            }

        timing = {
            "pre_ms": float(result.pre_ms),
            "npu_ms": float(result.npu_ms),
            "post_ms": float(result.post_ms),
        }
        return ball, timing

    def close(self):
        if self._handle:
            self._lib.yolo_ball_destroy(self._handle)
            self._handle = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
