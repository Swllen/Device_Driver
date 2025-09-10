import cv2
import threading
import time
from typing import Optional, Tuple, Union

class ELPCam:
    def __init__(
        self,
        device: Union[int, str] = 0,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
        fourcc: Optional[str] = "MJPG",  
        convert_rgb: bool = False,       # 默认不转 BGR，保留原始数据
    ):
        self.device = device
        self.cap = None
        self.width, self.height, self.fps = width, height, fps
        self.fourcc = fourcc
        self.convert_rgb = convert_rgb

        self._th: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._opened = False

        self.open()

    # 生命周期
    def open(self):
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_DSHOW)
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头: {self.device}")

        if self.fourcc:
            self.set_fourcc(self.fourcc)
        if self.width and self.height:
            self.set_resolution(self.width, self.height)
        if self.fps:
            self.set_fps(self.fps)

        self.set_convert_rgb(self.convert_rgb)
        self._opened = True

    def start_acquisition(self):
        if not self._opened:
            raise RuntimeError("摄像头未打开")
        if self._th and self._th.is_alive():
            return
        self._stop.clear()
        self._th = threading.Thread(target=self._grab_loop, daemon=True)
        self._th.start()

    def _grab_loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._latest = frame
        _ = self.cap.read()

    def stop_acquisition(self):
        self._stop.set()
        if self._th:
            self._th.join(timeout=1.0)
        self._th = None

    def acquire_frame(self, timeout: float = 1.0):
        if not self._opened:
            raise RuntimeError("摄像头未打开")
        if self._th and self._th.is_alive():
            t0 = time.time()
            while True:
                with self._lock:
                    frm = self._latest
                if frm is not None:
                    # 直接转单通道灰度
                    gray = cv2.cvtColor(frm, cv2.COLOR_BGR2GRAY)
                    return gray
                if (time.time() - t0) > timeout:
                    return None
                time.sleep(0.005)
        else:
            ok, frame = self.cap.read()
            if not ok:
                return None
            # 直接转单通道灰度
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    def release(self):
        self.stop_acquisition()
        if self.cap:
            self.cap.release()
        self._opened = False

    def decode_frame(self, frame, fourcc: Optional[str] = None, gray: bool = True):
        if frame is None:
            return None
        if fourcc is None:
            fourcc = self.get_fourcc()

        if fourcc == "MJPG":
            img = frame
        elif fourcc == "YUY2":
            img = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
        else:
            img = frame

        if gray:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    # 常用设置
    def set_resolution(self, w: int, h: int) -> bool:
        ok1 = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(w))
        ok2 = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(h))
        return ok1 and ok2

    def set_fps(self, fps: int) -> bool:
        return self.cap.set(cv2.CAP_PROP_FPS, float(fps))

    def set_fourcc(self, fourcc: str) -> bool:
        code = cv2.VideoWriter_fourcc(*fourcc)
        return self.cap.set(cv2.CAP_PROP_FOURCC, float(code))

    def set_convert_rgb(self, on: bool) -> bool:
        return self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1.0 if on else 0.0)

    def get_fourcc(self) -> str:
        code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        return "".join([chr((code >> (8 * i)) & 0xFF) for i in range(4)])

    def actual_format(self) -> Tuple[int, int, float, str]:
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        return w, h, fps, self.get_fourcc()

    # === 画质 / 曝光 / 增益等控制 ===
    def set_brightness(self, v: float) -> bool:   return self.cap.set(cv2.CAP_PROP_BRIGHTNESS, float(v))
    def set_contrast(self, v: float) -> bool:     return self.cap.set(cv2.CAP_PROP_CONTRAST, float(v))
    def set_saturation(self, v: float) -> bool:   return self.cap.set(cv2.CAP_PROP_SATURATION, float(v))
    def set_hue(self, v: float) -> bool:          return self.cap.set(cv2.CAP_PROP_HUE, float(v))
    def set_sharpness(self, v: float) -> bool:    return self.cap.set(cv2.CAP_PROP_SHARPNESS, float(v))
    def set_gamma(self, v: float) -> bool:        return self.cap.set(cv2.CAP_PROP_GAMMA, float(v))
    def set_gain(self, v: float) -> bool:         return self.cap.set(cv2.CAP_PROP_GAIN, float(v))

    def set_auto_exposure(self, on: bool) -> bool:
        """
        Windows 常见：
        - DirectShow: 1=手动, 3=自动
        - 有的驱动: 0.25/0.75
        """
        targets = [3.0 if on else 1.0]

        if self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, float(targets[0])):
            return True
        return False

    def set_exposure(self, v: float) -> bool:
        """
        Windows DirectShow 下曝光常用负值：-6 ~ -3。
        使用前请先关闭自动曝光。
        """
        return self.cap.set(cv2.CAP_PROP_EXPOSURE, float(v))


if __name__ == "__main__":
    cam = ELPCam(device=1, width=1920, height=1080, fps=30, fourcc="MJPG", convert_rgb=False)
    print("实际格式:", cam.actual_format())

    # 设置参数
    cam.set_auto_exposure(False)
    cam.set_exposure(-13)    # 曝光：常见范围 -13 ~ -1，具体依驱动
    cam.set_gain(0)
    cam.set_brightness(-4)
    cam.set_contrast(64)
    cam.set_saturation(0)
    cam.set_sharpness(3)
    cam.set_gamma(72)

    cam.start_acquisition()
    windowname = "ELP"
    cv2.namedWindow(windowname, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowname,960,540)
    while True:
        gray = cam.acquire_frame()
        # cv2.imwrite("1.png",gray)
        # gray = cam.decode_frame(raw, gray=True)
        print(gray.shape)
        if gray is None:
            continue
        cv2.imshow(windowname, gray)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

    cam.release()
    cv2.destroyAllWindows()
