# -*- coding: utf-8 -*-
from typing import Optional
import PySpin
import numpy as np


class SpinnakerCamera:
    def __init__(self, serial: Optional[str] = None, pixel_format=PySpin.PixelFormat_Mono8):
        """
        :param serial: Optional camera serial number. If None, pick the first camera found.
        :param pixel_format: Either an int (e.g. PySpin.PixelFormat_Mono8) or a str (e.g. "Mono8").
        """
        self._system = None
        self._cam = None
        self._nodemap = None
        self._nodemap_tldevice = None
        self._serial = serial
        self._pixel_format = pixel_format
        self._processor = PySpin.ImageProcessor()
        self._processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

    # ---------- Lifecycle ----------
    def open(self):
        """Connect to the first camera (or the one with the specified serial) and initialize it."""
        if self._cam is not None:
            return

        self._system = PySpin.System.GetInstance()
        cam_list = self._system.GetCameras()
        try:
            if cam_list.GetSize() == 0:
                raise RuntimeError("No camera detected")

            # Find camera by serial number, otherwise pick the first one
            target = None
            for i in range(cam_list.GetSize()):
                cam = cam_list.GetByIndex(i)
                if self._serial is None:
                    target = cam
                    break
                sn_node = PySpin.CStringPtr(cam.GetTLDeviceNodeMap().GetNode('DeviceSerialNumber'))
                sn = sn_node.GetValue() if PySpin.IsReadable(sn_node) else ""
                if sn == self._serial:
                    target = cam
                    break

            if target is None:
                raise RuntimeError(f"Camera with serial {self._serial} not found")

            self._cam = target
            self._cam.Init()  # must init before accessing nodemap
            self._nodemap = self._cam.GetNodeMap()
            self._nodemap_tldevice = self._cam.GetTLDeviceNodeMap()

            # Optional: set pixel format before acquisition (robust to int or str)
            self._safe_set_pixel_format(self._pixel_format)

        finally:
            cam_list.Clear()

    def _safe_set_pixel_format(self, pixfmt):
        pf_node = PySpin.CEnumerationPtr(self._nodemap.GetNode('PixelFormat'))
        if not (PySpin.IsReadable(pf_node) and PySpin.IsWritable(pf_node)):
            raise RuntimeError("PixelFormat node not accessible")

        if isinstance(pixfmt, str):
            entry = PySpin.CEnumEntryPtr(pf_node.GetEntryByName(pixfmt))
            if not PySpin.IsReadable(entry):
                raise RuntimeError(f"PixelFormat {pixfmt} not readable")
            pf_node.SetIntValue(entry.GetValue())
        else:
            pf_node.SetIntValue(int(pixfmt))

        print("Pixel format set to", pf_node.GetCurrentEntry().GetSymbolic())


    def close(self):
        """Stop acquisition (if needed) and release resources."""
        if self._cam is not None:
            # Stop acquisition if streaming (not all SDK versions have IsStreaming)
            try:
                if hasattr(self._cam, "IsStreaming") and self._cam.IsStreaming():
                    self.stop_acquisition()
            except Exception:
                pass
            try:
                self._cam.DeInit()
            finally:
                self._cam = None

        if self._system is not None:
            self._system.ReleaseInstance()
            self._system = None

    # ---------- Acquisition ----------
    def start_acquisition(self):
        """Start acquisition in Continuous mode."""
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        acq_mode = PySpin.CEnumerationPtr(self._nodemap.GetNode('AcquisitionMode'))
        if not (PySpin.IsReadable(acq_mode) and PySpin.IsWritable(acq_mode)):
            raise RuntimeError("AcquisitionMode not accessible")
        acq_mode.SetIntValue(acq_mode.GetEntryByName('Continuous').GetValue())

        self._cam.BeginAcquisition()  # start acquisition engine

    def stop_acquisition(self):
        """Stop acquisition."""
        if self._cam is None:
            return
        try:
            self._cam.EndAcquisition()
        except Exception:
            # If acquisition wasn't started, ignore
            pass

    def grab_numpy(self, timeout_ms: int = 1000):
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        img = self._cam.GetNextImage(timeout_ms)
        try:
            if img.IsIncomplete():
                raise RuntimeError(f"Incomplete image, status={int(img.GetImageStatus())}")

            # 解包 Mono12p → Mono16；其他格式按需再处理
            pf_name = str(img.GetPixelFormatName())  # 查询当前像素格式
            if "Mono12p" in pf_name or "Mono12Packed" in pf_name:
                img = self._processor.Convert(img, PySpin.PixelFormat_Mono12p)  # 解包到12位
            elif img.GetPixelFormat() != PySpin.PixelFormat_Mono8:
                # 若只是想预览，转成Mono8也可以；但会丢位深
                img = self._processor.Convert(img, PySpin.PixelFormat_Mono8)

            arr = img.GetNDArray()  # Mono12p时得到 dtype=uint16 的数组
            return np.array(arr, copy=True)
        finally:
            try:
                img.Release()
            except Exception:
                pass


    # ---------- Exposure ----------
    def set_exposure(self, auto: bool = False, time_us: Optional[float] = None):
        """
        Configure exposure:
        - auto=True enables auto exposure
        - auto=False and time_us provided → set manual timed exposure (microseconds)
        """
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        exp_auto = PySpin.CEnumerationPtr(self._nodemap.GetNode('ExposureAuto'))
        if PySpin.IsWritable(exp_auto):
            exp_auto.SetIntValue(exp_auto.GetEntryByName('Continuous' if auto else 'Off').GetValue())

        if not auto and time_us is not None:
            mode = PySpin.CEnumerationPtr(self._nodemap.GetNode('ExposureMode'))
            if PySpin.IsWritable(mode):
                mode.SetIntValue(mode.GetEntryByName('Timed').GetValue())
            exp_time = PySpin.CFloatPtr(self._nodemap.GetNode('ExposureTime'))
            if PySpin.IsWritable(exp_time):
                t = float(time_us)
                if t < exp_time.GetMin():
                    t = exp_time.GetMin()
                if t > exp_time.GetMax():
                    t = exp_time.GetMax()
                exp_time.SetValue(t)

    # ---------- Gamma ----------
    def set_gamma(self, enable: bool = True, gamma: Optional[float] = None):
        """Enable/disable Gamma and optionally set its value (~0.25–4 depending on camera)."""
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        g_en = PySpin.CBooleanPtr(self._nodemap.GetNode('GammaEnable'))
        if PySpin.IsWritable(g_en):
            g_en.SetValue(bool(enable))

        if enable and gamma is not None:
            g = PySpin.CFloatPtr(self._nodemap.GetNode('Gamma'))
            if PySpin.IsWritable(g):
                val = float(gamma)
                if val < g.GetMin():
                    val = g.GetMin()
                if val > g.GetMax():
                    val = g.GetMax()
                g.SetValue(val)

    # ---------- Gain ----------
    def set_gain(self, auto: bool = False, gain_db: Optional[float] = None):
        """
        Configure analog gain:
        - auto=True  -> GainAuto=Continuous（or On）
        - auto=False -> GainAuto=Off，gain_db（dB）
        """
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        node_gain_auto = PySpin.CEnumerationPtr(self._nodemap.GetNode('GainAuto'))
        if PySpin.IsWritable(node_gain_auto):
            entry = node_gain_auto.GetEntryByName('Continuous' if auto else 'Off')
            if PySpin.IsReadable(entry):
                node_gain_auto.SetIntValue(entry.GetValue())

        if auto or gain_db is None:
            return 

        try:
            gain_selector = PySpin.CEnumerationPtr(self._nodemap.GetNode('GainSelector'))
            if PySpin.IsWritable(gain_selector):
                for name in ('Analog', 'All'):
                    e = gain_selector.GetEntryByName(name)
                    if PySpin.IsReadable(e):
                        gain_selector.SetIntValue(e.GetValue())
                        break
        except Exception:
            pass

        node_gain = PySpin.CFloatPtr(self._nodemap.GetNode('Gain'))
        if not PySpin.IsWritable(node_gain):
            raise RuntimeError("Gain node not writable")

        val = float(gain_db)
        try:
            gmin, gmax = node_gain.GetMin(), node_gain.GetMax()
            if val < gmin: val = gmin
            if val > gmax: val = gmax
        except Exception:
            pass
        node_gain.SetValue(val)

    # ---------- ROI ----------
    def set_roi(self, x: int, y: int, w: int, h: int, center: bool = False):
        """
        Set ROI in pixels. If center=True, the ROI is centered and x,y are ignored.
        Note: ROI steps and valid ranges depend on camera, alignment is done to min/max/inc.
        """
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        width = PySpin.CIntegerPtr(self._nodemap.GetNode('Width'))
        height = PySpin.CIntegerPtr(self._nodemap.GetNode('Height'))
        offx = PySpin.CIntegerPtr(self._nodemap.GetNode('OffsetX'))
        offy = PySpin.CIntegerPtr(self._nodemap.GetNode('OffsetY'))

        def _align(node, val):
            vmin, vmax = node.GetMin(), node.GetMax()
            inc = node.GetInc() if hasattr(node, 'GetInc') else 1
            val = max(vmin, min(vmax, val))
            return vmin + ((val - vmin) // max(1, inc)) * max(1, inc)

        if center:
            cur_w = int(width.GetValue())
            cur_h = int(height.GetValue())
            w = _align(width, w)
            h = _align(height, h)
            x = _align(offx, max(0, (cur_w - w) // 2))
            y = _align(offy, max(0, (cur_h - h) // 2))
        else:
            w = _align(width, w)
            h = _align(height, h)
            x = _align(offx, x)
            y = _align(offy, y)

        # Some models require a specific order; try one order then the reverse if it fails.
        try:
            if PySpin.IsWritable(offx):
                offx.SetValue(x)
            if PySpin.IsWritable(offy):
                offy.SetValue(y)
            if PySpin.IsWritable(width):
                width.SetValue(w)
            if PySpin.IsWritable(height):
                height.SetValue(h)
        except PySpin.SpinnakerException:
            if PySpin.IsWritable(width):
                width.SetValue(w)
            if PySpin.IsWritable(height):
                height.SetValue(h)
            if PySpin.IsWritable(offx):
                offx.SetValue(x)
            if PySpin.IsWritable(offy):
                offy.SetValue(y)

    # ---------- Buffer: keep newest frame only ----------
    def set_latest_only(self, buffer_count: int = 3):
        """
        Keep only the newest frame in the host buffer (low latency).
        Call this BEFORE start_acquisition().
        :param buffer_count: total host buffers. Use a small number (>=2), e.g., 2~4.
        """
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        tl = self._cam.GetTLStreamNodeMap()

        # StreamBufferCountMode = Manual
        mode = PySpin.CEnumerationPtr(tl.GetNode('StreamBufferCountMode'))
        if PySpin.IsWritable(mode):
            entry = mode.GetEntryByName('Manual')
            if PySpin.IsReadable(entry):
                mode.SetIntValue(entry.GetValue())

        # StreamBufferCountManual = small number (e.g., 2~4)
        count = PySpin.CIntegerPtr(tl.GetNode('StreamBufferCountManual'))
        if PySpin.IsWritable(count):
            # keep it small to reduce latency (but >=2 for safety)
            try:
                bc = int(buffer_count)
            except Exception:
                bc = 3
            bc = max(2, bc)
            try:
                bc = min(bc, int(count.GetMax()))
            except Exception:
                pass
            count.SetValue(bc)

        # StreamBufferHandlingMode = NewestOnly
        handling = PySpin.CEnumerationPtr(tl.GetNode('StreamBufferHandlingMode'))
        if PySpin.IsWritable(handling):
            entry = handling.GetEntryByName('NewestOnly')
            if PySpin.IsReadable(entry):
                handling.SetIntValue(entry.GetValue())

def pyspin_image_to_numpy(img):
    """将 PySpin.Image 转成 numpy 数组（支持 Mono8 / Mono12p / Mono16）"""
    # 基本属性
    w = img.GetWidth()
    h = img.GetHeight()
    pf_name = str(img.GetPixelFormatName())  # 比如 'Mono12p'
    buf = img.GetData()
    size = img.GetBufferSize()

    # 直接从 buffer 建立 uint8 视图
    data_u8 = np.frombuffer(buf, dtype=np.uint8, count=size)

    if pf_name == 'Mono8':
        arr = data_u8.reshape(h, img.GetStride())[:, :w]  # 去掉可能的行填充
        return arr.copy()  # 返回紧凑数组

    if pf_name == 'Mono16':
        # 注意：相机流通常是 little-endian；若你的相机是 big-endian，需要 .byteswap()
        data_u16 = data_u8.view(np.uint16)
        arr = data_u16.reshape(h, img.GetStride() // 2)[:, :w]
        return arr.copy()

    if pf_name == 'Mono12p':
        # 12-bit packed：每3字节=2像素
        # bytes: [b0, b1, b2] -> p0 = b0 | ((b1 & 0x0F) << 8); p1 = (b2 << 4) | ((b1 & 0xF0) >> 4)
        # 先按行解包更安全（考虑 stride/padding）
        stride = img.GetStride()  # 每行字节数（包含 padding）
        row_bytes = w * 12 // 8   # 每行有效像素数据的字节数（packed后）
        assert row_bytes % 3 == 0, "Mono12p 每行有效字节应该是3的倍数"

        out = np.empty((h, w), dtype=np.uint16)

        offset = 0
        for r in range(h):
            row = data_u8[offset: offset + row_bytes]
            # 解包本行
            b0 = row[0::3].astype(np.uint16)
            b1 = row[1::3].astype(np.uint16)
            b2 = row[2::3].astype(np.uint16)

            p0 = b0 | ((b1 & 0x0F) << 8)
            p1 = (b2 << 4) | ((b1 & 0xF0) >> 4)

            # 拼回一行 w 个像素
            packed = np.empty(p0.size + p1.size, dtype=np.uint16)
            packed[0::2] = p0
            packed[1::2] = p1

            out[r, :] = packed[:w]

            offset += stride  # 跳到下一行（跨过 padding）

        return out

    # 其它格式：可以先用 ImageProcessor.Convert 转到 Mono8/Mono16 再转 numpy
    # from PySpin import ImageProcessor, PixelFormat_Mono8
    # proc = ImageProcessor()
    # img2 = proc.Convert(img, PySpin.PixelFormat_Mono16)
    # 然后对 img2 调用本函数的 Mono16 分支
    raise NotImplementedError(f'暂不支持像素格式: {pf_name}')

if __name__ == "__main__":
    cam = SpinnakerCamera(pixel_format=PySpin.PixelFormat_Mono12p)
    cam.open()

    # Optional: for feedback control to always use the latest frame
    cam.set_latest_only(buffer_count=3)
    cam.set_exposure(auto=False, time_us=200)  # microseconds
    cam.set_roi(100, 200, 640, 480)

    cam.start_acquisition()
    frame = cam.grab_numpy(timeout_ms=1000)
    cam.stop_acquisition()
    print("Frame shape:", frame.shape)


    cam.close()
