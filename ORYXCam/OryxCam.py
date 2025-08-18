# -*- coding: utf-8 -*-
import PySpin
import numpy as np

class SpinnakerCamera:
    def __init__(self, serial: str | None = None, pixel_format=PySpin.PixelFormat_Mono8):
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
        """Connect to the first camera (or the one with the specified serial) and initialize it"""
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
                sn = PySpin.CStringPtr(cam.GetTLDeviceNodeMap().GetNode('DeviceSerialNumber')).GetValue()
                if sn == self._serial:
                    target = cam
                    break
            if target is None:
                raise RuntimeError(f"Camera with serial {self._serial} not found")

            self._cam = target
            self._cam.Init()  # must init before accessing nodemap
            self._nodemap = self._cam.GetNodeMap()
            self._nodemap_tldevice = self._cam.GetTLDeviceNodeMap()

            # Optional: set pixel format (if writable)
            pf_node = PySpin.CEnumerationPtr(self._nodemap.GetNode('PixelFormat'))
            if PySpin.IsWritable(pf_node):
                pf_node.SetIntValue(pf_node.GetEntryByName(
                    self._pixel_format.name.split('_', 1)[-1]  # e.g. "Mono8"
                ).GetValue())
        finally:
            cam_list.Clear()

    def close(self):
        """Stop acquisition and release resources"""
        if self._cam is not None:
            try:
                # Stop acquisition if streaming
                if self._cam.IsStreaming():  # not all versions have this, safe to ignore errors
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
        """Start acquisition in Continuous mode"""
        if self._cam is None:
            raise RuntimeError("Camera not opened")
        acq_mode = PySpin.CEnumerationPtr(self._nodemap.GetNode('AcquisitionMode'))
        if not (PySpin.IsReadable(acq_mode) and PySpin.IsWritable(acq_mode)):
            raise RuntimeError("AcquisitionMode not accessible")
        acq_mode.SetIntValue(acq_mode.GetEntryByName('Continuous').GetValue())

        self._cam.BeginAcquisition()  # start acquisition engine
        # Must pair with EndAcquisition(), and each acquired image must be Released

    def stop_acquisition(self):
        """Stop acquisition"""
        if self._cam is None:
            return
        self._cam.EndAcquisition()

    def grab_numpy(self, timeout_ms: int = 1000) -> np.ndarray:
        """Grab a frame and return it as a numpy array (Mono8 by default)"""
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        img = self._cam.GetNextImage(timeout_ms)  # blocking until frame or timeout
        try:
            if img.IsIncomplete():
                raise RuntimeError(f"Incomplete image, status={int(img.GetImageStatus())}")

            # Convert to Mono8 if needed
            if img.GetPixelFormat() != PySpin.PixelFormat_Mono8:
                img = self._processor.Convert(img, PySpin.PixelFormat_Mono8)

            arr = img.GetNDArray()  # shape HxW or HxWxC depending on format
            return np.array(arr, copy=True)  # copy so array is valid after Release
        finally:
            try:
                img.Release()
            except Exception:
                pass

    # ---------- Exposure ----------
    def set_exposure(self, auto: bool = False, time_us: float | None = None):
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
                time_us = float(time_us)
                if time_us < exp_time.GetMin(): time_us = exp_time.GetMin()
                if time_us > exp_time.GetMax(): time_us = exp_time.GetMax()
                exp_time.SetValue(time_us)

    # ---------- Gamma ----------
    def set_gamma(self, enable: bool = True, gamma: float | None = None):
        """Enable/disable Gamma and optionally set its value (~0.25–4 depending on camera)"""
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        g_en = PySpin.CBooleanPtr(self._nodemap.GetNode('GammaEnable'))
        if PySpin.IsWritable(g_en):
            g_en.SetValue(bool(enable))

        if enable and gamma is not None:
            g = PySpin.CFloatPtr(self._nodemap.GetNode('Gamma'))
            if PySpin.IsWritable(g):
                val = float(gamma)
                if val < g.GetMin(): val = g.GetMin()
                if val > g.GetMax(): val = g.GetMax()
                g.SetValue(val)

    # ---------- ROI ----------
    def set_roi(self, x: int, y: int, w: int, h: int, center: bool = False):
        """
        Set ROI in pixels. If center=True, the ROI is centered and x,y are ignored.
        Note: ROI steps and valid ranges depend on camera, alignment is done to min/max/inc.
        """
        if self._cam is None:
            raise RuntimeError("Camera not opened")

        width  = PySpin.CIntegerPtr(self._nodemap.GetNode('Width'))
        height = PySpin.CIntegerPtr(self._nodemap.GetNode('Height'))
        offx   = PySpin.CIntegerPtr(self._nodemap.GetNode('OffsetX'))
        offy   = PySpin.CIntegerPtr(self._nodemap.GetNode('OffsetY'))

        def _align(node, val):
            vmin, vmax = node.GetMin(), node.GetMax()
            inc = node.GetInc() if hasattr(node, 'GetInc') else 1
            val = max(vmin, min(vmax, val))
            return vmin + ((val - vmin) // max(1, inc)) * max(1, inc)

        if center:
            cur_w = int(width.GetValue())
            cur_h = int(height.GetValue())
            w = _align(width,  w)
            h = _align(height, h)
            x = _align(offx,   max(0, (cur_w - w) // 2))
            y = _align(offy,   max(0, (cur_h - h) // 2))
        else:
            w = _align(width,  w)
            h = _align(height, h)
            x = _align(offx,   x)
            y = _align(offy,   y)

        try:
            if PySpin.IsWritable(offx):   offx.SetValue(x)
            if PySpin.IsWritable(offy):   offy.SetValue(y)
            if PySpin.IsWritable(width):  width.SetValue(w)
            if PySpin.IsWritable(height): height.SetValue(h)
        except PySpin.SpinnakerException:
            # Some cameras require different order (try again reversed)
            if PySpin.IsWritable(width):  width.SetValue(w)
            if PySpin.IsWritable(height): height.SetValue(h)
            if PySpin.IsWritable(offx):   offx.SetValue(x)
            if PySpin.IsWritable(offy):   offy.SetValue(y)

    def set_latest_only(self, buffer_count:int = 3):
        """
        Keep only the newest frame in the host buffer.
        Call this BEFORE start_acquisition().
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
            # keep it small to reduce latency (but >1 for safety)
            bc = max(2, min(int(buffer_count), int(count.GetMax())))
            count.SetValue(bc)

        # StreamBufferHandlingMode = NewestOnly
        handling = PySpin.CEnumerationPtr(tl.GetNode('StreamBufferHandlingMode'))
        if PySpin.IsWritable(handling):
            entry = handling.GetEntryByName('NewestOnly')
            if PySpin.IsReadable(entry):
                handling.SetIntValue(entry.GetValue())


if __name__ == "__main__":
    cam = SpinnakerCamera(pixel_format=PySpin.PixelFormat_Mono8)
    cam.open()
    cam.set_exposure(auto=False, time_us=200)
    cam.set_roi(100, 200, 640, 480)

    cam.start_acquisition()                         
    frame = cam.grab_numpy(timeout_ms=1000)         
    cam.stop_acquisition()                         

    cam.close()
