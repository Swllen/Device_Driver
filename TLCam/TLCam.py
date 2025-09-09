import numpy as np
import time
import os
import sys
import matplotlib.pyplot as plt

def configure_path():
    is_64bits = sys.maxsize > 2**32
    relative_path_to_dlls = '.' + os.sep + 'dlls' + os.sep
    relative_path_to_dlls += 'Native_64_lib' if is_64bits else '32_lib'
    absolute_path_to_file_directory = os.path.dirname(os.path.abspath(__file__))
    absolute_path_to_dlls = os.path.abspath(absolute_path_to_file_directory + os.sep + relative_path_to_dlls)
    os.environ['PATH'] = absolute_path_to_dlls + os.pathsep + os.environ['PATH']
    try:
        os.add_dll_directory(absolute_path_to_dlls)
    except AttributeError:
        pass

configure_path()

from thorlabs_tsi_sdk.tl_camera import TLCameraSDK, OPERATION_MODE, ROI

sdk = None

class TLCameraController:
    def __init__(self, camera_num=0):
        global sdk
        if sdk is None:
            sdk = TLCameraSDK()

        available_cameras = sdk.discover_available_cameras()
        if len(available_cameras) == 0:
            raise RuntimeError("No cameras detected!")

        if isinstance(camera_num, int):
            serial = available_cameras[camera_num]
        else:
            serial = camera_num

        self.camera = sdk.open_camera(serial)
        self.is_color_camera = self.camera.camera_sensor_type.name == "BAYER"
        self.bit_depth = self.camera.bit_depth
        self.max_pixel_value = 2 ** self.bit_depth - 1
        self.camera.image_poll_timeout_ms = 1000
        self.is_initialized = False
        self.is_continuous_trigger = False
        print(f"Camera {serial} initialized.")

    def setup_camera(self, exposure_us=500, gain=0, roi=None, binning=None):
        if self.camera.is_armed:
            self.camera.disarm()
        self.camera.exposure_time_us = exposure_us
        if gain >= 0:
            self.camera.gain = gain
        if roi:
            self.camera.roi = ROI(roi[0], roi[1], roi[2], roi[3])
        if binning:
            self.camera.roi_and_binning.set_binning(binning['BinX'], binning['BinY'])
        print(f"Camera setup complete. Exposure: {exposure_us} us, Gain: {gain}, ROI: {roi}, Bin: {binning}")

    def frames_per_trigger(self, frames_num=1):
        self.camera.operation_mode = OPERATION_MODE.SOFTWARE_TRIGGERED
        if frames_num > 0:
            self.camera.frames_per_trigger_zero_for_unlimited = frames_num
            self.camera.image_poll_timeout_ms = 2000
        elif frames_num == 0:
            self.camera.frames_per_trigger_zero_for_unlimited = 0
            self.camera.image_poll_timeout_ms = 2000
        self.is_initialized = True

    def initialize_acquisition(self):
        if not self.is_initialized:
            raise RuntimeError("Camera not initialized. Call frames_per_trigger first.")
        if not self.camera.is_armed:
            self.camera.arm(2)
            print("Camera armed.")

    def stop_acquisition(self):
        if self.camera.is_armed:
            self.camera.disarm()
            print("Camera disarmed.")

    def acquire_frame(self):
        if not self.is_initialized:
            raise RuntimeError("Camera not initialized.")
        if not self.camera.is_armed:
            self.camera.arm(2)
        self.camera.issue_software_trigger()
        frame = self.camera.get_pending_frame_or_null()
        if frame is None:
            raise RuntimeError("No image returned.")
        img = np.copy(frame.image_buffer).reshape(self.camera.image_height_pixels, self.camera.image_width_pixels)
        if self.is_color_camera:
            rgb_image = np.stack([img]*3, axis=-1)
            result = rgb_image / self.max_pixel_value
        else:
            result = img
        del frame
        return result

    def acquire_frame_continuous(self,frame_info):
        if not self.is_initialized:
            raise RuntimeError("Camera not initialized.")
        if not self.camera.is_armed:
            self.camera.arm(2)
        if not self.is_continuous_trigger:
            self.camera.issue_software_trigger()
            self.is_continuous_trigger = True

        frame = self.camera.get_pending_frame_or_null()
        if frame is None:
            raise RuntimeError("No image returned.")
        img = np.copy(frame.image_buffer).reshape(self.camera.image_height_pixels, self.camera.image_width_pixels)
        if self.is_color_camera:
            rgb_image = np.stack([img]*3, axis=-1)
            result = rgb_image / self.max_pixel_value
        else:
            result = img
        if frame_info:
            timestamp = frame.time_stamp_relative_ns_or_null
            frame_count = frame.frame_count
            del frame
            return result, timestamp, frame_count
        return result

    def release(self):
        if self.camera:
            if self.camera.is_armed:
                self.camera.disarm()
            self.camera.dispose()
        print("Camera resources released.")

def close_sdk():
    global sdk
    if sdk is not None:
        sdk.dispose()
        sdk = None
        print("SDK resources released.")
import cv2

if __name__ == '__main__':
    cam1 = TLCameraController(camera_num=0)
    cam1.setup_camera(exposure_us=500, gain=0)
    cam1.frames_per_trigger(1)
    cam1.initialize_acquisition()

    try:
        for i in range(10):
            time.sleep(0.1)
            img1 = cam1.acquire_frame()

            # 如果是 16bit 图像，先归一化到 8bit
            if img1.dtype != 'uint8':
                norm = cv2.normalize(img1, None, 0, 255, cv2.NORM_MINMAX)
                img_show = norm.astype('uint8')
            else:
                img_show = img1

            cv2.imshow("Camera1", img_show)

            # 按 q 或 ESC 退出
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cam1.stop_acquisition()
        cam1.release()
        close_sdk()
        cv2.destroyAllWindows()
    print("Program completed")
