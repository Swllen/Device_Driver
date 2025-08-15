import numpy as np
import time
import cv2
import os
import sys
from thorlabs_tsi_sdk.tl_camera import TLCameraSDK, OPERATION_MODE
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
        print(f"Camera {serial} initialized.")

    def setup_camera(self, exposure_us=500, gain=0, roi=None, binning=None):
        if self.camera.is_armed:
            self.camera.disarm()
        self.camera.exposure_time_us = exposure_us
        if gain >= 0:
            self.camera.gain = gain
        if roi:
            self.camera.roi_and_binning.set_roi(
                roi['OriginX'], roi['OriginY'], roi['Width'], roi['Height'])
        if binning:
            self.camera.roi_and_binning.set_binning(binning['BinX'], binning['BinY'])
        self.camera.operation_mode = OPERATION_MODE.SOFTWARE_TRIGGERED
        self.camera.frames_per_trigger_zero_for_unlimited = 1
        self.is_initialized = True
        print(f"Camera setup complete. Exposure: {exposure_us} us, Gain: {gain}, ROI: {roi}, Bin: {binning}")

    def initialize_acquisition(self):
        if not self.is_initialized:
            raise RuntimeError("Camera not initialized. Call setup_camera first.")
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

if __name__ == '__main__':
    configure_path()
    cam1 = TLCameraController(camera_num=0)
    cam2 = TLCameraController(camera_num=1)

    cam1.setup_camera(exposure_us=1000, gain=0)
    cam2.setup_camera(exposure_us=1500, gain=0)

    cam1.initialize_acquisition()
    cam2.initialize_acquisition()

    try:
        img1 = cam1.acquire_frame()
        img2 = cam2.acquire_frame()
        plt.imsave("cam1.png", img1, cmap='gray')
        plt.imsave("cam2.png", img2, cmap='gray')
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cam1.stop_acquisition()
        cam2.stop_acquisition()
        cam1.release()
        cam2.release()
        close_sdk()
    print("Program completed")
