import numpy as np
import time
import os
import sys
from thorlabs_tsi_sdk.tl_camera import TLCameraSDK, OPERATION_MODE
import matplotlib.pyplot as plt
# Configure DLL path for Thorlabs camera SDK
def configure_path():
    is_64bits = sys.maxsize > 2**32
    relative_path_to_dlls = '.' + os.sep + 'dlls' + os.sep

    if is_64bits:
        relative_path_to_dlls += 'Native_64_lib'
    else:
        relative_path_to_dlls += '32_lib'

    absolute_path_to_file_directory = os.path.dirname(os.path.abspath(__file__))
    absolute_path_to_dlls = os.path.abspath(absolute_path_to_file_directory + os.sep + relative_path_to_dlls)

    os.environ['PATH'] = absolute_path_to_dlls + os.pathsep + os.environ['PATH']
    try:
        os.add_dll_directory(absolute_path_to_dlls)
    except AttributeError:
        pass

# Camera Controller Class for Thorlabs Cameras
class TLCameraController:
    def __init__(self, camera_num=0):
        self.sdk = TLCameraSDK()  # Initialize the SDK directly
        self.available_cameras = self.sdk.discover_available_cameras()
        if len(self.available_cameras) == 0:
            raise RuntimeError("No cameras detected!")

        # Open the selected camera by serial number or index
        self.camera = self.sdk.open_camera(self.available_cameras[camera_num])
        self.is_color_camera = self.camera.camera_sensor_type.name == "BAYER"
        self.bit_depth = self.camera.bit_depth
        self.max_pixel_value = 2 ** self.bit_depth - 1
        # self.camera.frames_per_trigger_zero_for_unlimited = 0  # start camera in continuous mode
        self.camera.image_poll_timeout_ms = 1000  # 1 second polling timeout
        self.is_initialized = False
        print("Camera initialized.")

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
            self.camera.arm(2)  # Prepare for frame acquisition
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

        # Issue software trigger
        self.camera.issue_software_trigger()

        # Retrieve the frame
        frame = self.camera.get_pending_frame_or_null()
        if frame is None:
            raise RuntimeError("No image returned.")

        # Process the image
        img = np.copy(frame.image_buffer).reshape(self.camera.image_height_pixels, self.camera.image_width_pixels)
        if self.is_color_camera:
            rgb_image = np.stack([img]*3, axis=-1)  # Convert to RGB
            result = rgb_image / self.max_pixel_value
        else:
            result = img

        del frame
        return result

    def release(self):
        if self.camera:
            if self.camera.is_armed:
                self.camera.disarm()
            self.camera.dispose()  # Dispose of the camera resources
        self.sdk.dispose()  # Dispose of the SDK resources
        print("Camera and SDK resources released.")

if __name__ == '__main__':
    configure_path()
    cam = TLCameraController(camera_num=0)
    cam.setup_camera(exposure_us=1000, gain=0)  # Set your exposure and gain
    cam.initialize_acquisition()

    try:
        img = cam.acquire_frame()
        plt.imsave("test.png", img,cmap='gray')
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cam.stop_acquisition()
        cam.release()

    print("Program completed")
