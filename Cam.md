# TLCameraController - Python Interface for Thorlabs TSI Cameras

## Overview
`TLCameraController` is a Python wrapper for the Thorlabs Scientific Imaging (TSI) Camera SDK.  
It allows you to control a Thorlabs camera, configure parameters such as exposure, gain, ROI, and binning, and acquire frames for further processing or saving.

The implementation uses the official Thorlabs `thorlabs_tsi_sdk` Python package and the TSI Camera SDK DLLs.

---

## Requirements

- Python 3.x
- Thorlabs TSI Camera SDK installed (DLLs must be accessible to the script)
- `thorlabs_tsi_sdk` Python package
- `numpy`, `matplotlib`

Install required packages:
```bash
pip install numpy matplotlib
```

Ensure the Thorlabs DLL files are located in a `dlls/Native_64_lib` or `dlls/32_lib` subdirectory relative to the script, or adjust the `configure_path()` function accordingly.

---


## Build Instructions
> **Attention**: **My code load the DLL files directly**
1. If you have not done so already unzip the following folder to an accesible location on your drive. This contains the Camera SDK. 

   * 32-Bit - C:\Program Files (x86)\Thorlabs\Scientific Imaging\Scientific Camera Support\Scientific Camera Interfaces.zip
   * 64-Bit - C:\Program Files\Thorlabs\Scientific Imaging\Scientific Camera Support\Scientific Camera Interfaces.zip

2. The Python SDK is provided both as an installable package and as source files. To install the Python SDK in your environment, use a package manager such as pip to install from the package file. The zip folder will be found within the location in step 1: \Scientific Camera Interfaces\SDK\Python Toolkit

Example install command: 

```
python.exe -m pip install thorlabs_tsi_camera_python_sdk_package.zip
```

 This will install the thorlabs_tsi_sdk package into your current environment. The examples assume you are using this method. 
 If you want to use the source files directly, they are included in SDK\Python Camera Toolkit\source.


1. To use the examples on a Windows OS, copy the Native DLLs from 
     * 64-Bit - \Scientific Camera Interfaces\SDK\Native Toolkit\dlls\Native_64_lib
     * 32-Bit - \Scientific Camera Interfaces\SDK\Native Toolkit\dlls\Native_32_lib

   To a folder in your script location named __\dlls\64_lib__ for 64-bit machines and __\dlls\32_lib__ for 32-bit. This can be modified in the Configure_path() if you wish to change the location. 

## Usage

### 1. Configure DLL Path
```python
configure_path()
```
Adds the appropriate TSI SDK DLL path to the environment so the SDK can be loaded.

### 2. Initialize Camera
```python
cam = TLCameraController(camera_num=0)
```
- `camera_num`: Index of the detected camera (0 for first camera).

### 3. Setup Camera
```python
cam.setup_camera(exposure_us=1000, gain=0, roi=None, binning=None)
```
Parameters:
- `exposure_us`: Exposure time in microseconds.
- `gain`: Camera gain setting.
- `roi`: Optional dict with keys `OriginX`, `OriginY`, `Width`, `Height`.
- `binning`: Optional dict with keys `BinX`, `BinY`.

### 4. Initialize Acquisition
```python
cam.initialize_acquisition()
```
Prepares the camera for frame acquisition by arming it.

### 5. Acquire Frame
```python
frame = cam.acquire_frame()
```
Triggers a software acquisition and returns the captured image as a NumPy array.  
If the camera is a color sensor, the result is returned as an RGB array; otherwise, as a grayscale array.

### 6. Stop Acquisition
```python
cam.stop_acquisition()
```
Disarms the camera and stops acquisition.

### 7. Release Resources
```python
cam.release()
```
Disarms and disposes the camera, and disposes the SDK resources.

---

## Example
```python
if __name__ == '__main__':
    configure_path()
    cam = TLCameraController(camera_num=0)
    cam.setup_camera(exposure_us=1000, gain=0)
    cam.initialize_acquisition()

    try:
        img = cam.acquire_frame()
        plt.imsave("test.png", img, cmap='gray')
    finally:
        cam.stop_acquisition()
        cam.release()
```

---

## Methods Summary

### `configure_path()`
Sets the environment path to include the correct Thorlabs SDK DLL directory.

### `__init__(camera_num=0)`
Initializes the SDK, discovers cameras, and opens the specified camera.

### `setup_camera(exposure_us=500, gain=0, roi=None, binning=None)`
Configures exposure, gain, ROI, binning, and acquisition mode.

### `initialize_acquisition()`
Arms the camera for frame acquisition.

### `stop_acquisition()`
Stops acquisition and disarms the camera.

### `acquire_frame()`
Captures a single frame via software trigger and returns it as a NumPy array.

### `release()`
Disposes the camera and SDK resources.

