# In test.py
import sys
sys.path.append(r"E:\BU_Research\Nodeology_project\DeviceDriver\Device_Driver")

from SLM.CreatePattern import *
from BPC303.BPC3xx import *
from PAMC104 import *
from TLCam.TLCam import *

# Example usage
phase = create_vortex_phase_map_16bit(1200, 1920, 1920//2, 1200/2, l=1, max_value=1023, ROI=None) # this function is from CreatePattern
print(phase.shape)  # Should print (1200, 1920)
configure_path()
cam1 = TLCameraController(camera_num=0)
cam1.setup_camera(exposure_us=500, gain=0)
cam1.initialize_acquisition()