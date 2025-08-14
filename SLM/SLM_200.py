import ctypes
import numpy as np
import matplotlib.pyplot as plt
from ctypes import *
from scipy import misc
from time import sleep
import os
import pandas as pd
import matplotlib.pyplot as plt
import _slm_win as slm
from CreatePattern import *


class SantecSLM:
    def __init__(self):
        self.width = ctypes.c_ushort(0)
        self.height = ctypes.c_ushort(0)
        self.DisplayName =  ctypes.create_string_buffer(64)
        self.DispalyNumber = 0
        self.scan_displaynum()
    # Search LCOS-SLM
    def scan_displaynum(self):
        width = ctypes.c_ushort(0)
        height = ctypes.c_ushort(0)
        DisplayName =  ctypes.create_string_buffer(64)
        for DisplayNumber in range(1,8):        
            ret = slm.SLM_Disp_Info2(DisplayNumber, width, height, DisplayName)
            if(ret == slm.SLM_OK):
                Names = DisplayName.value.decode('mbcs').split(',')
                if('LCOS-SLM' in Names[0]): # 'LCOS-SLM,SOC,8001,2018021001'
                    print(DisplayNumber, Names, width, height)
                    self.DisplayNumber = DisplayNumber
                    break
    def open_display(self):
        ret = slm.SLM_Disp_Open(self.DisplayNumber) 
        if ret != slm.SLM_OK:
            print("Failed to open SLM display")

    def display(self,data):

        data_ptr = data.ctypes.data_as(ctypes.POINTER(ctypes.c_uint16))
        ret = slm.SLM_Disp_Data(self.DisplayNumber, 1920, 1200, slm.FLAGS_RATE120, data_ptr)

    def close_dispaly(self):
        slm.SLM_Disp_Close(self.DisplayNumber)

# Example
if __name__ == "__main__":
    file_path = "E:/BU_Research/Nodeology_project/MPLC_Wang_resources-20250630T191323Z-1-001/MPLC_Wang_resources/0811/3/780nm_OAM20_MAGx70_InS1p145_PSx365.mat"
    phase1,phase2=open_mat(file_path)

    phase1_shift = cirshift_xy(phase1,5,35)
    phase2_shift = cirshift_xy(phase2,104,22)

    pattern = merge_phase(phase1_shift,phase2_shift)

    '''
    DISPLAY
    '''
    SLM = SantecSLM()
    SLM.open_display()
    SLM.display(pattern)

    plt.figure(figsize=(15, 5))
    plt.subplot(1,3,1)
    plt.imshow(phase1_shift,cmap='gray')
    print(np.shape(phase1))
    plt.subplot(1,3,2)
    plt.imshow(phase2_shift,cmap='gray')
    plt.subplot(1,3,3)
    plt.imshow(pattern,cmap="gray")

    # SLM.close_dispaly()
