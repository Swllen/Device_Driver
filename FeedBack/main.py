import cv2
import numpy as np
import time
import sys
sys.path.append(r"E:\BU_Research\Nodeology_project\DeviceDriver\Device_Driver")
from PAMC104.PAMC104 import PAMC104Controller 
from TLCam.TLCam import *
from preprocess import *

DEADBAND_NORM = 0.05    
GAIN = 10.0               
MIN_PULSE = 50            
STEP_PULSE = 50         
SPEED = 1500             
YAW_CH = "A"
PITCH_CH = "B"

def get_original_position(frame):
    _, _, xc, yc = process_and_judge_direction(frame)
    return xc, yc

def update_position(cx, cy, width, height, originalx, originaly,
                    deadband_norm=DEADBAND_NORM, gain=GAIN):

    ox = originalx / float(width)
    oy = originaly / float(height)
    mx = cx / float(width)
    my = cy / float(height)

    err_x = mx - ox
    err_y = my - oy


    if abs(err_x) < deadband_norm:
        err_x = 0.0
    if abs(err_y) < deadband_norm:
        err_y = 0.0

    directionx = 'left' if err_x > 0 else ('right' if err_x < 0 else 'center')
    directiony = 'up'   if err_y > 0 else ('down'  if err_y < 0 else 'center')

    yaw_pulse = int(max(0, round(abs(err_x) * gain)))
    pitch_pulse = int(max(0, round(abs(err_y) * gain)))


    if 0 < yaw_pulse < MIN_PULSE: yaw_pulse = MIN_PULSE
    if 0 < pitch_pulse < MIN_PULSE: pitch_pulse = MIN_PULSE

    return yaw_pulse, pitch_pulse, directionx, directiony

def apply_direction_step(pamc, dirx, diry, step=STEP_PULSE, speed=SPEED,
                         channel_yaw=YAW_CH, channel_pitch=PITCH_CH):

    if dirx == "left":
        pamc.drive("NR", speed, step, channel_yaw)
    elif dirx == "right":
        pamc.drive("RR", speed, step, channel_yaw)

    if diry == "down":
        pamc.set_pitch("NR", speed, step, channel_pitch)
    elif diry == "up":
        pamc.set_pitch("RR", speed, step, channel_pitch)

def update_pamc(pamc: PAMC104Controller, yaw_pulse, pitch_pulse, directionx, directiony,
                channel_yaw=YAW_CH, channel_pitch=PITCH_CH, speed=SPEED):

    if yaw_pulse > 0:
        if directionx == "left":
            pamc.drive("NR", speed, yaw_pulse, channel_yaw)
        elif directionx == "right":
            pamc.drive("RR", speed, yaw_pulse, channel_yaw)

    if pitch_pulse > 0:
        if directiony == "down":
            pamc.set_pitch("NR", speed, pitch_pulse, channel_pitch)
        elif directiony == "up":
            pamc.set_pitch("RR", speed, pitch_pulse, channel_pitch)

def feedback_loop():
    camera = TLCameraController()
    camera.setup_camera(exposure_us=1000, gain=0)
    camera.frames_per_trigger(1)
    camera.initialize_acquisition()

    pamc = PAMC104Controller(port="COM7")
    print("Connection check:", pamc.connect_check())

    # first = camera.acquire_frame()
    # xc0, yc0 = get_original_position(first)
    # h0, w0 = first.shape[:2]

    # if xc0 is None or yc0 is None:
    #     originalx, originaly = w0 / 2.0, h0 / 2.0
    # else:
    #     originalx, originaly = xc0, yc0
    originalx = 0, originaly = 0
    try:
        while True:
            frame = camera.acquire_frame()
            pre_dx, pre_dy, xc, yc = process_and_judge_direction(frame)
            h, w = frame.shape[:2]

            if (xc is not None) and (yc is not None):
                yp, pp, dirx, diry = update_position(xc, yc, w, h, originalx, originaly)
                update_pamc(pamc, yp, pp, dirx, diry)

            elif (xc is None) and (yc is not None):
                _, pp, _, diry = update_position(originalx, yc, w, h, originalx, originaly)
                update_pamc(pamc, 0, pp, "center", diry)
                apply_direction_step(pamc, pre_dx, "center")

            elif (xc is not None) and (yc is None):
                yp, _, dirx, _ = update_position(xc, originaly, w, h, originalx, originaly)
                update_pamc(pamc, yp, 0, dirx, "center")
                apply_direction_step(pamc, "center", pre_dy)

            else:
                apply_direction_step(pamc, pre_dx, pre_dy)

            time.sleep(0.02)

    finally:
        camera.stop_acquisition()
        camera.release()
        close_sdk()
        print("Stop:", pamc.stop())
        pamc.close()
