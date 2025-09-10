import sys
sys.path.append(r"E:\BU_Research\AutoOAM\DeviceDriver\Device_Driver")

from feedback import PIDAxis,apply_direction_step,drive_outputs,print_log,get_original_position,draw_overlay
from preprocess import process_and_judge_direction
from PAMC104.PAMC104 import PAMC104Controller 
import time
# from TLCam.TLCam import *
from ELPcam.ELPcam import ELPCam
import cv2

# ===== Control & hardware parameters =====

"""
---PID Format---

PID_XX_PARAM = [26000, 100, 2.0, 26000, 100, 2.0] 

Sequece:

KP_NR_1 = 26000.0                 Proportional gain (normalized -> pulse)
KI_NR_1 = 100                     Integral gain (normalized-sum -> pulse)
KD_NR_1 = 2.0                     Derivative gain (normalized rate -> pulse)
KP_RR_1 = 26000.0                 
KI_RR_1 = 100                    
KD_RR_1 = 2.0
"""
DEADBAND_NORM = 0.01
PID_AB_PARAM = [26000, 100, 2.0, 26000, 100, 2.0]
PID_CD_PARAM = [26000, 100, 2.0, 26000, 100, 2.0]


INTEGRAL_CLAMP = 5000.0       # Clamp for integral accumulator (anti-windup)
DERIVATIVE_LPF_ALPHA = 0.2  # 0..1; lower = stronger smoothing (for D term)
MIN_PULSE = 10              # Do not send zero; any motion gets at least this
MAX_PULSE = 9999            # HW limit
STEP_PULSE = 5000             # Step used when center is missing for an axis
SPEED = 1500                # Motor speed

YAW_CH1 = "A"
PITCH_CH1 = "B"

YAW_CH2 = "C"
PITCH_CH2 = "D"

"""
CamParam = [
            "device_num", 
            "width", 
            "height", 
            "fps",
            "exposure", 
            "gain", 
            "brightness",
            "contrast", 
            "saturation", 
            "sharpness", 
            "gamma"

            ])
"""
CAMERA_1_PARAM = [1, 1920, 1080, 30, -13, 0, -4, 64, 0, 3, 72]
CAMERA_2_PARAM = [1, 1920, 1080, 30, -13, 0, -4, 64, 0, 3, 72]
# ===== Control & hardware parameters =====


# This is for the pamc feedback based on PID control
class feedback_system:
    def __init__(self,cam_param: list = None, pamc: PAMC104Controller = None,
                 yaw_ch: str = "", pitch_ch: str = "", pid_params: list = None):
        self.original_x = 0
        self.original_y = 0
        self.frame_width = 0
        self.frame_height = 0

        self.cam_param = cam_param
        self.pamc = pamc
        self.pid_yaw = None
        self.pid_pitch = None
        self.yaw_ch = yaw_ch
        self.pitch_ch = pitch_ch
        self.pid_params = pid_params

    def initialize(self):
        self._initialize_ELP_cam()
        time.sleep(1)
        self._initialize_pid()
        time.sleep(1)
        self._get_original_position_and_frame_size()

    def _get_original_position_and_frame_size(self):
        frame = self.cam.acquire_frame()
        self.frame_height,self.frame_width = frame.shape[:2]
        xc0, yc0 = get_original_position(frame)
        if xc0 is None or yc0 is None:
            self.original_x, self.original_y = 0.5,0.5
            # self.original_x = 0.5
            # self.original_y = 0.5
            print(f"[WARNING] Original position isn't acquired: X={xc0}, Y={yc0}")
        else:
            self.original_x, self.original_y = xc0/self.frame_width, yc0/self.frame_height
            # self.original_x = 0.5
            # self.original_y = 0.5
            print(f"[INIT] Original position acquired: X={xc0}, Y={yc0}")
        

    def _initialize_pid(self):
        kp_nr, ki_nr, kd_nr, kp_rr, ki_rr, kd_rr = self.pid_params

        self.pid_yaw = PIDAxis(kp_nr, ki_nr, kd_nr, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)
        self.pid_pitch = PIDAxis(kp_rr, ki_rr, kd_rr, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)

        print("[INIT] PID initialized")

    
    def _initialize_ELP_cam(self):
        device_num,width,height,fps,exposure,gain,brightness,contrast,saturation,sharpness,gamma = self.cam_param
        self.cam = ELPCam(device=device_num, width=width, height=height, fps=fps, fourcc="MJPG", convert_rgb=False)
        self.cam.set_auto_exposure(False)
        self.cam.set_exposure(exposure)    # -13 ~ -1
        self.cam.set_gain(gain)
        self.cam.set_brightness(brightness)
        self.cam.set_contrast(contrast)
        self.cam.set_saturation(saturation)
        self.cam.set_sharpness(sharpness)
        self.cam.set_gamma(gamma)
        print(f"[INIT] Camera{device_num} is initialized.")


    def reset_pid(self):
        self.pid_yaw.reset()
        self.pid_pitch.reset()


    def feedback_loop(self,display):
        self.reset_pid()
        t_prev = time.time()
        self.cam.start_acquisition()
        try:
            while True:
                frame = self.cam.acquire_frame()
                pre_dx, pre_dy, xc, yc = process_and_judge_direction(frame)

                
                t_now = time.time()
                dt = max(1e-3, t_now - t_prev)
                t_prev = t_now
                err_x = None
                err_y = None
                yaw_cmd = 0
                yaw_dir = ""
                pitch_cmd = 0
                pitch_dir = ""
                status_msg = ""

                # Branch by availability of centers
                if (xc is not None) and (yc is not None):
                    err_x = (xc / float(self.frame_width)) - self.original_x
                    err_y = (yc / float(self.frame_height)) - self.original_y
                    if abs(err_x) < DEADBAND_NORM and abs(err_y) < DEADBAND_NORM:
                        print("[INFO] \033[92mFeedBack Sucessful\033[0m")
                        self.reset_pid()
                        break

                    yaw_cmd, yaw_dir   = self.pid_yaw.update(err_x, dt, pos_dir_labels=("left","right"))
                    pitch_cmd, pitch_dir = self.pid_pitch.update(err_y, dt, pos_dir_labels=("up","down"))
                    
                    drive_outputs(self.pamc, yaw_cmd, yaw_dir, pitch_cmd, pitch_dir,channel_yaw=self.yaw_ch,channel_pitch=self.pitch_ch)
                    status_msg = "X and Y are in PID control"

                elif (xc is None) and (yc is not None):
                    # Y only: PID on pitch; yaw steps by inferred direction
                    err_y = (yc / float(self.frame_height)) - self.original_y
                    pitch_cmd, pitch_dir = self.pid_pitch.update(err_y, dt, pos_dir_labels=("up","down"))
                    drive_outputs(self.pamc, 0, "center", pitch_cmd, pitch_dir,channel_yaw=self.yaw_ch,channel_pitch=self.pitch_ch)
                    apply_direction_step(self.pamc, pre_dx, "center",channel_yaw=self.yaw_ch,channel_pitch=self.pitch_ch)
                    self.pid_yaw.i_term *= 0.95
                    status_msg = f"X lost -> yaw step {pre_dx or 'center'}"

                elif (xc is not None) and (yc is None):
                    # X only: PID on yaw; pitch steps by inferred direction
                    err_x = (xc / float(self.frame_width)) -  float(self.original_x)
                    yaw_cmd, yaw_dir = self.pid_yaw.update(err_x, dt, pos_dir_labels=("left","right"))
                    drive_outputs(self.pamc, yaw_cmd, yaw_dir, 0, "center",channel_yaw=self.yaw_ch,channel_pitch=self.pitch_ch)
                    apply_direction_step(self.pamc, "center", pre_dy,channel_yaw=self.yaw_ch,channel_pitch=self.pitch_ch)
                    self.pid_pitch.i_term *= 0.95
                    status_msg = f"Y lost -> pitch step {pre_dy or 'center'}"

                else:
                    self.reset_pid()
                    # Neither center found: step both axes by inferred directions
                    err_x = None
                    err_y = None
                    apply_direction_step(self.pamc, pre_dx, pre_dy,channel_yaw=self.yaw_ch,channel_pitch=self.pitch_ch)
                    self.pid_yaw.i_term *= 0.9
                    self.pid_pitch.i_term *= 0.9
                    status_msg = f"Both lost -> step X:{pre_dx or 'center'} Y:{pre_dy or 'center'}"

                print_log(dt,err_x,err_y,yaw_dir,yaw_cmd,pitch_dir,pitch_cmd,status_msg,xc,yc)
                
                # Dispaly
                if display:
                    disp = draw_overlay(frame, 720, 540, xc, yc,
                                    err_x=0.0 if err_x is None else err_x,
                                    err_y=0.0 if err_y is None else err_y,
                                    yaw_cmd=yaw_cmd, yaw_dir=yaw_dir,
                                    pitch_cmd=pitch_cmd, pitch_dir=pitch_dir,
                                    msg=status_msg)
                
                    norm = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
                    disp = norm.astype('uint8')
                    cv2.imshow("WINDOW_NAME", disp)
                    # Exit: key q or esc
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q'), ord('Q')):
                        print("[INFO] User ask for exit")
                        break

            time.sleep(0.01)
            self.cam.stop_acquisition()
        except Exception as e:
            ts = time.strftime("%H:%M:%S")
            print(f"[Error]:{e}")
        
    
    def close(self):
        # Clean all device
        try: self.cam.stop_acquisition()
        except Exception: pass
        try: self.cam.release()
        except Exception: pass
        try: print("Stop:", self.pamc.stop())
        except Exception: pass
        # try:
            # close_sdk()
        # except Exception: pass


def Initialize_FeedBack_System(pamc:PAMC104Controller = None, num:int = 1) -> feedback_system:
    if num == 1:
        sys1 = feedback_system(cam_param=CAMERA_1_PARAM,pamc=pamc,yaw_ch=YAW_CH1,pitch_ch=PITCH_CH1,pid_params=PID_AB_PARAM)
        sys1.initialize()

        return sys1
    elif num == 2:
        sys1 = feedback_system(cam_param=CAMERA_1_PARAM,pamc=pamc,yaw_ch=YAW_CH1,pitch_ch=PITCH_CH1,pid_params=PID_AB_PARAM)
        sys1.initialize()
        time.sleep(2)
        sys2 = feedback_system(cam_param=CAMERA_2_PARAM,pamc=pamc,yaw_ch=YAW_CH2,pitch_ch=PITCH_CH2,pid_params=PID_CD_PARAM)
        sys2.initialize()

        return sys1, sys2
    
    

    
if __name__ == "__main__":

    # camera = TLCameraController()
    # camera.setup_camera(exposure_us=15000, gain=0)
    # camera.frames_per_trigger(1)
    # camera.initialize_acquisition()
    try:
        
        pamc = PAMC104Controller(port="COM7")
        print("Connection check:", pamc.connect_check())
        sys1 = Initialize_FeedBack_System(pamc,1)
        time.sleep(1)
        pamc.drive("NR",1500,5000,"A")
        time.sleep(5000/1500 + 0.05)
        pamc.drive("NR",1500,5000,"B")
        time.sleep(5000/1500 + 0.05)
        time.sleep(5)
        sys1.feedback_loop(1)


        # print("Connection check:", pamc.connect_check())
        
        # pamc.drive("NR",1500,5000,"C")
        # time.sleep(5000/1500 + 0.05)
        # pamc.drive("NR",1500,5000,"D")
        # time.sleep(5000/1500 + 0.05)
        # sys2.feedback_loop()
        # sys1.close()
    finally:
        try: sys1.close()
        except NameError: pass
        except Exception: pass
        # try: sys2.close()
        except NameError: pass
        except Exception: pass
        try: pamc.close()
        except Exception: pass
