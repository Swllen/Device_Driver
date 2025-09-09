from feedback import *


# This is for the pamc feedback based on PID control
class feedback_system:
    def __init__(self,cam1=None,cam2=None,pamc=None):
        self.original_x = 0
        self.original_y = 0
        self.cam1 = cam1
        self.cam2 = cam2
        self.pamc = pamc
        self.frame_width = 0
        self.frame_height = 0
        self.pid_yaw_1 = None
        self.pid_pitch_1 = None
        self.pid_yaw_2 = None
        self.pid_pitch_2 = None

        self._initialize_pid()
        self.get_original_position()

    def get_original_position(self):
        frame = self.cam1.acquire_frame()
        xc0, yc0 = get_original_position(frame)
        self.frame_height,self.frame_width = frame.shape[:2]
        if xc0 is None or yc0 is None:
            self.original_x, self.original_y = self.frame_width / 2.0, self.frame_height / 2.0
        else:
            self.original_x, self.original_y = xc0/self.frame_width, yc0/self.frame_height

    def _initialize_pid(self):
        self.pid_yaw_1 = PIDAxis(KP, KI, KD, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)
        self.pid_pitch_1 = PIDAxis(KP, KI, KD, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)
        self.pid_yaw_2 = PIDAxis(KP, KI, KD, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)
        self.pid_pitch_2 = PIDAxis(KP, KI, KD, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)

    def reset(self):
        self.pid_yaw_1.reset()
        self.pid_pitch_1.reset()
        self.pid_yaw_2.reset()
        self.pid_pitch_1.reset()


    def feedback_loop(self):
        self.reset()
        t_prev = time.time()
        try:
            while True:
                frame = self.cam1.acquire_frame()
                pre_dx, pre_dy, xc, yc = process_and_judge_direction(frame)

                
                t_now = time.time()
                dt = max(1e-3, t_now - t_prev)
                t_prev = t_now

                # Branch by availability of centers
                if (xc is not None) and (yc is not None):
                    err_x = (xc / float(self.frame_width)) - self.original_x
                    err_y = (yc / float(self.frame_height)) - self.original_y
                    if err_x < 0.001 and err_y < 0.001:
                        break

                    yaw_cmd, yaw_dir   = pid_yaw.update(err_x, dt, pos_dir_labels=("left","right"))
                    pitch_cmd, pitch_dir = pid_pitch.update(err_y, dt, pos_dir_labels=("up","down"))
                    
                    drive_outputs(self.pamc, yaw_cmd, yaw_dir, pitch_cmd, pitch_dir)

                elif (xc is None) and (yc is not None):
                    err_y = (yc / float(self.frame_width)) - self.original_y
                    pitch_cmd, pitch_dir = pid_pitch.update(err_y, dt, pos_dir_labels=("up","down"))
                    drive_outputs(self.pamc, 0, "center", pitch_cmd, pitch_dir)
                    apply_direction_step(self.pamc, pre_dx, "center")
                    pid_yaw.i_term *= 0.95

                elif (xc is not None) and (yc is None):
                    err_x = (xc / float(self.frame_width)) -  float(self.original_x)
                    yaw_cmd, yaw_dir = pid_yaw.update(err_x, dt, pos_dir_labels=("left","right"))
                    drive_outputs(self.pamc, yaw_cmd, yaw_dir, 0, "center")
                    apply_direction_step(self.pamc, "center", pre_dy)
                    pid_pitch.i_term *= 0.95

                else:
                    apply_direction_step(self.pamc, pre_dx, pre_dy)
                    pid_yaw.i_term *= 0.9
                    pid_pitch.i_term *= 0.9


        finally:
            # Clean shutdown even if exceptions occur
            try: self.cam1.stop_acquisition()
            except Exception: pass
            try: self.cam1.release()
            except Exception: pass
            try: close_sdk()
            except Exception: pass
            try: print("Stop:", self.pamc.stop())
            except Exception: pass
            try: self.pamc.close()
            except Exception: pass


    
    if __name__ == "__main__":
        camera = TLCameraController()
        camera.setup_camera(exposure_us=1000, gain=0)
        camera.frames_per_trigger(1)
        camera.initialize_acquisition()

        pamc = PAMC104Controller(port="COM7")
        print("Connection check:", pamc.connect_check())
        sys = feedback_system(cam1=camera, pamc=pamc)
    # 开环运行（按需设置容差与沉降时间）
        feedback_system.feedback_loop()
        