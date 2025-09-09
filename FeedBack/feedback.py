import cv2
import numpy as np
import time
import sys
from collections import deque
sys.path.append(r"E:\BU_Research\AutoOAM\DeviceDriver\Device_Driver")
from PAMC104.PAMC104 import PAMC104Controller
from TLCam.TLCam import *
from preprocess import *

# ===== Control & hardware parameters =====
DEADBAND_NORM = 0.03        # Normalized deadband to ignore tiny errors
KP_NR = 26000.0                 # Proportional gain (normalized -> pulse)
KI_NR = 100                    # Integral gain (normalized-sum -> pulse)
KD_NR = 2.0
KP_RR = 26000.0                 # Proportional gain (normalized -> pulse)
KI_RR = 100                    # Integral gain (normalized-sum -> pulse)
KD_RR = 2.0                      # Derivative gain (normalized rate -> pulse)
INTEGRAL_CLAMP = 5000.0       # Clamp for integral accumulator (anti-windup)
DERIVATIVE_LPF_ALPHA = 0.2  # 0..1; lower = stronger smoothing (for D term)
MIN_PULSE = 10              # Do not send zero; any motion gets at least this
MAX_PULSE = 9999            # HW limit
STEP_PULSE = 5000             # Step used when center is missing for an axis
SPEED = 1500                # Motor speed
YAW_CH = "A"
PITCH_CH = "B"

YAW_CH1 = "A"
PITCH_CH1 = "B"

YAW_CH2 = "C"
PITCH_CH2 = "D"

# ===== Display & logging parameters =====
WINDOW_NAME = "Feedback Viewer"
LOG_INTERVAL_S = 0.2        
TEXT_SCALE = 0.6
TEXT_THICK = 1
TEXT_COLOR = (0, 255, 0)    
TEXT_COLOR_WARN = (0, 200, 255)
CROSS_COLOR = (255, 0, 0)   
CENTER_COLOR = (0, 255, 255)
POINT_COLOR = (0, 255, 0)   

def clamp_pulse(v: float) -> int:
    """Clamp a positive pulse to [MIN_PULSE, MAX_PULSE] and return int."""
    return int(max(MIN_PULSE, min(MAX_PULSE, int(round(v)))))

class PIDAxis:
    """
    Per-axis PID controller with:
      - deadband on error
      - integral clamp (anti-windup)
      - derivative low-pass filter
      - returns (pulse>=MIN_PULSE, direction) when motion needed; otherwise (0, 'center')
    """
    def __init__(self, kp, ki, kd, deadband_norm, i_clamp, d_alpha):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband_norm
        self.i_clamp = i_clamp
        self.d_alpha = d_alpha
        self.i_term = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.initialized = False

    def reset(self):
        self.i_term = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.initialized = False

    def update(self, err: float, dt: float, pos_dir_labels=("left","right")):
        """
        err: normalized error (positive means target is to the + direction)
        dt: elapsed seconds
        pos_dir_labels: ('pos_dir', 'neg_dir'), e.g. ('left','right') for yaw, ('up','down') for pitch
        Returns: (pulse_int, direction_str) or (0, 'center') if within deadband
        """
        # Deadband
        if abs(err) < self.deadband or dt <= 0:
            # Optional: light integral decay to avoid residual windup when in deadband
            self.i_term *= 0.95
            return 0, "center"

        # Proportional
        p = self.kp * err

        # Integral with clamp (accumulate true error, not abs)
        self.i_term += err * dt
        self.i_term = max(-self.i_clamp, min(self.i_clamp, self.i_term))
        i = self.ki * self.i_term

        # Derivative on error with low-pass filtering
        if not self.initialized:
            derr = 0.0
            self.d_filt = 0.0
            self.initialized = True
        else:
            derr = (err - self.prev_err) / dt
        self.d_filt = self.d_alpha * self.d_filt + (1.0 - self.d_alpha) * derr
        d = self.kd * self.d_filt

        self.prev_err = err

        # Raw control (can be negative -> carries direction)
        u = p + i + d

        # Map sign to direction string
        pos_dir, neg_dir = pos_dir_labels
        direction = pos_dir if u > 0 else neg_dir

        # Magnitude to pulse, enforce minimum since we must not send 0
        pulse = clamp_pulse(abs(u))

        return pulse, direction

# Build two PID controllers (yaw uses left/right; pitch uses up/down)
pid_yaw = PIDAxis(KP_NR, KI_NR, KD_NR, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)
pid_pitch = PIDAxis(KP_RR, KI_RR, KD_RR, DEADBAND_NORM, INTEGRAL_CLAMP, DERIVATIVE_LPF_ALPHA)

def get_original_position(frame):
    _, _, xc, yc = process_and_judge_direction(frame)
    return xc, yc

def apply_direction_step(pamc: PAMC104Controller, dirx, diry, step=STEP_PULSE, speed=SPEED,
                         channel_yaw=YAW_CH, channel_pitch=PITCH_CH):
    """When a center is missing on an axis, nudge that axis in the inferred direction."""
    # Yaw
    if dirx == "left":
        pamc.drive("RR", speed, clamp_pulse(step), channel_yaw)
    elif dirx == "right":
        pamc.drive("NR", speed, clamp_pulse(step), channel_yaw)
    time.sleep(clamp_pulse(step)/speed+0.05)
    # Pitch
    if diry == "down":
        pamc.drive("RR", speed, clamp_pulse(step), channel_pitch)
    elif diry == "up":
        pamc.drive("NR", speed, clamp_pulse(step), channel_pitch)
    time.sleep(clamp_pulse(step)/speed+0.05)
    

def drive_outputs(pamc: PAMC104Controller, yaw_cmd, yaw_dir, pitch_cmd, pitch_dir,
                  channel_yaw=YAW_CH, channel_pitch=PITCH_CH, speed=SPEED):
    """Send clamped non-zero pulses to hardware (if command > 0)."""
    if yaw_cmd > 0 and yaw_dir in ("left","right"):
        if yaw_dir == "left":
            pamc.drive("RR", speed, clamp_pulse(yaw_cmd), channel_yaw)
        else:
            pamc.drive("NR", speed, clamp_pulse(yaw_cmd), channel_yaw)
    time.sleep(clamp_pulse(yaw_cmd)/speed+0.05)
    if pitch_cmd > 0 and pitch_dir in ("up","down"):
        if pitch_dir == "down":
            pamc.drive("RR", speed, clamp_pulse(pitch_cmd), channel_pitch)
        else:
            pamc.drive("NR", speed, clamp_pulse(pitch_cmd), channel_pitch)
    time.sleep(clamp_pulse(pitch_cmd)/speed+0.05)

def draw_overlay(frame, originalx, originaly, xc, yc, err_x=None, err_y=None,
                 yaw_cmd=0, yaw_dir="center", pitch_cmd=0, pitch_dir="center",
                 fps=None, msg=""):
    """在画面上叠加可视化信息。"""
    # 确保三通道
    if frame.ndim == 2:
        disp = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:
        disp = frame.copy()

    h, w = disp.shape[:2]

    # 参考中心
    if originalx is not None and originaly is not None:
        cv2.drawMarker(disp, (int(originalx), int(originaly)), CENTER_COLOR,
                       markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)

    # 当前检测到的中心
    if xc is not None and yc is not None:
        cv2.circle(disp, (int(xc), int(yc)), 6, POINT_COLOR, 2)


    # 文本信息
    y0, dy = 24, 22
    lines = []
    if fps is not None:
        lines.append(f"FPS: {fps:.1f}")
    if err_x is not None and err_y is not None:
        lines.append(f"err_x={err_x:+.4f}, err_y={err_y:+.4f}")
    lines.append(f"yaw: {yaw_dir:>6}  cmd={yaw_cmd:4d}")
    lines.append(f"pit: {pitch_dir:>6}  cmd={pitch_cmd:4d}")
    if msg:
        lines.append(msg)

    for i, t in enumerate(lines):
        color = TEXT_COLOR if "center" not in t else TEXT_COLOR_WARN
        cv2.putText(disp, t, (10, y0 + i*dy), cv2.FONT_HERSHEY_SIMPLEX,
                    TEXT_SCALE, color, TEXT_THICK, cv2.LINE_AA)

    return disp

def feedback_loop():
    # Camera init
    camera = TLCameraController()
    camera.setup_camera(exposure_us=500, gain=0)
    camera.frames_per_trigger(1)
    camera.initialize_acquisition()

    # Motor controller init
    pamc = PAMC104Controller(port="COM7")
    print("Connection check:", pamc.connect_check())

    # OpenCV WINDOW
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME,720,540)

    # Reference center (fall back to image center if detection fails)
    first = camera.acquire_frame()
    
    # xc0, yc0 = get_original_position(first)
    #_________ DEBUG___________-
    xc0=720
    yc0=504

    h0, w0 = first.shape[:2]
    if xc0 is None or yc0 is None:
        originalx, originaly = w0 / 2.0, h0 / 2.0
        print("[Init] Center not detected, use center of the frame as refrence")
    else:
        originalx, originaly = xc0, yc0
        print(f"[Init] Initial refrence center: ({originalx:.1f}, {originaly:.1f})")

    # Reset PID states
    pid_yaw.reset()
    pid_pitch.reset()

    # Timing
    t_prev = time.time()
    last_log_t = 0.0
    fps_hist = deque(maxlen=30)

    try:
        while True:
            frame = camera.acquire_frame()
            pre_dx, pre_dy, xc, yc = process_and_judge_direction(frame)
            h, w = frame.shape[:2]

            t_now = time.time()
            dt = max(1e-3, t_now - t_prev)
            t_prev = t_now
            fps_hist.append(1.0 / dt)
            fps = sum(fps_hist) / len(fps_hist)

            yaw_cmd = 0
            yaw_dir = "center"
            pitch_cmd = 0
            pitch_dir = "center"
            status_msg = ""

            # Branch by availability of centers
            if (xc is not None) and (yc is not None):
                err_x = (xc / float(w)) - (originalx / float(w))
                err_y = (yc / float(h)) - (originaly / float(h))

                yaw_cmd, yaw_dir     = pid_yaw.update(err_x, dt, pos_dir_labels=("left","right"))
                pitch_cmd, pitch_dir = pid_pitch.update(err_y, dt, pos_dir_labels=("up","down"))

                drive_outputs(pamc, yaw_cmd, yaw_dir, pitch_cmd, pitch_dir)
                status_msg = "PID control"

            elif (xc is None) and (yc is not None):
                # Y only: PID on pitch; yaw steps by inferred direction
                err_x = None
                err_y = (yc / float(h)) - (originaly / float(h))
                pitch_cmd, pitch_dir = pid_pitch.update(err_y, dt, pos_dir_labels=("up","down"))
                drive_outputs(pamc, 0, "center", pitch_cmd, pitch_dir)
                apply_direction_step(pamc, pre_dx, "center")
                pid_yaw.i_term *= 0.95
                status_msg = f"X lost -> yaw step {pre_dx or 'center'}"

            elif (xc is not None) and (yc is None):
                # X only: PID on yaw; pitch steps by inferred direction
                err_x = (xc / float(w)) - (originalx / float(w))
                err_y = None
                yaw_cmd, yaw_dir = pid_yaw.update(err_x, dt, pos_dir_labels=("left","right"))
                drive_outputs(pamc, yaw_cmd, yaw_dir, 0, "center")
                apply_direction_step(pamc, "center", pre_dy)
                pid_pitch.i_term *= 0.95
                status_msg = f"Y lost -> pitch step {pre_dy or 'center'}"

            else:
                # Neither center found: step both axes by inferred directions
                err_x = None
                err_y = None
                apply_direction_step(pamc, pre_dx, pre_dy)
                # Slowly bleed integrators to avoid stale windup
                pid_yaw.i_term *= 0.9
                pid_pitch.i_term *= 0.9
                status_msg = f"Both lost -> step X:{pre_dx or 'center'} Y:{pre_dy or 'center'}"

            # print log
            if (t_now - last_log_t) >= LOG_INTERVAL_S:
                last_log_t = t_now
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] dt={dt*1000:.1f}ms  FPS={fps:.1f}  "
                      f"xc={None if xc is None else f'{xc:.1f}'}  "
                      f"yc={None if yc is None else f'{yc:.1f}'}  "
                      f"err_x={None if err_x is None else f'{err_x:+.4f}'}  "
                      f"err_y={None if err_y is None else f'{err_y:+.4f}'}  "
                      f"yaw=({yaw_dir},{yaw_cmd})  pitch=({pitch_dir},{pitch_cmd})  "
                      f"{status_msg}")

            # Dispaly
            disp = draw_overlay(frame, originalx, originaly, xc, yc,
                                err_x=0.0 if err_x is None else err_x,
                                err_y=0.0 if err_y is None else err_y,
                                yaw_cmd=yaw_cmd, yaw_dir=yaw_dir,
                                pitch_cmd=pitch_cmd, pitch_dir=pitch_dir,
                                fps=fps, msg=status_msg)
            
            norm = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
            disp = norm.astype('uint8')
            cv2.imshow(WINDOW_NAME, disp)

            # Exit: key q or esc
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                print("[Info] 用户请求退出。")
                break

            time.sleep(0.01)

    finally:
        # safely exit
        try:
            camera.stop_acquisition()
        except Exception:
            pass
        try:
            camera.release()
        except Exception:
            pass
        try:
            close_sdk()
        except Exception:
            pass
        try:
            print("Stop:", pamc.stop())
        except Exception:
            pass
        try:
            pamc.close()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    feedback_loop()
