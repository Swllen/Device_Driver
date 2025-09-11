from typing import Tuple
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

DEBUG = False
SMOOTH_K = 21
SUBPIXEL = True
SIGMA = 20
DISTANCE = 400
PROMINENCE = 2

def gaussian_blur(img: np.ndarray, sigma: float) -> np.ndarray:
    """
    Gaussian blur
    Removes fine details or high-frequency information and details.
    """
    if sigma <= 0:
        return img
    k = max(3, int(6*sigma) | 1)
    return cv2.GaussianBlur(img, (k, k), sigmaX=sigma, sigmaY=sigma)

def smooth1d(arr: np.ndarray, k: int) -> np.ndarray:
    """Average filter"""
    k = max(3, int(k) | 1)
    ker = np.ones(k, dtype=np.float32) / k
    return np.convolve(arr, ker, mode='same')

def quad_subpixel(y: np.ndarray, i: int) -> float:
    """Quadratic subpixel peak around index i; returns floating index."""
    if i <= 0 or i >= len(y)-1:
        return float(i)
    y0, y1, y2 = y[i-1], y[i], y[i+1]
    denom = (y0 - 2*y1 + y2) # base on parabola
    if abs(denom) < 1e-9:
        return float(i)
    delta = 0.5 * (y0 - y2) / denom
    return float(i + np.clip(delta, -1, 1))

def plot_debug(p,peaks):
    plt.figure(figsize=(8,4))
    plt.plot(p, label="Profile")
    plt.plot(peaks, p[peaks], "rx", label="Peaks")  
    plt.title("Profile Peaks")
    plt.xlabel("Pixel Index")
    plt.ylabel("Mean Intensity")
    plt.legend()
    plt.grid(True)
    plt.show()
    

def profile_peaks_center(profile: np.ndarray, smooth_k: int = 3, 
                         prominence: int = 2,distance_peaks: int = 200,
                         subpixel: bool = True, debug: bool = DEBUG) -> Tuple[float, float, float]:
    """Return (center_index, left_peak_index, right_peak_index) as floats (subpixel)."""
    p = smooth1d(profile, k=smooth_k) if smooth_k > 1 else profile.copy()
    # peaks, _ = find_peaks(p,height=0.5*np.max(p),prominence=PROMINENCE,distance=distance)
    peaks, _ = find_peaks(p,height=0.2*np.max(p),distance=distance_peaks)
    min_idx = int(np.argmin(p))   # if min index is on the left of peak(only single), the ring is right.
    if debug == True:
        plot_debug(p,peaks)

    # The ring is incomplete, to it is on which side.
    if len(peaks) == 1:
        if peaks[0] < min_idx:
            iR = peaks[0]
            if subpixel:
                iR = quad_subpixel(p, iR)
            return None, None, iR
        else:
            iL = peaks[0]
            if subpixel:
                iL = quad_subpixel(p, iL)
            return None, iL, None
        
    elif len(peaks) == 0:
        return None, None, None
    
    else:
        iL = peaks[0]
        iR = peaks[-1]
        if subpixel:
            iL = quad_subpixel(p, iL)
            iR = quad_subpixel(p, iR)
        c = 0.5 * (iL + iR)
        return c, iL, iR
   
def process_and_judge_direction(img: np.ndarray, sigma: float = 20.0, smooth_k: int = 21,
                                prominence=2, distance_peaks=200, subpixel: bool = True):
    """Compute center (xc, yc) and radius r from profiles; return diagnostics in dict."""
    blur = gaussian_blur(img, sigma) if sigma > 0 else img
    xprof = blur.mean(axis=0)
    yprof = blur.mean(axis=1)

    xc, lx, rx = profile_peaks_center(xprof, smooth_k, prominence, distance_peaks, subpixel)
    yc, ty, by = profile_peaks_center(yprof, smooth_k, prominence, distance_peaks, subpixel)

    if xc is None:
        if (lx is None) and (rx is None):
            directionx = "None"
        if (lx is None) and (rx is not None):
            directionx = "right"
        if (rx is None) and (lx is not None):
            directionx = "left"
    else:
        directionx = "Center"
    if yc is None:
        if (ty is None) and (by is None):
            directiony = "None"
        if (ty is None) and (by is not None):
            directiony = "down"
        if (by is None) and (ty is not None):
            directiony = "up"
    else:
        directiony = "Center"

    return directionx,directiony, xc, yc


if __name__ == "__main__":
    # image_path = r"E:\BU_Research\LabData\0814\Defocus_Sweep\20250814_1945\cam2\phase2_0.png"
    # image_path = "FeedBack\image.png"
    # image_path = r"D:\Desktop\2.png"
    image_path = r"E:\BU_Research\AutoOAM\DeviceDriver\Device_Driver\FeedBack\1.png"
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    directionx,directiony, xc, yc = process_and_judge_direction(img,smooth_k=SMOOTH_K,subpixel=SUBPIXEL,
                                                                distance_peaks=DISTANCE, prominence=PROMINENCE,
                                                                sigma=SIGMA)
    print(f"Direction X: {directionx}, Direction Y: {directiony}, xc: {xc}, yc: {yc}")