import numpy as np
import h5py
from scipy.special import erf

def create_periodic_matrix(height, width, high_value, low_value, num_cols_pattern=4):
    """
    Create a periodic matrix with alternating columns of a high value and a low value.

    Parameters:
    - height: Number of rows in the final matrix
    - width: Number of columns in the final matrix
    - high_value: Value for the columns where the pattern is high
    - low_value: Value for the columns where the pattern is low
    - num_cols_pattern: Number of columns for each block of the pattern (e.g., 5 columns of high_value and 5 columns of low_value)

    Returns:
    - A periodic matrix of shape (height, width)
    """
    # Create a 1×width column pattern with 10-bit values directly
    col_pattern = np.arange(width) // num_cols_pattern
    # Alternate between high and low values (both should be 10-bit values 0-1023)
    matrix_10bit = np.where(col_pattern % 2 == 0, high_value, low_value)
    # Tile the pattern to all rows
    matrix_10bit = np.tile(matrix_10bit, (height, 1)).astype(np.uint16)

    # Trim to the exact size
    return matrix_10bit

def create_vortex_phase_map_16bit(height, width, l=1, max_value=1023):
    """
    生成16位涡旋相位图，最大值max_value对应2π相位

    参数:
    - height: 图像高度
    - width: 图像宽度
    - l: 涡旋拓扑荷数（整数）
    - max_value: 最大相位值，对应2π

    返回:
    - numpy数组，dtype=uint16，范围0~max_value
    """
    x = np.arange(width) - width / 2
    y = np.arange(height) - height / 2
    X, Y = np.meshgrid(x, y)

    theta = np.arctan2(Y, X)  # -π 到 π

    phase = (l * theta) % (2 * np.pi)  # 0 到 2π

    phase_scaled = (phase / (2 * np.pi)) * max_value

    return phase_scaled.astype(np.uint16)

def create_vortex_phase_map_16bit(height, width, center_x, center_y, l=1, max_value=1023, ROI=None):
    """
    生成16位涡旋相位图，最大值max_value对应2π相位

    参数:
    - height: 图像高度
    - width: 图像宽度
    - l: 涡旋拓扑荷数（整数）
    - max_value: 最大相位值，对应2π
    - ROI: (x,y)
    返回:
    - numpy数组，dtype=uint16，范围0~max_value
    """
    if ROI is None:
        roi_height, roi_width = height, width
    else:
        roi_height, roi_width = ROI

    x = np.arange(roi_width) - roi_width / 2
    y = np.arange(roi_height) - roi_height / 2
    X, Y = np.meshgrid(x, y)

    theta = np.arctan2(Y, X)  # -π 到 π
    phase = (l * theta) % (2 * np.pi)  # 0 到 2π
    phase_scaled = (phase / (2 * np.pi)) * max_value
    roi_phase_map = phase_scaled.astype(np.uint16)

    phase_map = np.zeros((height, width), dtype=np.uint16)

    start_y = int(center_y - roi_height / 2)
    start_x = int(center_x - roi_width / 2)
    end_y = start_y + roi_height
    end_x = start_x + roi_width

    sy, ey = max(0, start_y), min(height, end_y)
    sx, ex = max(0, start_x), min(width, end_x)

    roi_sy = sy - start_y
    roi_sx = sx - start_x
    roi_ey = roi_sy + (ey - sy)
    roi_ex = roi_sx + (ex - sx)

    phase_map[sy:ey, sx:ex] = roi_phase_map[roi_sy:roi_ey, roi_sx:roi_ex]

    return phase_map

def create_pattern(height, width, center_x, center_y, array, max_value=1023, ROI=None, aperture_radius=None):
    """
    将一个局部相位图 array 放置到指定坐标 center_x, center_y 的大图中（height × width），
    并可选地应用一个圆形光阑（圆外设为0）。

    参数:
    - height, width: 输出图像大小
    - center_x, center_y: 放置 array 的中心位置
    - array: 相位图数组（uint16）
    - max_value: 最大相位值（默认1023）
    - ROI: 指定 array 的大小 (h, w)，否则自动获取
    - aperture_radius: 如果设置，将添加一个圆形光阑，单位为像素半径
    """
    if ROI is None:
        roi_height, roi_width = np.shape(array)
    else:
        roi_height, roi_width = ROI

    phase_map = np.zeros((height, width), dtype=np.uint16)

    start_y = int(center_y - roi_height / 2)
    start_x = int(center_x - roi_width / 2)
    end_y = start_y + roi_height
    end_x = start_x + roi_width

    sy, ey = max(0, start_y), min(height, end_y)
    sx, ex = max(0, start_x), min(width, end_x)

    roi_sy = sy - start_y
    roi_sx = sx - start_x
    roi_ey = roi_sy + (ey - sy)
    roi_ex = roi_sx + (ex - sx)

    roi_insert = array[roi_sy:roi_ey, roi_sx:roi_ex]

    if aperture_radius is not None:
        yy, xx = np.ogrid[sy:ey, sx:ex]
        mask = (xx - center_x)**2 + (yy - center_y)**2 <= aperture_radius**2
        masked_roi = np.zeros_like(roi_insert, dtype=np.uint16)
        masked_roi[mask] = roi_insert[mask]
        roi_insert = masked_roi

    phase_map[sy:ey, sx:ex] = roi_insert

    return phase_map

def create_checkerboard_phase_map_16bit(height, width, center_x=960, center_y=600, l=8, max_value=1023, ROI=None):
    """
    生成16位棋盘格相位图，棋盘格由0和max_value组成

    参数:
    - height: 图像高度
    - width: 图像宽度
    - l: 棋盘格每行/列格子数（整数）
    - max_value: 最大值，用于棋盘格白格
    - ROI: (roi_height, roi_width)，可选区域大小
    返回:
    - numpy数组，dtype=uint16，范围0~max_value
    """
    if ROI is None:
        roi_height, roi_width = height, width
    else:
        roi_height, roi_width = ROI

    x = np.arange(roi_width)
    y = np.arange(roi_height)
    X, Y = np.meshgrid(x, y)

    # 每个格子的像素大小
    box_height = roi_height / l
    box_width = roi_width / l

    # 计算格子编号并生成棋盘格模式
    x_idx = (X / box_width).astype(int)
    y_idx = (Y / box_height).astype(int)
    checkerboard = (x_idx + y_idx) % 2  # 黑白交错
    roi_phase_map = (checkerboard * max_value).astype(np.uint16)

    # 整图初始化并嵌入ROI
    phase_map = np.zeros((height, width), dtype=np.uint16)

    start_y = int(center_y - roi_height / 2)
    start_x = int(center_x - roi_width / 2)
    end_y = start_y + roi_height
    end_x = start_x + roi_width

    sy, ey = max(0, start_y), min(height, end_y)
    sx, ex = max(0, start_x), min(width, end_x)

    roi_sy = sy - start_y
    roi_sx = sx - start_x
    roi_ey = roi_sy + (ey - sy)
    roi_ex = roi_sx + (ex - sx)

    phase_map[sy:ey, sx:ex] = roi_phase_map[roi_sy:roi_ey, roi_sx:roi_ex]

    return phase_map

def create_background(height,width,value):
    phase = np.zeros((height,width))+1
    phase = phase*value
    phase = phase.astype(np.uint16)
    return phase

## For phase change
def open_mat(file_path):
    
    with h5py.File(file_path, 'r') as f:
        # print("Top-level keys:", list(f.keys()))

        mask_all = f['masks']['MASKS_ALL']
        # print("Data type:", mask_all.dtype)


        real_part = mask_all['real']
        imag_part = mask_all['imag']
        mask_complex = real_part + 1j * imag_part

        # print("mask shape:", np.shape(mask_complex))

        mask1 = mask_complex[0].T
        mask2 = mask_complex[1].T
        
        phase1 = np.angle(mask1)
        phase2 = np.angle(mask2)

        phase1 = np.mod(phase1 + np.pi, 2 * np.pi) / 2 / np.pi * 1023
        phase2 = np.mod(phase2 + np.pi, 2 * np.pi) / 2 / np.pi * 1023

        phase1 = phase1.astype(np.uint16)
        phase2 = phase2.astype(np.uint16)

    return phase1, phase2

def cirshift_xy(array,x_shift,y_shift):
    array_shift = np.roll(array,y_shift,axis=0)
    array_shift = np.roll(array_shift,x_shift,axis=1)

    return array_shift

def applyPhaseChange(original_phase, wavelength_nm, pixel_size_um=8, bits=10, tilt_x_mrad=0, tilt_y_mrad=0,
                    defocus_mm=0, axicon_mrad=0,A=0, B=0, C=0, xastig=0, yastig=0):

    # Shape
    H, W = original_phase.shape

    # Unit conversion
    pixel_size_mm = pixel_size_um * 1e-3
    lambda_mm = wavelength_nm * 1e-6

    # Create coordinate grid
    x = np.arange(1, W + 1) - (W / 2 + 0.5)
    y = np.arange(1, H + 1) - (H / 2 + 0.5)
    X, Y = np.meshgrid(x, y)

    # Convert to physical coordinates (in mm)
    X = X * pixel_size_mm
    Y = Y * pixel_size_mm

    # Convert to UV coordinates (astigmatic transformation)
    xastig_rad = np.deg2rad(xastig)
    yastig_safe = yastig if abs(yastig) > 1e-6 else 1e-6

    U = X * np.sin(xastig_rad) + Y * np.cos(xastig_rad)
    V = -X * np.cos(xastig_rad) + Y * np.sin(xastig_rad)

    # # Convert original phase from radians to grayscale levels
    radToGray = (2 ** bits) / (2 * np.pi)
    # original_phase_gray = (original_phase + np.pi) * radToGray

    # Tilt phase
    tilt_phase = -2 * np.pi / lambda_mm * (tilt_x_mrad * 1e-3 * X + tilt_y_mrad * 1e-3 * Y)

    # Defocus phase
    if defocus_mm != 0:
        defocus_phase = -np.pi / (lambda_mm * defocus_mm) * (U ** 2 + (1 / (yastig_safe ** 2)) * V ** 2)
    else:
        defocus_phase = 0

    # Axicon phase
    r = np.sqrt(U ** 2 + (1 / (yastig_safe ** 2)) * V ** 2)
    axicon_phase = -2 * np.pi / lambda_mm * axicon_mrad * 1e-3 * r

    # Two-plate ABC phase
    abc_phase = A * erf(B * (r - C))

    # Combine all phase contributions
    total_phase = tilt_phase + defocus_phase + axicon_phase + abc_phase
    total_phase_gray = np.uint16((total_phase % (2 * np.pi)) / (2 * np.pi) * 1023)
    
    # Final phase changed image
    phase_changed = (original_phase + total_phase_gray) % 1024

    return phase_changed


# Merge two phase in 1920*1200 map
def merge_phase(phase1,phase2):
    pattern = np.zeros((1200, 1920), dtype=np.uint16)
    pattern[:,0:1920//2] = phase1
    pattern[:,1920//2:1920] = phase2

    return pattern