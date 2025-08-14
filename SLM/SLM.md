# SantecSLM Control and Pattern Generation

## Overview
This repository provides:
- A `SantecSLM` Python class to control an LCOS-SLM device via the `_slm_win` SDK.
- Pattern generation functions in `CreatePattern.py` to create and manipulate phase maps for display on the SLM.
- Support for loading `.mat` mask files, applying shifts, and combining multiple phase maps.

The SLM control part:
- Automatically scans for connected LCOS-SLM devices.
- Opens the detected SLM and sends a `uint16` image buffer for display.
- Uses the SDK functions `SLM_Disp_Info2`, `SLM_Disp_Open`, `SLM_Disp_Data`, and `SLM_Disp_Close`.

The pattern generation part:
- Provides functions for vortex phase, checkerboard patterns, periodic matrices, aperture masks, and background generation.
- Includes `.mat` file loading for real/imaginary mask data and conversion to 10-bit phase levels.
- Allows phase shifting, rotation, defocus, tilt, and other transformations.

---

## Installation Requirements
- Python 3.x
- NumPy, SciPy, matplotlib, pandas, h5py
- `_slm_win` Python binding to the LCOS-SLM SDK
- Access to `SLMFunc.dll` which can be downloaded in Santec web.

