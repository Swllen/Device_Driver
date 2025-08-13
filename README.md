# BPC3xxController (Python, pythonnet)

A lightweight Python wrapper around **Thorlabs Kinesis .NET API** for **BPC3xx Benchtop Piezo** controllers.  
This class lets you pick which axes to connect (`X/Y/Z`), drive **single or multiple axes** with **parallel** execution, and switch between **closed-loop (position)** and **open-loop (voltage)** control.

---

## Features

- Connect to a BPC3xx device and **select which channels** to enable (`axes=('X','Z')`, etc.).
- **Closed-loop** (position in real units, typically µm) and **Open-loop** (voltage) modes.
- **Single-axis** and **multi-axis** commands with optional **parallel** execution.
- Convenience helpers: `pos()`, `volts()`, `zero()`, `zero_output()`.
- Safety checks against **travel** and **voltage** limits.
- Works with **simulation** (Kinesis Simulation Manager) for development.

---

## Requirements

- Windows with **Thorlabs Kinesis** installed.
- Python 3.x + **pythonnet** (`pip install pythonnet`).
- Kinesis DLLs available at:
```python
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericPiezoCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.Benchtop.PiezoCLI.dll")
```

If installed elsewhere, update the `clr.AddReference(...)` paths in your script.

---

## Quick Start

```python
    # Serial number of BPC3xx device.
    # For testing without hardware, use a simulation serial number  and set use_sim.
    serial_number = "71000001"  # Simulation example

    # Create controller instance for X and Z axes only
    dev = BPC3xxController(
        serial=serial_number,
        axes=('X', 'Z'),      
        # Control only X and Z channels.
        # You can also change The axes you want to activate.
        use_sim=True          # Enable simulation mode
    )

    # -------- Closed-loop control example (position control) --------
    dev.set_mode("CloseLoop")  # Switch all connected axes to closed-loop mode

    # Move both X and Z to given positions in micrometers, in parallel
    dev.move(X=5.0, Z=1.0,settle_s=5, parallel=True)
    # For single channel
    dev.set_position(axis: str, pos_real: float, settle_s=0.0):
    
    # Zero the output voltages for all active axes, in parallel
    dev.zero_output(parallel=True)
    # Zero specific axies
    dev.zero_output('X')

    # Read the current position of X only
    x_pos = dev.get_position('X')
    print(f"Current X position: {x_pos}")
    # Read mutiple position. e.g. Get current positions of all active axes
    print("All positions:", dev.pos())


    # -------- Open-loop control example (voltage control) --------
    dev.set_mode("OpenLoop")  # Switch to open-loop mode

    # Apply voltages to X and Z channels, in parallel
    dev.volt(X=10.0, Z=8.0, parallel=True)
    # For single channel
    dev.set_voltage(self, axis: str, volts: float, settle_s=0.0)

    # Read back the current output voltages
    print("All voltages:", dev.volts())
    print("X",dev.volts('x'))

    # -------- Clean up --------
    dev.close()
```
