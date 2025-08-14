import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
import clr
from System import Decimal

clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericPiezoCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.Benchtop.PiezoCLI.dll")


from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericPiezoCLI import *
from Thorlabs.MotionControl.GenericPiezoCLI import Piezo
from Thorlabs.MotionControl.GenericPiezoCLI import DeviceUnits
from Thorlabs.MotionControl.Benchtop.PiezoCLI import *

class BPC3xxController:
    def __init__(self, serial: str,
                 axes=('X', 'Y', 'Z'),
                 axis_map=None,                # e.g. {'X':1,'Y':2,'Z':3}
                 poll_ms=250,
                 use_sim=False):
        self.serial = serial
        self.axes = tuple(a.upper() for a in axes)
        self.axis_map = axis_map or {'X': 1, 'Y': 2, 'Z': 3}
        self.poll_ms = poll_ms
        self.use_sim = use_sim

        self._dev = None
        self._channels = {}                  # 'X'|'Y'|'Z' -> PiezoChannel '1 2 3'
        self._connected = False
        self.connect()

    def connect(self):
        if self._connected:
            return
        if self.use_sim:
            SimulationManager.Instance.InitializeSimulations()
        DeviceManagerCLI.BuildDeviceList()
        self._dev = BenchtopPiezo.CreateBenchtopPiezo(self.serial)
        self._dev.Connect(self.serial)

        for axis in self.axes:
            ch_id = int(self.axis_map[axis])
            ch = self._dev.GetChannel(ch_id)
            if not ch.IsSettingsInitialized():
                ch.WaitForSettingsInitialized(10000)
                if not ch.IsSettingsInitialized():
                    raise RuntimeError(f"channel {ch_id}（axies {axis}）initialize filed.")
            ch.StartPolling(self.poll_ms); time.sleep(self.poll_ms/1000.0)
            ch.EnableDevice();             time.sleep(0.2)
            self._channels[axis] = ch
        self._connected = True

    def close(self):
        try:
            for ch in self._channels.values():
                try: ch.StopPolling()
                except: pass
        finally:
            try:
                if self._dev and self._dev.IsConnected:
                    self._dev.Disconnect(True)
            except: pass
            if self.use_sim:
                try: SimulationManager.Instance.UninitializeSimulations()
                except: pass
            self._channels.clear()
            self._dev = None
            self._connected = False

    def __del__(self):
        self.close()

    # ---------------- helpers ----------------
    def _ch(self, axis: str):
        a = axis.upper()
        if a not in self._channels:
            raise KeyError(f"axies {a} is not activate.")
        return self._channels[a]

    def _axes_or_all(self, axes):
        return tuple(a.upper() for a in (axes or tuple(self._channels.keys())))

    def _run_parallel(self, funcs, parallel=True):
        """
        funcs: dict {'X': callable, ...}
        """
        if not parallel or len(funcs) <= 1:
            # Sequential execution
            for f in funcs.values(): f()
            return
        
        with ThreadPoolExecutor(max_workers=len(funcs)) as ex:
            futs = {ex.submit(f): name for name, f in funcs.items()}
            for fut in as_completed(futs):
                fut.result()

    def set_mode(self, mode: str):
        mode = mode.strip().lower()
        if mode not in ('openloop', 'closeloop'):
            raise ValueError("mode 只能是 'OpenLoop' 或 'CloseLoop'")
        target = PiezoControlModeTypes.OpenLoop if mode == 'openloop' else PiezoControlModeTypes.CloseLoop

        for a, ch in self._channels.items():
            ch.SetPositionControlMode(target)
            time.sleep(0.25)
            cur = f"ClosedLoop={getattr(ch,'IsClosedLoop',lambda:'NA')()}"
            print(f"[{a}] mode -> {cur}")

    # ---------------- Single-axis bottom layer----------------
    def set_position(self, axis: str, pos_real: float, settle_s=0.0):
        ch = self._ch(axis)
        p = Decimal(pos_real)
        mt = ch.GetMaxTravel()
        if not (Decimal(0) <= p <= mt):
            raise ValueError(f"[{axis}] position {p} exceed 0~{mt} μm.")
        ch.SetPosition(p)
        if settle_s > 0: time.sleep(settle_s)

    def get_position(self, axis: str):
        pos = self._ch(axis).GetPosition()
        time.sleep(0.25)
        return pos

    def set_voltage(self, axis: str, volts: float, settle_s=0.0):
        ch = self._ch(axis)
        v = Decimal(volts)
        mv = ch.GetMaxOutputVoltage()
        if not (Decimal(0) <= v <= mv):
            raise ValueError(f"[{axis}] voltage {v} exceed 0~{mv} V.")
        ch.SetOutputVoltage(v)
        if settle_s > 0: time.sleep(settle_s)

    def get_voltage(self, axis: str):
        vol = self._ch(axis).GetOutputVoltage()
        time.sleep(0.25)
        return vol

    # ---------------- Multi-axis parallel interface----------------
    def move(self, parallel=True, settle_s=0.0, **axes):
        """
        parallel=True: Control multiple axes simultaneously.
        settle_s: Wait for motion done.
        example: move(X=5.0, Z=2.5) (µm)
        """
        funcs = {a.upper(): (lambda A=a, v=v: (self.set_position(A, float(v), settle_s)))
                 for a, v in axes.items()}
        self._run_parallel(funcs, parallel=parallel)

    def volt(self, parallel=True, settle_s=0.0, **axes):
        """
        parallel=True: Control multiple axes simultaneously.
        settle_s: Wait for motion done.
        example: volt(Y=12.0, Z=9.5) (V) For OpenLoop only
        """
        
        funcs = {a.upper(): (lambda A=a, v=v: (self.set_voltage(A, float(v), settle_s)))
                 for a, v in axes.items()}
        self._run_parallel(funcs, parallel=parallel)

    def zero(self, *axes, parallel=True, wait_s=5):
        """Set Output to zero only"""
        axes = self._axes_or_all(axes)
        funcs = {
            a: (lambda A=a: (self._ch(A).SetZero(), time.sleep(wait_s)))
            for a in axes
        }
        self._run_parallel(funcs, parallel=parallel)

    def zero_output(self, *axes, parallel=True, wait_s=5):
        """Set all to Zero"""
        axes = self._axes_or_all(axes)
        funcs = {
            a: (lambda A=a: (self._ch(A).SetZeroOutput(), time.sleep(wait_s)))
            for a in axes
        }
        self._run_parallel(funcs, parallel=parallel)

    def pos(self, *axes):
        """pos() / pos('X','Z') -> dict"""
        axes = self._axes_or_all(axes)
        return {a: str(self.get_position(a)) for a in axes}

    def volts(self, *axes):
        axes = self._axes_or_all(axes)
        return {a: str(self.get_voltage(a)) for a in axes}
    
if __name__ == "__main__":
    # Serial number of BPC3xx device.
    # For testing without hardware, use a simulation serial number  and set use_sim.
    serial_number = "71000001"  # Simulation example

    # Create controller instance for X and Z axes only
    dev = BPC3xxController(
        serial=serial_number,
        axes=('X', 'Z'),      # Control only X and Z channels
        use_sim=True          # Enable simulation mode
    )

    # -------- Closed-loop control example (position control) --------
    dev.set_mode("CloseLoop")  # Switch all connected axes to closed-loop mode

    # Move both X and Z to given positions in micrometers, in parallel
    dev.move(X=5.0, Z=1.0,settle_s=5, parallel=True)

    # Zero the output voltages for all active axes, in parallel
    dev.zero_output(parallel=True)
    # dev.zero_output('X') Zero specifiv axies

    # Read the current position of X only
    x_pos = dev.get_position('X')
    print(f"Current X position: {x_pos}")

    # Move both axes again to new positions
    dev.move(X=5.0, Z=4.0, parallel=True)

    # Get current positions of all active axes
    print("All positions:", dev.pos())

    # -------- Open-loop control example (voltage control) --------
    dev.set_mode("OpenLoop")  # Switch to open-loop mode

    # Apply voltages to X and Z channels, in parallel
    dev.volt(X=10.0, Z=8.0, parallel=True)

    # Read back the current output voltages
    print("All voltages:", dev.volts())
    print("X",dev.volts('x'))

    # -------- Clean up --------
    dev.close()
