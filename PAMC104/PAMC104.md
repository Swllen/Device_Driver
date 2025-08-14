# PAMC104Controller - Python Serial Controller for PAMC-104

## 📖 Overview
`PAMC104Controller` is a Python class for controlling the **Mechano Transformer PAMC-104 Piezo Assist Motor Controller** over a serial (RS232) connection.  
It implements the official command protocol, allowing you to:

- Check connection
- Drive the motor clockwise or counterclockwise
- Stop motion
- Close the serial connection safely

---

## ⚙ Requirements

### 1. Python Package
```bash
pip install pyserial
```

### 2. Hardware & Cables
- PAMC-104 Controller
- USB-to-RS232 adapter (driver installed)
- Official SMB cables for motor connection

### 3. Communication Settings
As per the PAMC-104 manual:
```
Baud rate   : 115200
Data bits   : 8
Parity      : None
Stop bits   : 1
Flow control: None
New-line    : CR+LF
```

---

## 🚀 Usage

### 1. Import & Create Controller Instance
```python
from pamc104_controller import PAMC104Controller  # Save the class in pamc104_controller.py

controller = PAMC104Controller(port="COM9")  # Replace with your actual COM port
```

### 2. Check Connection
```python
print("Connection check:", controller.connect_check())
# Expected response: "OK"
```

### 3. Drive the Motor
```python
# Drive Axis1 (A) clockwise at 1500 Hz, 10 pulses
print("Drive motor:", controller.drive("NR", 1500, 10, "A"))

# Parameters:
# direction: "NR" = clockwise, "RR" = counterclockwise
# frequency: 1~1500 Hz
# pulses   : 0~9999 (0 = continuous)
# axis     : "A"=Axis1, "B"=Axis2, "C"=Axis3, "D"=Axis4
```

### 4. Stop Motion
```python
import time
time.sleep(2)  # Let it run for 2 seconds
print("Stop:", controller.stop())
# Expected response: "FIN"
```

### 5. Close the Connection
```python
controller.close()
```

---

## 📌 Example Script
```python
import time
from pamc104_controller import PAMC104Controller

controller = PAMC104Controller(port="COM9")

print("Connection check:", controller.connect_check())
print("Drive motor:", controller.drive("NR", 1500, 10, "A"))
time.sleep(2)
print("Stop:", controller.stop())

controller.close()
```

---

## 💡 Notes
- **Command Summary**:
  | Command       | Description |
  |---------------|-------------|
  | `CON`         | Check connection |
  | `NRxxxxYYYYZ` | Clockwise drive (`NR`) with frequency `xxxx` Hz, pulses `YYYY`, axis `Z` |
  | `RRxxxxYYYYZ` | Counterclockwise drive (`RR`) with frequency `xxxx` Hz, pulses `YYYY`, axis `Z` |
  | `S`           | Stop motion |

- **Continuous Motion**:  
  Set pulses = `0` for continuous drive. Stop manually with `stop()`.

- Always close the serial connection with `close()` after use.
- Only use official cables to avoid damaging connectors.

---



