import serial
import time

class PAMC104Controller:
    def __init__(self, port, baudrate=115200, timeout=1):
        """Initialize connection to PAMC-104 motor controller via serial port."""
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )

    def _send_cmd(self, cmd):
        """Send a command to the controller and return its response."""
        if not self.ser.is_open:
            self.ser.open()
        # Append CR+LF as required by the device protocol
        self.ser.write((cmd + '\r\n').encode())
        time.sleep(0.1)  # Wait for the device to respond
        response = self.ser.read_all().decode(errors='ignore').strip()
        return response

    def connect_check(self):
        """Check if the connection to the controller is successful."""
        return self._send_cmd("CON")

    def drive(self, direction, frequency, pulses, axis):
        """
        Drive the motor.

        Parameters:
            direction (str): 'NR' for clockwise, 'RR' for counterclockwise.
            frequency (int): Frequency in Hz (1~1500).
            pulses (int): Number of pulses (0~9999). Use 0 for continuous drive.
            axis (str): Output channel ('A'~'D'), where: A = Axis1, B = Axis2, C = Axis3, D = Axis4.
        """
        freq_str = f"{frequency:04d}"
        pulse_str = f"{pulses:04d}"
        cmd = f"{direction}{freq_str}{pulse_str}{axis}"
        return self._send_cmd(cmd)

    def stop(self):
        """Stop the current motor operation."""
        return self._send_cmd("S")

    def close(self):
        """Close the serial connection."""
        if self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    controller = PAMC104Controller(port="COM9")

    print("Connection check:", controller.connect_check())
    print("Drive motor:", controller.drive("NR", 1500, 10, "A"))
    time.sleep(2)
    print("Stop:", controller.stop())

    controller.close()
