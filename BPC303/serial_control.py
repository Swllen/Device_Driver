import serial
import struct
import time

CHAN_WORD = {1: 0x0001, 2: 0x0002, 3: 0x0004}
CHAN_IDENT = {1: 0x01, 2: 0x02, 3: 0x04} 

class BPC303_CONTROLLER:
    def __init__(self, port, baudrate=115200, dest=0x11, src=0x01, timeout=0.2):
        self.dest = dest
        self.src = src
        self.ser = serial.Serial(
                                    port,
                                    baudrate=baudrate,             
                                    bytesize=serial.EIGHTBITS,     
                                    parity=serial.PARITY_NONE,     
                                    stopbits=serial.STOPBITS_ONE,  
                                    timeout=timeout,
                                    write_timeout=timeout,
                                    rtscts=True,                   
                                    dsrdtr=False                  
                                )
        
    def _read_exact(self, n, timeout_s=None):
        end = time.time() + (timeout_s if timeout_s is not None else self.ser.timeout or 0.5)
        buf = bytearray()
        while len(buf) < n and time.time() < end:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)
    
    def identify(self,chan:int):
        msg_id = 0x0223
        c = CHAN_IDENT.get(chan)
        frame = struct.pack("<HBBBB",msg_id,c,0x00,self.dest,self.src)
        self.ser.write(frame)

    def set_channel_enable(self, chan: int, enable: bool):
        msg_id = 0x0210 
        c = CHAN_IDENT.get(chan)
        enable_state = 0x01 if enable else 0x02
        frame = struct.pack("<HBBBB",msg_id,c,enable_state,self.dest,self.src)
        self.ser.wirte(frame)

    def set_pos_control_mode(self, chan: int, closed_loop: bool):
        msg_id = 0x0640
        c = CHAN_IDENT.get(chan)
        mode = 0x02 if closed_loop else 0x01
        frame = struct.pack("<HBBBB",msg_id,c,mode,self.dest,self.src)
        self.ser.write(frame)

    def request_pos_control_mode(self,chan:int):
        msg_id = 0x0641
        c = CHAN_IDENT.get(chan)
        frame = struct.pack("<HBBBB",msg_id,c,0x00,self.dest,self.src)
        self.ser.write(frame)

    def get_pos_control_mode(self):
        hdr = self._read_exact(6)
        if len(hdr) != 6 or hdr[0] != 0x42 or hdr[1] != 0x06:
            return None
        chan_ident, mode = struct.unpack("B<B", hdr[2:4])
        chan = {v: k for k, v in CHAN_IDENT.items()}.get(chan_ident, 0)
        mode_str = 'OpenLoop' if mode == 0x01 else 'CloseLoop'
        return chan, mode_str

    def set_output_voltage_percent(self, chan: int, percent: float):
        msg_id = 0x0643
        c = CHAN_WORD.get(chan)
        pct = max(-100.0, min(100.0, percent))
        val = int(round(pct / 100.0 * 32767))
        header = struct.pack("<HHBB", msg_id, 4, self.dest | 0x80, self.src)
        data = struct.pack("<Hh", c, val)
        frame = header + data
        self.ser.write(frame)

    def set_output_voltage_volts(self, chan: int, volts: float, vmax: float):
        self.set_output_voltage_percent(chan, volts / float(vmax) * 100.0)

    def request_and_get_output_voltage(self,chan:int):
        msg_id = 0x0644
        c = chan_word.get(chan)
        frame = struct.pack("<HBBBB",msg_id,c,0x00,self.dest,self.src)
        self.ser.write(frame)

        hdr = self._read_exact(10)
        if len(hdr) != 10 or hdr[0] != 0x45 or hdr[1] != 0x06:
            return None
        data = hdr[6:]
        chan_word, val = struct.unpack("<Hh", data[:4])
        if c == chan_word:
            percent = val / 32767.0 * 100.0
        return percent
    
    def get_output_voltage(self,chan:int,Vmax:int):
        v = Vmax * self.request_and_get_output_voltage(chan)
        return v
    # ---------- 位置（闭环） ----------
    def set_output_position_percent(self, chan: int, percent: float):
        msg_id = 0x0646
        c = CHAN_WORD.get(chan)
        if c is None:
            raise ValueError("通道号必须是 1/2/3")
        pct = max(-100.0, min(100.0, percent))
        val = int(round(pct / 100.0 * 32767))
        
        header = struct.pack("<HHBB",msg_id, 4, self.dest | 0x80, self.src)
        data = struct.pack("<Hh", c, val)
        frame = header + data
        self.ser.write(frame)

    def request_and_get_output_position(self,chan:int):
        msg_id = 0x0647
        c = chan_word.get(chan)
        frame = struct.pack("<HBBBB",msg_id,c,0x00,self.dest,self.src)
        self.ser.write(frame)

        hdr = self._read_exact(10)
        if len(hdr) != 10 or hdr[0] != 0x48 or hdr[1] != 0x06:
            return None
        data = hdr[6:]
        chan_word, val = struct.unpack("<Hh", data[:4])
        if c == chan_word:
            percent = val / 32767 * 100.0
        return percent
    
    def get_output_pos(self,chan:int,Pmax:int):
        return Pmax * self.request_and_get_output_position(chan)
    
    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass
