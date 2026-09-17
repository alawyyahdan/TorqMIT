import sys
import os
import time
import struct
import threading
import csv
from datetime import datetime
import serial
import serial.tools.list_ports
from collections import deque
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QLabel, QLineEdit, QPushButton,
                               QComboBox, QRadioButton, QGroupBox, QTextEdit,
                               QSplitter, QGridLayout, QFrame, QScrollArea,
                               QCheckBox, QSpinBox, QButtonGroup, QMessageBox)
from PySide6.QtCore import QTimer, Qt, Signal, QObject
from PySide6.QtGui import QShortcut, QKeySequence
import pyqtgraph as pg
import numpy as np

try:
    import can
    HAS_CAN = True
except ImportError:
    HAS_CAN = False

# ==========================================
# UART SERVO MODE PROTOCOL (Driver Manual V1.0.18)
# Frame: [0x02][DataLen][DataFrame][CRC_HI][CRC_LO][0x03]
# ==========================================
def crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def build_packet(payload: bytes) -> bytes:
    packet = bytearray([0x02, len(payload)])
    packet.extend(payload)
    c = crc16(payload)
    packet.extend([(c >> 8) & 0xFF, c & 0xFF, 0x03])
    return bytes(packet)

# UART Servo Mode Command IDs (manual section 5.2.2)
COMM_GET_VALUES   = 4
COMM_SET_CURRENT  = 6
COMM_SET_RPM      = 8
COMM_SET_POS      = 9
COMM_SET_POS_SPD  = 91
COMM_ALIVE        = 30

# CAN Servo Mode Control IDs (section 5.1, extended frame bits[28:8])
CAN_PACKET_SET_DUTY          = 0
CAN_PACKET_SET_CURRENT       = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM           = 3
CAN_PACKET_SET_POS           = 4
CAN_PACKET_SET_ORIGIN_HERE   = 5
CAN_PACKET_SET_POS_SPD       = 6

# ==========================================
# AK40-10 DATASHEET CONSTANTS
# ==========================================
KT         = 0.056    # Nm/A (torque constant, pre-gearbox)
GEAR_RATIO = 10.0
GEAR_EFF   = 0.86
EFF_KT     = KT * GEAR_RATIO * GEAR_EFF  # 0.482 Nm/A at output shaft
POLE_PAIRS = 14
MAX_CURRENT = 7.3     # A peak (datasheet)
MAX_TORQUE  = 4.1     # Nm peak (datasheet)
BUF_SIZE    = 600     # ~30s at 20Hz

# ==========================================
# MIT MODE CONSTANTS (AK40-10, docs section 5.3)
# ==========================================
MIT_P_MIN  = -12.5    # rad
MIT_P_MAX  =  12.5
MIT_V_MIN  = -45.5    # rad/s
MIT_V_MAX  =  45.5
MIT_T_MIN  = -5.0     # Nm
MIT_T_MAX  =  5.0
MIT_KP_MIN =  0.0
MIT_KP_MAX =  500.0
MIT_KD_MIN =  0.0
MIT_KD_MAX =  5.0

# MIT special CAN commands
MIT_CMD_ENTER = bytes([0xFF]*7 + [0xFC])
MIT_CMD_EXIT  = bytes([0xFF]*7 + [0xFD])
MIT_CMD_ZERO  = bytes([0xFF]*7 + [0xFE])

# ==========================================
# MIT PROTOCOL HELPERS
# ==========================================
def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return float(x_int) * span / float((1 << bits) - 1) + x_min

def mit_pack_cmd(p_des, v_des, kp, kd, t_ff):
    p_int  = float_to_uint(p_des, MIT_P_MIN,  MIT_P_MAX,  16)
    v_int  = float_to_uint(v_des, MIT_V_MIN,  MIT_V_MAX,  12)
    kp_int = float_to_uint(kp,    MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_int = float_to_uint(kd,    MIT_KD_MIN, MIT_KD_MAX, 12)
    t_int  = float_to_uint(t_ff,  MIT_T_MIN,  MIT_T_MAX,  12)
    data = bytearray(8)
    data[0] = (p_int >> 8) & 0xFF
    data[1] = p_int & 0xFF
    data[2] = (v_int >> 4) & 0xFF
    data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    data[4] = kp_int & 0xFF
    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    data[7] = t_int & 0xFF
    return bytes(data)

def mit_unpack_reply(data):
    if len(data) < 8:
        return None
    motor_id = data[0]
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5]
    temp  = data[6]
    error = data[7]
    pos    = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16)
    speed  = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12)
    torque = uint_to_float(t_int, -MIT_T_MAX, MIT_T_MAX, 12)
    temperature = temp - 40
    return (motor_id, pos, speed, torque, temperature, error)

def servo_can_parse_feedback(arb_id, data):
    """Parse servo mode CAN extended frame feedback (section 5.2.1).
    Frame 0x29: real-time servo feedback.
    Extended ID = (function_id << 8) | motor_id
    """
    motor_id = arb_id & 0xFF
    func_id  = (arb_id >> 8) & 0xFF
    if func_id != 0x29 or len(data) < 8:
        return None
    pos_int = (data[0] << 8) | data[1]         # int16
    spd_int = (data[2] << 8) | data[3]         # int16
    cur_int = (data[4] << 8) | data[5]         # int16
    temp    = data[6]                           # int8 (signed)
    err     = data[7]                           # uint8
    # Convert from int16 (signed)
    if pos_int > 32767: pos_int -= 65536
    if spd_int > 32767: spd_int -= 65536
    if cur_int > 32767: cur_int -= 65536
    if temp > 127: temp -= 256
    position = pos_int * 0.1       # degrees
    speed    = spd_int * 10.0      # ERPM
    current  = cur_int * 0.01      # A
    return (motor_id, position, speed, current, temp, err)

def get_can_interfaces():
    interfaces = []
    try:
        for iface in os.listdir('/sys/class/net/'):
            if iface.startswith('can') or iface.startswith('vcan'):
                interfaces.append(iface)
    except:
        pass
    return sorted(interfaces)

# ==========================================
# SIGNALS BRIDGE
# ==========================================
class Signals(QObject):
    log_signal = Signal(str)

# ==========================================
# MOTOR CONTROLLER - UART SERVO MODE
# ==========================================
class Motor:
    def __init__(self, log_cb=None):
        self.ser = None
        self.connected = False
        self.simulation = False
        self.running = False
        self._thread = None
        self._lock = threading.RLock()
        self.log_cb = log_cb
        self.active = False
        self.target_current = 0.0
        self.target_torque = 0.0
        self.des_p = 0.0
        self.des_s = 5000.0
        self.des_a = 30000.0
        self.ramp_rate = 1.0
        self.ramped_current = 0.0
        self.fb_current = 0.0
        self.fb_pos = 0.0
        self.fb_speed = 0.0
        self.fb_mos_temp = 0.0
        self.fb_motor_temp = 0.0
        self.fb_voltage = 0.0
        self.fb_error = 0
        self.fb_torque = 0.0
        self._sim_i = 0.0

    def connect(self, port, baudrate=921600):
        if port == "Simulation":
            self.simulation = True
            self.connected = True
            self._start()
            self._log("Connected [Simulation Mode]")
            return True
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            self.ser.reset_input_buffer()
            self.simulation = False
            self.connected = True
            self._start()
            self._log(f"Connected to {port} @ {baudrate}")
            return True
        except Exception as e:
            self._log(f"ERROR: {e} -> Falling back to Simulation")
            self.simulation = True
            self.connected = True
            self._start()
            return False

    def disconnect(self):
        self.estop()
        self.running = False
        if self._thread: self._thread.join(timeout=1.5)
        with self._lock:
            if self.ser:
                try: self.ser.close()
                except: pass
                self.ser = None
        self.connected = False
        self._log("Disconnected")

    def estop(self):
        with self._lock:
            self.active = False
            self.target_current = 0.0
            self.target_torque = 0.0
            self.ramped_current = 0.0
        if not self.simulation and self.ser:
            try: self._write_current(0.0)
            except: pass
        self._log("!! E-STOP !!")

    def set_current(self, amps):
        amps = max(-MAX_CURRENT, min(MAX_CURRENT, amps))
        tau = amps * EFF_KT
        with self._lock:
            self.target_current = amps
            self.target_torque = tau
            self.active = True
        self._log(f"CMD Current: {amps:.3f} A  (tau: {tau:.3f} Nm)")

    def set_torque(self, nm):
        nm = max(-MAX_TORQUE, min(MAX_TORQUE, nm))
        cur = nm / EFF_KT if EFF_KT > 0 else 0.0
        cur = max(-MAX_CURRENT, min(MAX_CURRENT, cur))
        with self._lock:
            self.target_current = cur
            self.target_torque = nm
            self.active = True
        self._log(f"CMD Torque: {nm:.3f} Nm -> I: {cur:.3f} A")

    def stop_command(self):
        with self._lock:
            self.active = False
            self.target_current = 0.0
            self.target_torque = 0.0
            self.ramped_current = 0.0

    def _start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        DT = 0.025
        while self.running:
            t0 = time.time()
            if self.simulation: self._sim(DT)
            else: self._hw_cycle()
            rem = DT - (time.time() - t0)
            if rem > 0: time.sleep(rem)

    def _sim(self, dt):
        with self._lock:
            active = self.active; tgt_i = self.target_current; rr = self.ramp_rate
        if active:
            step = rr * dt
            diff = tgt_i - self._sim_i
            if abs(diff) < step: self._sim_i = tgt_i
            else: self._sim_i += step if diff > 0 else -step
        else:
            step = rr * dt
            if abs(self._sim_i) < step: self._sim_i = 0.0
            else: self._sim_i -= step if self._sim_i > 0 else -step
        noise_i = np.random.normal(0, 0.015)
        with self._lock:
            self.fb_current = round(self._sim_i + noise_i, 4)
            self.fb_speed = round(self._sim_i * 10000, 1)
            self.fb_torque = self.fb_current * EFF_KT
            self.fb_pos = 0.0
            self.fb_mos_temp = 34.0; self.fb_motor_temp = 36.0
            self.fb_voltage = 24.0; self.fb_error = 0

    def _hw_cycle(self):
        try:
            pkt = build_packet(bytes([COMM_GET_VALUES]))
            with self._lock: self.ser.write(pkt); self.ser.flush()
            self._log_hex("TX", pkt, "GET_VALUES")
            resp = self._read_frame()
            if resp:
                self._log_hex("RX", resp, "")
                self._parse(resp)
            with self._lock:
                active = self.active; tgt_i = self.target_current; rr = self.ramp_rate
            if active:
                step = rr * 0.025
                diff = tgt_i - self.ramped_current
                if abs(diff) < step: self.ramped_current = tgt_i
                else: self.ramped_current += step if diff > 0 else -step
                self._write_current(self.ramped_current)
            else:
                hb = build_packet(bytes([COMM_ALIVE]))
                with self._lock: self.ser.write(hb); self.ser.flush()
        except Exception as e:
            self._log(f"COMM ERR: {e}")

    def _write_current(self, amps):
        payload = bytearray([COMM_SET_CURRENT])
        payload.extend(int(amps * 1000).to_bytes(4, 'big', signed=True))
        pkt = build_packet(bytes(payload))
        with self._lock:
            if self.ser: self.ser.write(pkt); self.ser.flush()
        self._log_hex("TX", pkt, f"SET_I {amps:.3f}A")

    def _read_frame(self):
        buf = bytearray()
        t0 = time.time()
        while (time.time() - t0) < 0.022:
            with self._lock: w = self.ser.in_waiting if self.ser else 0
            if w > 0:
                with self._lock: b = self.ser.read(1)
                if not buf:
                    if b[0] == 0x02: buf.extend(b)
                    continue
                buf.extend(b)
                if len(buf) >= 2 and len(buf) == buf[1] + 5:
                    if buf[-1] == 0x03: return bytes(buf)
                    buf.clear()
        return None

    def _parse(self, data):
        if len(data) < 6: return
        payload = data[2:-3]
        if len(payload) < 2: return
        cmd_id = payload[0]; payload = payload[1:]
        if cmd_id != COMM_GET_VALUES: return
        if len(payload) < 53: return
        try:
            with self._lock:
                self.fb_mos_temp = struct.unpack('>h', payload[0:2])[0] / 10.0
                self.fb_motor_temp = struct.unpack('>h', payload[2:4])[0] / 10.0
                fb_phase = struct.unpack('>i', payload[4:8])[0] / 100.0
                if len(payload) >= 20:
                    self.fb_current = struct.unpack('>i', payload[16:20])[0] / 100.0
                else:
                    self.fb_current = fb_phase
                self.fb_torque = self.fb_current * EFF_KT
                self.fb_speed = struct.unpack('>i', payload[22:26])[0]
                self.fb_voltage = struct.unpack('>h', payload[26:28])[0] / 10.0
                self.fb_error = payload[52]
                if len(payload) >= 57:
                    self.fb_pos = struct.unpack('>i', payload[53:57])[0] / 1000000.0
        except: pass

    def _log(self, msg):
        if self.log_cb: self.log_cb(f"[{time.strftime('%H:%M:%S')}] {msg}")
    def _log_hex(self, direction, data, note):
        if self.log_cb:
            hx = " ".join(f"{b:02X}" for b in data[:20])
            if len(data) > 20: hx += " ..."
            self.log_cb(f"[{time.strftime('%H:%M:%S')}] {direction} ({len(data)}B) {hx}  {note}")


# ==========================================
# MOTOR CONTROLLER - CAN BUS
# Supports both Servo Mode (extended frame) and MIT Mode (standard frame)
# ==========================================
class MotorCAN:
    """Controls AK40-10 via CAN bus.

    Two CAN modes:
    SERVO: Extended frames. Current loop = same as UART but via CAN.
           CAN ID = (control_mode << 8) | motor_id
           Feedback on extended ID = 0x2900 | motor_id

    MIT:   Standard frames. Torque equation with damping:
           tau = kp*(p_des-p) + kd*(v_des-v) + t_ff
           Send on standard ID = motor_id
           Reply on standard ID with data[0] = motor_id
    """
    MODE_SERVO = "SERVO"
    MODE_MIT   = "MIT"

    def __init__(self, log_cb=None):
        self.bus = None
        self.connected = False
        self.simulation = False
        self.running = False
        self._thread = None
        self._lock = threading.RLock()
        self.log_cb = log_cb

        self.can_id = 2          # AK40-10 default = 2
        self.can_mode = self.MODE_MIT
        self.active = False

        # targets
        self.target_current = 0.0  # A
        self.target_torque  = 0.0  # Nm

        # MIT-specific targets
        self.target_p   = 0.0
        self.target_v   = 0.0
        self.target_kp  = 0.0
        self.target_kd  = 0.5
        self.target_tff = 0.0

        # ramp
        self.ramp_rate = 0.5     # Nm/s or A/s depending on mode
        self.ramped_value = 0.0  # ramped current (servo) or tff (MIT)

        # feedback
        self.fb_pos        = 0.0
        self.fb_speed      = 0.0   # ERPM for servo, rad/s for MIT
        self.fb_torque     = 0.0
        self.fb_current    = 0.0
        self.fb_motor_temp = 0.0
        self.fb_mos_temp   = 0.0
        self.fb_voltage    = 0.0
        self.fb_error      = 0

        self._tx_errors = 0
        self._last_log_t = 0
        self._rx_count = 0

        # serial monitor
        self.ser_mon = None
        self._ser_mon_thread = None
        self._ser_mon_running = False

        # sim state
        self._sim_v = 0.0; self._sim_p = 0.0; self._sim_i = 0.0
        self._SIM_J = 0.02

    def connect(self, channel, can_id=0, mode="MIT"):
        """Connect to CAN bus.
        can_id=0 = auto-detect from servo broadcast.
        Always tries to switch to MIT mode and verifies."""
        self.can_mode = self.MODE_MIT  # target
        self.can_id = can_id
        self._tx_errors = 0
        self._rx_count = 0
        self._connect_log = []  # collect log for popup

        if channel == "Simulation":
            self.simulation = True
            self.connected = True
            if can_id == 0: self.can_id = 1
            self._start()
            self._log("CAN Connected [Simulation - MIT]")
            return True
        if not HAS_CAN:
            self._log("ERROR: python-can not installed! pip install python-can")
            self.simulation = True
            self.connected = True
            if can_id == 0: self.can_id = 1
            self._start()
            return False
        try:
            # Auto bring-up CAN interface at 1Mbps
            import subprocess
            self._clog(f"Bringing up {channel} at 1Mbps...")
            # Try to bring down first (needed to change bitrate)
            subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'down'],
                           capture_output=True, timeout=3)
            # Set bitrate and bring up
            subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'type', 'can', 'bitrate', '1000000'],
                           capture_output=True, timeout=3)
            result = subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'up'],
                                    capture_output=True, timeout=3)
            if result.returncode == 0:
                self._clog(f"  {channel} UP at 1Mbps")
            else:
                err = result.stderr.decode().strip()
                self._clog(f"  ip link up warning: {err}")
            # Set TX queue length
            subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'txqueuelen', '100'],
                           capture_output=True, timeout=2)

            self.bus = can.interface.Bus(
                channel=channel, interface='socketcan', bitrate=1000000)

            self.simulation = False
            self.connected = True
            self._clog(f"CAN bus opened: {channel}")

            # ===== STEP 1: Auto-detect motor CAN ID =====
            if can_id == 0:
                self._clog("[1/4] Scanning for motor...")
                detected = self._auto_detect()
                if detected:
                    self.can_id = detected
                    self._clog(f"  FOUND motor CAN ID = {detected}")
                else:
                    self.can_id = 1
                    self._clog("  No motor found, defaulting to ID=1")
            else:
                self.can_id = can_id
                self._clog(f"[1/4] Manual CAN ID = {can_id}")

            # ===== STEP 2: Exit any existing mode first =====
            self._clog(f"[2/4] Resetting motor {self.can_id}...")
            self._flush_bus()
            # Send EXIT first to clean slate
            self._raw_send_std(self.can_id, MIT_CMD_EXIT)
            self._clog("  Sent MIT EXIT (clean slate)")
            time.sleep(0.1)
            self._flush_bus()

            # ===== STEP 3: Enter MIT mode (aggressive) =====
            self._clog(f"[3/4] Entering MIT mode (ID={self.can_id})...")
            mit_ok = False
            for attempt in range(5):
                self._flush_bus()
                self._raw_send_std(self.can_id, MIT_CMD_ENTER)
                self._clog(f"  TX MIT_ENTER attempt {attempt+1}/5")
                time.sleep(0.05)
                # Read all replies, skip echoes
                for _ in range(10):
                    reply = self.bus.recv(timeout=0.05)
                    if reply is None: break
                    # Skip echoes (our own MIT_ENTER reflected back)
                    if reply.data == MIT_CMD_ENTER: continue
                    if reply.data == MIT_CMD_EXIT: continue
                    hx = " ".join(f"{b:02X}" for b in reply.data)
                    ext = "EXT" if reply.is_extended_id else "STD"
                    self._clog(f"  RX (0x{reply.arbitration_id:X} {ext}) {hx}")
                    # MIT reply = standard frame, data[0] = motor_id
                    if not reply.is_extended_id and len(reply.data) >= 6:
                        if reply.data[0] == self.can_id:
                            mit_ok = True
                            self._clog(f"  >>> Motor {self.can_id} entered MIT mode! <<<")
                            break
                if mit_ok: break
                time.sleep(0.05)

            # ===== STEP 4: Verify with zero-torque command =====
            verified = False
            if mit_ok:
                self._clog(f"[4/4] Verifying MIT with zero-torque cmd...")
                verified = self._verify_mit()
                if verified:
                    self._clog("  >>> MIT VERIFIED - motor responding! <<<")
                else:
                    self._clog("  MIT ENTER got reply but verify failed")

            # ===== Determine result =====
            if verified:
                self.can_mode = self.MODE_MIT
                self._clog("")
                self._clog("=============================")
                self._clog("  RESULT: MIT MODE ACTIVE")
                self._clog(f"  Motor ID: {self.can_id}")
                self._clog("=============================")
            elif mit_ok:
                self.can_mode = self.MODE_MIT
                self._clog("")
                self._clog("=============================")
                self._clog("  RESULT: MIT MODE (unverified)")
                self._clog("  Got ENTER reply but verify")
                self._clog("  failed. Trying anyway.")
                self._clog("=============================")
            else:
                # Check if servo still alive
                servo_alive = self._check_servo_alive()
                if servo_alive:
                    self.can_mode = self.MODE_SERVO
                    self._clog("")
                    self._clog("=============================")
                    self._clog("  RESULT: SERVO MODE (fallback)")
                    self._clog("  Motor did NOT switch to MIT.")
                    self._clog("  Possible causes:")
                    self._clog("  - Motor firmware is servo-only")
                    self._clog("  - Need CubeMarsTool to switch")
                    self._clog("  - Try power cycling motor")
                    self._clog("  Current loop will work but")
                    self._clog("  NO damping (speed not limited)")
                    self._clog("=============================")
                else:
                    self.can_mode = self.MODE_MIT
                    self._clog("")
                    self._clog("=============================")
                    self._clog("  RESULT: NO REPLY FROM MOTOR")
                    self._clog("  Check wiring / power / CAN ID")
                    self._clog("  Trying MIT mode anyway...")
                    self._clog("=============================")

            # Flush connect log to monitor
            for line in self._connect_log:
                self._log(line)

            self._start()
            return True
        except Exception as e:
            self._clog(f"CAN ERROR: {e}")
            for line in self._connect_log:
                self._log(line)
            self.simulation = True
            self.connected = True
            if can_id == 0: self.can_id = 1
            self._start()
            return False

    def get_connect_result(self):
        """Return connect log for popup display."""
        return "\n".join(self._connect_log) if hasattr(self, '_connect_log') else ""

    def _clog(self, msg):
        """Log to both connect_log buffer and main log."""
        if not hasattr(self, '_connect_log'):
            self._connect_log = []
        self._connect_log.append(msg)

    def _raw_send_std(self, can_id, data):
        """Send a standard CAN frame."""
        try:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg, timeout=0.01)
        except Exception as e:
            self._clog(f"  TX error: {e}")

    def _enter_mit_mode(self):
        """Legacy - now handled in connect."""
        return False

    def _verify_mit(self):
        """Send a zero-torque MIT command and check for MIT-format reply."""
        try:
            # Send safe zero command: kp=0, kd=0.5, tff=0
            data = mit_pack_cmd(0.0, 0.0, 0.0, 0.5, 0.0)
            msg = can.Message(arbitration_id=self.can_id,
                              data=data, is_extended_id=False)
            self.bus.send(msg, timeout=0.01)
            # Look for MIT reply
            for _ in range(5):
                reply = self.bus.recv(timeout=0.05)
                if reply is None: continue
                if not reply.is_extended_id and len(reply.data) >= 6:
                    parsed = mit_unpack_reply(reply.data)
                    if parsed and parsed[0] == self.can_id:
                        mid, pos, spd, torq, temp, err = parsed
                        self._log(f"  MIT verify: pos={pos:.2f}rad spd={spd:.2f}rad/s "
                                  f"tau={torq:.3f}Nm T={temp:.0f}C err={err}")
                        return True
                elif reply.is_extended_id:
                    # Still getting servo frames = not switched yet
                    func = (reply.arbitration_id >> 8) & 0xFF
                    mid = reply.arbitration_id & 0xFF
                    self._log(f"  Got servo frame (func=0x{func:02X} motor={mid}) - not MIT")
        except Exception as e:
            self._log(f"  Verify error: {e}")
        return False

    def _check_servo_alive(self):
        """Check if servo broadcast frames are still coming."""
        t0 = time.time()
        while (time.time() - t0) < 0.3:
            msg = self.bus.recv(timeout=0.1)
            if msg and msg.is_extended_id:
                func = (msg.arbitration_id >> 8) & 0xFF
                if func == 0x29:
                    return True
        return False

    def _flush_bus(self):
        """Read and discard all pending messages."""
        while True:
            msg = self.bus.recv(timeout=0.01)
            if msg is None: break

    def _auto_detect(self):
        """Listen to CAN bus for ~0.5s and find motor IDs from servo feedback."""
        found_ids = set()
        t0 = time.time()
        while (time.time() - t0) < 0.5:
            msg = self.bus.recv(timeout=0.1)
            if msg is None: continue
            aid = msg.arbitration_id
            if msg.is_extended_id:
                func_id = (aid >> 8) & 0xFF
                motor_id = aid & 0xFF
                if func_id == 0x29 and len(msg.data) == 8:
                    found_ids.add(motor_id)
                    parsed = servo_can_parse_feedback(aid, msg.data)
                    if parsed:
                        mid, pos, spd, cur, tmp, err = parsed
                        self._log(f"  Found motor ID={mid}: "
                                  f"pos={pos:.1f}deg spd={spd:.0f}ERPM "
                                  f"I={cur:.2f}A T={tmp}C err={err}")
            else:
                # MIT mode reply: standard frame, data[0] = motor_id
                if len(msg.data) >= 8:
                    motor_id = msg.data[0]
                    if 0 < motor_id < 128:
                        found_ids.add(motor_id)
                        self._log(f"  Found motor ID={motor_id} (MIT frame)")
        if found_ids:
            return min(found_ids)  # return lowest ID
        return None

    def disconnect(self):
        self.estop()
        if self.can_mode == self.MODE_MIT and not self.simulation and self.bus:
            self._send_mit_special(MIT_CMD_EXIT)
            self._log("MIT EXIT sent")
        self.running = False
        if self._thread: self._thread.join(timeout=1.5)
        self.stop_serial_monitor()
        with self._lock:
            if self.bus:
                try: self.bus.shutdown()
                except: pass
                self.bus = None
        self.connected = False
        self._log("CAN Disconnected")

    def estop(self):
        with self._lock:
            self.active = False
            self.target_current = 0.0
            self.target_torque = 0.0
            self.target_tff = 0.0
            self.ramped_value = 0.0
        if not self.simulation and self.bus:
            try:
                if self.can_mode == self.MODE_MIT:
                    data = mit_pack_cmd(0.0, 0.0, 0.0, self.target_kd, 0.0)
                    msg = can.Message(arbitration_id=self.can_id,
                                      data=data, is_extended_id=False)
                else:
                    # Servo: send 0 current
                    eid = (CAN_PACKET_SET_CURRENT << 8) | self.can_id
                    data = int(0).to_bytes(4, 'big', signed=True)
                    msg = can.Message(arbitration_id=eid,
                                      data=data, is_extended_id=True)
                self.bus.send(msg)
            except: pass
        self._log("!! E-STOP !!")

    def set_current(self, amps):
        amps = max(-MAX_CURRENT, min(MAX_CURRENT, amps))
        nm = amps * EFF_KT
        nm = max(MIT_T_MIN, min(MIT_T_MAX, nm))
        with self._lock:
            self.target_current = amps
            self.target_torque = nm
            self.target_tff = nm
            self.active = True
        mode_s = self.can_mode
        self._log(f"CAN {mode_s} Current: {amps:.3f} A -> {nm:.3f} Nm")

    def set_torque(self, nm):
        nm = max(MIT_T_MIN, min(MIT_T_MAX, nm))
        cur = nm / EFF_KT if EFF_KT > 0 else 0.0
        cur = max(-MAX_CURRENT, min(MAX_CURRENT, cur))
        with self._lock:
            self.target_current = cur
            self.target_torque = nm
            self.target_tff = nm
            self.active = True
        self._log(f"CAN {self.can_mode} Torque: {nm:.3f} Nm (I={cur:.3f}A)")

    def stop_command(self):
        with self._lock:
            self.active = False
            self.target_current = 0.0
            self.target_torque = 0.0
            self.target_tff = 0.0
            self.ramped_value = 0.0

    def set_mit_params(self, kp, kd, p_des, v_des):
        with self._lock:
            self.target_kp = max(MIT_KP_MIN, min(MIT_KP_MAX, kp))
            self.target_kd = max(MIT_KD_MIN, min(MIT_KD_MAX, kd))
            self.target_p  = max(MIT_P_MIN, min(MIT_P_MAX, p_des))
            self.target_v  = max(MIT_V_MIN, min(MIT_V_MAX, v_des))

    # -- serial monitor --
    def start_serial_monitor(self, port, baudrate=921600):
        if self._ser_mon_running: self.stop_serial_monitor()
        try:
            self.ser_mon = serial.Serial(port, baudrate, timeout=0.1)
            self.ser_mon.reset_input_buffer()
            self._ser_mon_running = True
            self._ser_mon_thread = threading.Thread(
                target=self._serial_monitor_loop, daemon=True)
            self._ser_mon_thread.start()
            self._log(f"Serial monitor: {port}")
        except Exception as e:
            self._log(f"Serial monitor error: {e}")

    def stop_serial_monitor(self):
        self._ser_mon_running = False
        if self._ser_mon_thread:
            self._ser_mon_thread.join(timeout=1.0); self._ser_mon_thread = None
        if self.ser_mon:
            try: self.ser_mon.close()
            except: pass
            self.ser_mon = None

    def _serial_monitor_loop(self):
        while self._ser_mon_running and self.ser_mon:
            try:
                if self.ser_mon.in_waiting > 0:
                    data = self.ser_mon.read(self.ser_mon.in_waiting)
                    hx = " ".join(f"{b:02X}" for b in data[:30])
                    if len(data) > 30: hx += " ..."
                    self._log(f"UART_MON ({len(data)}B) {hx}")
                else:
                    time.sleep(0.05)
            except: break

    # -- internal --
    def _start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        DT = 0.01  # 100Hz
        while self.running:
            t0 = time.time()
            if self.simulation:
                self._sim(DT)
            elif self.can_mode == self.MODE_MIT:
                self._hw_cycle_mit(DT)
            else:
                self._hw_cycle_servo(DT)
            rem = DT - (time.time() - t0)
            if rem > 0: time.sleep(rem)

    # ---- SERVO MODE CAN ----
    def _hw_cycle_servo(self, dt):
        try:
            with self._lock:
                active = self.active
                tgt_i = self.target_current
                rr = self.ramp_rate

            # Ramp current
            if active:
                step = rr * dt
                diff = tgt_i - self.ramped_value
                if abs(diff) < step: self.ramped_value = tgt_i
                else: self.ramped_value += step if diff > 0 else -step
                send_i = self.ramped_value
            else:
                step = rr * dt
                if abs(self.ramped_value) < step: self.ramped_value = 0.0
                else: self.ramped_value -= step if self.ramped_value > 0 else -step
                send_i = self.ramped_value

            # Send current command (extended frame)
            eid = (CAN_PACKET_SET_CURRENT << 8) | self.can_id
            val = int(send_i * 1000)  # mA
            data = val.to_bytes(4, 'big', signed=True)
            msg = can.Message(arbitration_id=eid, data=data, is_extended_id=True)
            try:
                self.bus.send(msg, timeout=0.005)
                self._tx_errors = 0
            except can.CanOperationError as e:
                self._tx_errors += 1
                if self._tx_errors <= 3 or self._tx_errors % 100 == 0:
                    self._log(f"CAN TX err ({self._tx_errors}x): {e}")
                time.sleep(0.005)
                return

            # Log
            now = time.time()
            if (now - self._last_log_t) >= 1.0:
                self._last_log_t = now
                self._log(f"SERVO TX (EID=0x{eid:X}) I={send_i:.3f}A  "
                          f"RX={self._rx_count}")
                self._rx_count = 0

            # Read feedback (extended frame 0x29xx)
            reply = self.bus.recv(timeout=0.008)
            if reply and reply.is_extended_id and len(reply.data) == 8:
                parsed = servo_can_parse_feedback(reply.arbitration_id, reply.data)
                if parsed:
                    mid, pos, spd, cur, tmp, err = parsed
                    with self._lock:
                        self.fb_pos = pos
                        self.fb_speed = spd
                        self.fb_current = cur
                        self.fb_torque = cur * EFF_KT
                        self.fb_motor_temp = tmp
                        self.fb_mos_temp = tmp
                        self.fb_error = err
                    self._rx_count += 1

        except Exception as e:
            self._log(f"CAN ERR: {e}")
            time.sleep(0.02)

    # ---- MIT MODE CAN ----
    def _hw_cycle_mit(self, dt):
        try:
            with self._lock:
                active = self.active
                tgt_tff = self.target_tff
                rr = self.ramp_rate
                kp = self.target_kp
                kd = self.target_kd
                p_des = self.target_p
                v_des = self.target_v

            # Ramp torque
            if active:
                step = rr * dt
                diff = tgt_tff - self.ramped_value
                if abs(diff) < step: self.ramped_value = tgt_tff
                else: self.ramped_value += step if diff > 0 else -step
                tff_send = self.ramped_value
            else:
                step = rr * dt
                if abs(self.ramped_value) < step: self.ramped_value = 0.0
                else: self.ramped_value -= step if self.ramped_value > 0 else -step
                tff_send = self.ramped_value
                kp = 0.0; p_des = 0.0; v_des = 0.0

            data = mit_pack_cmd(p_des, v_des, kp, kd, tff_send)
            msg = can.Message(arbitration_id=self.can_id,
                              data=data, is_extended_id=False)
            try:
                self.bus.send(msg, timeout=0.005)
                self._tx_errors = 0
            except can.CanOperationError as e:
                self._tx_errors += 1
                if self._tx_errors <= 3 or self._tx_errors % 100 == 0:
                    self._log(f"CAN TX err ({self._tx_errors}x): {e}")
                time.sleep(0.005)
                return

            now = time.time()
            if (now - self._last_log_t) >= 1.0:
                self._last_log_t = now
                hx = " ".join(f"{b:02X}" for b in data)
                self._log(f"MIT TX (ID={self.can_id}) {hx}  "
                          f"tff={tff_send:.3f} kd={kd:.2f} RX={self._rx_count}")
                self._rx_count = 0

            # Read ALL replies, skip echoes (MCP2515 reflects our own frames)
            for _ in range(5):
                reply = self.bus.recv(timeout=0.005)
                if reply is None:
                    break
                # Skip echo of our own command
                if reply.data == data:
                    continue
                # Skip MIT special command echoes
                if reply.data in (MIT_CMD_ENTER, MIT_CMD_EXIT, MIT_CMD_ZERO):
                    continue
                # Skip extended (servo) frames
                if reply.is_extended_id:
                    continue
                # Parse MIT reply: data[0] = motor_id
                if len(reply.data) >= 6 and reply.data[0] == self.can_id:
                    parsed = mit_unpack_reply(reply.data)
                    if parsed:
                        mid, pos, spd, torq, temp, err = parsed
                        with self._lock:
                            self.fb_pos = pos
                            self.fb_speed = spd
                            self.fb_torque = torq
                            self.fb_current = torq / EFF_KT if EFF_KT > 0 else 0.0
                            self.fb_motor_temp = temp
                            self.fb_mos_temp = temp
                            self.fb_error = err
                        self._rx_count += 1
                    break  # got real reply, done

        except Exception as e:
            self._log(f"CAN ERR: {e}")
            time.sleep(0.02)

    # ---- SIMULATION ----
    def _sim(self, dt):
        with self._lock:
            active = self.active; tgt_i = self.target_current; rr = self.ramp_rate
            kp = self.target_kp; kd = self.target_kd
            p_des = self.target_p; v_des = self.target_v
            tgt_tff = self.target_tff

        if self.can_mode == self.MODE_MIT:
            # MIT sim
            if active:
                step = rr * dt
                diff = tgt_tff - self.ramped_value
                if abs(diff) < step: self.ramped_value = tgt_tff
                else: self.ramped_value += step if diff > 0 else -step
            else:
                step = rr * dt
                if abs(self.ramped_value) < step: self.ramped_value = 0.0
                else: self.ramped_value -= step if self.ramped_value > 0 else -step
                kp = 0.0; p_des = 0.0; v_des = 0.0
            tau = kp*(p_des-self._sim_p) + kd*(v_des-self._sim_v) + self.ramped_value
            tau = max(-MIT_T_MAX, min(MIT_T_MAX, tau))
            alpha = (tau - 0.01*self._sim_v) / self._SIM_J
            self._sim_v += alpha * dt
            self._sim_v = max(-MIT_V_MAX, min(MIT_V_MAX, self._sim_v))
            self._sim_p += self._sim_v * dt
            self._sim_p = max(MIT_P_MIN, min(MIT_P_MAX, self._sim_p))
            noise = np.random.normal(0, 0.005)
            with self._lock:
                self.fb_pos = round(self._sim_p, 4)
                self.fb_speed = round(self._sim_v + noise, 4)
                self.fb_torque = round(tau + np.random.normal(0, 0.01), 4)
                self.fb_current = round(self.fb_torque / EFF_KT, 4) if EFF_KT > 0 else 0
                self.fb_motor_temp = 36.0; self.fb_mos_temp = 34.0
                self.fb_voltage = 24.0; self.fb_error = 0
        else:
            # Servo sim (same as UART sim)
            if active:
                step = rr * dt
                diff = tgt_i - self._sim_i
                if abs(diff) < step: self._sim_i = tgt_i
                else: self._sim_i += step if diff > 0 else -step
            else:
                step = rr * dt
                if abs(self._sim_i) < step: self._sim_i = 0.0
                else: self._sim_i -= step if self._sim_i > 0 else -step
            noise = np.random.normal(0, 0.015)
            with self._lock:
                self.fb_current = round(self._sim_i + noise, 4)
                self.fb_speed = round(self._sim_i * 10000, 1)
                self.fb_torque = self.fb_current * EFF_KT
                self.fb_pos = 0.0
                self.fb_mos_temp = 34.0; self.fb_motor_temp = 36.0
                self.fb_voltage = 24.0; self.fb_error = 0

    def _send_mit_special(self, cmd_data):
        if self.bus:
            try:
                msg = can.Message(arbitration_id=self.can_id,
                                  data=cmd_data, is_extended_id=False)
                self.bus.send(msg)
                hx = " ".join(f"{b:02X}" for b in cmd_data)
                self._log(f"MIT SPECIAL (ID={self.can_id}) {hx}")
                reply = self.bus.recv(timeout=0.1)
                if reply:
                    hx = " ".join(f"{b:02X}" for b in reply.data)
                    self._log(f"CAN RX (ID=0x{reply.arbitration_id:X} "
                              f"{'EXT' if reply.is_extended_id else 'STD'}) {hx}")
            except Exception as e:
                self._log(f"MIT special err: {e}")

    def _log(self, msg):
        if self.log_cb: self.log_cb(f"[{time.strftime('%H:%M:%S')}] {msg}")


# ==========================================
# GUI
# ==========================================
class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AK40-10 Torque/Current Verification (UART + CAN)")
        self.resize(1200, 820)
        self.setStyleSheet(STYLE)

        self.signals = Signals()
        self.signals.log_signal.connect(self._append_log)
        _log_fn = lambda m: self.signals.log_signal.emit(m)

        self.motor = Motor(log_cb=_log_fn)
        self.motor_can = MotorCAN(log_cb=_log_fn)
        self._ctrl = None
        self._iface_type = "UART"

        self.t_buf    = deque(maxlen=BUF_SIZE)
        self.tgt_buf  = deque(maxlen=BUF_SIZE)
        self.act_buf  = deque(maxlen=BUF_SIZE)
        self.tgt2_buf = deque(maxlen=BUF_SIZE)
        self.act2_buf = deque(maxlen=BUF_SIZE)
        self._t0 = time.time()
        self._input_mode = "AMPS"
        self._graph_paused = False

        self._cap_pending = False
        self._cap_settle_t = 0.0
        self._cap_annotations = []

        # Recording
        self._rec_active = False
        self._rec_file = None
        self._rec_writer = None
        self._rec_path = ""
        self._rec_rows = 0

        self._build_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(50)

        QShortcut(QKeySequence(Qt.Key_Escape), self).activated.connect(self._estop)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_lay = QVBoxLayout(root)
        main_lay.setContentsMargins(6, 6, 6, 6)
        main_lay.setSpacing(4)

        # ====== TOP BAR ======
        top = QFrame()
        top_lay = QHBoxLayout(top)
        top_lay.setContentsMargins(4, 4, 4, 4)

        top_lay.addWidget(QLabel("Interface:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(180)
        btn_ref = QPushButton("R")
        btn_ref.setFixedWidth(28)
        btn_ref.clicked.connect(self._refresh_ports)
        top_lay.addWidget(self.cb_port)
        top_lay.addWidget(btn_ref)

        # CAN ID
        self.lbl_canid = QLabel("  CAN ID:")
        self.spin_canid = QSpinBox()
        self.spin_canid.setRange(0, 253)
        self.spin_canid.setValue(2)
        self.spin_canid.setSpecialValueText("Auto")
        self.spin_canid.setFixedWidth(60)
        self.spin_canid.setToolTip("Motor CAN ID. AK40-10 default=2. 0=auto-detect")
        top_lay.addWidget(self.lbl_canid)
        top_lay.addWidget(self.spin_canid)

        # CAN Mode label (auto-detected)
        self.lbl_canmode = QLabel("")
        self.lbl_canmode.setStyleSheet("color:#8f8; font-weight:bold; font-size:11px;")
        top_lay.addWidget(self.lbl_canmode)

        top_lay.addSpacing(8)
        self.btn_conn = QPushButton("Connect")
        self.btn_conn.setObjectName("btnGreen")
        self.btn_conn.clicked.connect(self._toggle_conn)
        top_lay.addWidget(self.btn_conn)

        top_lay.addSpacing(15)
        btn_estop = QPushButton("E-STOP [ESC]")
        btn_estop.setObjectName("btnRed")
        btn_estop.clicked.connect(self._estop)
        top_lay.addWidget(btn_estop)

        top_lay.addStretch()
        self.lbl_status = QLabel("  Disconnected")
        self.lbl_status.setStyleSheet("color:#e74c3c; font-weight:bold;")
        top_lay.addWidget(self.lbl_status)
        main_lay.addWidget(top)

        # ====== BODY ======
        body = QSplitter(Qt.Horizontal)

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setSpacing(6)

        # -- Input Mode --
        g_input = QGroupBox("Input Mode")
        il = QVBoxLayout(); g_input.setLayout(il)
        self.rb_amps = QRadioButton("Current (A) - default 0.5")
        self.rb_torque = QRadioButton("Torque (Nm) - convert via Kt")
        self.rb_amps.setChecked(True)
        self.rb_amps.toggled.connect(self._on_input_mode)
        il.addWidget(self.rb_amps); il.addWidget(self.rb_torque)

        # -- UART Parameters --
        self.g_uart_param = QGroupBox("UART Servo Parameters")
        pl = QGridLayout(); self.g_uart_param.setLayout(pl)
        pl.addWidget(QLabel("des P (deg)"), 0, 0)
        self.inp_desP = QLineEdit("0.00"); pl.addWidget(self.inp_desP, 0, 1)
        pl.addWidget(QLabel("des S (ERPM)"), 1, 0)
        self.inp_desS = QLineEdit("5000"); pl.addWidget(self.inp_desS, 1, 1)
        pl.addWidget(QLabel("des A (ERPM/s2)"), 2, 0)
        self.inp_desA = QLineEdit("30000"); pl.addWidget(self.inp_desA, 2, 1)
        pl.addWidget(QLabel("Ramp (A/s)"), 3, 0)
        self.inp_ramp_uart = QLineEdit("1.0"); pl.addWidget(self.inp_ramp_uart, 3, 1)

        # -- MIT Parameters --
        self.g_mit_param = QGroupBox("MIT Control Parameters")
        ml = QGridLayout(); self.g_mit_param.setLayout(ml)
        ml.addWidget(QLabel("Kp (pos gain)"), 0, 0)
        self.inp_kp = QLineEdit("0.0"); ml.addWidget(self.inp_kp, 0, 1)
        ml.addWidget(QLabel("Kd (damping)"), 1, 0)
        self.inp_kd = QLineEdit("0.5"); ml.addWidget(self.inp_kd, 1, 1)
        ml.addWidget(QLabel("p_des (rad)"), 2, 0)
        self.inp_pdes = QLineEdit("0.0"); ml.addWidget(self.inp_pdes, 2, 1)
        ml.addWidget(QLabel("v_des (rad/s)"), 3, 0)
        self.inp_vdes = QLineEdit("0.0"); ml.addWidget(self.inp_vdes, 3, 1)
        ml.addWidget(QLabel("Ramp (Nm/s)"), 4, 0)
        self.inp_ramp_can = QLineEdit("0.5"); ml.addWidget(self.inp_ramp_can, 4, 1)
        lbl_mit_eq = QLabel(
            "MIT equation:\n"
            "tau = kp*(p_des-p) + kd*(v_des-v) + t_ff\n\n"
            "Torque+damping: kp=0 v_des=0 kd>0\n"
            "  max speed = t_ff / kd\n\n"
            "RECOMMENDED:\n"
            "  kd=0.5 t_ff=0.5 => 1 rad/s\n"
            "  kd=0.3 t_ff=1.0 => 3.3 rad/s\n\n"
            "NOTE: Motor must be in MIT mode\n"
            "(switch via CubeMarsTool first)")
        lbl_mit_eq.setStyleSheet("color:#8f8; font-size:10px; font-family:Consolas,monospace;")
        lbl_mit_eq.setWordWrap(True)
        ml.addWidget(lbl_mit_eq, 5, 0, 1, 2)

        # -- CAN Servo Params --
        self.g_can_servo = QGroupBox("CAN Servo Parameters")
        csl = QGridLayout(); self.g_can_servo.setLayout(csl)
        csl.addWidget(QLabel("Ramp (A/s)"), 0, 0)
        self.inp_ramp_can_servo = QLineEdit("1.0"); csl.addWidget(self.inp_ramp_can_servo, 0, 1)
        lbl_servo_warn = QLabel(
            "SERVO mode: current loop via CAN\n"
            "Same protocol as UART but over CAN bus.\n"
            "Extended frame: (mode<<8)|motor_id\n"
            "Motor auto-broadcasts at 0x29|id\n\n"
            "WARNING: NO speed limit in current loop!\n"
            "Switch to MIT mode for damping.")
        lbl_servo_warn.setStyleSheet("color:#e74c3c; font-size:10px;")
        lbl_servo_warn.setWordWrap(True)
        csl.addWidget(lbl_servo_warn, 1, 0, 1, 2)

        # -- Serial Monitor for CAN --
        self.g_sermon = QGroupBox("Serial Monitor (optional)")
        sml = QHBoxLayout(); self.g_sermon.setLayout(sml)
        self.cb_ser_mon = QComboBox(); self.cb_ser_mon.setMinimumWidth(100)
        self.chk_ser_mon = QCheckBox("Enable")
        self.chk_ser_mon.toggled.connect(self._toggle_ser_mon)
        sml.addWidget(self.cb_ser_mon); sml.addWidget(self.chk_ser_mon)

        # -- Setpoint --
        g_sp = QGroupBox("Setpoint")
        sl = QVBoxLayout(); g_sp.setLayout(sl)
        self.lbl_sp = QLabel("Target Current (A):")
        self.inp_sp = QLineEdit("0.5")
        self.inp_sp.returnPressed.connect(self._send)
        self.lbl_calc = QLabel("")
        self.lbl_calc.setStyleSheet("color:#9cf; font-size:11px; font-family:Consolas,monospace;")
        self.lbl_calc.setWordWrap(True)
        self.btn_send = QPushButton("Send")
        self.btn_send.setObjectName("btnBlue")
        self.btn_send.clicked.connect(self._send)
        self.btn_stop = QPushButton("Stop Output (0)")
        self.btn_stop.clicked.connect(self._stop)
        self.btn_capture = QPushButton("Capture Now")
        self.btn_capture.clicked.connect(self._capture_now)
        for w in [self.lbl_sp, self.inp_sp, self.lbl_calc,
                  self.btn_send, self.btn_stop, self.btn_capture]:
            sl.addWidget(w)

        # -- Step Sweep --
        g_sweep = QGroupBox("Step Sweep")
        swl = QGridLayout(); g_sweep.setLayout(swl)
        swl.addWidget(QLabel("Start:"), 0, 0)
        self.inp_sweep_start = QLineEdit("0.1"); swl.addWidget(self.inp_sweep_start, 0, 1)
        swl.addWidget(QLabel("End:"), 1, 0)
        self.inp_sweep_end = QLineEdit("0.5"); swl.addWidget(self.inp_sweep_end, 1, 1)
        swl.addWidget(QLabel("Step:"), 2, 0)
        self.inp_sweep_step = QLineEdit("0.1"); swl.addWidget(self.inp_sweep_step, 2, 1)
        swl.addWidget(QLabel("Dwell (s):"), 3, 0)
        self.inp_sweep_dwell = QLineEdit("3.0"); swl.addWidget(self.inp_sweep_dwell, 3, 1)
        self.btn_sweep = QPushButton("Run Sweep")
        self.btn_sweep.setObjectName("btnBlue")
        self.btn_sweep.clicked.connect(self._start_sweep)
        self.btn_sweep_stop = QPushButton("Abort Sweep")
        self.btn_sweep_stop.setObjectName("btnRed")
        self.btn_sweep_stop.clicked.connect(self._abort_sweep)
        self.btn_sweep_stop.setEnabled(False)
        self.lbl_sweep_status = QLabel("")
        self.lbl_sweep_status.setStyleSheet("color:#ff0; font-size:11px; font-family:Consolas,monospace;")
        self.lbl_sweep_status.setWordWrap(True)
        swl.addWidget(self.btn_sweep, 4, 0)
        swl.addWidget(self.btn_sweep_stop, 4, 1)
        swl.addWidget(self.lbl_sweep_status, 5, 0, 1, 2)
        lbl_sweep_note = QLabel(
            "Uses current Input Mode (A or Nm).\n"
            "Params (Kp/Kd/etc) from MIT panel.\n"
            "Auto-captures at each step after dwell.\n"
            "Auto-records to CSV on start.")
        lbl_sweep_note.setStyleSheet("color:#888; font-size:10px;")
        lbl_sweep_note.setWordWrap(True)
        swl.addWidget(lbl_sweep_note, 6, 0, 1, 2)

        # -- Recording --
        g_rec = QGroupBox("Data Recording")
        rcl = QHBoxLayout(); g_rec.setLayout(rcl)
        self.btn_rec = QPushButton("Start Recording")
        self.btn_rec.setObjectName("btnGreen")
        self.btn_rec.clicked.connect(self._toggle_recording)
        self.lbl_rec_status = QLabel("Not recording")
        self.lbl_rec_status.setStyleSheet("color:#888; font-size:10px;")
        self.lbl_rec_status.setWordWrap(True)
        rcl.addWidget(self.btn_rec)
        rcl.addWidget(self.lbl_rec_status, stretch=1)

        # -- Reference --
        g_ref = QGroupBox("Conversion Reference")
        rl = QVBoxLayout(); g_ref.setLayout(rl)
        lbl_ref = QLabel(
            f"Kt={KT} Nm/A  Gear={GEAR_RATIO}:1  Eff={GEAR_EFF}\n"
            f"Effective Kt = {EFF_KT:.3f} Nm/A\n"
            f"tau = I x {EFF_KT:.3f}   I = tau / {EFF_KT:.3f}\n"
            f"Max: {MAX_CURRENT}A / {MAX_TORQUE}Nm")
        lbl_ref.setStyleSheet("color:#aaa; font-size:10px; font-family:Consolas,monospace;")
        rl.addWidget(lbl_ref)

        # -- Telemetry --
        g_tel = QGroupBox("Live Feedback")
        tl = QGridLayout(); g_tel.setLayout(tl)
        self.lbl_fb_cur  = QLabel("0.000 A")
        self.lbl_fb_tau  = QLabel("0.000 Nm")
        self.lbl_fb_spd  = QLabel("0")
        self.lbl_fb_pos  = QLabel("0.00")
        self.lbl_fb_tmp  = QLabel("-- C")
        self.lbl_fb_volt = QLabel("0.0 V")
        self.lbl_fb_err  = QLabel("0")
        for i, (name, w) in enumerate([
            ("Current:", self.lbl_fb_cur), ("Torque:", self.lbl_fb_tau),
            ("Speed:", self.lbl_fb_spd), ("Position:", self.lbl_fb_pos),
            ("Temp:", self.lbl_fb_tmp), ("Voltage:", self.lbl_fb_volt),
            ("Error:", self.lbl_fb_err)]):
            tl.addWidget(QLabel(name), i, 0); tl.addWidget(w, i, 1)

        ll.addWidget(g_input)
        ll.addWidget(self.g_uart_param)
        ll.addWidget(self.g_can_servo)
        ll.addWidget(self.g_mit_param)
        ll.addWidget(self.g_sermon)
        ll.addWidget(g_sp); ll.addWidget(g_sweep); ll.addWidget(g_rec); ll.addWidget(g_ref); ll.addWidget(g_tel)
        ll.addStretch()

        left_scroll = QScrollArea()
        left_scroll.setWidget(left); left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setMinimumWidth(310)

        # --- GRAPHS ---
        right = QWidget()
        right_lay = QVBoxLayout(right)
        pg.setConfigOption('background', '#1a1a1a')
        pg.setConfigOption('foreground', '#ccc')

        self.plot1 = pg.PlotWidget()
        self.plot1.setTitle("Current (A) : Target vs Actual")
        self.plot1.setLabel('left', 'A'); self.plot1.setLabel('bottom', 'Time (s)')
        self.plot1.showGrid(x=True, y=True, alpha=0.3)
        self.plot1.addLegend(offset=(10, 10))
        self.line_tgt1 = self.plot1.plot(pen=pg.mkPen('#e74c3c', width=2, style=Qt.DashLine), name="Target")
        self.line_act1 = self.plot1.plot(pen=pg.mkPen('#2ecc71', width=2), name="Actual")

        self.plot2 = pg.PlotWidget()
        self.plot2.setTitle("Torque (Nm) : Target vs Actual")
        self.plot2.setLabel('left', 'Nm'); self.plot2.setLabel('bottom', 'Time (s)')
        self.plot2.showGrid(x=True, y=True, alpha=0.3)
        self.plot2.addLegend(offset=(10, 10))
        self.line_tgt2 = self.plot2.plot(pen=pg.mkPen('#e67e22', width=2, style=Qt.DashLine), name="Target")
        self.line_act2 = self.plot2.plot(pen=pg.mkPen('#3498db', width=2), name="Actual")

        gc = QHBoxLayout()
        self.btn_pause = QPushButton("Pause Graph")
        self.btn_pause.clicked.connect(self._toggle_pause)
        btn_clr = QPushButton("Clear Graph")
        btn_clr.clicked.connect(self._clear_bufs)
        gc.addWidget(self.btn_pause); gc.addWidget(btn_clr); gc.addStretch()

        right_lay.addWidget(self.plot1); right_lay.addWidget(self.plot2)
        right_lay.addLayout(gc)

        body.addWidget(left_scroll); body.addWidget(right)
        body.setSizes([330, 870]); body.setChildrenCollapsible(False)
        main_lay.addWidget(body, stretch=1)

        # ====== MONITOR ======
        g_mon = QGroupBox("Communication Monitor")
        g_mon.setMaximumHeight(150)
        monl = QHBoxLayout(); g_mon.setLayout(monl)
        self.monitor = QTextEdit(); self.monitor.setReadOnly(True)
        self.monitor.setStyleSheet(
            "font-family:Consolas,monospace; font-size:11px; background:#111; color:#8f8;")
        btn_clr_mon = QPushButton("Clear"); btn_clr_mon.setFixedWidth(50)
        btn_clr_mon.clicked.connect(self.monitor.clear)
        monl.addWidget(self.monitor)
        monl.addWidget(btn_clr_mon, alignment=Qt.AlignTop)
        main_lay.addWidget(g_mon)

        # Connect signals AFTER all widgets exist
        self.cb_port.currentTextChanged.connect(self._on_iface_changed)
        # cb_canmode removed - mode is auto-detected
        self._refresh_ports()
        self._update_ui_for_iface()

    # ── port / interface ──
    def _refresh_ports(self):
        self.cb_port.blockSignals(True)
        self.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_port.addItem(f"UART: {p.device}")
        for iface in get_can_interfaces():
            self.cb_port.addItem(f"CAN: {iface}")
        self.cb_port.addItem("Simulation")
        self.cb_port.blockSignals(False)
        self._on_iface_changed(self.cb_port.currentText())
        self.cb_ser_mon.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_ser_mon.addItem(p.device)

    def _on_iface_changed(self, text):
        if text.startswith("CAN:"): self._iface_type = "CAN"
        elif text.startswith("UART:"): self._iface_type = "UART"
        else: self._iface_type = "SIM"
        self._update_ui_for_iface()

    def _on_canmode_changed(self, idx):
        pass  # mode is auto-detected now

    def _update_ui_for_iface(self):
        is_can = (self._iface_type == "CAN")
        is_sim = (self._iface_type == "SIM")
        self.lbl_canid.setVisible(is_can)
        self.spin_canid.setVisible(is_can)
        self.lbl_canmode.setVisible(is_can)
        self.g_uart_param.setVisible(not is_can)
        self.g_can_servo.setVisible(False)  # hidden until fallback detected
        self.g_mit_param.setVisible(is_can or is_sim)
        self.g_sermon.setVisible(is_can)

    # ── connect / disconnect ──
    def _toggle_conn(self):
        ctrl = self._ctrl
        if ctrl and ctrl.connected:
            if self._rec_active:
                self._stop_recording()
            ctrl.disconnect(); self._ctrl = None
            self.btn_conn.setText("Connect")
            self.btn_conn.setObjectName("btnGreen")
            self.btn_conn.setStyleSheet(self.btn_conn.styleSheet())
            self.lbl_status.setText("  Disconnected")
            self.lbl_status.setStyleSheet("color:#e74c3c; font-weight:bold;")
            return

        self._apply_params()
        port_text = self.cb_port.currentText()

        if self._iface_type == "CAN":
            channel = port_text.replace("CAN: ", "")
            can_id = self.spin_canid.value()  # 0 = auto
            self.motor_can.connect(channel, can_id)  # auto MIT
            self._ctrl = self.motor_can
            # Update mode label after connect
            mode = self.motor_can.can_mode
            self.lbl_canmode.setText(f"  Mode: {mode}")
            # Show correct param panel
            if mode == MotorCAN.MODE_SERVO:
                self.g_can_servo.setVisible(True)
                self.g_mit_param.setVisible(False)
            else:
                self.g_can_servo.setVisible(False)
                self.g_mit_param.setVisible(True)
            # Popup with connection result
            result_text = self.motor_can.get_connect_result()
            if not self.motor_can.simulation:
                if mode == MotorCAN.MODE_MIT:
                    mb = QMessageBox(self)
                    mb.setWindowTitle("CAN Connection Result")
                    mb.setIcon(QMessageBox.Information)
                    mb.setText(f"MIT MODE ACTIVE\nMotor ID: {self.motor_can.can_id}")
                    mb.setDetailedText(result_text)
                    mb.exec()
                else:
                    mb = QMessageBox(self)
                    mb.setWindowTitle("CAN Connection Result")
                    mb.setIcon(QMessageBox.Warning)
                    mb.setText(
                        f"MIT switch FAILED - using SERVO mode\n"
                        f"Motor ID: {self.motor_can.can_id}\n\n"
                        f"Motor did not respond to MIT ENTER.\n"
                        f"Possible causes:\n"
                        f"  - Firmware is servo-only\n"
                        f"  - Need power cycle\n"
                        f"  - CAN wiring issue\n\n"
                        f"SERVO mode works but has NO speed limit!")
                    mb.setDetailedText(result_text)
                    mb.exec()
        elif self._iface_type == "UART":
            port = port_text.replace("UART: ", "")
            self.motor.connect(port)
            self._ctrl = self.motor
        else:
            self.motor_can.connect("Simulation", 1)  # auto MIT sim
            self._ctrl = self.motor_can

        if self._ctrl and self._ctrl.connected:
            if self._ctrl.simulation: tag = "SIM"
            elif self._iface_type == "CAN":
                cid = self.motor_can.can_id
                cm = self.motor_can.can_mode
                tag = f"CAN:{port_text.replace('CAN: ','')} ID={cid} {cm}"
            else: tag = port_text.replace("UART: ", "")
            self.btn_conn.setText("Disconnect")
            self.btn_conn.setObjectName("btnGray")
            self.btn_conn.setStyleSheet(self.btn_conn.styleSheet())
            self.lbl_status.setText(f"  [{tag}]")
            self.lbl_status.setStyleSheet("color:#2ecc71; font-weight:bold;")
            self._t0 = time.time(); self._clear_bufs()
            # Auto-start recording on connect
            if not self._rec_active:
                self._start_recording()

    def _toggle_ser_mon(self, checked):
        if checked and self._ctrl is self.motor_can:
            port = self.cb_ser_mon.currentText()
            if port: self.motor_can.start_serial_monitor(port)
        else:
            self.motor_can.stop_serial_monitor()

    def _apply_params(self):
        try: self.motor.des_p = float(self.inp_desP.text())
        except: pass
        try: self.motor.des_s = float(self.inp_desS.text())
        except: pass
        try: self.motor.des_a = float(self.inp_desA.text())
        except: pass
        try: self.motor.ramp_rate = max(0.1, float(self.inp_ramp_uart.text()))
        except: pass
        try: kp = float(self.inp_kp.text())
        except: kp = 0.0
        try: kd = float(self.inp_kd.text())
        except: kd = 0.5
        try: p_des = float(self.inp_pdes.text())
        except: p_des = 0.0
        try: v_des = float(self.inp_vdes.text())
        except: v_des = 0.0
        try: self.motor_can.ramp_rate = max(0.05, float(self.inp_ramp_can.text()))
        except: pass
        try: self.motor_can.ramp_rate = max(0.05, float(self.inp_ramp_can_servo.text()))
        except: pass
        self.motor_can.set_mit_params(kp, kd, p_des, v_des)

    def _on_input_mode(self):
        self._input_mode = "AMPS" if self.rb_amps.isChecked() else "TORQUE"
        if self._input_mode == "AMPS":
            self.lbl_sp.setText("Target Current (A):")
            self.inp_sp.setPlaceholderText(f"max {MAX_CURRENT}")
        else:
            self.lbl_sp.setText("Target Torque (Nm):")
            self.inp_sp.setPlaceholderText(f"max {MIT_T_MAX}")

    def _send(self):
        if not self._ctrl or not self._ctrl.connected: return
        self._apply_params()
        try: val = float(self.inp_sp.text())
        except: return
        if self._input_mode == "AMPS":
            self._ctrl.set_current(val)
            tau = val * EFF_KT
            self.lbl_calc.setText(f"I={val:.3f}A  tau={tau:.3f}Nm")
        else:
            self._ctrl.set_torque(val)
            cur = val / EFF_KT if EFF_KT > 0 else 0
            self.lbl_calc.setText(f"tau={val:.3f}Nm  I={cur:.3f}A")

    # ── sweep ──
    _sweep_active = False
    _sweep_steps = []
    _sweep_idx = 0
    _sweep_dwell = 3.0
    _sweep_t_step = 0.0
    _sweep_captured = False

    def _start_sweep(self):
        if not self._ctrl or not self._ctrl.connected:
            self.lbl_sweep_status.setText("Not connected!")
            return
        try:
            start = float(self.inp_sweep_start.text())
            end = float(self.inp_sweep_end.text())
            step = abs(float(self.inp_sweep_step.text()))
            self._sweep_dwell = max(0.5, float(self.inp_sweep_dwell.text()))
        except:
            self.lbl_sweep_status.setText("Invalid input!")
            return
        if step < 0.001:
            self.lbl_sweep_status.setText("Step too small!")
            return

        # Generate steps
        self._sweep_steps = []
        val = start
        if start <= end:
            while val <= end + step * 0.01:
                self._sweep_steps.append(round(val, 6))
                val += step
        else:
            while val >= end - step * 0.01:
                self._sweep_steps.append(round(val, 6))
                val -= step

        if not self._sweep_steps:
            self.lbl_sweep_status.setText("No steps generated!")
            return

        unit = "A" if self._input_mode == "AMPS" else "Nm"
        steps_str = ", ".join(f"{s:.3f}" for s in self._sweep_steps)
        self.lbl_sweep_status.setText(
            f"Sweep: {steps_str} {unit}\n"
            f"Dwell: {self._sweep_dwell}s per step")

        self._sweep_idx = 0
        self._sweep_captured = False
        self._sweep_active = True
        self.btn_sweep.setEnabled(False)
        self.btn_sweep_stop.setEnabled(True)
        self.btn_send.setEnabled(False)

        # Apply params
        self._apply_params()

        # Auto-start recording if not already
        if not self._rec_active:
            self._start_recording()

        self._sweep_send_current_step()

        # Kill old timer if exists, create fresh one
        if hasattr(self, '_sweep_timer') and self._sweep_timer is not None:
            self._sweep_timer.stop()
            self._sweep_timer.deleteLater()
        self._sweep_timer = QTimer()
        self._sweep_timer.timeout.connect(self._sweep_tick)
        self._sweep_timer.start(100)

    def _sweep_send_current_step(self):
        if not self._ctrl or not self._ctrl.connected:
            self._sweep_finish("Sweep stopped: disconnected")
            return
        val = self._sweep_steps[self._sweep_idx]
        unit = "A" if self._input_mode == "AMPS" else "Nm"
        total = len(self._sweep_steps)
        self.lbl_sweep_status.setText(
            f"Step {self._sweep_idx+1}/{total}: {val:.3f} {unit}\n"
            f"Dwell: {self._sweep_dwell}s | Capturing after settle...")

        if self._input_mode == "AMPS":
            self._ctrl.set_current(val)
        else:
            self._ctrl.set_torque(val)

        self._sweep_t_step = time.time()
        self._sweep_captured = False

    def _sweep_tick(self):
        if not self._sweep_active:
            return
        if not self._ctrl or not self._ctrl.connected:
            self._sweep_finish("Sweep stopped: disconnected")
            return

        elapsed = time.time() - self._sweep_t_step
        val = self._sweep_steps[self._sweep_idx]
        unit = "A" if self._input_mode == "AMPS" else "Nm"
        total = len(self._sweep_steps)
        remain = max(0, self._sweep_dwell - elapsed)

        # Auto-capture at 80% of dwell (settled)
        if not self._sweep_captured and elapsed >= self._sweep_dwell * 0.8:
            self._sweep_captured = True
            self._cap_pending = True
            self._cap_settle_t = time.time()  # capture immediately

        # Update status
        self.lbl_sweep_status.setText(
            f"Step {self._sweep_idx+1}/{total}: {val:.3f} {unit}\n"
            f"Remaining: {remain:.1f}s")

        # Advance to next step
        if elapsed >= self._sweep_dwell:
            self._sweep_idx += 1
            if self._sweep_idx >= len(self._sweep_steps):
                # Sweep done
                self._sweep_finish("Sweep COMPLETE!")
            else:
                self._apply_params()
                self._sweep_send_current_step()

    def _abort_sweep(self):
        if self._ctrl:
            self._ctrl.stop_command()
        self._sweep_finish("Sweep ABORTED")

    def _sweep_finish(self, msg):
        self._sweep_active = False
        if hasattr(self, '_sweep_timer') and self._sweep_timer is not None:
            self._sweep_timer.stop()
            self._sweep_timer.deleteLater()
            self._sweep_timer = None
        self.btn_sweep.setEnabled(True)
        self.btn_sweep_stop.setEnabled(False)
        self.btn_send.setEnabled(True)
        # Stop motor
        if self._ctrl:
            self._ctrl.stop_command()
        # Auto-stop recording
        if self._rec_active:
            self._stop_recording()
        # Reset sweep state
        self._sweep_idx = 0
        self._sweep_captured = False
        self.lbl_sweep_status.setText(msg)

    # ── recording ──
    def _toggle_recording(self):
        if self._rec_active:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        # Create data dir
        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        os.makedirs(data_dir, exist_ok=True)
        # Filename: AK40-10_2026-09-16_14-30-25.csv
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        fname = f"AK40-10_{ts}.csv"
        self._rec_path = os.path.join(data_dir, fname)
        try:
            self._rec_file = open(self._rec_path, 'w', newline='')
            self._rec_writer = csv.writer(self._rec_file)
            # Header
            is_can = (self._ctrl is self.motor_can) if self._ctrl else False
            is_mit = is_can and self.motor_can.can_mode == MotorCAN.MODE_MIT
            mode_str = "MIT" if is_mit else "SERVO" if is_can else "UART"
            can_id = self.motor_can.can_id if is_can else "N/A"
            # Metadata rows
            self._rec_writer.writerow(["# AK40-10 Torque/Current Verification Log"])
            self._rec_writer.writerow([f"# Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"])
            self._rec_writer.writerow([f"# Mode: {mode_str}"])
            self._rec_writer.writerow([f"# CAN ID: {can_id}"])
            self._rec_writer.writerow([f"# Kt_eff: {EFF_KT:.4f} Nm/A"])
            self._rec_writer.writerow([f"# Gear Ratio: {GEAR_RATIO}"])
            self._rec_writer.writerow([f"# Input Mode: {self._input_mode}"])
            if is_mit:
                self._rec_writer.writerow([f"# MIT Kp: {self.motor_can.target_kp}"])
                self._rec_writer.writerow([f"# MIT Kd: {self.motor_can.target_kd}"])
                self._rec_writer.writerow([f"# MIT v_des: {self.motor_can.target_v}"])
                self._rec_writer.writerow([f"# MIT p_des: {self.motor_can.target_p}"])
            self._rec_writer.writerow([])
            # Column headers
            self._rec_writer.writerow([
                "time_s",
                "target_current_A", "actual_current_A",
                "target_torque_Nm", "actual_torque_Nm",
                "position_rad", "position_deg",
                "speed_rad_s", "speed_rpm",
                "motor_temp_C", "mos_temp_C",
                "voltage_V", "error_code",
                "sweep_step"  # empty if no sweep
            ])
            self._rec_active = True
            self._rec_rows = 0
            self.btn_rec.setText("Stop Recording")
            self.btn_rec.setObjectName("btnRed")
            self.btn_rec.setStyleSheet(self.btn_rec.styleSheet())
            self.lbl_rec_status.setText(f"REC: {fname}\n0 rows")
            self.lbl_rec_status.setStyleSheet("color:#e74c3c; font-weight:bold; font-size:10px;")
            self._append_log(f"Recording started: {self._rec_path}")
        except Exception as e:
            self._append_log(f"Recording error: {e}")
            self.lbl_rec_status.setText(f"ERROR: {e}")

    def _stop_recording(self):
        if self._rec_file:
            try:
                self._rec_file.close()
            except: pass
            self._rec_file = None
            self._rec_writer = None
        self._rec_active = False
        self.btn_rec.setText("Start Recording")
        self.btn_rec.setObjectName("btnGreen")
        self.btn_rec.setStyleSheet(self.btn_rec.styleSheet())
        self.lbl_rec_status.setText(f"Saved: {os.path.basename(self._rec_path)}\n{self._rec_rows} rows")
        self.lbl_rec_status.setStyleSheet("color:#2ecc71; font-size:10px;")
        self._append_log(f"Recording saved: {self._rec_path} ({self._rec_rows} rows)")

    def _rec_write_row(self, t, tgt_i, fb_i, tgt_tau, fb_tau, fb_pos, fb_spd, ctrl):
        """Write one data row to CSV if recording."""
        if not self._rec_active or not self._rec_writer:
            return
        try:
            is_mit = (ctrl is self.motor_can and
                      self.motor_can.can_mode == MotorCAN.MODE_MIT)
            if is_mit:
                pos_deg = np.degrees(fb_pos)
                spd_rpm = fb_spd * 60.0 / (2.0 * np.pi)
            else:
                pos_deg = fb_pos  # already in deg for servo
                spd_rpm = fb_spd / (POLE_PAIRS * GEAR_RATIO) if (POLE_PAIRS * GEAR_RATIO) > 0 else 0

            sweep_step = ""
            if self._sweep_active and self._sweep_idx < len(self._sweep_steps):
                sweep_step = f"{self._sweep_steps[self._sweep_idx]:.4f}"

            self._rec_writer.writerow([
                f"{t:.4f}",
                f"{tgt_i:.5f}", f"{fb_i:.5f}",
                f"{tgt_tau:.5f}", f"{fb_tau:.5f}",
                f"{fb_pos:.5f}", f"{pos_deg:.3f}",
                f"{fb_spd:.5f}", f"{spd_rpm:.3f}",
                f"{ctrl.fb_motor_temp:.1f}", f"{ctrl.fb_mos_temp:.1f}",
                f"{ctrl.fb_voltage:.1f}", f"{ctrl.fb_error}",
                sweep_step
            ])
            self._rec_rows += 1
            # Update status every 20 rows
            if self._rec_rows % 20 == 0:
                fname = os.path.basename(self._rec_path)
                self.lbl_rec_status.setText(f"REC: {fname}\n{self._rec_rows} rows")
        except Exception as e:
            self._append_log(f"Rec write error: {e}")

    def _stop(self):
        if self._ctrl: self._ctrl.stop_command()
        self.lbl_calc.setText("Stopped (0)")

    def _estop(self):
        self.motor.estop(); self.motor_can.estop()

    def _capture_now(self):
        if not self._ctrl or not self._ctrl.connected: return
        self._cap_pending = True; self._cap_settle_t = time.time() + 2.0
        self.lbl_calc.setText("Capturing in 2s...")

    def _toggle_pause(self):
        self._graph_paused = not self._graph_paused
        if self._graph_paused:
            self.btn_pause.setText("Resume")
            self.btn_pause.setStyleSheet("background:#e67e22; color:white; font-weight:bold;")
        else:
            self.btn_pause.setText("Pause Graph"); self.btn_pause.setStyleSheet("")

    def _clear_bufs(self):
        for b in [self.t_buf, self.tgt_buf, self.act_buf, self.tgt2_buf, self.act2_buf]:
            b.clear()
        self._cap_pending = False
        for item in self._cap_annotations: self.plot1.removeItem(item)
        self._cap_annotations.clear()
        self._t0 = time.time()

    # ── update loop (20Hz) ──
    def _update(self):
        ctrl = self._ctrl
        if not ctrl or not ctrl.connected: return

        t = time.time() - self._t0
        is_can = (ctrl is self.motor_can)
        is_mit = is_can and ctrl.can_mode == MotorCAN.MODE_MIT

        with ctrl._lock:
            fb_i = ctrl.fb_current; fb_spd = ctrl.fb_speed
            fb_pos = ctrl.fb_pos; fb_tau = ctrl.fb_torque
            tgt_i = ctrl.target_current; tgt_tau = ctrl.target_torque

        self.lbl_fb_cur.setText(f"{fb_i:.3f} A")
        self.lbl_fb_tau.setText(f"{fb_tau:.3f} Nm")

        if is_mit:
            rpm = fb_spd * 60.0 / (2.0 * np.pi)
            self.lbl_fb_spd.setText(f"{fb_spd:.2f} rad/s ({rpm:.1f} rpm)")
            self.lbl_fb_pos.setText(f"{fb_pos:.3f} rad ({np.degrees(fb_pos):.1f} deg)")
        else:
            rpm = fb_spd / (POLE_PAIRS * GEAR_RATIO) if (POLE_PAIRS * GEAR_RATIO) > 0 else 0
            self.lbl_fb_spd.setText(f"{fb_spd:.0f} ERPM ({rpm:.1f} rpm)")
            self.lbl_fb_pos.setText(f"{fb_pos:.2f} deg")

        self.lbl_fb_tmp.setText(f"MOS {ctrl.fb_mos_temp:.0f}C  Motor {ctrl.fb_motor_temp:.0f}C")
        self.lbl_fb_volt.setText(f"{ctrl.fb_voltage:.1f} V" if ctrl.fb_voltage > 0 else "N/A")
        self.lbl_fb_err.setText(str(ctrl.fb_error))

        if self._graph_paused: return

        self.t_buf.append(t)
        self.tgt_buf.append(tgt_i); self.act_buf.append(fb_i)
        self.tgt2_buf.append(tgt_tau); self.act2_buf.append(fb_tau)

        # Record to CSV
        self._rec_write_row(t, tgt_i, fb_i, tgt_tau, fb_tau, fb_pos, fb_spd, ctrl)

        if self._cap_pending and time.time() >= self._cap_settle_t:
            self._cap_pending = False
            err_i = abs(tgt_i - fb_i)
            txt = pg.TextItem(f"I={fb_i:.3f}A tau={fb_tau:.3f}Nm\nSpd={fb_spd:.1f}",
                              color='#fff', anchor=(0.5,1.2),
                              border=pg.mkPen('#888'), fill=pg.mkBrush('#333'))
            txt.setPos(t, fb_i); self.plot1.addItem(txt)
            self._cap_annotations.append(txt)
            dot = pg.ScatterPlotItem([t],[fb_i],size=8,
                pen=pg.mkPen('#fff',width=1),brush=pg.mkBrush('#2ecc71'))
            self.plot1.addItem(dot); self._cap_annotations.append(dot)
            self.lbl_calc.setText(f"Captured: I={fb_i:.3f}A tau={fb_tau:.3f}Nm err={err_i:.3f}A")

        ts = np.array(self.t_buf)
        self.line_tgt1.setData(ts, np.array(self.tgt_buf))
        self.line_act1.setData(ts, np.array(self.act_buf))
        self.line_tgt2.setData(ts, np.array(self.tgt2_buf))
        self.line_act2.setData(ts, np.array(self.act2_buf))
        if len(ts) > 1:
            xr = [max(0, ts[-1]-20), ts[-1]]
            self.plot1.setXRange(*xr, padding=0)
            self.plot2.setXRange(*xr, padding=0)

    def _append_log(self, msg):
        self.monitor.append(msg)
        sb = self.monitor.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, e):
        if self._rec_active:
            self._stop_recording()
        try: self.motor.disconnect()
        except: pass
        try: self.motor_can.disconnect()
        except: pass
        super().closeEvent(e)


# ==========================================
# STYLE
# ==========================================
STYLE = """
QMainWindow, QWidget { background: #2b2b2b; color: #dcdcdc; }
QGroupBox {
    border: 1px solid #555; border-radius: 4px;
    margin-top: 8px; font-weight: bold; color: #aaa;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; }
QLineEdit, QComboBox, QSpinBox {
    background: #3c3c3c; border: 1px solid #555;
    border-radius: 3px; padding: 4px 6px; color: #dcdcdc;
}
QPushButton {
    background: #3c3c3c; border: 1px solid #555;
    border-radius: 3px; padding: 5px 12px; color: #dcdcdc;
}
QPushButton:hover { background: #4a4a4a; }
QPushButton#btnGreen { background: #27ae60; color: white; font-weight: bold; }
QPushButton#btnRed   { background: #c0392b; color: white; font-weight: bold; }
QPushButton#btnBlue  { background: #2980b9; color: white; font-weight: bold; padding: 8px; }
QPushButton#btnGray  { background: #7f8c8d; color: white; font-weight: bold; }
QRadioButton, QCheckBox { color: #dcdcdc; }
QTextEdit { background: #111; color: #8f8; border: none; }
QLabel { color: #dcdcdc; }
QSplitter::handle { background: #444; }
QFrame { border: none; }
"""

# ==========================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = App()
    win.show()
    sys.exit(app.exec())
