#!/usr/bin/env python3
"""CAN Diagnostic - test raw communication with AK40-10"""
import can
import time
import struct
import sys

bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=1000000)

def flush():
    while bus.recv(timeout=0.01): pass

def send_recv(msg, label, timeout=0.2):
    flush()
    bus.send(msg)
    hx = ' '.join(f'{b:02X}' for b in msg.data)
    ext = 'EXT' if msg.is_extended_id else 'STD'
    print(f'  TX (0x{msg.arbitration_id:X} {ext}) {hx}  [{label}]')
    replies = []
    t0 = time.time()
    while (time.time() - t0) < timeout:
        r = bus.recv(timeout=0.05)
        if r:
            hx = ' '.join(f'{b:02X}' for b in r.data)
            ext = 'EXT' if r.is_extended_id else 'STD'
            print(f'  RX (0x{r.arbitration_id:X} {ext}) {hx}')
            replies.append(r)
    return replies

# ==============================
# STEP 1: Listen passively
# ==============================
print('='*60)
print('STEP 1: Listening to bus for 1 second...')
print('='*60)
flush()
heard = {}
t0 = time.time()
while (time.time() - t0) < 1.0:
    msg = bus.recv(timeout=0.1)
    if msg:
        key = (msg.arbitration_id, msg.is_extended_id)
        if key not in heard:
            heard[key] = msg
            hx = ' '.join(f'{b:02X}' for b in msg.data)
            ext = 'EXT' if msg.is_extended_id else 'STD'
            print(f'  {ext} 0x{msg.arbitration_id:X} [{len(msg.data)}B] {hx}')
            if msg.is_extended_id:
                func = (msg.arbitration_id >> 8) & 0xFF
                mid = msg.arbitration_id & 0xFF
                print(f'    -> func=0x{func:02X} motor_id={mid}')
                if func == 0x29:
                    d = msg.data
                    pos = ((d[0]<<8)|d[1])
                    if pos > 32767: pos -= 65536
                    spd = ((d[2]<<8)|d[3])
                    if spd > 32767: spd -= 65536
                    cur = ((d[4]<<8)|d[5])
                    if cur > 32767: cur -= 65536
                    print(f'    -> pos={pos*0.1:.1f}deg spd={spd*10:.0f}ERPM cur={cur*0.01:.2f}A temp={d[6]}C err={d[7]}')

if not heard:
    print('  NO DATA on bus! Check wiring.')
    sys.exit(1)

# Find motor IDs from servo frames
servo_ids = []
for (aid, is_ext), msg in heard.items():
    if is_ext:
        func = (aid >> 8) & 0xFF
        if func == 0x29:
            servo_ids.append(aid & 0xFF)

if servo_ids:
    motor_id = servo_ids[0]
    print(f'\n  => Detected motor ID = {motor_id} (servo mode)')
else:
    motor_id = int(input('  No servo frame detected. Enter motor ID manually: '))

# ==============================
# STEP 2: Try SERVO mode current command  
# ==============================
print(f'\n{"="*60}')
print(f'STEP 2: Sending SERVO current command (0.5A) to ID={motor_id}')
print('='*60)
eid = (1 << 8) | motor_id  # CAN_PACKET_SET_CURRENT=1
val = int(0.5 * 1000)  # 500 mA
data = struct.pack('>i', val)
msg = can.Message(arbitration_id=eid, data=data, is_extended_id=True)
replies = send_recv(msg, 'SET_CURRENT 0.5A')
if not replies:
    print('  NO REPLY to servo current command')
time.sleep(0.5)
# Send zero
data = struct.pack('>i', 0)
msg = can.Message(arbitration_id=eid, data=data, is_extended_id=True)
send_recv(msg, 'SET_CURRENT 0A (stop)', timeout=0.1)

# ==============================
# STEP 3: Try MIT ENTER
# ==============================
print(f'\n{"="*60}')
print(f'STEP 3: Trying MIT ENTER on ID={motor_id}')
print('='*60)
flush()
for attempt in range(3):
    flush()
    time.sleep(0.02)
    enter_data = bytes([0xFF]*7 + [0xFC])
    msg = can.Message(arbitration_id=motor_id, data=enter_data, is_extended_id=False)
    print(f'  Attempt {attempt+1}:')
    replies = send_recv(msg, 'MIT_ENTER', timeout=0.2)
    got_mit = False
    for r in replies:
        if not r.is_extended_id and len(r.data) >= 6:
            if r.data[0] == motor_id:
                print(f'  >>> MIT ENTER ACKNOWLEDGED by motor {motor_id}! <<<')
                got_mit = True
        elif r.is_extended_id:
            func = (r.arbitration_id >> 8) & 0xFF
            print(f'  (still servo frame func=0x{func:02X})')
    if got_mit:
        break
    # Also try with extended ID in case motor expects that
    msg2 = can.Message(arbitration_id=(8 << 8) | motor_id, data=enter_data, is_extended_id=True)
    replies2 = send_recv(msg2, 'MIT_ENTER (ext frame)', timeout=0.2)
    for r in replies2:
        if not r.is_extended_id and len(r.data) >= 6:
            if r.data[0] == motor_id:
                print(f'  >>> MIT ENTER via EXT ACKNOWLEDGED! <<<')
                got_mit = True
    if got_mit:
        break

if got_mit:
    # ==============================
    # STEP 4: Send MIT zero-torque
    # ==============================
    print(f'\n{"="*60}')
    print(f'STEP 4: MIT zero-torque command')
    print('='*60)
    def float_to_uint(x, xmin, xmax, bits):
        span = xmax - xmin
        x = max(xmin, min(xmax, x))
        return int((x - xmin) * ((1<<bits)-1) / span)
    p = float_to_uint(0, -12.5, 12.5, 16)
    v = float_to_uint(0, -45.5, 45.5, 12)
    kp = float_to_uint(0, 0, 500, 12)
    kd = float_to_uint(0.5, 0, 5, 12)
    tff = float_to_uint(0, -5, 5, 12)
    d = bytearray(8)
    d[0]=(p>>8)&0xFF; d[1]=p&0xFF
    d[2]=(v>>4)&0xFF; d[3]=((v&0xF)<<4)|((kp>>8)&0xF)
    d[4]=kp&0xFF; d[5]=(kd>>4)&0xFF
    d[6]=((kd&0xF)<<4)|((tff>>8)&0xF); d[7]=tff&0xFF
    msg = can.Message(arbitration_id=motor_id, data=bytes(d), is_extended_id=False)
    replies = send_recv(msg, 'MIT zero-torque kd=0.5')
    for r in replies:
        if not r.is_extended_id and len(r.data) >= 6 and r.data[0] == motor_id:
            print(f'  >>> MIT VERIFIED - motor is in MIT mode! <<<')
    
    # ==============================
    # STEP 5: Send MIT 0.5 Nm
    # ==============================
    print(f'\n{"="*60}')
    print(f'STEP 5: MIT torque 0.5 Nm with kd=0.5 (should spin slowly)')
    print('='*60)
    tff = float_to_uint(0.5, -5, 5, 12)
    d[6]=((kd&0xF)<<4)|((tff>>8)&0xF); d[7]=tff&0xFF
    msg = can.Message(arbitration_id=motor_id, data=bytes(d), is_extended_id=False)
    for i in range(20):  # send for 2 seconds
        bus.send(msg)
        r = bus.recv(timeout=0.08)
        if r and not r.is_extended_id and len(r.data) >= 6:
            def uint_to_float(xi, xmin, xmax, bits):
                return float(xi)*(xmax-xmin)/float((1<<bits)-1)+xmin
            pi = (r.data[1]<<8)|r.data[2]
            vi = (r.data[3]<<4)|(r.data[4]>>4)
            ti = ((r.data[4]&0xF)<<8)|r.data[5]
            pos = uint_to_float(pi, -12.5, 12.5, 16)
            spd = uint_to_float(vi, -45.5, 45.5, 12)
            tau = uint_to_float(ti, -5, 5, 12)
            print(f'  [{i:2d}] pos={pos:+.2f}rad spd={spd:+.2f}rad/s tau={tau:+.3f}Nm')
        time.sleep(0.1)
    # STOP
    tff = float_to_uint(0, -5, 5, 12)
    d[6]=((kd&0xF)<<4)|((tff>>8)&0xF); d[7]=tff&0xFF
    msg = can.Message(arbitration_id=motor_id, data=bytes(d), is_extended_id=False)
    bus.send(msg)
    print('  Stopped (0 torque)')
else:
    print(f'\n  MIT mode NOT available for motor {motor_id}')
    print('  Motor stays in servo mode.')
    print('  Servo current command WAS sent - did motor react at all?')

# EXIT MIT
flush()
exit_data = bytes([0xFF]*7 + [0xFD])
msg = can.Message(arbitration_id=motor_id, data=exit_data, is_extended_id=False)
bus.send(msg)
print('\nMIT EXIT sent. Done.')
bus.shutdown()
