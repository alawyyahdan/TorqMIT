#!/usr/bin/env python3
"""Continuous MIT test - send commands, print ALL bus traffic"""
import can, time, struct, sys

bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=1000000)

def float_to_uint(x, xmin, xmax, bits):
    span = xmax - xmin
    x = max(xmin, min(xmax, x))
    return int((x - xmin) * ((1<<bits)-1) / span)

def uint_to_float(xi, xmin, xmax, bits):
    return float(xi)*(xmax-xmin)/float((1<<bits)-1)+xmin

def make_mit_cmd(p=0, v=0, kp=0, kd=0.5, tff=0):
    pi = float_to_uint(p, -12.5, 12.5, 16)
    vi = float_to_uint(v, -45.5, 45.5, 12)
    kpi = float_to_uint(kp, 0, 500, 12)
    kdi = float_to_uint(kd, 0, 5, 12)
    ti = float_to_uint(tff, -5, 5, 12)
    d = bytearray(8)
    d[0]=(pi>>8)&0xFF; d[1]=pi&0xFF
    d[2]=(vi>>4)&0xFF; d[3]=((vi&0xF)<<4)|((kpi>>8)&0xF)
    d[4]=kpi&0xFF; d[5]=(kdi>>4)&0xFF
    d[6]=((kdi&0xF)<<4)|((ti>>8)&0xF); d[7]=ti&0xFF
    return bytes(d)

# Auto-detect motor ID
print('Scanning bus 0.5s...')
motor_id = None
t0 = time.time()
while (time.time()-t0) < 0.5:
    r = bus.recv(timeout=0.1)
    if r:
        hx = ' '.join(f'{b:02X}' for b in r.data)
        ext = 'EXT' if r.is_extended_id else 'STD'
        print(f'  {ext} 0x{r.arbitration_id:X} {hx}')
        if r.is_extended_id:
            func = (r.arbitration_id >> 8) & 0xFF
            mid = r.arbitration_id & 0xFF
            if func == 0x29:
                motor_id = mid
        elif len(r.data) >= 6:
            mid = r.data[0]
            if 0 < mid < 128:
                motor_id = mid

if motor_id is None:
    motor_id = int(input('Motor ID not found. Enter manually: '))
print(f'Motor ID = {motor_id}')

# Enter MIT
print('\nEntering MIT mode...')
while bus.recv(timeout=0.01): pass  # flush
enter = bytes([0xFF]*7+[0xFC])
for i in range(3):
    msg = can.Message(arbitration_id=motor_id, data=enter, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.05)
    r = bus.recv(timeout=0.1)
    if r:
        hx = ' '.join(f'{b:02X}' for b in r.data)
        ext = 'EXT' if r.is_extended_id else 'STD'
        print(f'  Reply: {ext} 0x{r.arbitration_id:X} {hx}')
        if not r.is_extended_id and len(r.data) >= 6 and r.data[0] == motor_id:
            print(f'  MIT ENTERED!')
            break

print(f'\n=== Sending MIT cmds at 50Hz for 10s ===')
print(f'=== Move motor by hand - should see position/speed change ===')
print(f'=== Press Ctrl+C to stop ===\n')

# Phase 1: 5s zero torque (just read feedback while you move motor)
try:
    count = 0
    rx_count = 0
    no_reply = 0
    t_start = time.time()
    
    while True:
        elapsed = time.time() - t_start
        if elapsed > 10:
            break
        
        # Switch to 0.5 Nm after 5 seconds
        if elapsed < 5:
            tff = 0.0
            phase = 'ZERO torque (move motor by hand)'
        else:
            tff = 0.5
            phase = '0.5 Nm torque (should spin slow)'
        
        data = make_mit_cmd(p=0, v=0, kp=0, kd=0.5, tff=tff)
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        bus.send(msg)
        
        # Read ALL replies (not just one)
        got_reply = False
        for _ in range(5):
            r = bus.recv(timeout=0.005)
            if r is None: break
            hx = ' '.join(f'{b:02X}' for b in r.data)
            ext = 'EXT' if r.is_extended_id else 'STD'
            
            if not r.is_extended_id and len(r.data) >= 6 and r.data[0] == motor_id:
                # Parse MIT reply
                pi = (r.data[1]<<8)|r.data[2]
                vi = (r.data[3]<<4)|(r.data[4]>>4)
                ti = ((r.data[4]&0xF)<<8)|r.data[5]
                pos = uint_to_float(pi, -12.5, 12.5, 16)
                spd = uint_to_float(vi, -45.5, 45.5, 12)
                tau = uint_to_float(ti, -5, 5, 12)
                tmp = r.data[6] - 40 if len(r.data) >= 7 else 0
                err = r.data[7] if len(r.data) >= 8 else 0
                rx_count += 1
                got_reply = True
                if count % 5 == 0:  # print every 5th
                    print(f'  [{elapsed:5.1f}s] {phase}')
                    print(f'         pos={pos:+7.3f}rad spd={spd:+7.3f}rad/s tau={tau:+6.3f}Nm T={tmp}C err={err}')
            elif r.is_extended_id:
                func = (r.arbitration_id >> 8) & 0xFF
                if count % 10 == 0:
                    print(f'  [{elapsed:5.1f}s] SERVO frame still coming (0x{func:02X}) - MIT may not be active!')
            else:
                if count % 10 == 0:
                    print(f'  [{elapsed:5.1f}s] Unknown: {ext} 0x{r.arbitration_id:X} {hx}')
        
        if not got_reply:
            no_reply += 1
            if no_reply == 1 or no_reply % 50 == 0:
                print(f'  [{elapsed:5.1f}s] No reply (x{no_reply}) - check if motor is in MIT')
        
        count += 1
        time.sleep(0.02)  # ~50Hz

except KeyboardInterrupt:
    print('\nStopped by user')

# Stop & Exit
data = make_mit_cmd(p=0, v=0, kp=0, kd=0.5, tff=0)
msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
bus.send(msg)
time.sleep(0.05)
exit_data = bytes([0xFF]*7+[0xFD])
msg = can.Message(arbitration_id=motor_id, data=exit_data, is_extended_id=False)
bus.send(msg)
print(f'\nDone. TX={count} RX={rx_count} no_reply={no_reply}')
bus.shutdown()
