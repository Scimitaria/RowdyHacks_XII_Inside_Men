"""Measure each wheel's top speed in encoder ticks/s, to set MAX_TARGET_TPS
in pi_pico_PID/main.c.

Needs the open-loop pi_pico_code firmware on the Pico (it drives both wheels
at 100% duty with "7 100 100"). Lift the wheels off the ground first.

Usage: python3 get_slots_per_sec.py [port] [seconds]   (default /dev/ttyACM0, 2 s)
"""

import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
measure_s = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
SPIN_UP_S = 1.0          # let the motors reach full speed before measuring
MARGIN = 0.9             # leave headroom so the PI loop isn't pinned at 100%


def command(pico, line):
    """Send one command and return the reply lines up to its "OK"/"ERR"."""
    pico.reset_input_buffer()   # drop anything left over from before
    pico.write((line + '\n').encode())
    replies = []
    while True:
        reply = pico.readline().decode(errors='ignore').strip()
        if not reply:
            raise RuntimeError(f'No reply from the Pico to {line!r}')
        replies.append(reply)
        if reply.startswith('ERR'):
            raise RuntimeError(f'Pico rejected {line!r}: {reply} '
                               '(is it running the pi_pico_code firmware?)')
        if reply.startswith('OK'):
            return replies


def read_encoders(pico):
    """Return (count_a, count_b, time_s) from the Pico's "ENC" reply."""
    for reply in command(pico, '6'):
        parts = reply.split()
        if len(parts) == 4 and parts[0] == 'ENC':
            return int(parts[1]), int(parts[2]), int(parts[3]) * 1e-6
    raise RuntimeError('No ENC line in the reply to "6"')


# timeout: how long readline waits for the Pico before giving up
with serial.Serial(port, 115200, timeout=0.5, write_timeout=0.5) as pico:
    try:
        command(pico, '7 100 100')
        print(f'Both wheels at 100%, spinning up for {SPIN_UP_S} s...')
        time.sleep(SPIN_UP_S)

        a0, b0, t0 = read_encoders(pico)
        time.sleep(measure_s)
        a1, b1, t1 = read_encoders(pico)
    finally:
        pico.write(b'5\n')   # always stop, even on an error or Ctrl+C

    dt = t1 - t0
    left = (a1 - a0) / dt
    right = (b1 - b0) / dt
    print(f'Measured over {dt:.2f} s:')
    print(f'  left  (A): {left:8.0f} ticks/s')
    print(f'  right (B): {right:8.0f} ticks/s')

    if left <= 0 or right <= 0:
        print('A wheel counted down (or not at all) while driving forward: check its\n'
              'ENC_*_REVERSED / MOTOR_B_REVERSED setting and wiring before using the PI firmware.')
        sys.exit(1)

    max_tps = int(min(left, right) * MARGIN)
    kp = 30 / max_tps
    print('\nFor pi_pico_PID/main.c:')
    print(f'  #define MAX_TARGET_TPS     {max_tps}')
    print(f'  #define KP                 {kp:.4f}f    // starting point')
    print(f'  #define KI                 {5 * kp:.4f}f    // starting point')
