"""Send '6' to the Pico once and print what it replies.

Usage: python3 get_slot_node.py [port]   (default /dev/ttyACM0)
"""

import sys

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

# timeout: how long readline waits for the Pico before giving up
with serial.Serial(port, 115200, timeout=0.5, write_timeout=0.5) as pico:
    pico.reset_input_buffer()   # drop anything left over from before
    pico.write(b'6\n')

    # The Pico answers with "ENC <count_a> <count_b> <time_us>", then "OK 6".
    while True:
        line = pico.readline().decode(errors='ignore').strip()
        if not line:
            print('No (more) reply from the Pico')
            break
        print(line)
        if line.startswith(('OK', 'ERR')):
            break
