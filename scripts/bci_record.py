import serial
import sys
import time
import struct
import numpy as np

# EDIT !
PORT = '/dev/tty.usbserial-DB00MAB7'
TO_CSV = True

START_BYTE = 0xA0
END_BYTE = 0xC0

BAUD_RATE = 115200

USE_DAISY = True

print('Initializing...')

ser = serial.Serial(PORT, BAUD_RATE, timeout=10)

time.sleep(1)

ser.write(b'v')

time.sleep(1)


def read(n):
    b = ser.read(n)
    if not b:
        print 'ops'
        sys.exit(2)
    return b


# reset state
line = ''
while '$$$' not in line:
    try:
        c = ser.read().decode('utf-8')
        line += c
        # sys.stdout.write(str(c))
    except UnicodeDecodeError, e:
        ser.write(b's')
        ser.close()
        print('Error: %s' % e)
        sys.exit(2)

print('Received header')

state = 1

ser.write(b'b')

if TO_CSV:
    outfile = open('bci_%d.csv' % time.time(), 'w')

row = np.zeros(10 + 8 * int(USE_DAISY))
row_count = 0
is_daisy_sample = False
is_running = True
while is_running:
    try:
        # header
        if state == 1:

            b = read(1)
            if struct.unpack('B', b)[0] == START_BYTE:
                sample_n = struct.unpack('B', read(1))[0]
                row[0] = time.time()
                row[1] = sample_n
                state = 2

        elif state == 2:

            # channels
            for c in xrange(8):
                bc = read(3)
                ubc = struct.unpack('3B', bc)
                # 3byte int in 2s compliment
                if (ubc[0] >= 127):
                    pre_fix = '\xFF'
                else:
                    pre_fix = '\x00'
                bcr = pre_fix + bc
                mi = struct.unpack('>i', bcr)[0]
                value = mi * 4.5 / float((pow(2, 23) - 1)) / 24. * 1000000.
                row[2 + c + 8 * int(is_daisy_sample)] = value

            state = 3

        elif state == 3:

            # accelerometer
            for a in xrange(3):
                acc = struct.unpack('>h', read(2))[0]

            state = 4

        elif state == 4:
            if is_daisy_sample or not USE_DAISY:
                if len(row) == 2 + 8 + 8 * int(USE_DAISY):
                    # print row
                    if TO_CSV:
                        outfile.write(','.join(map(str, row)) + '\n')
                    row_count += 1
                else:
                    print('Wrong channel count!')
                    ser.write(b's')
                    ser.close()
                    sys.exit(2)
                row = np.zeros(10 + 8 * int(USE_DAISY))
            is_daisy_sample = USE_DAISY and not is_daisy_sample
            # end
            be = struct.unpack('B', read(1))[0]

            if row_count > 4:
                raise KeyboardInterrupt

            state = 1

    except KeyboardInterrupt:
        is_running = False
        print('Read %d events' % row_count)
        ser.write(b's')
        ser.close()
        if TO_CSV:
            outfile.close()
