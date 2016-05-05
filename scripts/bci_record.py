import serial
import sys
import time
import struct

# EDIT !
PORT = '/dev/tty.usbserial-DB00MAB7'
TO_CSV = True

START_BYTE = 0xA0
END_BYTE = 0xC0

BAUD_RATE = 115200

ser = serial.Serial(PORT, BAUD_RATE, timeout=10)

time.sleep(1)

ser.write(b'v')

time.sleep(1)


def read(n):
    b = ser.read(n)
    if not b:
        print 'ops'
        sys.exit()
    return b


# reset state
line = ''
while '$$$' not in line:
    c = ser.read().decode('utf-8')
    line += c
    sys.stdout.write(str(c))


state = 1

ser.write(b'b')


outfile = open('bci_%d.csv' % time.time(), 'w')
row = []
row_count = 0
while True:
    try:
        # header
        if state == 1:

            b = read(1)
            if struct.unpack('B', b)[0] == START_BYTE:
                sample_n = struct.unpack('B', read(1))[0]
                row = [time.time(), sample_n]

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
                row += [value]

            state = 3

        elif state == 3:

            # accelerometer
            for a in xrange(3):
                acc = struct.unpack('>h', read(2))[0]

            state = 4

        elif state == 4:
            if len(line) == 10:
                outfile.write(','.join(map(str, row)) + '\n')
                row_count += 1
            else:
                assert False
            # end
            be = struct.unpack('B', read(1))[0]

            state = 1

    except KeyboardInterrupt:
        print 'Wrote %d events' % row_count
        ser.write(b's')
        ser.close()
        outfile.close()
        sys.exit(0)
