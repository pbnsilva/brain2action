import serial
import sys
import time
import struct


START_BYTE = 0xA0
END_BYTE = 0xC0

port = "/dev/tty.usbserial-DB00MAB7"
baudrate = 115200

ser = serial.Serial(port, baudrate, timeout=10)

# Reset the state
time.sleep(1)
ser.write(b'v')
time.sleep(1)

line = ''
while '$$$' not in line:
    c = ser.read().decode('utf-8')
    line += c
    sys.stdout.write(str(c))

def read(n):
    b = ser.read(n)
    if not b:
        print 'ops'
        sys.exit()
    return b

state = 1

# Starts the stream
ser.write(b'b')

while True:
    try:
        # header
        if state == 1:
            b = read(1)
            if struct.unpack('B', b)[0] == START_BYTE:
                sample_n = struct.unpack('B', read(1))[0]
                print 'sample ', str(sample_n)
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
                print 'EEG ', mi * 4.5 / float((pow(2, 23) - 1)) / 24. * 1000000.
            state = 3

        elif state == 3:
            # accelerometer
            for a in xrange(3):
                acc = struct.unpack('>h', read(2))[0]
                print 'ACC ', acc * 0.002 / pow(2, 4)
            state = 4

        elif state == 4:
            # end
            be = struct.unpack('B', read(1))[0]
            state = 1

    except KeyboardInterrupt:
        print 'closing'
        ser.write(b's')
        ser.close()
        sys.exit()
