import sys
import time
import serial


# EDIT !
PORT = "/dev/cu.usbmodem1411"
TO_CSV = True

BAUD_RATE = 115200

ser = serial.Serial(PORT, BAUD_RATE)
ser.timeout = 10

row_count = 0
# outfile = open('imu_%d.csv' % time.time(), 'w')
while True:
    try:
        line = ser.readline().strip()
        ts = time.time()
        line = '{},'.format(ts) + line
        # outfile.write(line + '\n')
        # print line
        row_count += 1

    except KeyboardInterrupt:
        break

print 'Wrote %d events' % row_count
ser.close()
# outfile.close()
