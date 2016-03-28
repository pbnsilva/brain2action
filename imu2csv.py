import math
import serial
import time
import numpy as np

# Notes
# When initialising have the imus facing outside and not forward!!!


port = "/dev/cu.usbserial-A4007TOu"
baudrate = 115200
outfile = 'imu.csv'

ser = serial.Serial(port, baudrate)

outfile = open(outfile, 'w')


while True:
    try:
        line = ser.readline().strip()
        ts = time.time()
        line = '{},'.format(ts) + line
        print line
        if line.count(',')>5:
            outfile.write(line+'\n')    

    except KeyboardInterrupt:
        ser.close()
