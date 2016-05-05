import mpu6050
import math
import smbus
from mux import Multiplex

mx = Multiplex(1)

sensors = [2, 3, 4, 5]

# acc x, y, z, gyro x, y, z
offsets = {
    2: [2563, 2128, 1306, 85, -18, 22],
    3: [-1523, -276, 1773, 60, -13, -3],
    4: [-3131, 1430, 1332, 72, 10, 26],
    5: [-1944, -823, 1969, 121, -11, 20]
}

mpus = {}

# initialize
for i in sensors:
    mpus[i] = mpu6050.MPU6050()
    mpus[i].dmpInitialize()
    mpus[i].setDMPEnabled(True)
    mpus[i].setXAccelOffset(offsets[i][0])
    mpus[i].setYAccelOffset(offsets[i][1])
    mpus[i].setZAccelOffset(offsets[i][2])
    mpus[i].setXGyroOffset(offsets[i][3])
    mpus[i].setYGyroOffset(offsets[i][4])
    mpus[i].setZGyroOffset(offsets[i][5])
    packetSize = mpus[i].dmpGetFIFOPacketSize()


while True:
    for i in sensors:
        mpu = mpus[i]

        mx.channel(0x70, i)

        fifoCount = mpu.getFIFOCount()
        if fifoCount == 1024:
            mpu.resetFIFO()
            #print('FIFO overflow!')

        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()

        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        ypr = mpu.dmpGetYawPitchRoll(q, g)

        print i, q['x'], q['y'], q['z'], q['w']

        #print(ypr['yaw'] * 180 / math.pi),
        #print(ypr['pitch'] * 180 / math.pi),
        #print(ypr['roll'] * 180 / math.pi)

        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize
