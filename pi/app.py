import mpu6050
import math
import smbus

sensor_count = 2

mpus = {}

# initialize
for i in xrange(sensor_count):
    mpus[i] = mpu6050.MPU6050()
    mpus[i].dmpInitialize()
    mpus[i].setDMPEnabled(True)
    packetSize = mpus[i].dmpGetFIFOPacketSize()


while True:
    for i in xrange(sensor_count):
        mpu = mpus[i]

        fifoCount = mpu.getFIFOCount()
        if fifoCount == 1024:
            mpu.resetFIFO()
            print('FIFO overflow!')

        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        ypr = mpu.dmpGetYawPitchRoll(q, g)

        print i
        print(ypr['yaw'] * 180 / math.pi),
        print(ypr['pitch'] * 180 / math.pi),
        print(ypr['roll'] * 180 / math.pi)

        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize
