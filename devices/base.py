import serial


class SerialDevice(object):

    def __init__(self, port, baud_rate=115200):
        self._ser = serial.Serial(port, baud_rate)
        self._ser.timeout = 10

    def read_line(self):
        return self._ser.readline().strip()

    def close(self):
        self._ser.close()
