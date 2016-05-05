import smbus

class Multiplex:

  def __init__(self, bus):
    self.bus = smbus.SMBus(bus)

  def channel(self, address=0x70, channel=0):
    if (channel == 0): action = 0x00
    elif (channel == 1): action = 0x02
    elif (channel == 2): action = 0x04
    elif (channel == 3): action = 0x08
    elif (channel == 4): action = 0x10
    elif (channel == 5): action = 0x20
    elif (channel == 6): action = 0x40
    elif (channel == 7): action = 0x80    
    else: raise Exception("ERROR: Wrong mux port")

    self.bus.write_byte(address, action)


if __name__ == '__main__':
  bus = 1
  address = 0x70
  plexer = Multiplex(bus)
  plexer.channel(address, 4)
  print 'Now run i2cdetect'
