import serial
import time

ser_receiver = None

while not ser_receiver:
    try:
        ser_receiver = serial.Serial(
            port='COM6',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
    except:
        print('Unable to connect receiver')

print("connected to: " + ser_receiver.portstr)

power = -1
sf = 7
reset = 0


with open('{}.txt'.format(sf), 'w') as f:
    while 1:
        try:
            line_recv = ser_receiver.read(10000000000000000000000000000000000000)
            if line_recv != b'':
                f.write(line_recv.decode('utf-8'))
        except:
            break



