import serial
import time
from threading import Thread
import re

ser_sender = None
ser_prevent = None

while not ser_sender:
    try:
        ser_sender = serial.Serial(
            port='COM4',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
    except:
        print('Unable to connect sender')

while not ser_prevent:
    try:
        ser_prevent = serial.Serial(
            port='COM6',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
    except:
        print('Unable to connect prevent')

print("connected to: " + ser_prevent.portstr)

power = -1
sf_sender = 7
sf_prevent = 7
reset = 0

time.sleep(5)


def read_prevent():
    while 1:
        line_prev = ser_prevent.read(10000000000000000000000000000000000000)
        if line_prev != b'':
            print(line_prev.decode('utf-8'), end='')


th = Thread(target=read_prevent)
th.start()

line_send = ''

while 1:
    line_send_read = ser_sender.read(10000000000000000000000000000000000000)
    if line_send_read != b'':
        line_send = line_send + line_send_read.decode('utf-8')
    else:
        continue
    # print(line_send, end='')
    if 'Po' in line_send:
        # print(line_send)
        try:
            power = int(re.findall(r'[-+]?\d+', line_send)[0])
        except:
            print('Не смог найти число в строке')
        ser_prevent.write(b'%a\n\r' % power)
        ser_prevent.write(b'%a\n\r' % sf_prevent)
    elif 'Do' in line_send:
        print('HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHERE')
        sf_prevent += 1
        power = -1
        ser_prevent.write(b'%a\n\r' % power)
        ser_prevent.write(b'%a\n\r' % sf_prevent)
        time.sleep(5)
    if '\n' in line_send:
        line_send = ''
