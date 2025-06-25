# testing script designed to send high or low signals via serial port

import serial

def send_byte(x):
    ser = serial.Serial('/dev/ttyUSB0')
    print(ser.name)
    ser.write(bytes[x])
    ser.close()


send_byte(1)
