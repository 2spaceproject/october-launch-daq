#!/usr/bin/python3
import serial

with serial.Serial(
        port='/dev/ttyUSB0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        timeout=1,
        write_timeout=0.1,
    ) as ser:

    while True:
        line = ser.readline()
        print(line)