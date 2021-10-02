#!/usr/bin/python3

import picamera
import threading
import Adafruit_BMP.BMP085 as BMP085
from mpu9250_i2c import *
from time import time, sleep
import csv
import serial

# from mpu6050_i2c import mpu6050_conv

TIMEOUT = 5
start_time = int(time())
csv_header = ['temp', 'pres', 'alt', 'ax', 'ay', 'az']

def record():
    camera = picamera.PiCamera(framerate=60)
    camera.resolution = (640, 480)
    camera.start_recording(f'/home/pi/recording_{start_time}.h264')
    camera.wait_recording(TIMEOUT)
    camera.stop_recording()

if __name__ == '__main__':
    bmp = BMP085.BMP085()

    ser = serial.Serial(
        port='/dev/serial0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        timeout=1,
        write_timeout=0.1
    )

    if not ser.is_open:
        ser.open()

    t1 = threading.Thread(target=record, daemon=True)
    t1.start()

    with open(f'/home/pi/data_{start_time}.csv', 'w', encoding='UTF-8') as f:
        writer = csv.writer(f)
        writer.writerow(csv_header)

        while int(time()) - start_time < TIMEOUT:
            # try:
            ax, ay, az, _, _, _ = mpu6050_conv()
            data = [
                '{0:0.2f}'.format(bmp.read_temperature()),
                '{0:0.2f}'.format(bmp.read_pressure()),
                '{0:0.2f}'.format(bmp.read_altitude()),
                '{0:0.2f}'.format(ax),
                '{0:0.2f}'.format(ay),
                '{0:0.2f}'.format(az)
                ]

            writer.writerow(data)
            ser_data = f'{",".join(data)}#\n'.encode('utf-8')
            ser.write(ser_data)
            ser.flush()
        # except:
            #     continue

            sleep(0.5)

    t1.join()
    ser.close()
