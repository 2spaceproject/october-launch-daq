#!/usr/bin/python3

import picamera
import threading
import Adafruit_BMP.BMP085 as BMP085
from mpu9250_i2c import *
from time import time, sleep, ctime
import csv
import serial
import RPi.GPIO as GPIO

TIMEOUT = 99999
start_time = int(time())
csv_header = ['temp', 'pres', 'alt', 'ax', 'ay', 'az']

threshold_alt = 4000.0

def kalman_calc(altitude):
    global kalman_x_last
    global kalman_p_last
    kalman_x_temp = kalman_x_last
    kalman_p_temp = kalman_p_last + kalman_r
    kalman_k = (f_1/(kalman_p_temp + kalman_q)) * kalman_p_temp
    kalman_x = kalman_x_temp + (kalman_k * (altitude - kalman_x_temp))
    kalman_p = (f_1 - kalman_k) * kalman_p_temp
    kalman_x_last = kalman_x
    kalman_p_last = kalman_p

    return kalman_x

f_1 = 1.0
kalman_k = 0.0
kalman_q = 4.0001
kalman_r = .20001
kalman_x = 0.0
kalman_p = 0.0
kalman_x_temp = 0.0
kalman_p_temp = 0.0
kalman_x_last = 0.0
kalman_p_last = 0.0
last_altitude = 0.0
measures = 10
apogee_altitude = 0.0
apogee_fired = False
main_deploy_altitude = 450.0
main_fired = False

apogee_pin = 21
main_pin = 26


def record():
    camera = picamera.PiCamera(framerate=60)
    camera.resolution = (1280, 720)
    camera.start_recording(f'/home/pi/recording_{start_time}.h264')
    camera.wait_recording(TIMEOUT)
    camera.stop_recording()

if __name__ == '__main__':

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(apogee_pin, GPIO.OUT)
    GPIO.setup(main_pin, GPIO.OUT)
    GPIO.output(apogee_pin, False)
    GPIO.output(main_pin, False)

    bmp = BMP085.BMP085()

    ser = serial.Serial(
        port='/dev/serial0',
        baudrate = 9600,
        timeout=1,
        write_timeout=0.1
    )

    if not ser.is_open:
        ser.open()

    #t1 = threading.Thread(target=record, daemon=True)
    #t1.start()

    print('Calibrating Kalman filter...')

    for step in range(50):
        try:
            kalman_calc(bmp.read_altitude())
        except:
            continue


    print(f'Computing initial altitude...')
    reads = 0
    sum_alts = 0.0
    while reads < 10:
        try:
            print(reads)
            alt = bmp.read_altitude()
            sum_alts += alt
            reads += 1
        except:
            continue

    initial_alt = sum_alts / reads

    print('Starting to log and transmit data...')

    with open(f'/home/pi/data_{start_time}.csv', 'w', encoding='UTF-8') as f:
        writer = csv.writer(f)
        writer.writerow(csv_header)

        while int(time()) - start_time < TIMEOUT:
            try:
                sleep(0.5)
                ax, ay, az, _, _, _ = mpu6050_conv()
                current_altitude = kalman_calc(bmp.read_altitude()) - initial_alt
                print(f'Kalman Computed current altitude: {current_altitude}')
                if not apogee_fired and current_altitude > threshold_alt:
                    if current_altitude < last_altitude:
                        is_valid = True
                        for _ in range(measures):
                            sleep(0.02)
                            current_altitude = kalman_calc(bmp.read_altitude()) - initial_alt
                            if current_altitude > last_altitude:
                                is_valid = False
                                break
                            else:
                                last_altitude = current_altitude


                        if is_valid:
                            GPIO.output(apogee_pin, True)
                            sleep(2)
                            apogee_fired = True
                            GPIO.output(apogee_pin, False)
                            try:
                                row = f'apogee deployed at {ctime()}#\r\n'
                                writer.writerow(row)
                                ser.write(row.encode('utf-8'))
                            except:
                                sleep(0.1)

                        apogee_altitude = current_altitude

                if apogee_fired and not main_fired and current_altitude <= main_deploy_altitude:
                    GPIO.output(main_pin, True)
                    sleep(2)
                    main_fired = True
                    GPIO.output(main_pin, False)
                    try:
                        row = f'main deployed at {ctime()}#\r\n'
                        writer.writerow(row)
                        ser.write(row.encode('utf-8'))
                    except:
                        sleep(0.1)

                last_altitude = current_altitude

                data = [
                    '{0:0.2f}'.format(bmp.read_temperature()),
                    '{0:0.2f}'.format(bmp.read_pressure()),
                    '{0:0.2f}'.format(current_altitude),
                    '{0:0.2f}'.format(ax),
                    '{0:0.2f}'.format(ay),
                    '{0:0.2f}'.format(az)
                    ]

                writer.writerow(data)
                ser_data = f'{",".join(data)}#\r\n'.encode('utf-8')
                print(ser_data)
                ser.write(ser_data)
                ser.flush()
            except KeyboardInterrupt:
                GPIO.cleanup()
                ser.close()
                exit()


    #t1.join()
    ser.close()
