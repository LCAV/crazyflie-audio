#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
serial_motors.py: Class to control serial motors (linear and rotational)
"""
import serial
import time

# Ubuntu:
# to find serial port, run python -m serial.tools.list_ports
SERIAL_PORT = "/dev/ttyACM0"

TIME_FOR_10_CM = 15. # seconds
TIME_FOR_90_DEG = 5. # seconds (4)
TIME_FOR_27_DEG = 3 # seconds (2) 
TIME_FOR_360_DEG = 17. # seconds (16)

class SerialMotors(object):
    def __init__(self, port=SERIAL_PORT, baudrate=115200):
        self.port = SERIAL_PORT
        self.serial = serial.Serial(port, baudrate)

    def turn(self, angle_deg):
        if angle_deg == 360:
            # start rotating and continue immediately
            self.serial.write(b"i")
        elif angle_deg == 90:
            # wait until rotation is one
            self.serial.write(b"o")
            time.sleep(TIME_FOR_90_DEG)
        elif angle_deg in [27, 54, 81]:
            for i in range(angle_deg // 27):
                self.serial.write(b"p")
                time.sleep(TIME_FOR_27_DEG) # wait for turning to be done

    def turn_back(self, angle_deg):
        if angle_deg == 360:
            self.serial.write(b"j")
            time.sleep(TIME_FOR_360_DEG)
        elif angle_deg == 90:
            self.serial.write(b"k")
            time.sleep(TIME_FOR_90_DEG) # wait for turning to be done
        elif angle_deg in [27, 54, 81]:
            for i in range(angle_deg // 27):
                self.serial.write(b"l")
                time.sleep(4) # wait for turning to be done

    def move(self, delta_cm):
        if delta_cm > 0:
            for i in range(delta_cm // 10):
                self.serial.write(b"q")
                time.sleep(TIME_FOR_10_CM) # wait for linear movement to be done

    def move_back(self, delta_cm):
        for i in range(delta_cm // 10):
            self.serial.write(b"a")
            print('moving 10cm, waiting for', TIME_FOR_10_CM)
            time.sleep(TIME_FOR_10_CM) # wait for linear movement to be done
            print('done')


if __name__ == "__main__":
    sm = SerialMotors()

    #sm.turn(360)
    #time.sleep(TIME_FOR_360_DEG)
    #sm.turn_back(360)

    #sm.turn(90)
    #sm.turn_back(90)

    #sm.turn(27)
    #sm.turn_back(27)

    #sm.move(10)
    sm.move_back(10)
