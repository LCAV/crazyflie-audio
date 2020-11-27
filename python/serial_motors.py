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

# (distance (cm), command, time (s))
move = {
    'forward': [
        (0.1,  b"q", 2.0),
        (1, b"w", 5.0),
        (10, b"e", 34.0)
    ],
    'backward': [
        (0.1,  b"a", 2.0),
        (1, b"s", 5.0),
        (10, b"d", 34.0)
    ]
}
turn = {
    'forward': [
        (27,  b"p", 3),
        (90, b"o", 5),
        (360, b"i", 17)
    ],
    'backward': [
        (27,  b"l", 3),
        (90, b"k", 5),
        (360, b"j", 17)
    ]
}

class SerialMotors(object):
    def __init__(self, port=SERIAL_PORT, baudrate=115200, verbose=False):
        self.port = SERIAL_PORT
        self.serial = serial.Serial(port, baudrate)
        self.verbose = verbose

    # turn is by default non-blocking because when we do the 360 degrees we 
    # want to recording DURING, not after, as for the others.
    def turn(self, angle_deg, blocking=False):
        self.move_in_chunks(turn['forward'], angle_deg, blocking=blocking)

    def turn_back(self, angle_deg, blocking=True):
        self.move_in_chunks(turn['backward'], angle_deg, blocking=blocking)

    def move(self, delta_cm, blocking=True):
        self.move_in_chunks(move['forward'], delta_cm, blocking=blocking)

    def move_back(self, delta_cm, blocking=True):
        self.move_in_chunks(move['backward'], delta_cm, blocking=blocking)

    def move_in_chunks(self, commands, total, blocking=True):
        leftover = total
        commands_decreasing = sorted(commands, key=lambda tuple_: tuple_[0])[::-1]
        for partial, command, time_s in commands_decreasing:
            num_commands = int(leftover // partial)

            if self.verbose:
                print(f'moving {leftover} in chunks of {partial}')

            if (num_commands > 1) and not blocking:
                if self.verbose:
                    print(f'cannot move by {leftover} in non-blocking mode.')
                blocking = True

            for i in range(num_commands):
                if self.verbose:
                    print(f'running command {i+1}/{num_commands}')
                self.serial.write(command)

                if blocking:
                    time.sleep(time_s) # wait for linear movement to be done

            leftover = leftover % partial

        if leftover != 0:
            print(f'Warning: not moving by last {leftover} cm.')


if __name__ == "__main__":
    sm = SerialMotors(verbose=True)
    #sm.turn(360)
    #sm.turn_back(360)
    sm.move(1)