#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.callbacks import Caller
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
import logging

import numpy as np

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
id = 'radio://0/80/2M'

class Audio_CRTP(object):
    """
    Crazyflie console is used to receive characters printed using printf
    from the firmware.
    """

    def __init__(self, crazyflie):
        """
        Initialize the console and register it to receive data from the copter.
        """
        self.corr_matrix = np.zeros((32,32))
        self.packet_count = 141
        self.packet_payload = 29
        self.receivedChar = Caller()
        self.array = np.zeros(4096)
        self.start = 0
        self.index = 0
        self.start_time = 0
        self.cf = crazyflie
        self.cf.add_port_callback(CRTPPort.AUDIO, self.incoming)

    def incoming(self, packet):
        """
        Callback for data received from the copter.
        """
        # This might be done prettier ;-)
        receive_byte = packet.data
        if packet.channel == 1:
            if packet.datal[0] == 0:
                if self.index != 0 and self.index != self.packet_count+1:
                    print("packets loss")
                self.index = 0
                self.start = 1
                self.start_time = time.time()
        elif self.start:
            if self.index == self.packet_count:
                self.array[self.index * self.packet_payload:self.index * self.packet_payload + 7] = packet.datal[0:7]
                self.array.astype(np.uint8).tofile("bin")
                self.corr_matrix = np.fromfile("bin",np.float32).reshape((32,32))
                print("Elapsed time = {}s ".format(time.time()-self.start_time))
            else:
                self.array[self.index*self.packet_payload:(self.index+1)*self.packet_payload] = packet.datal
            self.index += 1


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(id) as scf:
        time.sleep(5)
        cf = scf.cf
        audio_crtp = Audio_CRTP(cf)
        while True:
            time.sleep(1)

