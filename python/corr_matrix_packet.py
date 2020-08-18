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

FFTSIZE = 32
nMic = 4
CRTP_PAYLOAD = 29
N_BYTES = FFTSIZE*nMic*nMic*2*4 # *2 for complex numbers, *4 for bytes in float
N_PACKETS,N_BYTES_LAST_PACKET = divmod(N_BYTES,CRTP_PAYLOAD) # N_PACKETS = number of packets -1

class Audio_CRTP(object):
    """
    Audio CRTP recovers the correlation matrix through the CRTP protocol. The matrix is sent in 142 packets of 29 bytes.
    The protocol has a start condition and then counts the incoming packets.
    """

    def __init__(self, crazyflie):
        """
        Initialisation
        """
        self.corr_matrix = np.zeros((FFTSIZE,nMic*nMic*2))
        self.receivedChar = Caller()
        self.array = np.zeros(N_BYTES)
        self.start = 0
        self.index = 0
        self.start_time = 0
        self.cf = crazyflie
        self.cf.add_port_callback(CRTPPort.AUDIO, self.incoming)

    def incoming(self, packet):
        """
        Callback for data received from the copter.
        """
        if packet.channel == 1: # Channel 1 is used for start condition
            if packet.datal[0] == 0:
                if self.index != 0 and self.index != N_PACKETS+1: # n packets received != expectation
                    print("packets loss")
                self.index = 0 # reset index
                self.start = 1
                self.start_time = time.time()
        elif self.start:
            if self.index == N_PACKETS:
                self.array[self.index * CRTP_PAYLOAD:self.index * CRTP_PAYLOAD + N_BYTES_LAST_PACKET] = \
                    packet.datal[0:N_BYTES_LAST_PACKET] # last bytes
                self.array.astype(np.uint8).tofile("bin") # write array to dummy bin file to convert uint8 to float32
                self.corr_matrix = np.fromfile("bin",np.float32).reshape((FFTSIZE,nMic*nMic*2))
                print("Elapsed time = {}s ".format(time.time()-self.start_time))
            else:
                self.array[self.index*CRTP_PAYLOAD:(self.index+1)*CRTP_PAYLOAD] = packet.datal
            self.index += 1


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        audio_CRTP = Audio_CRTP(cf)
        while True:
            time.sleep(1)

