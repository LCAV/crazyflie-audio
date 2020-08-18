#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import time

import numpy as np

from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.callbacks import Caller
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"

FFTSIZE = 32
N_MICS = 4
CRTP_PAYLOAD = 29
FLOAT_PRECISION = 4
N_FLOATS = FFTSIZE * N_MICS * 2  # *2 for complex numbers
N_BYTES = N_FLOATS * FLOAT_PRECISION
N_FULL_PACKETS, N_BYTES_LAST_PACKET = divmod(N_BYTES, CRTP_PAYLOAD)


class Audio_CRTP(object):
    """
    Audio CRTP recovers the correlation matrix through the CRTP protocol. 
    The matrix is sent in packets of CRTP_PAYLOAD bytes each.
    The protocol has a start condition (channel==1) and then counts the incoming packets.
    """

    def __init__(self, crazyflie):
        self.array = np.zeros(N_BYTES, dtype=np.uint8)
        self.corr_matrix = np.zeros(N_FLOATS, dtype=np.float32)

        self.receivedChar = Caller()
        self.start = False
        self.index = 0
        self.start_time = 0
        self.cf = crazyflie
        self.cf.add_port_callback(CRTPPort.AUDIO, self.callback_incoming)

    def callback_incoming(self, packet):
        # We send the first package in channel 1 to identify the start of new audio data.
        if packet.channel == 1:
            if (self.index != 0) and (self.index != N_FULL_PACKETS + 1):
                print(f"packets loss: received only {self.index}/{N_FULL_PACKETS+1}")
            self.index = 0  # reset index
            self.start = True
            self.start_time = time.time()

        if self.start:
            # received all full packets, read remaining bytes
            if self.index == N_FULL_PACKETS:
                self.array[
                    self.index * CRTP_PAYLOAD : self.index * CRTP_PAYLOAD
                    + N_BYTES_LAST_PACKET
                ] = packet.datal[
                    0:N_BYTES_LAST_PACKET
                ]  # last bytes

                self.corr_matrix = np.frombuffer(self.array, dtype=np.float32) # conversion of uint8 data to float32

                print(
                    f"Elapsed time for receiving audio data = {time.time() - self.start_time}s"
                )
            else:
                self.array[
                    self.index * CRTP_PAYLOAD : (self.index + 1) * CRTP_PAYLOAD
                ] = packet.datal
            self.index += 1


if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        audio_CRTP = Audio_CRTP(cf)
        while True:
            time.sleep(1)
