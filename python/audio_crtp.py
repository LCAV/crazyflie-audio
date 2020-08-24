#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import time

import numpy as np

from cflib.utils.callbacks import Caller
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp

# TODO(FD) for now, below only works with modified cflib.
# from cflib.crtp.crtpstack import CRTPPort
# CRTP_PORT_AUDIO = CRTPPort.AUDIO

CRTP_PORT_AUDIO = 0x09

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"

FFTSIZE = 32
N_MICS = 4
CRTP_PAYLOAD = 29 # number of bytes per package
FLOAT_PRECISION = 4 # number of bytes for float32
N_FLOATS = FFTSIZE * N_MICS * 2  # *2 for complex numbers
N_BYTES = N_FLOATS * FLOAT_PRECISION
N_FULL_PACKETS, N_BYTES_LAST_PACKET = divmod(N_BYTES, CRTP_PAYLOAD)

def set_thrust(cf,thrust):
    thrust_str = '%d' % thrust
    cf.param.set_value('motorPowerSet.m4', thrust_str)
    cf.param.set_value('motorPowerSet.m1', thrust_str)
    cf.param.set_value('motorPowerSet.m2', thrust_str)
    cf.param.set_value('motorPowerSet.m3', thrust_str)
    cf.param.set_value('motorPowerSet.enable', '1')

class AudioCRTP(object):
    """
    Audio CRTP recovers the correlation matrix through the CRTP protocol. 
    The matrix is sent in packets of CRTP_PAYLOAD bytes each.
    The protocol has a start condition (channel==1) and then counts the incoming packets.
    """

    def __init__(self, crazyflie):
        self.array = np.zeros(N_BYTES, dtype=np.uint8)

        self.receivedChar = Caller()
        self.start = False
        self.index = 0
        self.start_time = 0
        self.cf = crazyflie
        self.cf.add_port_callback(CRTPPort.AUDIO, self.callback_audio)

        # this data can be read and published by ROS nodes
        self.audio_data = {'time': None, 'data': None, 'published': True}
        self.motion_data = {'time': None, 'data': None, 'published': True}

    def callback_audio(self, packet):
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
                
                self.audio_data['data'] = = np.frombuffer(self.array, dtype=np.float32)
                self.audio_data['time'] = time.time()
                self.audio_data['published'] = False

                print(
                    f"Elapsed time for receiving audio data = {self.audio_data['time'] - self.start_time}s"
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
        set_thrust(cf, 43000)
        audio_CRTP = AudioCRTP(cf)
        while True:
            time.sleep(1)
