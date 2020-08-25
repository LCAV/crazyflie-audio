#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import time

import numpy as np

from cflib.utils.callbacks import Caller
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
import cflib.crtp

# TODO(FD) for now, below only works with modified cflib.
# from cflib.crtp.crtpstack import CRTPPort
# CRTP_PORT_AUDIO = CRTPPort.AUDIO

CRTP_PORT_AUDIO = 0x09

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"



# TODO(FD): figure out if below changes when the Crazyflie actually flies.
# Tests have shown that when the flowdeck is attached to the Crazyflie and 
# it is moved around (without flying), then 
# yaw:
# - stabilizer.yaw, controller.yaw and stateEstimate.yaw are all the same. 
#   They are in degrees and clipped to -180, 180.
# - mag.x sometimes gives values similar to above 3, 
#   but sometimes it is constantly zero (should it only be used outside? 
# - gyro.z gives the raw yaw rate (or acceleration?).
# dx/dy: 
# - motion.deltaX and motion.deltaY are in milimeters can be very noisy, especially when 
#   the ground is not textured. 
# - stabilizer.vx and stabilizer.vy are in m/s (not 100% sure) and more stable 
#  but of course would need to be integrated for a position estimate.
# z:
# - range.zrange gives the raw data in milimeters (uint16), which is quite accurate.
# - stateEstimateZ.z in milimeters (uint16), is more smooth but also overshoots a little bit, and even 
#   goes negative sometimes which is impossible. 
# - stateEstimate.z in meters (float), from barometer. Surprisingly accurate. 
CHOSEN_LOGGERS = {
    'yaw': 'stabilizer.yaw', 
    'dx': 'motion.deltaX',
    'dy': 'motion.deltaY',
    'z': 'range.zrange'
}


FFTSIZE = 32
N_MICS = 4
CRTP_PAYLOAD = 29 # number of bytes per package
FLOAT_PRECISION = 4 # number of bytes for float32
N_FLOATS = FFTSIZE * N_MICS * 2  # *2 for complex numbers
N_BYTES = N_FLOATS * FLOAT_PRECISION
N_FULL_PACKETS, N_BYTES_LAST_PACKET = divmod(N_BYTES, CRTP_PAYLOAD)

def set_thrust(cf,thrust):
    thrust_str = f'{thrust}'
    cf.param.set_value('motorPowerSet.m4', thrust_str)
    cf.param.set_value('motorPowerSet.m1', thrust_str)
    cf.param.set_value('motorPowerSet.m2', thrust_str)
    cf.param.set_value('motorPowerSet.m3', thrust_str)
    cf.param.set_value('motorPowerSet.enable', '1')

class ReaderCRTP(object):
    """
    ReaderCRTP recovers the data sent through the CRTP protocol and publishes them. 
    There are different schemes for different data types:

    - audio_data: 
    The audio data (the FFT signals at N_FREQUENCY bins, recorded from N_MICS microphones) is sent in packets of CRTP_PAYLOAD bytes each.
    The new data frame starts when the start condition is met(channel==1) and we count the incoming packets to make sure there is no packet loss.

    - motion_data: 
    We read the current motion estimate through the standard logging framework provided by the Crazyflie, and then publish the estimate as a Pose.
    """
    def __init__(self, crazyflie, verbose=False):
        self.array = np.zeros(N_BYTES, dtype=np.uint8)

        self.receivedChar = Caller()
        self.start = False
        self.index = 0
        self.start_time = 0
        self.cf = crazyflie
        self.verbose = verbose

        lg_motion = LogConfig(name='Motion2D', period_in_ms=300)
        for log_value in CHOSEN_LOGGERS.values():
            lg_motion.add_variable(log_value, 'float')
        self.cf.log.add_config(lg_motion)
        lg_motion.data_received_cb.add_callback(self.callback_logging)
        lg_motion.start()

        self.cf.add_port_callback(CRTP_PORT_AUDIO, self.callback_audio)

        # this data can be read and published by ROS nodes
        self.audio_data = {'timestamp': None, 'data': None, 'published': True}
        self.motion_data = {'timestamp': None, 'data': None, 'published': True}

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
                
                self.audio_data['data'] = np.frombuffer(self.array, dtype=np.float32)
                self.audio_data['timestamp'] = time.time()
                self.audio_data['published'] = False

                if self.verbose:
                    packet_time = self.audio_data['timestamp'] - self.start_time
                    print(f"callback_audio: time for all packets: {packet_time}s")
            else:
                self.array[
                    self.index * CRTP_PAYLOAD : (self.index + 1) * CRTP_PAYLOAD
                ] = packet.datal # packet in list format
            self.index += 1

    def callback_logging(self, timestamp, data, logconf):
        if self.verbose:
            print('callback', timestamp, data, logconf.name)

        # TODO(FD): figure out if this timestamp is correct.
        self.motion_data['timestamp'] = timestamp
        self.motion_data['published'] = False
        self.motion_data['data'] = {
            key: data[val] for key, val in CHOSEN_LOGGERS.items()
        }

if __name__ == "__main__":
    verbose = True
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        #set_thrust(cf, 43000)
        reader_crtp = ReaderCRTP(cf, verbose=verbose)

        while True:
            time.sleep(1)
