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
LOGGING_PERIOD_MS = 10 # logging period in ms

FFTSIZE = 32
N_MICS = 4
CRTP_PAYLOAD = 29 # number of bytes per package
FLOAT_PRECISION = 4 # number of bytes for float32
INT_PRECISION = 2 # number of bytes per int16

# precalculations for audio signals 
N_FLOATS = FFTSIZE * N_MICS * 2  # *2 for complex numbers
N_BYTES_AUDIO = N_FLOATS * FLOAT_PRECISION
N_FULL_AUDIO, N_BYTES_LAST_AUDIO = divmod(N_BYTES_AUDIO, CRTP_PAYLOAD)

# precalculations for frequency bins
N_INTS = FFTSIZE 
N_BYTES_FBINS = N_INTS * INT_PRECISION
N_FULL_FBINS, N_BYTES_LAST_FBINS = divmod(N_BYTES_FBINS, CRTP_PAYLOAD)

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
        self.audio_array = np.zeros(N_BYTES_AUDIO, dtype=np.uint8)
        self.fbins_array = np.zeros(N_BYTES_FBINS, dtype=np.uint8)

        self.receivedChar = Caller()
        self.start_audio = False
        self.start_fbins = False
        self.index_audio = 0
        self.index_fbins = 0
        self.packet_start_time_audio = 0
        self.packet_start_time_fbins = 0
        self.cf = crazyflie
        self.verbose = verbose

        lg_motion = LogConfig(name='Motion2D', period_in_ms=LOGGING_PERIOD_MS)
        for log_value in CHOSEN_LOGGERS.values():
            lg_motion.add_variable(log_value, 'float')
        self.cf.log.add_config(lg_motion)
        lg_motion.data_received_cb.add_callback(self.callback_logging)
        lg_motion.start()

        self.cf.add_port_callback(CRTP_PORT_AUDIO, self.callback_crtp)

        # this data can be read and published by ROS nodes
        self.start_time = time.time()
        self.audio_data = {'timestamp': None, 'data': None, 'published': True}
        self.fbins_data = {'timestamp': None, 'data': None, 'published': True}
        self.motion_data = {'timestamp': None, 'data': None, 'published': True}

        # start sending audio data
        self.cf.param.set_value("audio.send_audio_enable", 1)
        print("set audio")

    def get_time_ms(self):
        return int((time.time()-self.start_time)*1000)

    def callback_crtp(self, packet):
        # We send the first package this channel to identify the start of new audio data.
        if packet.channel == 1:
            print('callback audio')
            if (self.index_audio != 0) and (self.index_audio != N_FULL_FBINS + 1):
                print(f"audio packets loss: received only {self.index}/{N_FULL_FBINS+1}")
            self.index_audio = 0  # reset index
            self.start_audio = True
            self.packet_start_time_audio = time.time()

        # TODO: there is a lot of copy-pasting going on here, could refactor this 
        # at some point.

        if (packet.channel == 2) and self.start_audio:
            print('callback fbins')
            #if (self.index_fbins != 0) and (self.index_fbins != N_FULL_FBINS + 1):
            #    print(f"fbins packets loss: received only {self.index_fbins}/{N_FULL_FBINS+1}")
            # TODO(FD): simplify this
            if not self.start_fbins:
                self.index_fbins = 0  # reset index
                self.start_fbins = True
                self.packet_start_time_fbins = time.time()

        elif (packet.channel == 2):
            print('callback fbins without start_audio')


        if self.start_audio:
            # received all full packets, read remaining bytes
            if self.index_audio == N_FULL_FBINS:
                self.audio_array[
                    self.index_audio * CRTP_PAYLOAD : self.index_audio * CRTP_PAYLOAD
                    + N_BYTES_LAST_FBINS
                ] = packet.datal[
                    0:N_BYTES_LAST_FBINS
                ]  # last bytes
                
                self.audio_data['data'] = np.frombuffer(self.audio_array, dtype=np.float32)
                self.audio_data['timestamp'] = self.get_time_ms()
                self.audio_data['published'] = False

                # TODO(FD): do we need this? It was not here before. 
                self.start_audio = False

                if self.verbose:
                    packet_time = time.time() - self.packet_start_time_audio
                    print(f"callback_crtp: time for all audio packets: {packet_time}s")

            else:
                self.audio_array[
                    self.index_audio * CRTP_PAYLOAD : (self.index_audio + 1) * CRTP_PAYLOAD
                ] = packet.datal # packet in list format
            self.index_audio += 1

        if self.start_fbins:
            # received all full packets, read remaining bytes
            if self.index_fbins == N_FULL_FBINS:
                self.fbins_array[
                    self.index_fbins * CRTP_PAYLOAD : self.index_fbins * CRTP_PAYLOAD
                    + N_BYTES_LAST_FBINS
                ] = packet.datal[
                    0:N_BYTES_LAST_FBINS
                ]  # last bytes
                
                self.fbins_data['data'] = np.frombuffer(self.fbins_array, dtype=np.uint16)
                self.fbins_data['timestamp'] = self.get_time_ms()
                self.fbins_data['published'] = False

                # TODO(FD): do we need this? It was not here before. 
                self.start_fbins = False

                if self.verbose:
                    packet_time = time.time() - self.packet_start_time_fbins
                    print(f"callback_crtp: time for all fbins packets: {packet_time}s")

            else:
                self.fbins_array[
                    self.index_fbins * CRTP_PAYLOAD : (self.index_fbins + 1) * CRTP_PAYLOAD
                ] = packet.datal # packet in list format
            self.index_fbins += 1

    def callback_logging(self, timestamp, data, logconf):
        self.motion_data['timestamp'] = self.get_time_ms()
        self.motion_data['published'] = False
        self.motion_data['data'] = {
            key: data[val] for key, val in CHOSEN_LOGGERS.items()
        }
        #if self.verbose:
        #    print('callback_logging:', logconf.name, self.motion_data['data'])

if __name__ == "__main__":
    import argparse
    verbose = True
    cflib.crtp.init_drivers(enable_debug_driver=False)

    parser = argparse.ArgumentParser(description='Read CRTP data from Crazyflie.')
    parser.add_argument('id', metavar='ID', type=int, help='number of Crazyflie ("radio://0/ID/2M")',
                        default=69)
    args = parser.parse_args()
    id = f"radio://0/{args.id}/2M"


    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        #set_thrust(cf, 43000)
        reader_crtp = ReaderCRTP(cf, verbose=verbose)

        try:
            while True:
                time.sleep(1)
        except:
            print("unset audio.send_audio_enable")
            cf.param.set_value("audio.send_audio_enable", 0)
