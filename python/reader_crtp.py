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

from cflib.crtp.crtpstack import CRTPPort
# TODO(FD) for now, below only works with modified cflib.
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

# audio signals data
N_FLOATS = FFTSIZE * N_MICS * 2  # *2 for complex numbers
N_BYTES_AUDIO = N_FLOATS * 4 # 4 is number of bytes per float32

# frequency bins data
N_INTS = FFTSIZE 
N_BYTES_FBINS = N_INTS * 2 # 2 number of bytes for int16

def set_thrust(cf,thrust):
    thrust_str = f'{thrust}'
    cf.param.set_value('motorPowerSet.m4', thrust_str)
    cf.param.set_value('motorPowerSet.m1', thrust_str)
    cf.param.set_value('motorPowerSet.m2', thrust_str)
    cf.param.set_value('motorPowerSet.m3', thrust_str)
    cf.param.set_value('motorPowerSet.enable', '1')


class ArrayCRTP(object):
    def __init__(self, data_dict, dtype, n_bytes, name="array"):
        """
        :param data_dict: data dictionary containing snapshot of latest data 
        :param dtype: the type of data to be read (np.float32/np.uint16/etc.)
        :param n_bytes: the number of bytes to form one array, we will read 
                        CRTP packets until we reach this number of bytes.
        :param name: name of this array (used for printing only)
        """
        self.name = name 
        self.n_bytes = n_bytes
        self.n_packets_full, self.n_bytes_last = divmod(n_bytes, CRTP_PAYLOAD)
        self.array = np.zeros(n_bytes, dtype=np.uint8)
        self.index = 0
        self.dtype = dtype
        self.packet_start_time = time.time()
        self.data_dict = data_dict

    def fill_array_from_crtp(self, packet, timestamp=0, verbose=False):
        """
        :param packet: CRTP packet
        :param timestamp: current timestamp

        :returns: True if the array was filled, False if it is not full yet.
        """
        if verbose and (self.name == "fbins"):
            print(f"ReaderCRTP: filling fbins {self.index} (every second should be zero or one): {packet.datal}")
        elif verbose and (self.name == "audio"):
            print(f"ReaderCRTP: filling audio {self.index} (first 6 floats): {packet.datal[:6*4]}")


        # received all full packets, read remaining bytes
        if self.index == self.n_packets_full:
            self.array[
                self.index * CRTP_PAYLOAD:
                self.index * CRTP_PAYLOAD + self.n_bytes_last
            ] = packet.datal[:self.n_bytes_last] 
            
            self.data_dict['data'] = np.frombuffer(self.array, dtype=self.dtype)
            self.data_dict['timestamp'] = timestamp 
            self.data_dict['published'] = False

            self.index += 1
            return True
        else:
            if(self.index == 0):
                self.packet_start_time = time.time()

            assert (self.index + 1)*CRTP_PAYLOAD < len(self.array), \
            f"{self.name}: index {self.index * CRTP_PAYLOAD} exceeds length {len(self.array)}"

            self.array[
                self.index * CRTP_PAYLOAD: 
                (self.index + 1) * CRTP_PAYLOAD
            ] = packet.datal

            self.index += 1
            return False

    def reset_array(self):
        if (self.index != 0) and (self.index != self.n_packets_full + 1):
            print(f"{self.name}: packets loss, received only {self.index}/{self.n_packets_full+1}")
        self.index = 0


class ReaderCRTP(object):
    """
    ReaderCRTP recovers the data sent through the CRTP protocol and stores it. 
    There are different schemes for different data types:

    - audio: 
    The audio data (the FFT signals at N_FREQUENCY bins, recorded from N_MICS microphones) is sent in packets of CRTP_PAYLOAD bytes each.
    The new data frame starts when the start condition is met(channel==1) and we count the incoming packets to make sure there is no packet loss.

    - motion: 
    We read the current motion estimate through the standard logging framework provided by the Crazyflie, and then publish the estimate as a Pose.

    - console: 
    We read whatever is published using DEBUG_PRINT in the firmware and print it out. 

    """
    def __init__(self, crazyflie, verbose=False):

        self.receivedChar = Caller()
        self.start_audio = False
        self.cf = crazyflie
        self.verbose = verbose

        lg_motion = LogConfig(name='Motion2D', period_in_ms=LOGGING_PERIOD_MS)
        for log_value in CHOSEN_LOGGERS.values():
            lg_motion.add_variable(log_value, 'float')
        self.cf.log.add_config(lg_motion)
        lg_motion.data_received_cb.add_callback(self.callback_logging)
        # lg_motion.start()

        self.cf.add_port_callback(CRTP_PORT_AUDIO, self.callback_audio)
        self.cf.add_port_callback(CRTPPort.CONSOLE, self.callback_console)

        # this data can be read and published by ROS nodes
        self.start_time = time.time()
        self.audio_dict = {'timestamp': None, 'data': None, 'published': True}
        self.fbins_dict = {'timestamp': None, 'data': None, 'published': True}
        self.motion_dict = {'timestamp': None, 'data': None, 'published': True}

        self.audio_array = ArrayCRTP(self.audio_dict, np.float32, N_BYTES_AUDIO, "audio")
        self.fbins_array = ArrayCRTP(self.fbins_dict, np.uint16, N_BYTES_FBINS, "fbins")

        # start sending audio data
        self.cf.param.set_value("audio.send_audio_enable", 1)
        if self.verbose:
            print("ReaderCRTP: set audio.send_audio_enable")

    def get_time_ms(self):
        return int((time.time() - self.start_time) * 1000)


    def callback_audio(self, packet):
        # We send the first package this channel to identify the start of new audio data.
        if packet.channel == 1:
            self.start_audio = True
            self.audio_array.reset_array()
            self.fbins_array.reset_array()

        if self.start_audio and packet.channel != 2: # channel is either 0 or 1: read data
            filled = self.audio_array.fill_array_from_crtp(packet, self.get_time_ms(), verbose=False)

            if self.verbose and filled:
                packet_time = time.time() - self.audio_array.packet_start_time
                print(f"ReaderCRTP audio callback: time for all packets: {packet_time}s")

        elif self.start_audio and packet.channel == 2: # channel is 2: read fbins
            filled = self.fbins_array.fill_array_from_crtp(packet, self.get_time_ms(), verbose=False)

            if self.verbose and filled:
                packet_time = time.time() - self.fbins_array.packet_start_time
                print(f"ReaderCRTP fbins callback: time for all packets: {packet_time}s")

    def callback_console(self, packet):
        message = ''.join(chr(n) for n in packet.datal)
        print(message, end='')

    def callback_logging(self, timestamp, data, logconf):
        self.motion_dict['timestamp'] = self.get_time_ms()
        self.motion_dict['published'] = False
        self.motion_dict['data'] = {
            key: data[val] for key, val in CHOSEN_LOGGERS.items()
        }
        if self.verbose:
            print('ReaderCRTP logging callback:', logconf.name)


if __name__ == "__main__":
    import argparse
    verbose = False
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
            print("ReaderCRTP: unset audio.send_audio_enable")
            cf.param.set_value("audio.send_audio_enable", 0)
