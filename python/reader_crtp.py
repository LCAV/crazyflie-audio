#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pdb
import logging
import time

import numpy as np

from cflib.utils.callbacks import Caller
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
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
    'yaw_rate': 'gyro.z', 
    'dx': 'motion.deltaX',
    'dy': 'motion.deltaY',
    'z': 'range.zrange'
}
LOGGING_PERIOD_MS = 10 # logging period in ms

BATTERY_PERIOD_MS = 1000 # logging period of battery in ms (set to 0 for none) 

FFTSIZE = 32
N_MICS = 4
CRTP_PAYLOAD = 29 # number of bytes per package

# audio signals data
N_FRAMES_AUDIO = FFTSIZE * N_MICS * 2  # *2 for complex numbers
AUDIO_DTYPE = np.float32
N_FRAMES_FBINS = FFTSIZE 
FBINS_DTYPE = np.uint16

# the timestamp is sent in the end of the fbins messages, as an uint32. 
N_BYTES_TIMESTAMP = 4
ALLOWED_DELTA_US = 1e6

class ArrayCRTP(object):
    def __init__(self, dtype, n_frames, name="array", extra_bytes=0):
        """
        :param n_bytes: the number of bytes to form one array, we will read 
                        CRTP packets until we reach this number of bytes.
        :param dtype: the type of data to be read (np.float32/np.uint16/etc.)
        :param name: name of this array (used for printing only)
        """
        self.name = name 
        self.n_frames = n_frames
        self.n_bytes = n_frames * np.dtype(dtype).itemsize + extra_bytes
        self.n_packets_full, self.n_bytes_last = divmod(self.n_bytes, CRTP_PAYLOAD)
        print(f"{name}: waiting for {self.n_bytes} bytes.")
        self.packet_counter = 0
        self.dtype = dtype
        self.packet_start_time = time.time()

        self.bytes_array = np.zeros(self.n_bytes, dtype=np.uint8)
        self.array = np.zeros(n_frames, dtype=dtype)

    def fill_array_from_crtp(self, packet, verbose=False):
        """
        :param packet: CRTP packet
        :param timestamp: current timestamp

        :returns: True if the array was filled, False if it is not full yet.
        """
        if verbose and (self.name == "fbins"):
            print(f"ReaderCRTP: filling fbins {self.packet_counter} (every second should be zero or one): {packet.datal}")
        elif verbose and (self.name == "audio"):
            print(f"ReaderCRTP: filling audio {self.packet_counter} (first 6 floats): {packet.datal[:6*4]}")

        # received all full packets, read remaining bytes
        if self.packet_counter == self.n_packets_full:
            self.bytes_array[
                self.packet_counter * CRTP_PAYLOAD:
                self.packet_counter * CRTP_PAYLOAD + self.n_bytes_last
            ] = packet.datal[:self.n_bytes_last] 

            self.array = np.frombuffer(self.bytes_array, dtype=self.dtype)

            # increase the counter to test for package loss
            self.packet_counter += 1
            return True
        elif self.packet_counter < self.n_packets_full:
            if (self.packet_counter == 0):
                self.packet_start_time = time.time()

            assert (self.packet_counter + 1)*CRTP_PAYLOAD < len(self.bytes_array), \
            f"{self.name}: index {self.packet_counter * CRTP_PAYLOAD} exceeds length {len(self.array)}"

            self.bytes_array[
                self.packet_counter * CRTP_PAYLOAD: 
                (self.packet_counter + 1) * CRTP_PAYLOAD
            ] = packet.datal

            self.packet_counter += 1
            return False
        else:
            print("unexpected packet")

    def reset_array(self):
        if (self.packet_counter != 0) and (self.packet_counter != self.n_packets_full + 1):
            print(f"{self.name}: packets loss, received only {self.packet_counter}/{self.n_packets_full+1}")
        self.packet_counter = 0


class ReaderCRTP(object):
    """
    ReaderCRTP recovers the data sent through the CRTP protocol and stores it. 
    There are different schemes for different data types:

    - audio: 
    The audio data (the FFT signals at N_FREQUENCY bins, recorded from N_MICS microphones, and the corresponding frequency bins) is sent in packets of CRTP_PAYLOAD bytes each.
    A new data frame starts when the start condition is met(channel==1) and we count the incoming packets to make sure there is no packet loss. The frequency data is sent after the audio data on channel 2.

    - motion: 
    We read the current motion estimate through the standard logging framework provided by the Crazyflie, and then publish the estimate as a Pose.

    - console: 
    We read whatever is published using DEBUG_PRINT in the firmware and print it out. 

    """

    # TODO(FD) potentially replace with constant
    BATTERY_OK = 3.83

    def __init__(self, crazyflie, verbose=False, log_motion=False, log_battery=True):

        self.receivedChar = Caller()
        self.frame_started = False
        self.cf = crazyflie
        self.verbose = verbose

        self.mc = MotionCommander(self.cf)

        if log_motion:
            lg_motion = LogConfig(name='Motion2D', period_in_ms=LOGGING_PERIOD_MS)
            for log_value in CHOSEN_LOGGERS.values():
                lg_motion.add_variable(log_value, 'float')
            self.cf.log.add_config(lg_motion)
            lg_motion.data_received_cb.add_callback(self.callback_logging)
            lg_motion.start()

        if log_battery:
            lg_battery = LogConfig(name='Battery', period_in_ms=BATTERY_PERIOD_MS)
            lg_battery.add_variable('pm.vbat', 'float')
            self.cf.log.add_config(lg_battery)
            lg_battery.data_received_cb.add_callback(self.callback_battery)
            lg_battery.start()

        self.cf.add_port_callback(CRTP_PORT_AUDIO, self.callback_audio)
        self.cf.add_port_callback(CRTPPort.CONSOLE, self.callback_console)

        # this data can be read and published by ROS nodes
        self.start_time = time.time()
        self.audio_dict = {
                'timestamp': None, 
                'audio_timestamp': None, 
                'signals_f_vect': None, 
                'fbins': None,
                'published': True
        }
        self.motion_dict = {
                'timestamp': None, 
                'data': None, 
                'published': True
        }

        self.audio_array = ArrayCRTP(AUDIO_DTYPE, N_FRAMES_AUDIO, "audio")
        self.fbins_array = ArrayCRTP(FBINS_DTYPE, N_FRAMES_FBINS, "fbins")
        self.audio_timestamp = 0

        self.battery = None

        # for debugging only
        self.update_rate = 0
        self.last_packet_time = 0

        # start sending audio data
        self.cf.param.set_value("audio.send_audio_enable", 1)
        if self.verbose:
            print("ReaderCRTP: set audio.send_audio_enable")

    def get_time_ms(self):
        return int((time.time() - self.start_time) * 1000)


    def callback_audio(self, packet):
        # We send the first package this channel to identify the start of new audio data.
        # Channel order:    1 | 0 0 ... 0 0 |  2 2 2
        #         audio start |    audio    |  fbins
        if packet.channel == 1:
            self.frame_started = True
            self.audio_array.reset_array()
            self.fbins_array.reset_array()

        if self.frame_started and packet.channel != 2: # channel is either 0 or 1: read data
            filled = self.audio_array.fill_array_from_crtp(packet, verbose=False)

        elif self.frame_started and packet.channel == 2: # channel is 2: read fbins
            filled = self.fbins_array.fill_array_from_crtp(packet, verbose=False)

            if filled: 
                # read the timestamp from the last packet
                timestamp_bytes = np.array(
                        packet.datal[self.fbins_array.n_bytes_last:
                        self.fbins_array.n_bytes_last + N_BYTES_TIMESTAMP], 
                        dtype=np.uint8)

                # frombuffer returns array of length 1
                new_audio_timestamp = int(np.frombuffer(timestamp_bytes, dtype=np.uint32)[0])

                # reject faulty packages
                if self.audio_timestamp and (new_audio_timestamp > self.audio_timestamp + ALLOWED_DELTA_US):
                    return 
                self.audio_timestamp = new_audio_timestamp

                self.audio_dict['published'] = False
                self.audio_dict['signals_f_vect'] = self.audio_array.array
                self.audio_dict['fbins'] = self.fbins_array.array
                self.audio_dict['audio_timestamp'] = self.audio_timestamp
                self.audio_dict['timestamp'] = self.get_time_ms()
                    
                if self.verbose:
                    packet_time = time.time() - self.audio_dict['timestamp']
                    print(f"ReaderCRTP callback: time for all packets: {packet_time}s, current timestamp: {new_audio_timestamp}, update rate: {self.update_rate:.2f}")
                    self.update_rate = 1/(time.time() - self.last_packet_time)
                    self.last_packet_time = time.time()

    def callback_console(self, packet):
        message = ''.join(chr(n) for n in packet.datal)
        print(message, end='')

    def callback_logging(self, timestamp, data, logconf):
        self.motion_dict['timestamp'] = self.get_time_ms()
        self.motion_dict['published'] = False
        self.motion_dict['data'] = {
            key: data[val] for key, val in CHOSEN_LOGGERS.items()
        }

    def callback_battery(self, timestamp, data, logconf):
        self.battery = data['pm.vbat']
        if self.verbose:
            print('ReaderCRTP battery callback:', self.battery)

    def battery_ok(self):
        if self.battery is None:
            return True
        elif self.battery <= self.BATTERY_OK: # corresponds to 20 %
            print(f"Warning: battery only at {self.battery}, not executing command")
            return False
        return True

    def send_thrust_command(self, value, motor='all'):
        if (value > 0) and not self.battery_ok():
            return False
        if motor == 'all':
            [self.cf.param.set_value(f"motorPowerSet.m{i}", value) for i in range(1, 5)]
        else:
            self.cf.param.set_value(f"motorPowerSet.{motor}", value)
        if value > 0:
            self.cf.param.set_value("motorPowerSet.enable", 1)
        return True

    def send_hover_command(self, height):
        if not self.battery_ok():
            return False
        self.mc.take_off(height)
        time.sleep(1)
        return True

    def send_turn_command(self, angle_deg):
        if angle_deg > 0:
            self.mc.turn_left(angle_deg)
        else:
            self.mc.turn_right(-angle_deg)
        return True
        # do not need this because  it is part of turn_*
        # time.sleep(1)

    def send_land_command(self, velocity=0):
        if velocity > 0:
            print('Warning: using default velocity')

        self.mc.land()
        time.sleep(1)
        self.mc.stop()
        return True

    def send_buzzer_effect(self, effect):
        self.cf.param.set_value("sound.effect", effect)

    def send_buzzer_freq(self, freq):
        self.cf.param.set_value("sound.freq", freq)


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

        reader_crtp = ReaderCRTP(cf, verbose=verbose)

        try:
            while True:
                time.sleep(1)
        except:
            print("ReaderCRTP: unset audio.send_audio_enable")
            cf.param.set_value("audio.send_audio_enable", 0)
            time.sleep(3)
