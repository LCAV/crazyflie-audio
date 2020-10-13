import logging
import time
import sounddevice as sd
import numpy as np

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

record_dir = "insert_path_here"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Sound card selection
# sd.default.device = 'ASIO4ALL v2'
# sd.default.device = 'Yamaha Steinberg USB ASIO'
sd.default.device = 'M Audio M-Track Eight ASIO'

fs = 48000
sd.default.samplerate = fs
sd.default.channels = 4

id = 'radio://0/69/2M'


def create_white_noise(fs,duration):
    wn = np.random.rand(fs * duration)
    return wn

def create_sinus(fs,f,duration):
    dt = 1/fs
    t = np.arange(0, duration-dt, dt)
    sin = np.sin(2 * np.pi * f * t)
    return sin

def set_thrust(cf,thrust):
    thrust_str = '%d' % thrust
    cf.param.set_value('motorPowerSet.m4', thrust_str)
    cf.param.set_value('motorPowerSet.m1', thrust_str)
    cf.param.set_value('motorPowerSet.m2', thrust_str)
    cf.param.set_value('motorPowerSet.m3', thrust_str)
    cf.param.set_value('motorPowerSet.enable', '1')

if __name__ == '__main__':
    duration = 10 #sec
    #signal = create_white_noise(fs, duration)
    signal = create_sinus(fs, 200, duration)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        set_thrust(cf, 43000)
        time.sleep(2)

        # recording = sd.rec(duration * fs, blocking=True) #uncomment if no sound should be played
        recording = sd.playrec(signal, blocking=True)

        cf.param.set_value('motorPowerSet.enable', '0')
        time.sleep(1)

        np.save(record_dir, recording)
