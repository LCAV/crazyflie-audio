"""
dummy_flight.py: Make drone hover at fixed height, and land on Keyboardinterrupt.

Requires the Optical Flow deck for zranger data. 
"""

import logging
import time
from threading import Thread
import matplotlib.pyplot as plt
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
PLOTTING = True

z_ranger_logs = np.empty(1)
z_filtered_logs = np.empty(1)
vz_logs = np.empty(1)

id = "radio://0/80/2M"
is_deck_attached = False

def initialization(scf):
    cf = scf.cf
    activate_high_level_commander(cf)
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2)
    return cf.high_level_commander

def param_deck_flow(name, value):
    global is_deck_attached
    print(value)
    if value:
        is_deck_attached = True
        print('Deck is attached!')
    else:
        is_deck_attached = False
        print('Deck is NOT attached!')

def low_pass(z, z_old, dt, fc):
    tau = 1 / (2 * np.pi * fc)
    alpha = dt / tau
    return alpha * z + (1 - alpha) * z_old


def activate_high_level_commander(cf):
    cf.param.set_value("commander.enHighLevel", "1")


def log_func(scf):
    dt = 10  # ms
    fc = 1  # Hz

    lg_stab = LogConfig(name="Stabilizer", period_in_ms=dt)
    lg_stab.add_variable("range.zrange", "float")  # Z ranger measurement
    lg_stab.add_variable("stateEstimate.x", "float")  # estimated X coordinate
    lg_stab.add_variable("stateEstimate.y", "float")  # estimated Y coordinate
    lg_stab.add_variable("stabilizer.thrust", "float")  # estimated Y coordinate

    global z_ranger_logs
    global z_filtered_logs
    global vz_logs

    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            data = log_entry[1]

            z_ranger_logs = np.append(z_ranger_logs, data["range.zrange"])
            z = low_pass(data["range.zrange"], z_filtered_logs[-1], dt * 1e-3, fc)
            vz = (z - z_filtered_logs[-1]) / dt
            vz_logs = np.append(vz_logs, vz)
            z_filtered_logs = np.append(z_filtered_logs, z)
            print(data["stabilizer.thrust"])


def take_off(commander):
    height = 0.6
    print("trying to take off to", height)
    commander.takeoff(height, 0.5)
    time.sleep(1.0)

def param_update_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)


def simple_param_async(scf, groupstr, namestr, value):
    cf = scf.cf
    full_name = groupstr + '.' + namestr

    cf.param.add_update_callback(group=groupstr, name=namestr,
                                 cb=param_update_callback)
    #time.sleep(1)
    cf.param.set_value(full_name, value)


if __name__ == "__main__":
    id = "radio://0/80/2M"
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(id, cf=Crazyflie(rw_cache="./cache")) as scf:
        commander = initialization(scf)
        time_start = time.process_time()

        take_off(commander)
        
        try:
            Thread(target=log_func, args=[scf]).start()

            while (time.process_time() - time_start) < 3:
                time.sleep(1)
        except KeyboardInterrupt:
            commander.land(0.0, 1.0)
            time.sleep(2)
            commander.stop()
        

        simple_param_async(scf, 'sound', 'effect', 12)
        simple_param_async(scf, 'sound', 'freq', 600)
        simple_param_async(scf, 'sound', 'ratio', 0)
        
        print("Flying")
        time.sleep(2)
        
        print("Landing")
        commander.land(0.0, 1.0)
        commander.stop()

        simple_param_async(scf, 'sound', 'effect', 0)
        time.sleep(2)


    if PLOTTING:

        fig, axs = plt.subplots(2)
        axs[0].plot(z_ranger_logs)
        axs[0].plot(z_filtered_logs)
        axs[1].plot(vz_logs[10:-1])
        axs[1].set_xlabel("time index")
        axs[1].set_ylabel("z velocity, filtered")
        axs[0].set_xlabel("time index")
        axs[0].set_ylabel("z range, filtered")
        plt.show()
        print(z_ranger_logs)

        print(z_filtered_logs)
