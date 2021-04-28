import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = "radio://0/80/2M/E7E7E7E7E7"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def param_update_callback(name, value):
    print("The crazyflie has parameter " + name + " set at number: " + value)


def simple_param_async(scf, groupstr, namestr, value):
    cf = scf.cf
    full_name = groupstr + "." + namestr

    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_update_callback)
    # time.sleep(1)
    cf.param.set_value(full_name, value)


def log_stab_callback(timestamp, data, logconf):
    print("[%d][%s]: %s" % (timestamp, logconf.name, data))


def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()


def simple_log(scf, logconf):

    with SyncLogger(scf, lg_stab) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print("[%d][%s]: %s" % (timestamp, logconf_name, data))

            break


def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")


if __name__ == "__main__":
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
    lg_stab.add_variable("stabilizer.roll", "float")
    lg_stab.add_variable("stabilizer.pitch", "float")
    lg_stab.add_variable("stabilizer.yaw", "float")

    group = "sound"
    name = "effect"

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        simple_param_async(scf, "sound", "effect", 0)
        time.sleep(2)

        simple_param_async(scf, "sound", "effect", 12)
        simple_param_async(scf, "sound", "freq", 2000)
        simple_param_async(scf, "sound", "ratio", 1)
        time.sleep(5)

        simple_param_async(scf, "sound", "effect", 12)
        simple_param_async(scf, "sound", "freq", 600)
        simple_param_async(scf, "sound", "ratio", 0)
        time.sleep(5)
        simple_param_async(scf, "sound", "effect", 0)
        time.sleep(2)

        simple_param_async(scf, "sound", "effect", 12)
        simple_param_async(scf, "sound", "freq", 600)
        simple_param_async(scf, "sound", "ratio", 10)
        time.sleep(5)

        simple_param_async(scf, "sound", "effect", 12)
        simple_param_async(scf, "sound", "freq", 600)
        simple_param_async(scf, "sound", "ratio", 0)
        time.sleep(5)
        simple_param_async(scf, "sound", "effect", 0)
        time.sleep(2)
