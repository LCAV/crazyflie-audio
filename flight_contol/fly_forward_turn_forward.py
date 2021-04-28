import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


URI = "radio://0/80/2M/E7E7E7E7E7"
DEFAULT_HEIGHT = 0.5

is_deck_attached = False

logging.basicConfig(level=logging.ERROR)


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Take off")
        time.sleep(1)

        print("Forward")
        mc.forward(0.25)
        time.sleep(1)

        print("Turn left")
        mc.turn_left(180)
        time.sleep(1)

        print("Forward")
        mc.forward(0.25)
        time.sleep(1)

        print("Land")


def param_deck_flow(name, value):
    global is_deck_attached

    print(f"deck attached {value}")

    print(type(value))

    if value == "1":
        is_deck_attached = True
        print("Deck is attached!")
    else:
        is_deck_attached = False
        print("Deck is NOT attached!")


if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:

        scf.cf.param.add_update_callback(
            group="deck", name="bcFlow2", cb=param_deck_flow
        )
        time.sleep(1)

        if is_deck_attached:
            move_linear_simple(scf)

        print("Landed")
