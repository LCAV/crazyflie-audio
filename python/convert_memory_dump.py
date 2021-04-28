import numpy as np
import matplotlib.pyplot as plt

datadir = "recordings_14_7_20/external_source_and_propellers/20deg/"

fs = 32000
dt = 1 / fs
left_2 = np.fromfile(datadir + "left_2", np.int16)
np.save(datadir + "front_right", left_2)

left_3 = np.fromfile(datadir + "left_3", np.int16)
np.save(datadir + "back_right", left_3)

right_2 = np.fromfile(datadir + "right_2", np.int16)
np.save(datadir + "front_left", right_2)

right_3 = np.fromfile(datadir + "right_3", np.int16)
np.save(datadir + "back_left", right_3)

# t = np.arange(0,right_3.size/fs,dt)
# plt.plot(t, left_2)
# plt.plot(t,right_2)
# plt.plot(t,left_3)
# plt.plot(t,right_3)
# plt.show()
