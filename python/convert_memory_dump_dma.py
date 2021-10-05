import numpy as np
import matplotlib.pyplot as plt

datadir = "recordings_14_7_20/test_2/"

fs = 32000
dt = 1 / fs
dma_2 = np.fromfile(datadir + "dma_2", np.int16)
left2 = dma_2[0:-1:2]
right2 = dma_2[1:-2:2]

np.save(datadir + "front_right", left2)
np.save(datadir + "front_left", right2)

dma_3 = np.fromfile(datadir + "dma_3", np.float16)
left3 = dma_3[0:-1:2]
right3 = dma_3[1:-1:2]

np.save(datadir + "back_right", left3)
np.save(datadir + "back_left", right2)

# t = np.arange(0,left2.size/fs,dt)
# plt.plot(t, left2/65535,'.-')
# plt.show()
