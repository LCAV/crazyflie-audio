#!/usr/bin/env python3
# coding: utf-8

import os

import matplotlib.pylab as plt
import numpy as np
import sys


import os
from file_parser import parameters, read_recordings
import pandas as pd


## we have only 0deg for low and normal
dir_names = [
    "recordings_9_7_20",  # audio shield measurements with buffer length 256, only 200Hz source
    "recordings_14_7_20",  # audio shield measurements with buffer length 8060, only 200Hz source
    "recordings_16_7_20",  # recordings with measurement mics, 200Hz and white_noise source
]
sources = [
    "200Hz",
    "white_noise",
]  # source types (single frequency at 200Hz or white noise)

loudnesses = ["high", "normal", "low"]  # loudness of source

gt_degrees_list = [0, 20, 40]  # rotation of drone with respect to source.

df = pd.DataFrame(
    columns=[
        "loudness",
        "gt_degrees",
        "dir_name",
        "source",
        "signals_props",
        "signals_source",
        "signals_all",
    ]
)



signals_props, signals_source, signals_all = read_recordings( dir_names[1], loudness=loudnesses[1], gt_degrees=gt_degrees_list[1], source=sources[1], )

#signals_props = np.transpose(signals_props)

f = 32000

thrust = 43000

om_0 = 3.27258551 * np.sqrt(thrust) - 26.41814899

om_0s = om_0 * np.append([0.5, 1, 1.5], np.arange(2, 10, 1))

NSamples = 1024

chosen_bin = np.arange(1, 32, 1)

for signal in signals_props:
    freq = np.fft.rfftfreq(len(signal[0:NSamples]), 1 / f)
    Y = np.fft.rfft(signal[0:NSamples]) / len(signal[0:NSamples])
    plt.plot(freq, np.abs(Y) , label = f"{dir_names[1]} {loudnesses[1]} {gt_degrees_list[1]} {sources[1]}")

#print(len(freq))

for line in om_0s:
    plt.axvline(line, c = 'r', ls = '--')



bins_basic_candidates = np.ones(np.size(freq))

DELTA_F_PROP = 100
NUMBER_FINAL_BINS = 32

hi_pass_f = 200
lo_pass_f = 10000

# Remove propeler sound +- delta f
for i in range(np.size(freq)):
    in_prop_range = 0
    for om_0_i in om_0s:
        if( np.abs((freq[i] - om_0_i )) < DELTA_F_PROP):
            in_prop_range = 1
            break
    if in_prop_range:
        bins_basic_candidates[i] = 0

# Count candidate that are still available
sum_candidate = 0
for i in range(np.size(bins_basic_candidates)):
    if(bins_basic_candidates[i] == 1):
        sum_candidate += 1

# Remove Hi-pass and low-pass candidates
for i in range(np.size(freq)):
    if((freq[i] < hi_pass_f) | (freq[i] > lo_pass_f )):
        bins_basic_candidates[i] = 0


# Backup 
bins_snr_avoid_prop = np.copy(bins_basic_candidates)
bins_uniforme_avoid_prop = np.copy(bins_basic_candidates)

## samples uniformly among remaining candidates
decimation = int(np.floor(sum_candidate / NUMBER_FINAL_BINS))
print(f'decimation =  {decimation}')

for i in range(np.size(freq) - decimation):
    print(f'i = {i}')

    # first samples are zeroed anyway
    if(i >= decimation):
        print(f'\ti - decimation =  {i - decimation}')
        
        sum = 0
        # check if any of the previous decimation bits where 1
        for j in range(decimation - 1):
            sum += bins_uniforme_avoid_prop[i - j] 

        # if previous samples where 0, then ok to place a one
        print(f'\tsum =  {sum}')
        if(sum == 0):
            bins_uniforme_avoid_prop[i] = 1
            print('\tAdded 1 to buffer')
        else:
            bins_uniforme_avoid_prop[i] = 0
    else:
        bins_uniforme_avoid_prop[i] = 0

    # Debug print buffer while constructing it
    print(bins_uniforme_avoid_prop[max(0, i - decimation) : i])


plt.scatter(freq, + 1000 * bins_basic_candidates, label = 'bins_basic_candidates')
plt.scatter(freq, + 600 * bins_uniforme_avoid_prop, label = 'bins_uniforme_avoid_prop')
plt.legend()

plt.autoscale(enable = True, axis = 'x', tight = True)

plt.show()
