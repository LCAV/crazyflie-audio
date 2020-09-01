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

om_0s = om_0 * np.append([0.5, 1, 1.5], np.arange(2, 27, 1))
print(f'Number of harmonics {len(om_0s)}')
NSamples = 1024

for signal in signals_props:
    freq = np.fft.rfftfreq(NSamples, 1 / f)
    Y = np.fft.rfft(signal[0:NSamples]) / NSamples
    #plt.plot(range(len(freq)), np.abs(Y) , label = f"{dir_names[1]} {loudnesses[1]} {gt_degrees_list[1]} {sources[1]}")
    plt.plot(freq, np.abs(Y) , label = f"{dir_names[1]} {loudnesses[1]} {gt_degrees_list[1]} {sources[1]}")

#print(freq)
bins_basic_candidates = np.ones(np.size(freq))

DELTA_F_PROP = 100
FFTSIZE_SENT = 32

hi_pass_f = 100
lo_pass_f = 10000

lo_pass_index = lo_pass_f/(f/NSamples)
hi_pass_index = hi_pass_f/(f/NSamples)

print(f/NSamples)

print(f'hi_pass_index = {hi_pass_index } ({hi_pass_f}), lo_pass_index = {lo_pass_index } ({lo_pass_f})')

# Remove propeler sound +- delta f
for i in range(np.size(freq)):
    in_prop_range = 0
    for om_0_i in om_0s:
        if( np.abs((freq[i] - om_0_i )) < DELTA_F_PROP):
            in_prop_range = 1
            break
    if in_prop_range:
        bins_basic_candidates[i] = 0

# Remove Hi-pass and low-pass candidates
for i in range(np.size(freq)):
    if((freq[i] < hi_pass_f) | (freq[i] > lo_pass_f )):
        bins_basic_candidates[i] = 0

for i in range(np.size(freq)):
    print(f'i = {i}: bins_basic_candidates = {bins_basic_candidates[i]}')

# Count candidate that are still available
sum_candidate = 0
for i in range(np.size(bins_basic_candidates)):
    if(bins_basic_candidates[i] == 1):
        sum_candidate += 1

# Backup 
bins_snr_avoid_prop = np.zeros(np.shape(bins_basic_candidates))
bins_uniforme_avoid_prop = np.zeros(np.shape(bins_basic_candidates))

## samples uniformly among remaining candidates
decimation = int(np.ceil(sum_candidate / FFTSIZE_SENT)) + 1
print(f'candidates = {sum_candidate}, decimation =  {decimation}')

number_selected_candidates = 0
selected_bins = np.zeros(FFTSIZE_SENT)
for i in range(np.size(freq)):

    if((bins_basic_candidates[i] == 1)):
        
        sum = 0
        # check if any of the previous decimation bits where 1
        for j in range(decimation):
            sum += bins_uniforme_avoid_prop[i - j] 

        # if previous samples where 0, then ok to place a one
        if( (sum == 0) & (number_selected_candidates < FFTSIZE_SENT)):
            bins_uniforme_avoid_prop[i] = 1
            selected_bins [number_selected_candidates] = i
            number_selected_candidates += 1
        else:
            bins_uniforme_avoid_prop[i] = 0
    else:
        bins_uniforme_avoid_prop[i] = 0

print(selected_bins)

sum = 0
for j in range(len(bins_uniforme_avoid_prop)):
    sum += bins_uniforme_avoid_prop[j] 
print(f'Final chosen bins_uniforme_avoid_prop count is: {number_selected_candidates} ')

#plt.scatter(range(len(freq)), + 1000 * bins_basic_candidates, label = 'bins_basic_candidates')
#plt.scatter(range(len(freq)), + 600 * bins_uniforme_avoid_prop, label = 'bins_uniforme_avoid_prop')
plt.scatter(freq, + 1000 * bins_basic_candidates, label = 'bins_basic_candidates')
plt.scatter(freq, + 600 * bins_uniforme_avoid_prop, label = 'bins_uniforme_avoid_prop')
plt.legend()

plt.autoscale(enable = True, axis = 'x', tight = True)

plt.show()
