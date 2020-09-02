#!/usr/bin/env python
# coding: utf-8
"""
file_parser.py: functions to parse audio measurements into standard numpy array format.
"""

import os

from mic_array import get_square_array

import numpy as np
from scipy.io.wavfile import read

root_dir = os.path.abspath(os.path.dirname(__file__) + "/../data/")

FILENAMES = [
    "back_left.npy",  # 0, should be 2
    "back_right.npy",  # 1, should be 0
    "front_left.npy",  # 2, should be 1
    "front_right.npy",  # 3, should be 3
]
# Note that these positions were not very accurate
MEMS_MIC_POSITIONS = get_square_array(baseline=0.11, delta=0) 
MEMS_SOURCE_DISTANCE = 1.925

# These positions were more accurate
MEAS_MIC_POSITIONS = get_square_array(baseline=0.11, delta=0)
MEAS_SOURCE_DISTANCE = 1.965

SIM_MIC_POSITIONS = get_square_array(baseline=0.11, delta=0)
SIM_SOURCE_DISTANCE = 1.925

parameters = {
        "recordings_9_7_20": {"Fs": 32000, "time_index": 100, "mic_positions": MEMS_MIC_POSITIONS, "source_distance": MEMS_SOURCE_DISTANCE},
        "recordings_14_7_20": {"Fs": 32000, "time_index": 100, "mic_positions": MEMS_MIC_POSITIONS, "source_distance": MEMS_SOURCE_DISTANCE},
        "recordings_16_7_20": {"Fs": 48000, "time_index": 42000, "mic_positions": MEAS_MIC_POSITIONS, "source_distance": MEAS_SOURCE_DISTANCE},
        "analytical": {"Fs": 44100, "time_index": 0, "mic_positions": SIM_MIC_POSITIONS, "source_distance": SIM_SOURCE_DISTANCE} 
        "pyroomacoustics": {"Fs": 44100, "time_index": 400, "mic_positions": SIM_MIC_POSITIONS, "source_distance": SIM_SOURCE_DISTANCE} 
}



def read_all(dir_name, Fs=None, verbose=False, fnames=FILENAMES):
    n_times = None
    for i, fname in enumerate(fnames):
        fullname = f"{dir_name}/{fname}"
        if fullname.split('.')[-1] == "npy":
            # Unfortunately we cannot check that the sampling rate matches for 
            # numpy arrays as it is not saved.
            signal0 = np.load(fullname)
        elif fullname.split('.')[-1] == "wav":
            Fs_new, signal0 = read(fullname)
            # Check that the sampling rate matches for wav files.
            if Fs is not None:
                assert Fs == Fs_new
            Fs = Fs_new
        if verbose:
            print("read", fullname)
        signal0 = signal0 - np.mean(signal0)
        if n_times is None:
            n_times = len(signal0)
            signals = np.empty((len(fnames), n_times))
        signals[i, :] = signal0
    return signals, Fs


def read_recording_9_7_20(loudness="high", gt_degrees=0, verbose=False, type_="props"):
    if type_ == "props":
        dir_name = root_dir + "recordings_9_7_20/propellers_only/"
    elif type_ == "source":
        dir_name = root_dir + f"recordings_9_7_20/200Hz/{loudness}_sound/without_prop/"
    elif type_ == "all":
        dir_name = root_dir + f"recordings_9_7_20/200Hz/{loudness}_sound/with_prop/{gt_degrees}_deg/"
    Fs = parameters["recordings_9_7_20"]
    return read_all(dir_name, Fs, verbose)


def read_recording_14_7_20(gt_degrees=0, verbose=False, type_="props"):
    if type_ == "props":
        dir_name = root_dir + "/recordings_14_7_20/all_propellers/"
    elif type_ == "source":
        dir_name = root_dir + f"/recordings_14_7_20/external_source_only/"
    elif type_ == "all":
        dir_name = root_dir + f"/recordings_14_7_20/external_source_and_propellers/{gt_degrees}deg/"

    Fs = parameters["recordings_14_7_20"]
    return read_all(dir_name, Fs, verbose)


def read_recordings_9_7_20(loudness="high", gt_degrees=0, verbose=False):
    signals_props,*_ = read_recording_9_7_20(loudness, gt_degrees, verbose, type_="props")
    signals_source,*_ = read_recording_9_7_20(loudness, gt_degrees, verbose, type_="source")
    signals_all,*_ = read_recording_9_7_20(loudness, gt_degrees, verbose, type_="all")
    return signals_props, signals_source, signals_all


def read_recordings_14_7_20(gt_degrees=0, verbose=False):
    signals_props,*_ = read_recording_14_7_20(gt_degrees, verbose, type_="props")
    signals_source,*_ = read_recording_14_7_20(gt_degrees, verbose, type_="source")
    signals_all,*_ = read_recording_14_7_20(gt_degrees, verbose, type_="all")
    return signals_props, signals_source, signals_all


def read_recording_16_7_20(fname, verbose):
    signals_wrong = np.load(fname).astype(np.float).T
    if verbose:
        print("read", fname)

    # fix order, because the signals are ordered as
    # 3  0              2  3
    # 2  1  instead of  0  1
    signals = np.empty(signals_wrong.shape)
    signals[2, :] = signals_wrong[3, :]
    signals[3, :] = signals_wrong[0, :]
    signals[0, :] = signals_wrong[2, :]
    signals[1, :] = signals_wrong[1, :]
    return signals


def read_recordings_16_7_20(loudness="high", gt_degrees=0, source="white_noise", verbose=False):
    file_props = root_dir + "/recordings_16_7_20/propellers_only/all.npy"
    if source == "white_noise":
        file_source = root_dir + f"/recordings_16_7_20/white_noise/{loudness}/{gt_degrees}deg/wn_only.npy"
        file_all = root_dir + f"/recordings_16_7_20/white_noise/{loudness}/{gt_degrees}deg/wn_and_props.npy"
    elif source == "200Hz":
        file_source = root_dir + f"/recordings_16_7_20/200Hz/{loudness}/{gt_degrees}deg/200Hz_only.npy"
        file_all = root_dir + f"/recordings_16_7_20/200Hz/{loudness}/{gt_degrees}deg/200Hz_and_props.npy"
    else:
        raise ValueError(source)

    signals_props = read_recording_16_7_20(file_props, verbose)
    signals_source = read_recording_16_7_20(file_source, verbose)
    signals_all = read_recording_16_7_20(file_all, verbose)
    n_times = min(
        signals_props.shape[1],
        signals_all.shape[1],
        signals_source.shape[1],
    )
    signals_props = signals_props[:, -n_times:]
    signals_source = signals_source[:, -n_times:]
    signals_all = signals_all[:, -n_times:]
    return signals_props, signals_source, signals_all


def read_recordings(dir_name, loudness, gt_degrees, source=None):
    if dir_name == "recordings_9_7_20":
        return read_recordings_9_7_20(loudness, gt_degrees)
    elif dir_name == "recordings_14_7_20":
        return read_recordings_14_7_20(gt_degrees)
    elif dir_name == "recordings_16_7_20":
        return read_recordings_16_7_20(loudness, gt_degrees, source)
    else:
        raise ValueError(dir_name)


def read_simulation(type_="analytical", verbose=False):
    Fs = parameters[type_]
    data_dir = root_dir + "/simulated/"
    fnames_source = [f"{type_}_source_mic{i}.wav" for i in range(1, 5)]
    signals_source = read_all(data_dir, Fs, verbose, fnames_source)
    fnames_all = [f"{type_}_all_mic{i}.wav" for i in range(1, 5)]
    signals_all = read_all(data_dir, Fs, verbose, fnames_all)
    fnames_props = [f"{type_}_props_mic{i}.wav" for i in range(1, 5)]
    signals_props = read_props(data_dir, Fs, verbose, fnames_props)
    return signals_props, signals_source, signals_all
