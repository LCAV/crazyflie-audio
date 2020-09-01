#!/usr/bin/env python
# coding: utf-8
"""
file_parser.py: functions to parse audio measurements into standard numpy array format.
"""

import os

import numpy as np
from scipy.io.wavfile import read

root_dir = os.path.abspath(os.path.dirname(__file__) + "/../data/")

parameters = {
    "recordings_9_7_20": {"Fs": 32000, "time_index": 100},
    "recordings_14_7_20": {"Fs": 32000, "time_index": 100},
    "recordings_16_7_20": {"Fs": 42000, "time_index": 42000},
}

FILENAMES = [
    "back_left.npy",  # 0, should be 2
    "back_right.npy",  # 1, should be 0
    "front_left.npy",  # 2, should be 1
    "front_right.npy",  # 3, should be 3
]

def read_all(dir_name, verbose=False, fnames=FILENAMES):
    n_times = None
    Fs = None
    for i, fname in enumerate(fnames):
        fullname = f"{dir_name}/{fname}"
        if fullname.split('.')[-1] == "npy":
            signal0 = np.load(fullname)
        elif fullname.split('.')[-1] == "wav":
            Fs_new, signal0 = read(fullname)
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
    return read_all(dir_name, verbose)


def read_recording_14_7_20(gt_degrees=0, verbose=False, type_="props"):
    if type_ == "props":
        dir_name = root_dir + "/recordings_14_7_20/all_propellers/"
    elif type_ == "source":
        dir_name = root_dir + f"/recordings_14_7_20/external_source_only/"
    elif type_ == "all":
        dir_name = root_dir + f"/recordings_14_7_20/external_source_and_propellers/{gt_degrees}deg/"
    return read_all(dir_name, verbose)


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


def read_simulation(type_="analytical_source", verbose=False):
    data_dir = root_dir + "/simulated/"
    fnames = [f"{type_}_mic{i}.wav" for i in range(1, 5)]
    return read_all(data_dir, verbose, fnames)
