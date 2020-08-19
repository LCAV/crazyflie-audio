#!/usr/bin/env python
# coding: utf-8
"""
file_parser.py: functions to parse audio measurements into standard numpy array format.
"""

import numpy as np

parameters = {
    "recordings_9_7_20": {"Fs": 32000, "time_index": 100},
    "recordings_14_7_20": {"Fs": 32000, "time_index": 100},
    "recordings_16_7_20": {"Fs": 42000, "time_index": 42000},
}

filenames = [
    "back_left",  # 0, should be 2
    "back_right",  # 1, should be 0
    "front_left",  # 2, should be 1
    "front_right",  # 3, should be 3
]


def read_recordings_9_7_20(loudness="high", gt_degrees=0, verbose=False):
    dir_props = "../data/recordings_9_7_20/propellers_only/"
    dir_source = f"../data/recordings_9_7_20/200Hz/{loudness}_sound/without_prop/"
    dir_all = (
        f"../data/recordings_9_7_20/200Hz/{loudness}_sound/with_prop/{gt_degrees}_deg/"
    )

    n_times = None
    for i, fname in enumerate(filenames):

        fullname = f"{dir_props}/{fname}.npy"
        signal0_props = np.load(fullname)
        if verbose:
            print("read", fullname)

        fullname = f"{dir_source}/{fname}.npy"
        signal0_source = np.load(fullname)
        if verbose:
            print("read", fullname)

        fullname = f"{dir_all}/{fname}.npy"
        signal0_all = np.load(fullname)
        if verbose:
            print("read", fullname, "\n")

        signal0_props -= np.mean(signal0_props)
        signal0_source -= np.mean(signal0_source)
        signal0_all -= np.mean(signal0_all)

        if n_times is None:
            n_times = len(signal0_props)
            signals_props = np.empty((len(filenames), n_times))
            signals_source = np.empty((len(filenames), n_times))
            signals_all = np.empty((len(filenames), n_times))

        signals_props[i, :] = signal0_props
        signals_source[i, :] = signal0_source
        signals_all[i, :] = signal0_all

    return signals_props, signals_source, signals_all


def read_recordings_14_7_20(loudness="high", gt_degrees=0, verbose=False):
    dir_props = "../data/recordings_14_7_20/all_propellers/"
    dir_source = f"../data/recordings_14_7_20/external_source_only/"
    dir_all = (
        f"../data/recordings_14_7_20/external_source_and_propellers/{gt_degrees}deg/"
    )

    n_times = None
    for i, fname in enumerate(filenames):

        fullname = f"{dir_props}/{fname}.npy"
        signal0_props = np.load(fullname).astype(np.float)
        if verbose:
            print("read", fullname)

        fullname = f"{dir_source}/{fname}.npy"
        signal0_source = np.load(fullname).astype(np.float)
        if verbose:
            print("read", fullname)

        fullname = f"{dir_all}/{fname}.npy"
        signal0_all = np.load(fullname).astype(np.float)
        if verbose:
            print("read", fullname, "\n")

        signal0_props -= np.mean(signal0_props)
        signal0_source -= np.mean(signal0_source)
        signal0_all -= np.mean(signal0_all)

        if n_times is None:
            n_times = len(signal0_props)
            signals_props = np.empty((len(filenames), n_times))
            signals_source = np.empty((len(filenames), n_times))
            signals_all = np.empty((len(filenames), n_times))

        signals_props[i, :] = signal0_props
        signals_source[i, :] = signal0_source
        signals_all[i, :] = signal0_all
    return signals_props, signals_source, signals_all


def read_recordings_16_7_20(
    loudness="high", gt_degrees=0, source="white_noise", verbose=False
):
    file_props = "../data/recordings_16_7_20/propellers_only/all.npy"
    if source == "white_noise":
        file_source = f"../data/recordings_16_7_20/white_noise/{loudness}/{gt_degrees}deg/wn_only.npy"
        file_all = f"../data/recordings_16_7_20/white_noise/{loudness}/{gt_degrees}deg/wn_and_props.npy"
    elif source == "200Hz":
        file_source = f"../data/recordings_16_7_20/200Hz/{loudness}/{gt_degrees}deg/200Hz_only.npy"
        file_all = f"../data/recordings_16_7_20/200Hz/{loudness}/{gt_degrees}deg/200Hz_and_props.npy"
    else:
        raise ValueError(source)

    signals_props_wrong = np.load(file_props).astype(np.float).T
    signals_source_wrong = np.load(file_source).astype(np.float).T
    signals_all_wrong = np.load(file_all).astype(np.float).T
    if verbose:
        print("read", file_props, file_source, file_all)

    n_times = min(
        signals_props_wrong.shape[1],
        signals_all_wrong.shape[1],
        signals_source_wrong.shape[1],
    )

    signals_props_wrong = signals_props_wrong[:, -n_times:]
    signals_source_wrong = signals_source_wrong[:, -n_times:]
    signals_all_wrong = signals_all_wrong[:, -n_times:]

    # fix order, because the signals are ordered as
    # 3  0              2  3
    # 2  1  instead of  0  1
    signals_props = np.empty(signals_props_wrong.shape)
    signals_source = np.empty(signals_source_wrong.shape)
    signals_all = np.empty(signals_all_wrong.shape)

    signals_source[2, :] = signals_source_wrong[3, :]
    signals_source[3, :] = signals_source_wrong[0, :]
    signals_source[0, :] = signals_source_wrong[2, :]
    signals_source[1, :] = signals_source_wrong[1, :]

    signals_all[2, :] = signals_all_wrong[3, :]
    signals_all[3, :] = signals_all_wrong[0, :]
    signals_all[0, :] = signals_all_wrong[2, :]
    signals_all[1, :] = signals_all_wrong[1, :]

    signals_props[2, :] = signals_props_wrong[3, :]
    signals_props[3, :] = signals_props_wrong[0, :]
    signals_props[0, :] = signals_props_wrong[2, :]
    signals_props[1, :] = signals_props_wrong[1, :]

    return signals_props, signals_source, signals_all


def read_recordings(dir_name, loudness, gt_degrees, source=None):
    if dir_name == "recordings_9_7_20":
        return read_recordings_9_7_20(loudness, gt_degrees)
    elif dir_name == "recordings_14_7_20":
        return read_recordings_14_7_20(loudness, gt_degrees)
    elif dir_name == "recordings_16_7_20":
        return read_recordings_16_7_20(loudness, gt_degrees, source)
    else:
        raise ValueError(dir_name)
