#!/usr/bin/env python3
# coding: utf-8

import numpy as np

from crazyflie_description_py.parameters import N_BUFFER, FS, FFTSIZE


def get_frequencies():
    return np.fft.rfftfreq(N_BUFFER, 1 / FS)


def generate_sweep(key):
    from crazyflie_description_py.parameters import SOUND_EFFECTS

    freqs = get_frequencies()

    if key == "sweep_all":
        t_sec = 0.5  # duration of each note in seconds
        bins = list(range(len(freqs)))
    elif key == "sweep_buzzer":
        # obtained from WallAnalysis.ipynb
        t_sec = 1.0
        freqs_buzz = [
            1000.0,
            1015.625,
            1031.25,
            1046.875,
            1062.5,
            1078.125,
            1093.75,
            1109.375,
            1125.0,
            1140.625,
            1156.25,
            1171.875,
            1187.5,
            1203.125,
            1218.75,
            1234.375,
            1250.0,
            1265.625,
            1281.25,
            1296.875,
            1312.5,
            1328.125,
            1343.75,
            1359.375,
            1375.0,
            1390.625,
            1406.25,
            1421.875,
            1437.5,
            1453.125,
            1468.75,
            1484.375,
            1500.0,
            1515.625,
            1531.25,
            1546.875,
            1562.5,
            1578.125,
            1593.75,
            1609.375,
            1625.0,
            1640.625,
            1656.25,
            1671.875,
            1687.5,
            1703.125,
            1718.75,
            1734.375,
            1750.0,
            1765.625,
            1781.25,
            1796.875,
            1812.5,
            1828.125,
            1843.75,
            1859.375,
            1875.0,
            1890.625,
            1906.25,
            1921.875,
            1937.5,
            1953.125,
            1968.75,
            1984.375,
            2000.0,
            2015.625,
            2031.25,
            2046.875,
            2062.5,
            2078.125,
            2093.75,
            2109.375,
            2125.0,
            2140.625,
            2156.25,
            2171.875,
            2187.5,
            2203.125,
            2218.75,
            2234.375,
            2250.0,
            2265.625,
            2281.25,
            2296.875,
            2312.5,
            2328.125,
            2343.75,
            2359.375,
            2375.0,
            2390.625,
            2406.25,
            2421.875,
            2437.5,
            2453.125,
            2468.75,
            2484.375,
            2500.0,
            2515.625,
            2519.384765625,
            2562.451171875,
            2627.05078125,
            2691.650390625,
            2777.783203125,
            2885.44921875,
            3014.6484375,
            3122.314453125,
            3186.9140625,
            3273.046875,
            3380.712890625,
            3466.845703125,
            3531.4453125,
            3574.51171875,
            3617.578125,
            3682.177734375,
            3725.244140625,
            3768.310546875,
            3811.376953125,
            3854.443359375,
            3919.04296875,
            3962.109375,
            4005.17578125,
            4048.2421875,
            4112.841796875,
            4155.908203125,
            4220.5078125,
            4263.57421875,
            4328.173828125,
            4371.240234375,
            4435.83984375,
            4500.439453125,
            4565.0390625,
            4629.638671875,
            4694.23828125,
            4758.837890625,
            4823.4375,
            4909.5703125,
            4974.169921875,
            5060.302734375,
            5124.90234375,
            5211.03515625,
            5297.16796875,
            5383.30078125,
            5469.43359375,
            5577.099609375,
            5663.232421875,
            5770.8984375,
            5857.03125,
            5964.697265625,
            6072.36328125,
            6201.5625,
            6309.228515625,
            6438.427734375,
            6567.626953125,
            6696.826171875,
            6847.55859375,
            6998.291015625,
            7149.0234375,
            7299.755859375,
            7472.021484375,
            7644.287109375,
            7816.552734375,
            8010.3515625,
            8204.150390625,
            8419.482421875,
            8634.814453125,
            8871.6796875,
            9130.078125,
            9388.4765625,
            9668.408203125,
            9948.33984375,
            10271.337890625,
            10594.3359375,
            10938.8671875,
            11326.46484375,
            11735.595703125,
            12166.259765625,
            12639.990234375,
            13135.25390625,
            13695.1171875,
            14276.513671875,
            14922.509765625,
            15633.10546875,
        ]
        bins = [np.argmin(np.abs(freqs - f)) for f in freqs_buzz]
    elif key == "sweep_hard":
        t_sec = 0.5  # duration of each note in seconds
        # obtained from WallAnalysis.ipynb
        freqs_hard = [
            1171.875,
            1234.375,
            1390.625,
            3015.625,
            3125.0,
            3531.25,
            3687.5,
            4156.25,
            4265.625,
            5125.0,
            5218.75,
            6437.5,
            6687.5,
            8000.0,
            8421.875,
            10937.5,
            13140.625,
            14921.875,
        ]
        bins = [np.argmin(np.abs(freqs - f)) for f in freqs_hard]
    elif key == "sweep_slow":
        t_sec = 2.5  # duration of each frequency
        freqs_hard = [1750, 2375, 3125, 3875]
        bins = [np.argmin(np.abs(freqs - f)) for f in freqs_hard]
    elif key == "sweep_fast":
        t_sec = 0.5  # duration of each frequency
        freqs_hard = [1750, 2375, 3125, 3875]
        bins = [np.argmin(np.abs(freqs - f)) for f in freqs_hard]
    elif key == "sweep_short":
        t_sec = 1.0  # duration of each frequency
        min_freq, max_freq = SOUND_EFFECTS[key][1]
        bins = select_frequencies(
            min_freq=min_freq, max_freq=max_freq, n_freqs=int(FFTSIZE // 2)
        )
    else:
        t_sec = 1.0  # duration of each note in seconds
        min_freq, max_freq = SOUND_EFFECTS[key][1]
        bins = select_frequencies(min_freq=min_freq, max_freq=max_freq)
    return bins, t_sec


def select_frequencies(
    n_buffer=N_BUFFER,
    fs=FS,
    thrust=0,
    min_freq=100,
    max_freq=10000,
    filter_snr=False,
    buffer_f=None,
    delta_freq=100,
    ax=None,
    verbose=False,
    n_freqs=FFTSIZE,
):
    freq = get_frequencies()
    n_frequencies = len(freq)
    df = fs / n_buffer
    np.testing.assert_allclose(freq, df * np.arange(n_buffer // 2 + 1))

    min_index = int(round(min_freq * n_buffer / fs))
    max_index = int(round(max_freq * n_buffer / fs))
    min_idx = np.argmin(np.abs(freq - min_freq))
    max_idx = np.argmin(np.abs(freq - max_freq))
    assert min_index == min_idx, (min_idx, min_index, min_index * df, freq[min_idx])
    assert max_index == max_idx, (max_idx, max_index, max_index * df, freq[max_idx])
    assert freq[min_idx] == df * min_index
    assert freq[max_idx] == df * max_index

    assert (
        max_index < n_frequencies
    ), f"given max frequency {max_freq}Hz too high for sampling frequency {fs}Hz"

    potential_indices = []

    # Calculate propeller sound bins
    if thrust > 0:
        prop_freq = 3.27258551 * np.sqrt(thrust) - 26.41814899
        prop_factors = np.append([0.5, 1, 1.5], np.arange(2, 27, 1))

    # Remove propeller sound +- delta f
    for i in range(min_index, max_index):
        use_this = True
        # if this frequency is not in propellers, add it to potential bins.
        if thrust > 0:
            for j in range(len(prop_factors)):
                if abs((i * df) - (prop_factors[j] * prop_freq)) < delta_freq:
                    if ax is not None:
                        ax.scatter(i * df, 0, color="red")
                    use_this = False
                    break
        if use_this:
            potential_indices.append(i)

    if verbose:
        print(f"selecting {FFTSIZE} from {len(potential_indices)}")
    if not potential_indices:
        potential_indices = [min_index]

    # Select indices based on snr or uniformly.
    selected_indices = []
    if not filter_snr:  # choose uniformly every k-th bin
        if len(potential_indices) > n_freqs:
            decimation = float(len(potential_indices) / n_freqs)
            for i in range(n_freqs):
                idx = int(round(i * decimation))
                selected_indices += [potential_indices[idx]]
        else:
            selected_indices = potential_indices

    else:  # choose the highest K amplitude bins.
        # C-like structure to be used in qsort.
        signals_amp_list = []
        for i in potential_indices:
            sum_ = 0
            for j in range(buffer_f.shape[0]):  # buffer_f is of shape n_mics x n_bins_
                sum_ += np.abs(buffer_f[j, i])
            struct = {"amplitude": sum_, "index": i}
            signals_amp_list.append(struct)

        # below would be replaced by qsort
        sorted_signals_amp_list = sorted(
            signals_amp_list, key=lambda elem: elem["amplitude"]
        )[::-1]

        for i in range(n_freqs):
            # thanks to
            elements = sorted_signals_amp_list[i : i + 1]
            selected_indices += [e["index"] for e in elements]

    if len(selected_indices) < n_freqs:
        if verbose:
            print("Warning: found less indices than asked for. Filling with zeros")
        selected_indices = np.r_[
            selected_indices, [0] * (n_freqs - len(selected_indices))
        ]

    assert len(selected_indices) == n_freqs, len(selected_indices)
    return selected_indices
