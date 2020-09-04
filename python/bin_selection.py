#!/usr/bin/env python3
# coding: utf-8

import numpy as np

N_BUFFER = 1024

# TODO(FD) should below two parameters be input to function? 
DELTA_F_PROP = 100
FFTSIZE_SENT = 32

def select_frequencies(n_buffer, fs, thrust=None, min_freq=100, max_freq=10000, filter_snr=False, buffer_f=None, ax=None):
    freq = np.fft.rfftfreq(n_buffer, 1 / fs)
    n_frequencies = len(freq)

    min_index = int(min_freq * n_buffer / fs)
    max_index = int(max_freq * n_buffer / fs)

    assert max_index < n_frequencies, f"{max_index, n_frequencies}"

    potential_indices = []

    # Calculate propeller sound bins
    if thrust is not None:
        prop_freq = 3.27258551 * np.sqrt(thrust) - 26.41814899
        prop_indices = np.append([0.5, 1, 1.5], np.arange(2, 27, 1))

    # Remove propeller sound +- delta f
    for i in np.arange(min_index, max_index):
        use_this = True
        # if this frequency is not in propellers, add it to potential bins.
        if thrust is not None:
            for prop_i in prop_indices:
                if (abs(freq[i] - (prop_i * prop_freq)) < DELTA_F_PROP):
                    if ax is not None:
                        ax.scatter(freq[i], 0, color='red')
                    use_this = False
                    break
        if use_this:
            potential_indices.append(i)

    assert len(potential_indices) >= FFTSIZE_SENT
    print(f'selecting {FFTSIZE_SENT} from {len(potential_indices)}')

    # Select indices based on snr or uniformly. 
    selected_indices = []
    if not filter_snr: # choose uniformly every k-th bin
        decimation = len(potential_indices) / FFTSIZE_SENT
        for i in range(FFTSIZE_SENT):
            selected_indices.append(potential_indices[round(i * decimation)])

    else: # choose the highest K amplitude bins. 
        # C-like structure to be used in qsort.
        signals_amp_list = []
        for i in potential_indices:
            sum_ = 0
            for j in range(buffer_f.shape[0]):
                sum_ += np.abs(buffer_f[j, i])
            struct = {'amplitude': sum_, 'index': i}
            signals_amp_list.append(struct)

        # below would be replaced by qsort
        sorted_signals_amp_list = sorted(signals_amp_list, key=lambda elem: elem['amplitude'])[::-1]

        for i in range(FFTSIZE_SENT):
            selected_indices.append(sorted_signals_amp_list[i]['index'])

    return selected_indices

if __name__ == "__main__":
    import matplotlib.pylab as plt
    from file_parser import read_recordings, parameters

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

    dir_name = dir_names[1]
    loudness = loudnesses[1]
    gt_degrees = gt_degrees_list[1]
    source = sources[1]
    Fs = parameters[dir_name]["Fs"]

    signals_props, signals_source, signals_all = read_recordings(dir_name=dir_name, 
            loudness=loudness, gt_degrees=gt_degrees, source=source)

    buffer_f = np.fft.rfft(signals_props[:, :N_BUFFER])
    freq = np.fft.rfftfreq(N_BUFFER, d=1/Fs)

    fig, ax = plt.subplots()

    thrust = 43000
    min_freq = 100
    max_freq = 4000
    assert max_freq <= Fs / 2
    bin_uniform_avoid_props = select_frequencies(N_BUFFER, Fs, thrust=thrust, min_freq=min_freq, max_freq=max_freq, filter_snr=False, ax=ax)
    bin_uniform_avoid_props_snr = select_frequencies(N_BUFFER, Fs, thrust=thrust, min_freq=min_freq, max_freq=max_freq, filter_snr=True, buffer_f=buffer_f, ax=ax)

    ### plotting
    max_amp = np.max(np.abs(buffer_f[:, (freq<max_freq) & (freq>min_freq)]))
    for i, buffer_f_i in enumerate(buffer_f):
        ax.plot(freq, np.abs(buffer_f_i), label = f"mic{i}")
    ax.scatter(freq[bin_uniform_avoid_props], [0.3 * max_amp] * FFTSIZE_SENT, label = 'bins_avoid_props')
    ax.scatter(freq[bin_uniform_avoid_props_snr], [0.6 * max_amp] * FFTSIZE_SENT, label = 'bins_avoid_props_snr')
    ax.set_xlim(0.5*min_freq, 1.5*max_freq)
    ax.set_ylim(-1, max_amp)
    ax.legend()
    plt.show()
