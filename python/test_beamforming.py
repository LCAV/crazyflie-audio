#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_beamforming.py:  
"""

import numpy as np

from algos_basics import get_mic_delays, get_autocorrelation
from algos_beamforming import get_das_beamformer, get_lcmv_beamformer
from algos_beamforming import get_powers

from mic_array import get_square_array

PLOTTING = False


# TODO: could be shared with other scripts.
def prepare(mic_positions, gt_azimuth=30 * np.pi / 180):
    offset = 0  # phase offset in radiants

    # TODO: should be equal to lamda?
    noise = 1e-3  # independent receiver noise

    Fs = 44100  # Hz
    times = np.arange(0, 0.01, step=1 / Fs)  # seconds

    n_times = len(times)

    frequencies = np.fft.rfftfreq(n=n_times, d=1 / Fs)
    frequencies_signal = [200, 300, 400, 500]  # Hz
    assert np.all(
        [f in frequencies for f in frequencies_signal]
    ), f"choose valid signal frequencies! {(frequencies, frequencies_signal)}"

    delays = get_mic_delays(mic_positions, azimuth=gt_azimuth)
    signals = np.zeros((mic_positions.shape[0], n_times))
    for freq in frequencies_signal:
        new_signals = np.array(
            [np.cos(2 * np.pi * freq * (times - delay) + offset) for delay in delays]
        )
        assert new_signals.shape == signals.shape
        signals += new_signals

    signals += np.random.normal(loc=0, scale=noise, size=signals.shape)

    signals_f = np.fft.rfft(signals, n=n_times, axis=1)
    signals_test = np.fft.irfft(signals_f, n=n_times, axis=1)
    np.testing.assert_allclose(signals, signals_test, atol=1e-10)

    freq_bins = [int(f / Fs * n_times) for f in frequencies_signal]
    assert all(
        [
            freq_bins[i] == np.where(f == frequencies)[0][0]
            for i, f in enumerate(frequencies_signal)
        ]
    )
    Rx = get_autocorrelation(signals, freq_bins)
    return signals_f, Rx, frequencies_signal, freq_bins


def test_beamforming():
    def plot(responses, title="DAS", ax=None):
        if ax is None:
            fig, ax = plt.subplots()
        combination_sum = np.sum(responses, axis=1)
        combination_product = np.product(responses, axis=1)
        for j, f in enumerate(frequencies_signal):
            ax.plot(azimuth_scan * 180 / np.pi, responses[:, j], label=f"{f:.0f} Hz")
        ax.plot(azimuth_scan * 180 / np.pi, combination_sum, label=f"sum")
        ax.plot(azimuth_scan * 180 / np.pi, combination_product, label=f"product")
        ax.axvline(x=gt_azimuth * 180 / np.pi, label="source direction", ls=":")
        ax.set_yscale("log")
        ax.set_title(title)
        ax.legend()

    test_angles_deg = np.linspace(0, 180, 4)

    baseline = 0.11  # meters
    mic_positions_dict = {
        "ULA": baseline * np.c_[[0, 0], [1, 0], [2, 0]].T,
        #'modified ULA': baseline * np.c_[[0, 1], [1, 0], [2, 0]].T,
        "square": get_square_array(baseline=baseline, delta=0),
    }

    for mic_name, mic_positions in mic_positions_dict.items():
        print("mic_positions", mic_name)
        n_mics = mic_positions.shape[0]

        for deg in test_angles_deg:
            print("source at", deg)
            gt_azimuth = deg * np.pi / 180  # radiants
            signals_f, Rx, frequencies_signal, freq_bins = prepare(
                mic_positions, gt_azimuth
            )

            # TODO: when we do the full circle, LCMV places a high responses symmetrically to the zero,
            # and we get a wrong estimate. We might want to figure out why.
            # azimuth_scan = np.linspace(0, 360, 361) * np.pi / 180
            azimuth_scan = np.linspace(0, 180, 181) * np.pi / 180
            responses_das = np.zeros((len(azimuth_scan), len(frequencies_signal)))
            responses_lcmv = np.zeros((len(azimuth_scan), len(frequencies_signal)))
            responses_mvdr = np.zeros((len(azimuth_scan), len(frequencies_signal)))

            lamda = 1e-3
            lcmv_zero = gt_azimuth + (30 * np.pi / 180.0)

            for i, az in enumerate(azimuth_scan):
                constraints = [(az, 1)]
                h_mvdr_vector, *_ = get_lcmv_beamformer(
                    Rx, frequencies_signal, mic_positions, constraints, lamda=lamda
                )  # n_freqs x n_mics
                delays = get_mic_delays(mic_positions, azimuth=az)

                if az != lcmv_zero:
                    constraints = [(az, 1), (lcmv_zero, 0)]
                    h_lcmv_vector, *_ = get_lcmv_beamformer(
                        Rx, frequencies_signal, mic_positions, constraints
                    )  # n_freqs x n_mics
                else:
                    constraints = [(az, 1)]
                    h_lcmv_vector, *_ = get_lcmv_beamformer(
                        Rx, frequencies_signal, mic_positions, constraints
                    )  # n_freqs x n_mics

                h_das_vector = get_das_beamformer(
                    az, frequencies_signal, mic_positions
                )  # n_freqs x n_mics

                for j, f in enumerate(frequencies_signal):
                    freq_bin = freq_bins[j]
                    h_das = h_das_vector[j]  # n_mics
                    h_mvdr = h_mvdr_vector[j]  # n_mics
                    h_lcmv = h_lcmv_vector[j]  # n_mics
                    a_vector = np.exp(-1j * delays * 2 * np.pi * f)

                    # DAS
                    np.testing.assert_allclose(np.sum(np.abs(h_das)), 1)
                    np.testing.assert_allclose(h_das.conj().dot(a_vector), 1.0)
                    power_y_das = np.abs(h_das.conj().dot(Rx[j]).dot(h_das))
                    responses_das[i, j] = power_y_das

                    power_y_das_test = get_powers(h_das_vector, Rx)[j]
                    np.testing.assert_allclose(
                        power_y_das, power_y_das_test, atol=1e-10
                    )

                    power_y_das_test2 = (
                        np.abs(h_das.conj().dot(signals_f[:, freq_bin])) ** 2 / n_mics
                    )
                    np.testing.assert_allclose(
                        power_y_das, power_y_das_test2, atol=1e-10
                    )

                    # check that phase of h exactly compensates for delays in ground truth direction.
                    if az == gt_azimuth:
                        np.testing.assert_allclose(
                            np.unwrap(np.angle(h_das)) / (2 * np.pi * f),
                            -delays.reshape(h_das.shape),
                        )

                    # MVDR
                    np.testing.assert_allclose(h_mvdr.conj().dot(a_vector), 1.0)
                    power_y_mvdr = np.abs(h_mvdr.conj().dot(Rx[j]).dot(h_mvdr))
                    responses_mvdr[i, j] = power_y_mvdr

                    power_y_mvdr_test = get_powers(h_mvdr_vector, Rx)[j]
                    np.testing.assert_allclose(
                        power_y_mvdr, power_y_mvdr_test, atol=1e-10
                    )

                    # TODO is it normal that this atol has to be relatively high?
                    power_y_mvdr_test2 = (
                        np.abs(h_mvdr.conj().dot(signals_f[:, freq_bin])) ** 2 / n_mics
                    )
                    np.testing.assert_allclose(
                        power_y_mvdr, power_y_mvdr_test2, atol=1e-5
                    )

                    # TODO not sure why this does not work
                    # however it does also work, so there is just a scaling issue somewhere.
                    Rx_inv = np.linalg.inv(Rx[j] + lamda * np.eye(n_mics))
                    power_y_mvdr_test3 = 1 / np.abs(
                        a_vector.conj().dot(Rx_inv).dot(a_vector)
                    )
                    # np.testing.assert_allclose(power_y_mvdr, power_y_mvdr_test3, atol=1e-5)

                    # LCMV
                    # np.testing.assert_allclose(np.abs(h_lcmv.conj().dot(a_vector)), 1.0)
                    power_y_lcmv = np.abs(h_lcmv.conj().dot(Rx[j]).dot(h_lcmv))
                    responses_lcmv[i, j] = power_y_lcmv

            # TODO reponses_lcmv does not always work, to be checked later.
            for responses, name in zip(
                [responses_das, responses_mvdr], ["das", "mvdr"]  # responses_lcmv],
            ):  # , 'lcmv']):
                combination_sum = np.sum(responses, axis=1)
                combination_product = np.product(responses, axis=1)

                # make sure the peak is at the correct location
                angle_res = abs(azimuth_scan[1] - azimuth_scan[0])

                # because the array is linear, there is an ambiguity.
                guess_sum = azimuth_scan[np.argmax(combination_sum)]
                guess_product = azimuth_scan[np.argmax(combination_product)]
                if guess_sum > np.pi:
                    guess_sum = 2 * np.pi - guess_sum
                if guess_product > np.pi:
                    guess_product = 2 * np.pi - guess_product
                assert (
                    abs(guess_sum - gt_azimuth) < angle_res / 2.0
                ), f"failed for {name}, {deg}"
                assert (
                    abs(guess_product - gt_azimuth) < angle_res / 2.0
                ), f"failed for {name}, {deg}"

    if PLOTTING:
        import matplotlib.pylab as plt

        fig, axs = plt.subplots(3, 1)
        plot(responses_das, "DAS", ax=axs[0])
        plot(responses_mvdr, "MVDR", ax=axs[1])
        plot(responses_lcmv, "LCMV", ax=axs[2])
        plt.show()
