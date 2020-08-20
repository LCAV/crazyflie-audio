#!/usr/bin/env python
# coding: utf-8
"""
algos_beamforming.py: algorithms for beamforming
"""
import numpy as np

from algos_basics import get_autocorrelation, get_mic_delta, get_mic_delays
from constants import SPEED_OF_SOUND

def get_lcmv_beamformer(
    Rx, frequencies, mic_positions, constraints, rcond=0, lamda=1e-3, elevation=None
):
    """

    Solves the problem
    h = min E(|y(t)|**2)
    s.t. 

    :param Rx: n_frequencies x n_mics x n_mics covariance matrix.
    :param frequencies: frequencies in Hz. 
    :param mic_positions: n_mics x dim positions of mic array
    :param constraints: list of constraint tuples of form (angle: response)
    :param rcond: regularization parameter. 
    
    :returns: list of n_mics complex microphone gains for LCMV algorithm. 
    """
    assert mic_positions.ndim == 2, "watch out, changed argument order!!"
    
    n_mics = Rx.shape[1]

    assert mic_positions.shape[0] == n_mics

    H = np.zeros(
        (len(frequencies), n_mics), dtype=np.complex128
    )  # n_frequencies x n_mics

    condition_numbers = []
    condition_numbers_big = []
    for j, freq in enumerate(frequencies):

        # generate constraints
        C = np.zeros((n_mics, 0), dtype=np.complex128)
        c = np.zeros((0,), dtype=np.complex128)
        for theta, value in constraints:
            delays = get_mic_delays(mic_positions, theta, elevation)
            exponent = delays * 2 * np.pi * freq
            C_row = np.exp(-1j * exponent)

            C = np.c_[C, np.array(C_row).reshape((mic_positions.shape[0], -1))]
            c = np.r_[c, value]

        assert C.shape[1] == len(constraints)
        assert c.shape[0] == len(constraints)

        # solve the optimization problem
        Rx_inv = np.linalg.pinv(Rx[j, :, :] + lamda * np.eye(Rx.shape[1]), rcond=rcond)
        big_mat = C.conj().T @ Rx_inv @ C
        big_inv = np.linalg.pinv(big_mat, rcond=rcond)
        H[j, :] = Rx_inv @ C @ big_inv @ c
        condition_numbers.append(np.linalg.cond(Rx[j, :, :]))
        condition_numbers_big.append(np.linalg.cond(big_mat))
    return H, condition_numbers, condition_numbers_big


def get_das_beamformer(
    azimuth, frequencies, mic_positions, elevation=None, verbose=False
):
    """
    :param azimuth: azimuth angle in rad
    :param frequencies: frequencies in Hz. 
    :returns: list of n_mics complex microphone gains for DAS algorithm. 
    """
    delays = get_mic_delays(mic_positions, azimuth, elevation)
    gains = np.exp(-1j * 2 * np.pi * np.outer(frequencies, delays))
    if verbose:
        print("outer", np.outer(frequencies, delays))
        print("delays", delays)
        print("real, imaginary", np.real(gains), np.imag(gains))
        print("mag, phase", np.abs(gains), np.angle(gains))
        theo = np.unwrap(-2 * np.pi * np.outer(frequencies, delays))
        np.testing.assert_allclose(np.angle(gains), theo)
    return gains / mic_positions.shape[0]


# TODO: redundant function, to be deleted soon.
def get_das_powers(azimuth, frequencies, mic_positions, Rx):
    n_frequencies = Rx.shape[0]
    beamformer = das(azimuth, frequencies, mic_positions)
    powers = np.empty(n_frequencies)
    for j in range(n_frequencies):
        a = beamformer[j, :]
        powers[j] = np.abs(a.conj() @ Rx[j, :, :] @ a / (a.conj() @ a))
    return powers


def get_beampattern(azimuth, frequencies, mic_positions, H):
    mic_ref = mic_positions[0]
    elevation = None
    delays = np.array(
        [
            get_mic_delta(mic_ref, mic_other, azimuth, elevation) / SPEED_OF_SOUND
            for mic_other in mic_positions
        ]
    )
    exponent = 2 * np.pi * np.outer(frequencies, delays)
    if (azimuth <= np.pi / 2) and (azimuth >= 0):
        assert np.all(exponent >= 0)
    directional_response = np.exp(1j * exponent)

    y = np.sum(np.multiply(H.conj(), directional_response), axis=1)  # (n_frequencies,)
    powers = np.abs(y) ** 2
    return powers


def get_powers(H, Rx):
    """ Return power of beamformed signal

    :param H: beamformer (n_frequencies x n_mics)
    :param Rx: covariance matrix (n_frequencies x n_mics x n_mics) 

    :return: vector of powers at each frequency (n_frequencies)

    """
    n_frequencies = Rx.shape[0]
    n_mics = Rx.shape[1]
    assert H.shape[0] == n_frequencies
    assert H.shape[1] == n_mics

    powers = np.empty(n_frequencies)
    for j in range(n_frequencies):
        h = H[j, :]
        powers[j] = np.abs(h.conj() @ Rx[j, :, :] @ h)
    return powers


def select_frequencies(
    n_buffer,
    Fs,
    method,
    max_freq=1000,
    min_freq=100,
    buffer_f=None,
    num_frequencies=None,
    amp_ratio=2,
    delta=4,
):
    """
    Implementation of frequency selection schemes.
    """
    # TODO(FD) replace this hard-coded part.
    propeller_frequency = 600  # Hz, function of thrust command
    external_frequency = 200  # Hz, frequency of external source

    frequencies = np.fft.rfftfreq(n=n_buffer, d=1 / Fs)

    if method == "uniform":
        max_bin = int(max_freq * n_buffer // Fs)
        min_bin = int(min_freq * n_buffer // Fs)
        frequency_bins = np.linspace(min_bin, max_bin, num_frequencies).astype(np.int)
    elif method == "single":
        frequency_bins = [int(external_frequency * n_buffer / Fs)]
    elif method == "harmonics":
        propeller_bin = int(propeller_frequency * n_buffer / Fs)
        if num_frequencies is None:
            num_frequencies = int(max_freq / propeller_frequency)
        frequency_bins = propeller_bin * np.arange(1, num_frequencies + 1)
    elif "between" in method:
        # choose which frequencies to exclude
        exclude_frequencies = propeller_frequency * np.arange(
            1, int(max_freq / propeller_frequency)
        )
        # TODO(FD) maybe rethink this hard-coded part
        delta_f = delta * Fs / n_buffer  # Hz

        all_frequencies = np.fft.rfftfreq(n=n_buffer, d=1 / Fs)

        candidate_bins = []
        for i, f in enumerate(all_frequencies):
            if (
                (f > min_freq)
                and (f < max_freq)
                and all(abs(f - exclude_frequencies) > delta_f)
            ):
                candidate_bins.append(i)

        frequency_bins = candidate_bins

        # sort the frequency bins by snr
        if "between_snr" == method:
            total_avg_amp = np.mean(np.abs(buffer_f))

            filtered_bins = []
            filtered_freqs = []
            for i in candidate_bins:
                avg_amp = np.mean(np.abs(buffer_f[:, i]))
                if avg_amp / total_avg_amp > amp_ratio:
                    filtered_bins.append(i)
                    filtered_freqs.append(all_frequencies[i])

            frequency_bins = np.array(filtered_bins)[np.argsort(filtered_freqs)]

    # make sure we choose the correct num_frequencies
    if num_frequencies is not None and (num_frequencies > len(frequency_bins)):
        print(
            f"Warning: num_frequencies {num_frequencies} is higher than available {len(frequency_bins)}"
        )
    elif num_frequencies is not None and (num_frequencies < len(frequency_bins)):
        # choose the highest snr frequencies among the available ones (bins are sorted)
        if method == "between_snr":
            frequency_bins = frequency_bins[:num_frequencies]
        else:
            frequency_bins = np.random.choice(
                frequency_bins, size=num_frequencies, replace=False
            )
    return sorted(frequency_bins)
