#!/usr/bin/env python
# coding: utf-8
"""
algos_beamforming.py: algorithms for beamforming
"""
import numpy as np

from algos_basics import get_autocorrelation, get_mic_delays, low_rank_inverse
from constants import SPEED_OF_SOUND

INVERSE = "pinv"
# INVERSE = 'low-rank'

def get_lcmv_beamformer(
    Rx, frequencies, mic_positions, constraints, rcond=0, lamda=1e-3, elevation=None, 
    inverse=INVERSE,
):
    """
    Solves the problem
    h = min E(|y(t)|**2)
    s.t. h.conj()C=c

    :param Rx: n_frequencies x n_mics x n_mics covariance matrix.
    :param frequencies: frequencies in Hz. 
    :param mic_positions: n_mics x dim positions of mic array
    :param constraints: list of constraint tuples of form (angle: response)
    :param rcond: regularization parameter. 
    
    :returns: list of n_mics complex microphone gains for LCMV algorithm. 
    """

    print("Deprecation warning: use get_lcmv_beamformer_fast for better performance.")
    assert mic_positions.ndim == 2, "watch out, changed argument order!!"

    n_mics = Rx.shape[1]

    assert mic_positions.shape[0] == n_mics

    H = np.zeros(
        (len(frequencies), n_mics), dtype=np.complex128
    )  # n_frequencies x n_mics

    constraints = trim_constraints(constraints)

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
        if inverse == "pinv":
            Rx_inv = np.linalg.pinv(Rx[j, :, :] + lamda * np.eye(Rx.shape[1]), rcond=rcond)
        elif inverse == "low-rank":
            Rx_inv = np.empty(Rx.shape, dtype=np.complex)
            Rx_inv = low_rank_inverse(Rx[j, ...], rank=1)
        else:
            raise NotImplementedError(inverse)
        # (n_constraints x n_mics) (n_mics  x n_mics) (n_mics x n_constraints)
        # = n_constraints x n_constraints
        big_mat = C.conj().T @ Rx_inv @ C + 1e-5 * np.eye(C.shape[-1])
        big_inv = np.linalg.pinv(big_mat, rcond=rcond)
        H[j, :] = Rx_inv @ C @ big_inv @ c
    return H


def get_lcmv_beamformer_fast(
    Rx,
    frequencies,
    mic_positions,
    constraints,
    rcond=0,
    lamda=1e-3,
    elevation=None,
    inverse=INVERSE,
    cancel_centre=False
):
    """
    Solves the problem: h = min E(|y(t)|**2)
    under the given constraints

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

    # generate constraints
    C = np.zeros((len(frequencies), n_mics, 0), dtype=np.complex128)
    constraints = trim_constraints(constraints, eps=1e-3)

    c = np.array([c[1] for c in constraints])
    all_thetas = np.array([c[0] for c in constraints])
       
    for theta in all_thetas:
        delays = get_mic_delays(mic_positions, theta, elevation)
        exponent = 2 * np.pi * delays[None, :] * frequencies[:, None]  # n_freq x n_mics
        C_row = np.exp(-1j * exponent)
        C = np.concatenate([C, C_row[:, :, None]], axis=2)  # n_freq x n_mics x n_constr
        
    if cancel_centre:
        C_row = np.ones((C.shape[0], C.shape[1], 1))
        C = np.concatenate([C, C_row], axis=2)  # n_freq x n_mics x n_constr
        c = np.r_[c, 0.0]

    # solve the optimization problem
    if inverse == "pinv":
        Rx_inv = np.linalg.pinv(
            Rx + lamda * np.eye(Rx.shape[1])[None, :, :], rcond=rcond
        )  # n_freq x n_mics x n_mics

    elif inverse == "low-rank":
        Rx_inv = np.empty(Rx.shape, dtype=np.complex)
        for i in range(Rx.shape[0]):
            Rx_inv[i, ...] = low_rank_inverse(Rx[i, ...], rank=1)
    # big_mat should have dimension n_freq x n_constr x n_constr
    # (n_freq x n_constr x n_mics) (n_freq x n_mics x n_mics) = n_freq x n_constr x n_mics
    # @ n_freq x n_mics x n_constr = n_freq x n_constr x n_constr
    big_mat = np.transpose(C.conj(), (0, 2, 1)) @ Rx_inv @ C  + 10 * np.eye(C.shape[-1])[None, :, :]
    try:
        big_inv = np.linalg.inv(big_mat)
    except:
        rconds = np.linalg.cond(big_mat)
        singular_freqs = np.where(rconds > 0)[0]
        print("LCMV warning: singular matrix at frequency index", singular_freqs, constraints)
        for f in singular_freqs:
            big_mat[f] += 1e-5 * np.eye(big_mat[f].shape[0])
        big_inv = np.linalg.inv(big_mat)
    # n_freq x n_mics x n_mics @ # n_freq x n_mics x n_constr = n_freq x n_mics x n_constr
    H = Rx_inv @ C @ big_inv @ c
    #for j in range(H.shape[0]):
        #print(H[j, :].conj() @  C[j, :, :] - c)
    return H.conj()


def get_das_beamformer(azimuth, frequencies, mic_positions, elevation=None):
    """
    :param azimuth: azimuth angle in rad
    :param frequencies: frequencies in Hz. 
    :returns: list of n_mics complex microphone gains for DAS algorithm. 
    """
    delays = get_mic_delays(mic_positions, azimuth, elevation)
    gains = np.exp(-1j * 2 * np.pi * np.outer(frequencies, delays))
    return gains / mic_positions.shape[0]


def get_beampattern(azimuth, frequencies, mic_positions, H):
    mic_ref = mic_positions[0]
    elevation = None
    delays = get_mic_delays(mic_positions, azimuth, elevation)
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
    # (n_frequencies x 1 x n_mics) (n_frequencies x n_mics x n_mics) (n_frequencies x n_mics x 1)
    # n_frequencies x 1 x n_mics (n_frequencies x n_mics x 1)
    powers = np.abs(H.conj()[:, None, :] @ Rx @ H[:, :, None]).squeeze()
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

def trim_constraints(constraints, eps=1e-3):
    """ 
    If we are scanning at an angle (c_i=1) where we want to 
    place 0 (c_i=0) then we need to ignore that 1 constraint.
    
    In general: we keep always the minimum constraint if 
    there are multiple for one angle.
    """
    these = range(len(constraints)-1)
    keep_list = []
    i = 0
    all_thetas = np.array([c[0] for c in constraints])
    c = np.array([c[1] for c in constraints], dtype=np.complex128)
    found_duplicate = False
    visited = []
    while i < len(these)+1:
        if i in visited:
            i += 1
            continue
        # because thetas are angles, we need to take the  angle difference
        angle_diff = np.abs(all_thetas[i:] - all_thetas[i])
        angle_diff[angle_diff > np.pi] = 2*np.pi - angle_diff[angle_diff > np.pi]
        angle_diff[angle_diff < 0] += 2*np.pi

        # close_idx sees where the close-by angles are
        close_idx = np.where(angle_diff < eps)[0]
        close_idx += i
        
        constraints_double= c[close_idx]
        keep = np.argmin(constraints_double)
        # keep list sees which of the angles need to be kept.
        keep_list.append(keep + i)
        visited += list(close_idx)
        i += 1
        if len(close_idx) > 1:
            found_duplicate = True
    constraints = list(zip(all_thetas[keep_list], c[keep_list]))
    return constraints
    
def test_trim_constraints():
    all_thetas = np.array([1, 1, 1, 2, 3, 3, 4])
    c = np.array([1, 0.1, 0, 1, 1, 0, 1])
    constraints = list(zip(all_thetas, c))
    constraints = trim_constraints(constraints)
    assert len(constraints) == 4
    assert np.all(constraints == [(1, 0), (2,1), (3,0), (4, 1)])

if __name__ == "__main__":
    test_trim_constraints()
    print("tests ok")
