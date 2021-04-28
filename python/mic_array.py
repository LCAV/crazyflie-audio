"""
mic_array.py: Functions to generate different mic array geometries.
"""

import numpy as np

from constants import SPEED_OF_SOUND


def get_square_array(baseline=1, delta=0):
    """ Generate square array

    :param baseline: square side 
    :param delta: offset towards inside of square (use for propellers)
    """
    xx, yy = np.array(np.meshgrid([delta, baseline - delta], [delta, baseline - delta]))
    mic_positions = np.c_[xx.flatten(), yy.flatten()]
    mic_positions -= np.mean(mic_positions, axis=0)[None, :]
    return mic_positions


def get_min_distance(mic_positions):
    dm = np.linalg.norm(mic_positions[None, :, :] - mic_positions[:, None, :], axis=2)
    return min(dm[dm > 0])


def get_max_distance(mic_positions):
    dm = np.linalg.norm(mic_positions[None, :, :] - mic_positions[:, None, :], axis=2)
    return max(dm[dm > 0])


def get_uniform_array(mic_number, baseline=1, dimension=2):
    """ Create uniform array along x axis. """
    mic_positions_x = np.linspace(0, baseline, mic_number)
    mic_positions = np.zeros((mic_number, dimension))  # M x d
    mic_positions[:, 0] = mic_positions_x
    return mic_positions


def get_tetrahedron_array(dimension=2, baseline=0.05):
    """ Create tetrahedron (in 2 dimensions, simply 2. """
    if not dimension in (2, 3):
        raise ValueError(dimension)

    tetrahedron = np.array(
        [
            [baseline * np.sqrt(3) / 3, 0, 0],
            [-baseline * np.sqrt(3) / 6, baseline / 2.0, 0],
            [-baseline * np.sqrt(3) / 6, -baseline / 2.0, 0],
            [0, 0, baseline * np.sqrt(6) / 3],
        ]
    )
    assert tetrahedron.shape[0] == 4
    assert tetrahedron.shape[1] == 3
    if dimension == 2:
        # only return the first 3 mics, which are located in one plane.
        return tetrahedron[:3, :2]
    else:
        return tetrahedron


def ambiguity_test(mic_positions, freq):
    delta = get_min_distance(mic_positions)
    lamda = SPEED_OF_SOUND / freq
    print(
        f"signal wavelength/2: {lamda/2:.1e}, minimum distance between mics {delta:.1e}"
    )
    print(
        f"signal wavelength is {lamda/delta:.1f} times bigger than minimum mic distance. (Make sure it is more than 1)"
    )


if __name__ == "__main__":

    eps = 1e-10

    baseline = 0.5  # 1
    num_mics = 5

    for dimension in [2, 3]:
        mic_positions = get_uniform_array(
            num_mics, baseline=baseline, dimension=dimension
        )
        min_dist = get_min_distance(mic_positions)
        min_dist_theo = baseline / (num_mics - 1)
        assert (
            abs(min_dist - min_dist_theo) < eps
        ), f"{dimension}D test failed for uniform: {min_dist}!={min_dist_theo}"

        mic_positions = get_tetrahedron_array(baseline=baseline, dimension=dimension)
        min_dist = get_min_distance(mic_positions)
        assert (
            abs(min_dist - baseline) < eps
        ), f"{dimension}D test failed for tetra: {min_dist}!={baseline}"
        max_dist = get_max_distance(mic_positions)
        assert (
            abs(max_dist - baseline) < eps
        ), f"{dimension}D test failed for tetra: {max_dist}!={baseline}"
    print("all tests ok.")
