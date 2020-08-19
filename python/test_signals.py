#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_signals.py:  
"""

from signals import *


def test_power():
    length = 1000000

    # 1-dimensional signals
    sigma = 1e-3
    signal = np.random.normal(scale=sigma, size=length)
    power = sigma**2
    assert abs(power - get_power(signal, dB=False)) < 1e-1

    a = 5
    f = 200
    times = np.arange(length) / (10 * f)
    signal = a * np.sin(2 * np.pi * f * times)
    assert signal.shape == (length, )
    power = a**2 / 2
    assert abs(power - get_power(signal, dB=False)) < 1e-1

    # multi-dimensional signals
    num_mics = 3

    sigma = 1e-3
    signal = np.random.normal(scale=sigma, size=(num_mics, length))
    power = sigma**2
    assert abs(power - get_power(signal, dB=False)) < 1e-1

    a = 5
    f = 200
    times = np.arange(length) / (10 * f)
    signal = np.r_[[a * np.sin(2 * np.pi * f * times + offset) for offset in np.random.rand(num_mics)]]
    assert signal.shape == (num_mics, length)
    power = a**2 / 2
    assert abs(power - get_power(signal, dB=False)) < 1e-1
