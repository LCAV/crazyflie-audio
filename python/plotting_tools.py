#!/usr/bin/env python
# coding: utf-8
"""
plotting_tools.py: tools used project-wide for plotting.
"""

import matplotlib.pylab as plt
import numpy as np


def plot_symmetric_angles(array, thetas=None, negative=True, log=True, ax=None, **kwargs):
    if thetas is None:
        thetas_deg = np.linspace(0, 360, len(array))
    elif all(thetas <= 2 * np.pi):
        thetas_deg = thetas * 180 / np.pi
    else:
        thetas_deg = thetas

    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = plt.gcf()

    if log:
        ax.semilogy(thetas_deg, array, **kwargs)
    else:
        ax.plot(thetas_deg, array, **kwargs)
    ax.set_xticks(np.linspace(0, max(thetas_deg), 9))

    yticks_max = max([abs(min(array)), abs(max(array))])
    if negative:
        yticks = np.linspace(-yticks_max, yticks_max, 5)
    else:
        yticks = np.linspace(min(array), yticks_max, 5)

    if log:
        ax.set_yticks(yticks)
        #ax.set_yticklabels([f"$10^{{{y:.1f}}}$" for y in yticks])
        ax.set_yticklabels([f"{y:.1f}" for y in yticks])
    else:
        ax.set_yticks(yticks)
        ax.set_yticklabels([f"10{y:.1e}" for y in yticks])
    ax.grid()
    return fig, ax


def plot_autocorrelation(Rx):
    fig, ax = plt.subplots()
    ax.matshow(Rx)


def plot_polar_response(responses, thetas=None, log=True):
    if thetas is None:
        thetas = np.linspace(0, 2 * np.pi, len(responses))

    if log:
        responses_plot = responses.copy()
        responses_plot[responses_plot > 0] = np.log10(responses_plot[responses_plot > 0])
    else:
        responses_plot = responses

    plt.figure()
    ax = plt.subplot(111, projection='polar')
    ax.scatter(thetas, responses_plot)
    ax.set_title('log-polar plot', y=1.1)


def add_colorbar(ax, im):
    from mpl_toolkits.axes_grid1.axes_divider import make_axes_locatable
    ax_divider = make_axes_locatable(ax)

    cax = ax_divider.append_axes("right", size="8%", pad="2%")
    plt.colorbar(im, cax=cax)
