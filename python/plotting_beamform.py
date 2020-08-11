import numpy as np
import matplotlib.pylab as plt


def get_max_bins(signals, min_idx=0):
    """ Get max signal bin, ignoring the first min_idx samples. """
    signals_f = np.fft.rfft(signals, axis=1)
    mag = np.abs(signals_f)
    max_bins = np.argmax(mag[:, min_idx:], axis=1) + min_idx
    if len(np.unique(max_bins)) == 1:
        return max_bins[:1]
    print("Warning: max peak differs for different signals")
    return max_bins


def plot_autocorrelation(Rx, bins=[0, 100]):
    vmin = np.min(np.abs(Rx))
    vmax = np.max(np.abs(Rx))
    a_vmin = np.min(np.angle(Rx))
    a_vmax = np.max(np.angle(Rx))

    fig, axs = plt.subplots(2, len(bins))
    for j, bin_ in enumerate(bins):
        im = axs[0, j].matshow(np.abs(Rx[bin_]), vmin=vmin, vmax=vmax)
        im2 = axs[1, j].matshow(np.angle(Rx[bin_]), vmin=a_vmin, vmax=a_vmax)
        print(f"magnitudes the same: bin{bin_} {np.allclose(*np.abs(Rx[bin_]))}")
        print(f"    angles the same: bin{bin_} {np.allclose(*np.angle(Rx[bin_]))}")

    fig.colorbar(im, ax=axs[0, :].flat)
    fig.colorbar(im2, ax=axs[1, :].flat)
    axs[0, 0].set_ylabel("magnitude")
    axs[1, 0].set_ylabel("phase")


def plot_signals(signals, Fs, plot_frequencies=[]):
    freqs = np.fft.rfftfreq(n=signals.shape[1], d=1 / Fs)
    fig, ax = plt.subplots()
    fig.set_size_inches(10, 5)
    for i, signal in enumerate(signals):
        signal_f = np.fft.rfft(signal)
        mag = np.abs(signal_f)
        ax.loglog(freqs, mag)

    for i, freq in enumerate(plot_frequencies):
        ax.axvline(x=freq, color=f"C{i}", linestyle=":", label=f"{freq:.0f}Hz")
    ax.legend()
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("amplitude")
    return fig, ax


def plot_setup(mic_positions, source_positions, propeller_positions=[[]]):
    mic_ref = mic_positions[0]
    fig, ax = plt.subplots()
    for i, mic in enumerate(mic_positions):
        ax.scatter(*mic, color=f"C{i}", label=f"mic{i}", s=50)

    for i, prop in enumerate(propeller_positions):
        if not len(prop):
            break
        ax.scatter(*prop, color=f"C{i}", label=f"prop{i}", s=50, marker=".")

    for i, source in enumerate(source_positions):
        ax.scatter(*source, color=f"C{i}", label=f"source{i}", s=50, marker="*")

    ax.scatter(*mic_ref.T, label="ref mic", color="k", marker="x", s=100.0)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    plt.legend()
    plt.axis("equal")


def plot_spectrum_linear(
    mvdr_scan, thetas_scan, frequencies_hz, gt_azimuth=None, ax=None, combination=None
):
    if ax is None:
        fig, ax = plt.subplots()

    colors = get_gradual_colors(mvdr_scan.shape[0])

    max_legend_entries = 10
    for i in range(mvdr_scan.shape[0]):
        label = None
        if (i <= max_legend_entries // 2) or (
            i >= mvdr_scan.shape[0] - max_legend_entries // 2
        ):
            label = f"{frequencies_hz[i]:.0f}Hz"
        ax.semilogy(thetas_scan, mvdr_scan[i, :], color=colors[i], label=label)
    if gt_azimuth is not None:
        ax.axvline(x=gt_azimuth, color="k", linestyle=":", label="source")

    if combination is not None:
        ax.semilogy(
            thetas_scan,
            combination(mvdr_scan, axis=0),
            color="k",
            linestyle="-.",
            label=combination.__name__,
        )

    ax.legend(loc="lower left")

    ax.set_xlabel("theta [deg]")
    ax.set_ylabel("response")
    ax.grid()

    new_ticks = np.linspace(min(thetas_scan), max(thetas_scan), len(ax.get_xticks()))
    ax.set_xticks(new_ticks)
    ax.set_xticklabels([f"{tick*180/np.pi:.0f}" for tick in new_ticks])


def get_gradual_colors(num_colors, cmap_name="inferno"):
    cmap = plt.get_cmap(cmap_name)
    return [cmap(i) for i in np.linspace(0.1, 0.9, num_colors)]


def plot_spectrum_polar(
    mvdr_scan, thetas_scan, frequencies_hz, gt_azimuth=None, ax=None, constraints=None
):

    colors = get_gradual_colors(mvdr_scan.shape[0])

    if ax is None:
        fig = plt.figure()
        fig.set_size_inches(10, 10)
        ax = fig.add_subplot(121, polar=True)

    for i in range(mvdr_scan.shape[0]):
        color = colors[i]
        label = f"f = {frequencies_hz[i]} Hz"
        ax.plot(thetas_scan, np.log(mvdr_scan[i]), label=label, color=color)

    if constraints is not None:
        constraint_colors = {True: "green", False: "red"}
        for c, val in constraints:
            ax.axvline(x=c, color=constraint_colors[val > 0], ls=":")

    ax.set_xticklabels([])


def normalize(a):
    """
    Noramlize signals in array per row.
    """
    if a.ndim > 1:
        normalized = (a - np.min(a, axis=1)[:, None]) / (
            np.max(a, axis=1)[:, None] - np.min(a, axis=1)[:, None]
        )
        return normalized
    else:
        if abs(np.max(a) - np.min(a)) < 1e-10:
            print("Warning: not normalizing, because almost constant.")
            return a
        normalized = (a - np.min(a)) / (np.max(a) - np.min(a))
        return normalized


def plot_sine_signals(signals_source, normalize_signal=True, length=100):
    if normalize_signal:
        signals_plot = normalize(signals_source)
    else:
        signals_plot = signals_source
    start_idx = np.argmax(signals_plot, axis=1)[0]
    indices = np.arange(start_idx, start_idx + length)
    plt.figure()
    for j in range(signals_plot.shape[0]):
        plt.plot(indices, signals_plot[j, indices], label=f"mic{j}")
    plt.legend()
