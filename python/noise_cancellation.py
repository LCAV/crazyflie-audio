import matplotlib.pylab as plt
import numpy as np
from scipy import signal


def iircomb(f, Fs, ftype="notch", Q=30):
    """  Calculate comb filter.
    
    Inspired from unreleased scipy feature:
    https://github.com/scipy/scipy/pull/12476/files
    """
    M = int(Fs // f)
    w0 = 2 * np.pi * f / Fs
    w_delta = w0 / Q
    print(f"filtering out at {Fs / M:.0f} instead of {f:.0f}")

    # notch
    if ftype == "notch":
        G0, G = [1, 0]
    elif ftype == "peak":
        G0, G = [0, 1]
    else:
        raise ValueError(ftype)
    GB = 1 / np.sqrt(2)

    # Compute beta
    # Eq. 11.5.3 (p. 591) from reference [1]
    beta = np.sqrt((GB ** 2 - G0 ** 2) / (G ** 2 - GB ** 2)) * np.tan(M * w_delta / 4)

    # Compute filter coefficients
    # Eq 11.5.1 (p. 590) variables a, b, c from reference [1]
    ax = (1 - beta) / (1 + beta)
    bx = (G0 + G * beta) / (1 + beta)
    cx = (G0 - G * beta) / (1 + beta)

    # Compute numerator coefficients
    # Eq 11.5.1 (p. 590) or Eq 11.5.4 (p. 591) from reference [1]
    # b - cz^-N or b + cz^-N
    b = np.zeros(M + 1)
    b[0] = bx
    b[-1] = cx
    if ftype == "notch":
        b[-1] = -cx

    # Compute denominator coefficients
    # Eq 11.5.1 (p. 590) or Eq 11.5.4 (p. 591) from reference [1]
    # 1 - az^-N or 1 + az^-N
    a = np.zeros(M + 1)
    a[0] = 1
    a[-1] = ax
    if ftype == "notch":
        a[-1] = -ax
    return b, a


def filter_iir_comb(buffer, f, Fs, ftype="notch", Q=30, plot=False):
    b, a = iircomb(f=f, Fs=Fs, ftype=ftype, Q=Q)
    sos = signal.tf2sos(b, a)
    buffer_filtered_sos = signal.sosfilt(sos, buffer)

    if plot:
        sos = signal.tf2sos(b, a)
        w, h = signal.sosfreqz(sos, Fs, fs=Fs)
        num_freqs = 10
        plt.plot(w[w < f * num_freqs], np.abs(h[w < f * num_freqs]), label=ftype)
        [plt.axvline(x=f * n, color="k", ls=":") for n in np.arange(num_freqs)]

    return buffer_filtered_sos


def filter_iir_bandpass(buffer, fmin, fmax, Fs, order=1, method="cheby2", plot=False):
    if method == "cheby2":
        sos = signal.iirfilter(
            order,
            [fmin, fmax],
            rs=20,
            btype="band",
            analog=False,
            ftype="cheby2",
            output="sos",
            fs=Fs,
        )
    elif method == "single_peak":
        Q = (fmax - fmin) * 10
        f = (fmax + fmin) / 2
        b, a = signal.iirpeak(f, Q=Q, fs=Fs)
        sos = signal.tf2sos(b, a)  # 1 x 6
    else:
        raise ValueError(method)

    buffer_filtered_sos = signal.sosfilt(sos, buffer)

    #### DEBGUGING
    buffer_filtered = buffer
    for filter_coeffs in sos:
        a = filter_coeffs[3:]
        b = filter_coeffs[:3]
        buffer_filtered = signal.lfilter(b, a, buffer_filtered)
    np.testing.assert_allclose(buffer_filtered_sos, buffer_filtered, atol=1e-5)
    #### DEBGUGING

    if plot:
        w, h = signal.sosfreqz(sos, Fs, fs=Fs)
        fig, axs = plt.subplots(2, 2)

        # magnitude plots
        axs[0, 0].semilogx(w, 20 * np.log10(np.abs(h) + 1e-20))
        axs[0, 1].semilogx(w, 20 * np.log10(np.abs(h) + 1e-20))
        axs[0, 0].set_ylabel("amplitude [dB]")
        axs[0, 0].set_ylim([-100, 10])
        axs[0, 1].set_ylim([-100, 10])
        axs[0, 1].set_xlim([2 * fmin - fmax, 2 * fmax - fmin])

        # phase plots
        axs[1, 0].semilogx(w, np.unwrap(np.angle(h)), label="unwarped")
        axs[1, 0].semilogx(w, np.angle(h), label="raw", ls=":")
        axs[1, 1].semilogx(w, np.unwrap(np.angle(h)), label="unwarped")
        axs[1, 1].semilogx(w, np.angle(h), label="raw", ls=":")
        axs[1, 1].set_xlim([2 * fmin - fmax, 2 * fmax - fmin])
        axs[1, 0].set_ylabel("phase [rad]")

        # decoration
        fig.suptitle(method)
        [ax.grid(which="both", axis="both") for ax in axs.flatten()]
        [ax.axvline(x=fmin, color="k", ls=":") for ax in axs.flatten()]
        [ax.axvline(x=fmax, color="k", ls=":") for ax in axs.flatten()]
        axs[0, 0].get_xaxis().set_visible(False)
        axs[0, 1].get_xaxis().set_visible(False)
        axs[0, 1].get_yaxis().set_visible(False)
        axs[1, 1].get_yaxis().set_visible(False)
        axs[1, 1].get_xaxis().set_visible(False)
        axs[1, 0].set_xlabel("frequency [Hz]")
        plt.legend()

    return buffer_filtered_sos
