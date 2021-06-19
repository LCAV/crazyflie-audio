import numpy as np
import pandas as pd

TARGET_F = (
    3250  # In Hz so 50.0 is 0.020 seconds period and 0.25 is 4 seconds period
)
CLOCK_MCU = 84000000  # corresponds to ca. 329500 * 255
TOLERANCE = 3  # tolerance in Hz of frequencies
PERIOD = 255
PSC = 1
MAX_INT = 65536

RESTRICT_BINS = True
N_BUFFER = 1024
FS = 64000

# -----------------------------------------------------


def get_available_freqs(fmin, fmax, n_freqs):
    uniform = np.linspace(fmin, fmax, n_freqs).astype(np.int)
    if RESTRICT_BINS:
        all_frequencies = np.fft.rfftfreq(N_BUFFER, 1 / FS)
        keep_indices = np.argmin(
            np.abs(all_frequencies[None, :] - uniform[:, None]), axis=1
        )
        return all_frequencies[keep_indices]
    else:
        return uniform


def get_frequency(prescaler=PSC, period=PERIOD):
    f = CLOCK_MCU / (prescaler + 1) / (period + 1)
    return f


def perfect_divisors(target_f=TARGET_F):
    exacts = []
    for psc in range(1, MAX_INT):
        period = CLOCK_MCU / (target_f * (psc + 1)) - 1
        if CLOCK_MCU % psc == 0:
            if period <= MAX_INT:
                exacts.append(psc)
    return exacts


def possible_prescaler_value(target_f=TARGET_F):
    """ Get all possible prescalers for given frequency. 

    For each possible prescaler, calculate feasible frequency range based on ratios, and only keep prescaler if that includes the target frequency.

    """
    possible_prescalers = []
    for psc in range(MAX_INT):
        f1 = get_frequency(psc, 0)
        f2 = get_frequency(psc, MAX_INT - 1)
        if f1 >= target_f >= f2:
            possible_prescalers.append(psc)
    return possible_prescalers


def close_divisor(psc, tolerance=TOLERANCE, target_f=TARGET_F):
    period = CLOCK_MCU / (target_f * (psc + 1)) - 1
    if period >= MAX_INT:
        return

    f = get_frequency(psc, int(period))
    error = abs(f - target_f)
    if error < tolerance:
        return {
            "PSC": int(psc),
            "ARR": int(period),
            "F": int(f),
            "ERROR": round(error, 1),
        }
    else:
        return None


def get_all(target_f=TARGET_F):
    df = pd.DataFrame(columns=["PSC", "ARR", "F", "ERROR"], dtype=np.double)

    poss_prescalers = possible_prescaler_value(target_f=target_f)
    for pre in poss_prescalers:
        row = close_divisor(pre, target_f=target_f)
        if row is not None:
            df = df.append(pd.Series(row), ignore_index=True)

    df = df.sort_values(["ERROR", "PSC"])
    return df


def get_sweep_fixed_period(period=PERIOD, fmin=0, fmax=20000, n_freqs=16):
    df = pd.DataFrame(columns=["PSC", "ARR", "F", "ERROR"])

    prescalers = np.arange(2000)  # MAX_INT, step=1)
    for psc in prescalers:
        f = int(round(get_frequency(psc, period)))

        # add window because best fit might lie slightly outside the desired window
        if fmax + 1000 >= f >= fmin - 1000:
            df.loc[len(df), :] = dict(PSC=psc, ARR=period, F=f, ERROR=0)

    df.sort_values("F", axis=0, inplace=True)
    n_total = len(df)
    if n_freqs is not None:
        keep_frequencies = get_available_freqs(fmin, fmax, n_freqs)
        keep_indices = np.argmin(
            np.abs(keep_frequencies[None, :] - df.F[:, None]), axis=0
        )
        df = df.iloc[keep_indices]
        df.loc[:, "ERROR"] = df.F - keep_frequencies

    # if we have multiple frequencies, keep only the one with
    # the smallest error.
    df.sort_values(["F", "ERROR"], axis=0, inplace=True)
    df.drop_duplicates(inplace=True, subset="F", keep="first")

    df = df.apply(pd.to_numeric, axis=0)
    return df


def get_sweep_approximate_period(period=PERIOD, fmin=0, fmax=20000, n_freqs=16):
    df = pd.DataFrame(columns=["PSC", "ARR", "F", "ERROR"], dtype=np.double)
    chosen_dict = {}

    frequencies = get_available_freqs(fmin, fmax, n_freqs)

    for f in frequencies:
        print("treating frequency", f)
        df_all = get_all(target_f=f)
        if not len(df_all):
            print("Warning: no combis found for", f)
            continue

        chosen = df_all.iloc[np.argmin(np.abs(df_all.ARR.values - period))]
        df = df.append(chosen, ignore_index=True)
    return df


def print_correct_format(df):
    print(r"freq_list_t freq_list_tim[] = {")
    for i, row in df.iterrows():
        print(
            "\t"
            + r"{"
            + f"{row.F:.0f}, {row.PSC:.0f}, {row.ARR:.0f}"
            + r"},"
            + f"// error: {row.ERROR}"
        )
    print(r"};")


if __name__ == "__main__":
    # sweep
    params = dict(fmin=2500, fmax=4400, period=48, n_freqs=16)

    # mono
    # params = dict(fmin=2000, fmax=2001, period=48, n_freqs=1)

    df = get_sweep_approximate_period(**params)
    print("approximate period:")
    print_correct_format(df)

    df = get_sweep_fixed_period(**params)
    print("fixed period:")
    print_correct_format(df)

    print(f"std of diff {np.std(df.F.values[:-1] - df.F.values[1:]):.1f}")
