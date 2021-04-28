import numpy as np
import pandas as pd

TARGET_F = 3250  # In Hz so 50.0 is 0.020 seconds period and 0.25 is 4 seconds period
CLOCK_MCU = 84000000  # corresponds to ca. 329500 * 255
TOLERANCE = 0.0001
PERIOD = 255
PSC = 1
MAX_INT = 65536

# -----------------------------------------------------
def abs_error(num1, num2):
    return abs((num1 - num2) / num1)


def get_frequency(prescaler=PSC, period=PERIOD, clock=CLOCK_MCU):
    f = clock / (prescaler * period)
    return f


def perfect_divisors(target_f=TARGET_F):
    exacts = []
    for psc in range(1, MAX_INT):
        period = CLOCK_MCU / (target_f * psc)
        if CLOCK_MCU % psc == 0:
            if period <= MAX_INT:
                exacts.append(psc)
    return exacts


def add_exact_period(prescaler, target_f=TARGET_F):
    period = CLOCK_MCU / (target_f * prescaler)
    if period == int(period):
        return np.array([prescaler, period, target_f, 0.0]).reshape((1, 4))
    else:
        return None


def possible_prescaler_value(target_f=TARGET_F):
    possible_prescalers = []
    exact_prescalers = perfect_divisors()
    for psc in range(1, MAX_INT):
        if psc in exact_prescalers:
            continue
        f1 = get_frequency(psc, 1)
        f2 = get_frequency(psc, MAX_INT)
        if f1 >= target_f >= f2:
            possible_prescalers.append(psc)
    return possible_prescalers


def close_divisor(psc, tolerance, target_f=TARGET_F):
    period = CLOCK_MCU / (target_f * psc)
    error = abs_error(int(period), period)
    if error < tolerance and period < MAX_INT:
        h = get_frequency(psc, int(period))
        return np.array([psc, int(period), h, error]).reshape((1, 4))
    else:
        return None


def print_errors():
    # Make a dataframe to hold results as we compute them
    df = pd.DataFrame(columns=["PSC", "ARR", "F", "ERROR"], dtype=np.double)

    # Get exact prescalers first.
    exact_prescalers = perfect_divisors()
    for pre in exact_prescalers:
        row = add_exact_period(pre)
        if row is not None:
            df = df.append(pd.DataFrame(row, columns=df.columns))

    # Get possible prescalers.
    poss_prescalers = possible_prescaler_value()
    close_prescalers = []
    for pre in poss_prescalers:
        row = close_divisor(pre, TOLERANCE)
        if row is not None:
            df = df.append(pd.DataFrame(row, columns=df.columns))

    #  Adjust PSC and ARR values by -1 to reflect the way you'd code them.
    df["PSC"] = df["PSC"] - 1
    df["ARR"] = df["ARR"] - 1

    #  Sort first by errors (zeroes and lowest errors at top of list, and
    #  then by prescaler value (ascending).
    df = df.sort_values(["ERROR", "PSC"])

    # Make and populate column indicating if combination is exact.
    df["EXACT"] = pd.Series("?", index=df.index)
    df["EXACT"] = np.where(df["ERROR"] == 0.0, "YES", "NO")

    #  Format for output.
    df["PSC"] = df["PSC"].map("{:.0f}".format)
    df["ARR"] = df["ARR"].map("{:.0f}".format)
    df["F"] = df["F"].map("{:.6f}".format)
    df["ERROR"] = df["ERROR"].map("{:.10f}".format)

    output = df.to_string()
    print(output)
    print(
        "\nthese are the ",
        df.shape[0],
        " total combination meeting your tolerance requirement",
    )


def print_frequencies(period=PERIOD, fmin=0, fmax=np.inf, n_freqs=None):
    df = pd.DataFrame(columns=["PSC", "ARR", "F", "ERROR"])

    prescalers = np.arange(1, MAX_INT, step=1)
    for psc in prescalers:
        f = get_frequency(psc, period)
        if fmax >= f >= fmin:
            df.loc[len(df), :] = dict(PSC=psc, ARR=period, F=int(round(f)), ERROR=0)
    df.sort_values("F", axis=0, inplace=True)
    n_total = len(df)
    if n_freqs is not None:
        # keep_indices = np.linspace(0, n_total-1, n_freqs).astype(int)
        keep_frequencies = np.linspace(fmin, fmax, n_freqs)
        keep_indices = np.argmin(
            np.abs(keep_frequencies[None, :] - df.F[:, None]), axis=0
        )
        df = df.iloc[keep_indices]
        df.loc[:, "ERROR"] = keep_frequencies - df.F
    df = df.astype(np.int)
    return df


def print_correct_format(df):
    print(r"freq_list_t freq_list_tim[] = {")
    for i, row in df.iterrows():
        print("\t" + r"{" + f"{row.F}, {row.PSC}, {row.ARR}, {row.ERROR}" + r"},")
    print(r"};")


if __name__ == "__main__":
    # print_errors()
    df = print_frequencies(period=256, fmin=3000, fmax=4875, n_freqs=16)
    print(df)
    print_correct_format(df)
