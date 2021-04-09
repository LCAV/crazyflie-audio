import numpy as np
import pandas as pd

TARGET_F = 3000  # In Hz so 50.0 is 0.020 seconds period and 0.25 is 4 seconds period
CLOCK_MCU = 84000000
TOLERANCE = 0.0005

# -----------------------------------------------------


def abs_error(num1, num2):
    return abs((num1 - num2) / num1)


def hertz(clock, prescaler, period):
    f = clock / (prescaler * period)
    return f


def perfect_divisors():
    exacts = []
    for psc in range(1, 65536):
        arr = CLOCK_MCU / (TARGET_F * psc)
        if CLOCK_MCU % psc == 0:
            if arr <= 65536:
                exacts.append(psc)
    return exacts


def add_exact_period(prescaler):
    entries = []
    arr = CLOCK_MCU / (TARGET_F * prescaler)
    if arr == int(arr):
        entry = [prescaler, arr, TARGET_F, 0.0]
        entries.append(entry)
    return entries


def possible_prescaler_value():
    possibles = []
    for psc in range(1, 65536):
        if psc in exact_prescalers:
            continue
        h1 = hertz(CLOCK_MCU, psc, 1)
        h2 = hertz(CLOCK_MCU, psc, 65536)
        if h1 >= TARGET_F >= h2:
            possibles.append(psc)
    return possibles


def close_divisor(psc, tolerance):
    arr = CLOCK_MCU / (TARGET_F * psc)
    error = abs_error(int(arr), arr)
    if error < tolerance and arr < 65536.0:
        h = hertz(CLOCK_MCU, psc, int(arr))
        return psc, int(arr), h, error
    else:
        return None


#  ------------------------------------------------------------------------

# Make a dataframe to hold results as we compute them
df = pd.DataFrame(columns=['PSC', 'ARR', 'F', 'ERROR'], dtype=np.double)

# Get exact prescalars first.
exact_prescalers = perfect_divisors()
exact_values = []
for index in range(len(exact_prescalers)):
    rows = add_exact_period(exact_prescalers[index])
    for rowindex in range(len(rows)):
        df = df.append(pd.DataFrame(np.array(rows[rowindex]).reshape(1, 4), columns=df.columns))

# Get possible prescalers.
poss_prescalers = possible_prescaler_value()
close_prescalers = []
for index in range(len(poss_prescalers)):
    value = close_divisor(poss_prescalers[index], TOLERANCE)
    if value is not None:
        close_prescalers.append((value[0], value[1], value[2], value[3]))
df = df.append(pd.DataFrame(np.array(close_prescalers).reshape(len(close_prescalers), 4), columns=df.columns))

#  Adjust PSC and ARR values by -1 to reflect the way you'd code them.
df['PSC'] = df['PSC'] - 1
df['ARR'] = df['ARR'] - 1

#  Sort first by errors (zeroes and lowest errors at top of list, and
#  then by prescaler value (ascending).
df = df.sort_values(['ERROR', 'PSC'])

# Make and populate column indicating if combination is exact.
df['EXACT'] = pd.Series("?", index=df.index)
df['EXACT'] = np.where(df['ERROR'] == 0.0, "YES", "NO")

#  Format for output.
df['PSC'] = df['PSC'].map('{:.0f}'.format)
df['ARR'] = df['ARR'].map('{:.0f}'.format)
df['F'] = df['F'].map('{:.6f}'.format)
df['ERROR'] = df['ERROR'].map('{:.10f}'.format)

output = df.to_string()
print(output)
print()
print('these are the ', df.shape[0], ' total combination meeting your tolerance requirement')
exit(0)