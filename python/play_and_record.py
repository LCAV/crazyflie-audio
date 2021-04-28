import time

import numpy as np

import wave
from scipy.io import wavfile

from signals import generate_signal, amplify_signal

FS = 44100  # sampling frequency in Hz
N_MICS = 1  # number of mics
DURATION = 10  # duration of recording in seconds
MIN_DB = -60  # loudness, dB
MAX_DB = -10
IN_FILE = None
FREQ = 4100  # 440 # Hz
# SIGNAL_TYPE = "mono"
SIGNAL_TYPE = "mono"
# SIGNAL_TYPE = "random"
# SIGNAL_TYPE = "random_linear"
# SIGNAL_TYPE = "mono_linear"
# SIGNAL_TYPE = "sweep"
# SIGNAL_TYPE = "real"; IN_FILE = "../data/propellers/44000.wav"


def get_usb_soundcard_ubuntu(fs=FS, n_mics=N_MICS):
    import sounddevice as sd

    # Sound card selection: input, output
    name = "UAC-2"
    sd.default.device = name, name  # os-specific
    sd.default.samplerate = fs
    sd.default.dtype = np.float32
    sd.default.channels = n_mics, 1
    return sd


if __name__ == "__main__":
    np.random.seed(1)

    signal = generate_signal(
        FS,
        duration_sec=DURATION,
        signal_type=SIGNAL_TYPE,
        frequency_hz=FREQ,
        min_dB=MIN_DB,
        max_dB=MAX_DB,
        fname=IN_FILE,
    )
    out_file = f"{SIGNAL_TYPE}"

    sd = get_usb_soundcard_ubuntu()

    print("using devices:")
    print(sd.query_devices())

    try:
        print("start recording")
        recording = sd.playrec(signal, blocking=True)
    except:
        raise

    # print('sleeping...')
    # time.sleep(DURATION)
    print("status (empty is ok):", sd.get_status())

    # convert 64 to 32 float
    recording_float32 = recording.astype(np.float32)

    wavfile.write(f"{out_file}_scipy.wav", FS, recording_float32)

    with wave.open(f"{out_file}.wav", "w") as f:
        f.setnchannels(N_MICS)
        f.setframerate(FS)
        f.setsampwidth(4)  # number of bytes for int32
        recording_int32 = (recording_float32 * (2 ** 31 - 1)).astype(np.int32)
        f.writeframes(recording_int32)

    np.save(f"{out_file}.npy", recording_float32)
    print(f"wrote files as {out_file}.wav, {out_file}_scipy.wav and {out_file}.npy.")
