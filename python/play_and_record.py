import time

import numpy as np

import wave
from scipy.io import wavfile

from signals import generate_signal, amplify_signal

FS = 44100 # sampling frequency in Hz
N_MICS = 1 # number of mics
DURATION = 30 # duration of recording in seconds
TARGET_DB = -30 # loudness, dB, set to None for now scaling
IN_FILE = None
FREQ = 800 #440 # Hz
SIGNAL_TYPE = "mono"
#SIGNAL_TYPE = "random"
#SIGNAL_TYPE = "random_linear"
#SIGNAL_TYPE = "mono_linear"
#SIGNAL_TYPE = "real"; IN_FILE = "../data/propellers/44000.wav"


def get_usb_soundcard_ubuntu(fs=FS, n_mics=N_MICS):
    import sounddevice as sd

    # Sound card selection: input, output
    sd.default.device = 'USB Audio', 'USB Audio'  # os-specific
    sd.default.samplerate = fs 
    sd.default.dtype = np.float32
    sd.default.channels = n_mics, 1
    return sd


if __name__ == '__main__':
    np.random.seed(1)

    signal = generate_signal(FS, duration_sec=DURATION, signal_type=SIGNAL_TYPE, frequency_hz=FREQ, fname=IN_FILE)
    out_file = f"../data/test/{SIGNAL_TYPE}"

    if TARGET_DB is not None:
        signal = amplify_signal(signal, target_dB=TARGET_DB, verbose=True)

    sd = get_usb_soundcard_ubuntu()

    print('using devices:')
    print(sd.query_devices())

    try:
        print('start recording')
        recording = sd.playrec(signal, blocking=False)
    except:
        raise

    print('sleeping...')
    time.sleep(DURATION)
    print('status (empty is ok):', sd.get_status())

    # convert 64 to 32 float
    recording_float32 = recording.astype(np.float32)

    wavfile.write(f'{out_file}_scipy.wav', FS, recording_float32)

    with wave.open(f'{out_file}.wav', 'w') as f:
        f.setnchannels(N_CHANNELS)
        f.setframerate(FS)
        f.setsampwidth(4) # number of bytes for int32
        recording_int32 = (recording_float32 * (2**31 - 1)).astype(np.int32)
        f.writeframes(recording_int32)

    np.save(f'{out_file}.npy', recording_float32)
    print(f'wrote files as {out_file}.wav, {out_file}_scipy.wav and {out_file}.npy.')
