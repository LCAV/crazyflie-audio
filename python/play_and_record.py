import time

import numpy as np

import wave
from scipy.io import wavfile

from signals import generate_signal, amplify_signal

FS = 44100
N_CHANNELS = 1
DURATION = 30 # seconds
FREQ = 440


def get_usb_soundcard_ubuntu(fs=FS, n_channels=N_CHANNELS):
    import sounddevice as sd

    # Sound card selection: input, output
    sd.default.device = 'USB Audio', 'USB Audio'  # os-specific
    sd.default.samplerate = fs 
    sd.default.dtype = np.float32
    sd.default.channels = n_channels
    return sd


if __name__ == '__main__':
    np.random.seed(1)

    in_file = None
    #signal_type = "mono"
    #signal_type = "random"
    #signal_type = "random_linear"
    signal_type = "mono_linear"
    #signal_type = "real"; in_file = "../data/propellers/44000.wav"
    signal = generate_signal(FS, duration_sec=DURATION, signal_type=signal_type, frequency_hz=FREQ, fname=in_file)
    out_file = f"../data/test/{signal_type}"

    #signal = amplify_signal(signal, target_dB=-10, verbose=True)

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
