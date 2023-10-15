import pyaudio
import wave
import struct
from array import array

import algo

# Specify the path to the input audio file
INPUT_WAV_FILE = 'test.wav'

# Open the input WAV file
input_wav_file = wave.open(INPUT_WAV_FILE, 'rb')
CHANNELS = input_wav_file.getnchannels()
RATE = input_wav_file.getframerate()

# Set audio format
FORMAT = pyaudio.paInt16

# Set the buffer size
CHUNK = 1024

# Initialize PyAudio for output stream
audio = pyaudio.PyAudio()

# Open an output stream
output_stream = audio.open(format=FORMAT,
                           channels=CHANNELS,
                           rate=RATE,
                           output=True,
                           frames_per_buffer=CHUNK)

# Start the test file a number of frames in
start_offset_seconds = 240
start_offset_frames = start_offset_seconds*RATE
input_wav_file.setpos(start_offset_frames)

# Read audio data from the input file
frames = input_wav_file.readframes(CHUNK)

# Initialise our frequency analyser
analyser = algo.FreqAnalyser(buffer_size=CHUNK, 
                             sample_rate=RATE, 
                             num_bands=9,
                             min_freq=20,
                             max_freq=20000,
                             plot=True)

# Main program loop
while frames:
    # Unpack data as 16-bit
    sample_data = array('h', frames)
    
    # Scale samples to floating point format between 1.0 and -1.0
    samples = [sample_datum / (2**15) for sample_datum in sample_data]

    # Extract a single channel
    single_channel = samples[::2]

    # Call the frequency analyser with our single channel of sample
    magnitude_data = analyser.analyse(single_channel)
    
    # Directly pass the chunk of original samples to the output buffer so 
    # we can listen to the audio we just analysed.
    output_stream.write(b''.join(struct.pack('h', sample) for sample in sample_data))

    # Read the next chunk of audio data from the input file
    frames = input_wav_file.readframes(CHUNK)

# Close the input file and the output stream
input_wav_file.close()
output_stream.stop_stream()
output_stream.close()
audio.terminate()
