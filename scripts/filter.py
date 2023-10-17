import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, freqz, lfilter

class LowPassFilter:
    def __init__(self, sampling_rate, cutoff_frequency):
        self.sampling_rate = sampling_rate
        self.cutoff_frequency = cutoff_frequency
        self.b, self.a = self.create_filter()
        self.zi = np.zeros(max(len(self.a), len(self.b)) - 1)

    def create_filter(self):
        normal_cutoff = self.cutoff_frequency / self.sampling_rate
        b, a = butter(N=2, Wn=normal_cutoff, btype='low', analog=False, output='ba')
        return b, a

    def run(self, sample):
        filtered_sample, self.zi = lfilter(self.b, self.a, [sample], zi=self.zi)
        return filtered_sample[0]
    
    def plot_frequency_response(self, b, a):
        w, h = freqz(b, a)
        nyquist_frequency = 0.5 * self.sampling_rate
        plt.figure()
        plt.plot(nyquist_frequency * w / np.pi, np.abs(h), 'b')
        plt.title("Lowpass Filter Frequency Response")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("Gain")
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # Example usage
    sampling_rate = 1000  # Sample rate in Hz
    cutoff_frequency = 1  # Cutoff frequency in Hz
    rolloff = 0.5

    # Create an array of filter objects
    filter = LowPassFilter(sampling_rate, cutoff_frequency)

    # Plot frequency response for a specific filter
    filter_index_to_plot = 0
    b, a = filter.create_filter()
    a = [1, 1.4312, -0.5604]
    b= [ 0.0323, 0.0646, 0.032303]
    
    filter.plot_frequency_response(b, a)
    
