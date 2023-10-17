import numpy as np
import math
import matplotlib
matplotlib.use('Gtk3Agg')
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
# import matplotlib.animation as animation
import filter

class FreqAnalyser:

    class Fft:
        def __init__(self, buffer_size, sample_rate, fft_size):
            self.buffer_size: int= buffer_size
            self.sample_rate: int= sample_rate
            self.fft_size:int = fft_size
            self.precision: float = sample_rate/fft_size
            self.resolution: float = sample_rate/buffer_size
            self.num_bins: int = int(fft_size/2)

    class Analysis:          
        def __init__(self, num_bands, min_freq, max_freq, frame_rate, smoothing_factor=0.15):
            self.num_bands: int = num_bands
            self.min_freq: int = min_freq
            self.max_freq: int = max_freq
            self.freq_bands = self._calculate_freq_bands(self.num_bands, self.min_freq, self.max_freq)
            self.frame_rate = frame_rate
            self.filter_cutoff = smoothing_factor*(frame_rate/2) # minimum smoothing = 2 frames (roughly 20Hz?)
            self.band_filters = self._create_filters(self.num_bands, frame_rate, self.filter_cutoff)
        
        def _calculate_freq_bands(self, num_bands, min_freq, max_freq):
            # Calculate the logarithmic scale factor
            s = (math.log(max_freq) - math.log(min_freq)) / num_bands

            band_array = []
            for i in range(num_bands):
                start_freq = min_freq * math.exp(i * s)
                end_freq = min_freq * math.exp((i + 1) * s)
                band_array.append((start_freq, end_freq))

            return band_array
        
        def _create_filters(self, num_bands, frame_rate, cutoff_Hz):
            band_filters = []
            for i in range(num_bands):
                band_filters.append(filter.LowPassFilter(frame_rate, cutoff_Hz))

            return band_filters


    class Params:
        def __init__(self, gain_dB=0, contrast=0, compensation=0):
            # Take in decibel gain and convert to linear gain - applying then 
            # becomes a single multiply operation
            self.linear_gain = 10 ** (gain_dB / 20)
            # Linear value for contrast (-1->1) is transformed onto a 2^x exponential scale...
            # To centre the scale around a decibel mapping, we have an offset of log2(10)
            # this means the "normal" transformed exponent is 10, equal to the decibel mapping.
            # Then we have sensitivity which if set to 3 makes our lowest contrast almost
            # equal to a linear mapping. After experimentation it is likely that the 
            # sensitivity can be reduced.
            contrast_offset = math.log2(10)
            contrast_sensitivity=3
            self.contrast_exponent = pow(2, (contrast*contrast_sensitivity) +contrast_offset)
            # Compensate for the averaging out of higher frequencies due to band sizing.
            # Input value should be between 0 and 1
            self.compensation = compensation
        
    def __init__(self, 
                 buffer_size=1024, 
                 sample_rate=44100,
                 num_bands=9,
                 min_freq=20,
                 max_freq=20000,
                 plot=False):

        frame_rate = sample_rate/buffer_size
        
        self.fft = self.Fft(buffer_size, sample_rate, buffer_size)
        self.analysis = self.Analysis(num_bands, min_freq, max_freq, frame_rate)

        # Dynamic params
        self.params = self.Params(gain_dB=12, contrast=-0.9, compensation=0.6)

        # Plotting
        if plot is True:
           self._plot_init()
        else:
            self._fig = None

    def _apply_window(self, input_signal):
        window = np.kaiser(len(input_signal), 0.5)  # Create a Hamming window of the same length as the input_signal
        windowed_input_signal = input_signal * window  # Element-wise multiplication with the Hamming window
        return windowed_input_signal

    def _fft(self, input_signal):
        """
        Perform FFT analysis on the input signal.
        
        Args:
        input_signal (numpy.array): Input real signal for FFT analysis.
        
        Returns:
        numpy.array: Magnitude spectrum of the input signal.
        """
        # Ensure the input signal matches the specified buffer size
        input_signal = input_signal[:self.fft.buffer_size]

        zero_padded_buffer = np.pad(input_signal, (0, self.fft.fft_size - self.fft.buffer_size), 'constant')

        # Perform FFT and compute magnitude spectrum
        fft_result = np.fft.rfft(zero_padded_buffer)
        magnitude_spectrum = np.abs(fft_result) / self.fft.fft_size

        # This gives us buffer_size/2 + 1 elements. To keep our buffers
        # radix 2 sizing, we just discard the final element.
        return magnitude_spectrum[:-1]

    def _banding(self, bin_mags):
        num_bins = self.fft.num_bins
        results = [0.0] * self.analysis.num_bands
        bin_freq = 0
        band_index = 0
        bin_count = 0
        band_comp = self._banding_compensation_factor(band_index)  # Initial calculation

        for i in range(num_bins):
            band_start_freq, band_end_freq = self.analysis.freq_bands[band_index]

            # If the frequency is within the current band, accumulate
            if band_start_freq <= bin_freq <= band_end_freq:
                results[band_index] += bin_mags[i] * band_comp

            # If bin freq is above the current band, average the current band and set up the next one
            elif bin_freq > band_end_freq:
                results[band_index] /= bin_count
                bin_count = 0
                if (band_index + 1) < self.analysis.num_bands:
                    band_index += 1
                    band_comp = self._banding_compensation_factor(band_index)  # Recalculate for the next band
                    results[band_index] = bin_mags[i] * band_comp
                else:
                    break

            # Add bin range on each loop
            bin_freq += self.fft.precision
            bin_count += 1

        # Catch the case that the freq_ranges is greater than that of the fft
        if bin_freq < band_end_freq:
            results[band_index] /= bin_count

        return results
    
    def _banding_compensation_factor(self, band_index):
        # Calculate the bands compensation factor based on our selected 
        # compensation exponent.
        # The factor is calculated using the generic exp form: y = a * exp(b*x)
        # where x is the bands normalised value and b is the compensation gain
        max_gain = 7
        b = self.params.compensation*max_gain
        x = band_index/self.analysis.num_bands
        return math.exp(b*x)
        
    def _apply_gain(self, input):
        # Convert dB gain to linear scale and apply gain to the input
        return input * self.params.linear_gain

    def _apply_limit(self, input_value):
        # Catch invalid input (clipping)
        # if linear_magnitude > 1.0 or linear_magnitude < 0.0:
        #     raise ValueError("Invalid input: linear_magnitude should be between 0 and 1.")

        # Apply limit to the input (limit the input to a maximum of 1.0)
        return np.maximum(np.minimum(1.0, input_value),0.0)

    def _calculate_logarithmic_magnitude(self, linear_magnitude):
        # Handle the case where linear_magnitude is zero
        if linear_magnitude == 0:
            return 0

        # ...which is then used to define the exponential mapping of the linear
        # input onto an exponential scale giving us the resulting log-magnitude
        logarithmic_magnitude = pow(linear_magnitude, self.params.contrast_exponent)

        return logarithmic_magnitude

    def _apply_smoothing(self, input, filter_inst):
        return filter_inst.run(input)

    def _plot_init(self):

        # Create plot with titles and log x scale.
        self._fig, self._ax = plt.subplots()
        self._ax.set_xlabel('Frequency')
        self._ax.set_ylabel('Magnitude')
        self._ax.set_title('Ray-dance Simulator')
        self._ax.set_xscale('log')

        # Create empty data set
        self._fft_plot_x = np.arange(self.fft.num_bins) * self.fft.precision
        self._fft_plot_y = np.linspace(0,1,self.fft.num_bins)
        self._band_bar_x = [start for start, _ in self.analysis.freq_bands]
        self._band_bar_y = np.linspace(0,1,self.analysis.num_bands)
        self._band_bar_w = [end - start for start, end in self.analysis.freq_bands]

        # Plot it and set to animated
        self._fft_plot, = self._ax.plot(self._fft_plot_x, self._fft_plot_y, color='red', animated=True)
        # self._band_bar = self._ax.bar(self._band_bar_x, self._band_bar_y, width=self._band_bar_w, align='edge', color='blue', animated=True)
        self._band_rectangles = [Rectangle((x, 0), w, y) for x, y, w in zip(self._band_bar_x, self._band_bar_y, self._band_bar_w)]
        self._band_bar = PatchCollection(self._band_rectangles, color='blue', animated=True)
        self._ax.add_collection(self._band_bar)

        # make sure the window is raised, but the script keeps going
        plt.show(block=False)
        plt.pause(1)

        # Get a copy of the figure without the animated plot  
        self._background = self._fig.canvas.copy_from_bbox(self._fig.bbox)

        # Draw the plot!
        self._ax.draw_artist(self._fft_plot)
        self._ax.draw_artist(self._band_bar)

        # Show!
        self._fig.canvas.blit(self._fig.bbox)

        # This will verify the initial render
        # self._fig.canvas.flush_events()
        # plt.pause(1)

    def _update_plot(self, frame):
        # restore background
        self._fig.canvas.restore_region(self._background)

        # Update bar plot recatangles
        for rect, new_height in zip(self._band_rectangles, self.band_mags):
            rect.set_height(new_height) 

        # Update the plot
        self._band_bar.set_paths(self._band_rectangles)
        self._ax.draw_artist(self._band_bar)
        self._fft_plot.set_ydata(self.bin_mags)
        self._ax.draw_artist(self._fft_plot)
        

        # Show!
        self._fig.canvas.blit(self._ax.bbox)
        self._fig.canvas.flush_events()


    def analyse(self, input_signal):
        self.bin_mags = self._fft(input_signal)
        # self.windowed = self._apply_window(input_signal)
        # self.bin_mags = self._fft(self.windowed)
        self.band_mags = self._banding(self.bin_mags)

        self.results = self.band_mags

        for i in range(self.analysis.num_bands):
            self.results[i] = self._apply_smoothing(self.results[i], self.analysis.band_filters[i])
            self.results[i] = self._apply_gain(self.band_mags[i])
            self.results[i] = self._apply_limit(self.results[i])
            # self.results[i] = self._calculate_logarithmic_magnitude(self.results[i])

        self._update_plot(0)

        return self.results


if __name__ == "__main__":
    # Example usage
    buffer_size = 1024
    sample_rate = 44100
    analyser = FreqAnalyser(buffer_size)

    # Generate Multifrequency Input signal
    frequencies = [60, 250, 1000, 4000]  # Frequencies of the input sine waves in Hz
    amplitudes = [1, 1, 1, 1]    # Amplitudes of the sine waves (adjust as needed)
    duration = buffer_size / sample_rate
    time = np.linspace(0, duration, buffer_size, endpoint=False)
    input_signal = sum(amplitude * np.sin(2 * np.pi * frequency * time) for frequency, amplitude in zip(frequencies, amplitudes))

    # Perform FFT analysis on the input signal
    magnitude_spectrum = analyser.analyse(input_signal)

    # Create figure
    figure = plt.figure(figsize=(8, 6))

    # Plot Magnitude histogram
    freq_band_widths = [end - start for start, end in analyser.analysis.freq_bands]
    freq_band_edges = [start for start, _ in analyser.analysis.freq_bands]
    colors = ['blue', 'green']
    plt.bar(freq_band_edges, analyser.band_mags, width=freq_band_widths, align='edge', color=colors)

    # Plot FFT
    plt.plot(np.arange(len(analyser.bin_mags))*analyser.fft.precision, analyser.bin_mags, 'red')

    plt.xlabel('Frequency')
    plt.ylabel('Magnitude')
    plt.title('Ray-dance Simulator')
    plt.xscale('log')
    plt.grid(True)
    plt.show()

