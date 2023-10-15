
import matplotlib
matplotlib.use('Gtk3Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
import serial

class RaydancePlotter:
    def __init__(self):
        self._fig, self._ax = plt.subplots()
        self._ax.set_xlabel('Frequency')
        self._ax.set_ylabel('Magnitude')
        self._ax.set_title('Ray-dance Simulator')
        self._background = None
        self._band_rectangles = []
        self._fft_plot = None
        self._band_bar = None

    def init_both(self, num_bins, fft_precision, freq_bands):
        self.fft_precision = fft_precision
        self._ax.set_xscale('log')
        self._plot_init(num_bins, freq_bands)

    def init_bars(self, num_bands):
        freq_bands = [(i, i + 1) for i in np.linspace(0, num_bands - 1, num_bands)]
        self.fft_precision = 1
        self._plot_init(num_bands, freq_bands)


    def _plot_init(self, num_bins, freq_bands):
        """
        Create the empty data sets for the plot and configures them to be animated data sets.
        """
        
        # Create empty data set
        self._fft_plot_x = np.arange(num_bins) * self.fft_precision
        self._fft_plot_y = np.linspace(0, 1, num_bins)
        self._band_bar_x = [start for start, _ in freq_bands]
        self._band_bar_y = np.linspace(0, 1, len(freq_bands))
        self._band_bar_w = [end - start for start, end in freq_bands]

        # Plot it and set to animated
        self._fft_plot, = self._ax.plot(self._fft_plot_x, self._fft_plot_y, color='red', animated=True)
        self._band_rectangles = [Rectangle((x, 0), w, y) for x, y, w in zip(self._band_bar_x, self._band_bar_y, self._band_bar_w)]
        self._band_bar = PatchCollection(self._band_rectangles, color='blue', animated=True)
        self._ax.add_collection(self._band_bar)

        # Make sure the window is raised, but the script keeps going
        plt.show(block=False)
        plt.pause(1)

        # Get a copy of the figure without the animated plot  
        self._background = self._fig.canvas.copy_from_bbox(self._fig.bbox)

        # Draw the plot!
        self._ax.draw_artist(self._band_bar)
        self._ax.draw_artist(self._fft_plot)

        # Show!
        self._fig.canvas.blit(self._fig.bbox)

        # Pause to view empty graph
        self._fig.canvas.flush_events()
        plt.pause(2)
        
    def update_plot(self, bin_mags, band_mags):
        """
        Updates the plot with new bin magnitudes and band magnitudes.

        Args:
            bin_mags (numpy.array or None): Magnitudes of FFT bins.
            band_mags (numpy.array or None): Magnitudes of frequency bands.
        """
        # Replace None values with existing data sets
        bin_mags = self._fft_plot_y if bin_mags is None else bin_mags
        band_mags = self._band_bar_y if band_mags is None else band_mags
        
        # Restore the background
        self._fig.canvas.restore_region(self._background)

        # Update bar plot rectangles with new band magnitudes
        for rect, new_height in zip(self._band_rectangles, band_mags):
            rect.set_height(new_height) 
        # Update FFT Plot Data
        self._fft_plot.set_ydata(bin_mags)

        # Draw the New plots
        self._band_bar.set_paths(self._band_rectangles)
        self._ax.draw_artist(self._band_bar)
        self._ax.draw_artist(self._fft_plot)

        # Redraw the figure canvas and flush events
        self._fig.canvas.blit(self._ax.bbox)
        self._fig.canvas.flush_events()

    def read_band_mags(self, serial_port):
        """
        Reads band magnitudes from the specified serial port and updates the plot.
        """
        delimiter = "[bands]"
        buffer = ""

        with serial.Serial(serial_port, baudrate=9600) as ser:
            while True:
                buffer += ser.read().decode()
                while delimiter in buffer:
                    start_index = buffer.find(delimiter)
                    data_start = start_index + len(delimiter)
                    end_index = buffer.find(delimiter, data_start)
                    if end_index != -1:
                        data = buffer[data_start:end_index]  # Directly extract data between delimiters
                        band_mags = np.array([int(value) / 255 for value in data.split()], dtype=float)
                        self.update_plot(None, band_mags)  # Example: Use None for bin mags
                        buffer = buffer[end_index + len(delimiter):]  # Remove processed data from the buffer
                    else:
                        break  # Incomplete data, wait for more input


if __name__ == "__main__":
    num_bins = 10  # Provide the number of FFT bins
    fft_precision = 1  # Provide the FFT precision
    freq_bands = [(0, 1), (2, 3), (4, 5)]  # Provide the frequency bands as a list of tuples (start, end)
    
    # Specify the serial port (e.g., '/dev/ttyUSB0' on Linux or 'COM3' on Windows)
    serial_port = '/dev/ttyUSB1'  # Modify this according to your setup
    
    simulator = RayDanceSimulator(num_bins, fft_precision, freq_bands)
    simulator.read_band_mags(serial_port)
