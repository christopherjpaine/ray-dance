import serial
import plot

# Initialise Serial
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

# Initialise plot to just display bars
plotter = plot.RaydancePlotter()
plotter.init_bars(9)


while True:
    line = ser.readline().decode().strip()
    if line:
        # try - means any corrupt data just kinda gets ignored
        try:
            if line.startswith("[algo]"):  # Check if the line starts with the marker
                data = line.replace("[algo]", "") # Remove the marker from the line
                values = list(map(float, data.split()))  # Split values by space and convert them to floats
                # print("Received data:", values)  # Process the received data here
                plotter.update_plot(None, values)
            else:
                print("rx: " + str(line))
        except ValueError:
            pass
    else:
        print("Timeout")