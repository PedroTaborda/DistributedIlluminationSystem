from matplotlib import pyplot as plt
from matplotlib import animation as animation
import serial

SERIAL_PORTS = ['/dev/cu.usbmodem101', '/dev/cu.usbmodem1101']
BAUD_RATE = 115200

variable_data = dict()

# Create figure for plotting
fig = plt.figure()
axes = dict()
n_rows = 1
n_cols = 1
is_plotting = True


def setupLumminancePlot(ax, manage_limits=True):
    ax.set_title('Measured Luminance')
    if manage_limits:
        ax.set_ylim(bottom=0)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Luminance [Lux]')

def setupDutyPlot(ax, manage_limits=True):
    ax.set_title('Duty Cycle')
    if manage_limits:
        ax.set_ylim(bottom=0, top=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Duty Cycle')
    
def setupJitterPlot(ax, manage_limits=True):
    ax.set_title('Jitter')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Jitter[us]')

def setupIntegralErrorPlot(ax, manage_limits=True):
    ax.set_title('Integral Error')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Integral Error [Lux.s]')

def setupTrackingErrorPlot(ax, manage_limits=True):
    ax.set_title('Tracking Error')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Tracking Error [Lux]')

def setupSimulatorPlot(ax, manage_limits=True):
    ax.set_title('Simulator Luminance Prediction')
    if manage_limits:
        ax.set_ylim(bottom=0)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Luminance [Lux]')

def setupReferencePlot(ax, manage_limits=True):
    ax.set_title('Reference')
    if manage_limits:
        ax.set_ylim(bottom=0)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Luminance [Lux]')


plot_setups = {'l': setupLumminancePlot,
               'd': setupDutyPlot,
               'j': setupJitterPlot,
               'i': setupIntegralErrorPlot,
               'e': setupTrackingErrorPlot,
               's': setupSimulatorPlot,
               'r': setupReferencePlot}

def graph_update_function(i, serial_port: serial.Serial, variable_data):
    global n_rows, n_cols
    while serial_port.in_waiting > 0:
        line = serial_port.readline().strip().decode()
        try:
            stream_command, variable, lumminaire_index, value, timestamp = line.split(' ')
            value = float(value)
            # convert to seconds
            timestamp = float(timestamp) / 1e3

            if not variable in variable_data:
                variable_data[variable] = [[], []]

            variable_data[variable][0] += [timestamp]
            variable_data[variable][1] += [value]

        except Exception as e:
            print(line)
            continue

    for key in variable_data:
        data = variable_data[key]
        
        if not key in axes:
            grid_change = False
            if not n_rows == (len(axes) // 3) + 1:
                n_rows = (len(axes) // 3) + 1
                grid_change = True
            if not n_cols == min(len(axes) + 1, 3):
                n_cols = min(len(axes) + 1, 3)
                grid_change = True

            if grid_change:
                for index, ax_key in enumerate(axes):
                    fig.delaxes(axes[ax_key])
                    axes[ax_key] = fig.add_subplot(n_rows, n_cols, index + 1)
                    axes[ax_key].clear()
                    if(len(variable_data[ax_key][0]) > 2000):
                        axes[ax_key].plot(variable_data[ax_key][0][-2000:], variable_data[ax_key][1][-2000:])
                    else:
                        axes[ax_key].plot(variable_data[ax_key][0], variable_data[ax_key][1])

            axes[key] = fig.add_subplot(n_rows, n_cols, len(axes) + 1)
            fig.tight_layout()
        axes[key].clear()
        
        if(len(data[0]) > 2000):
            axes[key].plot(data[0][-2000:], data[1][-2000:])
        else:
            axes[key].plot(data[0], data[1])
        plot_setups[key](axes[key])


if __name__=="__main__":
    serial_port = serial.Serial(baudrate=BAUD_RATE, timeout=None)
    first_timestamp = 0
    is_first_timestamp = True

    port = 0
    while not serial_port.is_open and port < len(SERIAL_PORTS):
        try:
            serial_port.port = SERIAL_PORTS[port]
            serial_port.open()
        except:
            port += 1

    if port == len(SERIAL_PORTS):
        print("No serial ports were available. Terminating.")
        exit()

    # Startup the real-time panel which plots the streamed variables
    ani = animation.FuncAnimation(fig, graph_update_function, fargs=(serial_port, variable_data), interval=100)
    plt.show()
    plt.close()

    # After the user has finished the real-time analysis, show all data gathered in different figures
    for index, key in enumerate(variable_data):
        fig = plt.figure(index + 1)
        axes = fig.add_subplot(1, 1, 1)
        axes.plot(variable_data[key][0], variable_data[key][1])
        plot_setups[key](axes)

    if 'l' in variable_data and 's' in variable_data:
        fig = plt.figure(len(variable_data) + 1)
        axes = fig.add_subplot(1, 1, 1)
        axes.plot(variable_data['l'][0], variable_data['l'][1])
        axes.plot(variable_data['s'][0], variable_data['s'][1])
        axes.set_xlabel('Time [s]')
        axes.set_ylim(bottom=0)
        axes.set_ylabel('Luminance [Lux]')
        axes.set_title('Simulator vs Measure')
        axes.legend(('Measurement', 'Simulation'))

    if 'l' in variable_data and 'r' in variable_data:
        fig = plt.figure(len(variable_data) + 1)
        axes = fig.add_subplot(1, 1, 1)
        axes.plot(variable_data['l'][0], variable_data['l'][1])
        axes.plot(variable_data['r'][0], variable_data['r'][1])
        axes.set_xlabel('Time [s]')
        axes.set_ylim(bottom=0)
        axes.set_ylabel('Luminance [Lux]')
        axes.set_title('Reference vs Measure')
        axes.legend(('Measurement', 'Reference'))

    if 'l' in variable_data and 's' in variable_data and 'r' in variable_data:
        fig = plt.figure(len(variable_data) + 2)
        axes = fig.add_subplot(1, 1, 1)
        axes.plot(variable_data['l'][0], variable_data['l'][1])
        axes.plot(variable_data['s'][0], variable_data['s'][1])
        axes.plot(variable_data['r'][0], variable_data['r'][1])
        axes.set_xlabel('Time [s]')
        axes.set_ylim(bottom=0)
        axes.set_ylabel('Luminance [Lux]')
        axes.set_title('Reference vs Simulator vs Measure')
        axes.legend(('Measurement', 'Simulation', 'Reference'))

    plt.show()
