import threading
import time
import serial
import queue
import PySimpleGUI as sg
import matplotlib.figure as figure
import matplotlib.pyplot as plt


class Artist:
    MAX_SAMPLES = 5000
    def __init__(self, ports) -> None:
        self.ports = ports

        self.rows = 2
        self.cols = 2
        plt.ion()
        self.subplots = [plt.subplots(self.rows, self.cols, num=i) for i in range(len(ports))]

        self.figures:list[figure.Figure] = [subplot[0] for subplot in self.subplots]
        self.axes: list[tuple[tuple[plt.Axes]]] = [subplot[1] for subplot in self.subplots] # self.rows by self.cols

        self.latest_time = 0


        # illuminance
        self.l = [[]] * len(ports)
        self.l_t = [[]] * len(ports)
        self.l_lines : list[plt.Line2D] = [ax[0][0].plot([], label='illuminance')[0] for ax in self.axes]

        # reference
        self.r = [[]] * len(ports)
        self.r_t = [[]] * len(ports)
        self.r_lines : list[plt.Line2D] = [ax[0][0].plot([], label='reference')[0] for ax in self.axes]

        # duty cycle
        self.DC = [[]] * len(ports)
        self.DC_t = [[]] * len(ports)
        self.DC_lines : list[plt.Line2D] = [ax[0][1].plot([], label='duty cycle')[0] for ax in self.axes]

        # integral error
        self.IE = [[]] * len(ports)
        self.IE_t = [[]] * len(ports)
        self.IE_lines : list[plt.Line2D] = [ax[1][0].plot([], label='integral error')[0] for ax in self.axes]
        
        # tracking error
        self.TE = [[]] * len(ports)
        self.TE_t = [[]] * len(ports)
        self.TE_lines : list[plt.Line2D] = [ax[1][1].plot([], label='tracking error')[0] for ax in self.axes]

    def setup_figures(self):
        for idx, figure in enumerate(self.figures):
            figure.suptitle(f"Luminaire at '{self.ports[idx]}'")

    def clear_figures(self):
        for idx in range(len(self.ports)):
            for row in range(self.rows):
                for col in range(self.cols):
                    ax = self.axes[idx][row][col]
                    for line in ax.get_lines():
                        line.set_data([], [])
        self.update_figures()

    def update_figures(self):
        for idx in range(len(self.ports)):
            for row in range(self.rows):
                for col in range(self.cols):
                    ax = self.axes[idx][row][col]
                    if ax.get_lines():
                        ax.relim()
                        ax.autoscale_view()
                        ax.legend()

    @staticmethod 
    def wrap(lst: list):
        return lst[-Artist.MAX_SAMPLES:]

    def func_t(self, idx, t):
        self.time[idx].append(t)
        self.time[idx] = self.wrap(self.time[idx])
        self.time_lines[idx].set_data(self.time[idx], self.time[idx])

    def func_TE(self, idx, tracking_error, time):
        self.TE_t[idx].append(time)
        self.TE[idx].append(tracking_error)
        self.TE_t[idx] = self.wrap(self.TE_t[idx])
        self.TE[idx] = self.wrap(self.TE[idx])
        self.TE_lines[idx].set_data(self.TE_t[idx], self.TE[idx])

    def func_l(self, idx, illuminance, time):
        self.l_t[idx].append(time)
        self.l[idx].append(illuminance)
        self.l_t[idx] = self.wrap(self.l_t[idx])
        self.l[idx] = self.wrap(self.l[idx])
        self.l_lines[idx].set_data(self.l_t[idx], self.l[idx])

    def func_r(self, idx, reference, time):
        self.r_t[idx].append(time)
        self.r[idx].append(reference)
        self.r_t[idx] = self.wrap(self.r_t[idx])
        self.r[idx] = self.wrap(self.r[idx])
        self.r_lines[idx].set_data(self.r_t[idx], self.r[idx])

    def func_IE(self, idx, integral_error, time):
        self.IE_t[idx].append(time)
        self.IE[idx].append(integral_error)
        self.IE_t[idx] = self.wrap(self.IE_t[idx])
        self.IE[idx] = self.wrap(self.IE[idx])
        self.IE_lines[idx].set_data(self.IE_t[idx], self.IE[idx])

    def func_DC(self, idx, duty_cycle, time):
        self.DC_t[idx].append(time)
        self.DC[idx].append(duty_cycle)
        self.DC_t[idx] = self.wrap(self.DC_t[idx])
        self.DC[idx] = self.wrap(self.DC[idx])
        self.DC_lines[idx].set_data(self.DC_t[idx], self.DC[idx])


serial_ports = ['COM4', 'COM5', 'COM6']

artist = Artist(serial_ports)
artist.setup_figures()

commands = (
    ('t ', 'time', artist.func_t),
    ('s t ', 'tracking error', artist.func_TE),
    ('s l ', 'illuminance', artist.func_l),
    ('s r ', 'reference', artist.func_r),
    ('s i ', 'integral error', artist.func_IE),
    ('s d ', 'duty cycle', artist.func_DC),
)

QUEUE_SIZE = 1000

input_height = 5
output_height = 7

inputs_str = [""]*input_height
outputs_str = [[""]*output_height for _ in range(len(serial_ports))]

def serial_read(serial: serial.Serial, data_queue_out: queue.Queue, data_queue_in: queue.Queue):
    buf = b''
    while True:
        buf = serial.read_all()
        if buf:
            data_queue_out.put(buf)
        try:
            data_in = data_queue_in.get(block=False)
            if data_in:
                print(f"Writing: {data_in}")
                serial.write(data_in)
                serial.flush()
                data_in = None
        except queue.Empty:
            pass

def update_str_lst(lst, new_str):
    lst[:-1] = lst[1:]
    lst[-1] = new_str

queues_out = [queue.Queue(QUEUE_SIZE) for _ in range(len(serial_ports))]
queues_in = [queue.Queue(QUEUE_SIZE) for _ in range(len(serial_ports))]
serial_objects = []
for port in serial_ports:
    serial_objects += [serial.Serial(port, timeout=None)]
    time.sleep(0.2)
    
threads = [threading.Thread(target=serial_read, args=(serial_objects[i], queues_out[i], queues_in[i]), daemon=True) for i in range(len(serial_objects))]

[thread.start() for thread in threads]

data_read_buffers = [b'' for _ in range(len(serial_ports))]

sg.theme('Python')
column = [[sg.Text(f'{port}:'), sg.Text(size=(80,output_height), key=port)] for port in serial_ports]

column += [[sg.Text(f'[{serial_ports[0]}] Input:'), sg.Text(size=(80, input_height), key='input_win')]]

TOGGLE_RL_KEYS = [f"{port}_toggle_rl" for port in serial_ports]
toggle_rl = [sg.Button(f"{serial_ports[i]}", key=TOGGLE_RL_KEYS[i]) for i in range(len(serial_ports))]

TOGGLE_TE_KEYS = [f"{port}_toggle_te" for port in serial_ports]
toggle_te = [sg.Button(f"{serial_ports[i]}", key=TOGGLE_TE_KEYS[i]) for i in range(len(serial_ports))]

TOGGLE_IE_KEYS = [f"{port}_toggle_ie" for port in serial_ports]
toggle_ie = [sg.Button(f"{serial_ports[i]}", key=TOGGLE_IE_KEYS[i]) for i in range(len(serial_ports))]

TOGGLE_DC_KEYS = [f"{port}_toggle_dc" for port in serial_ports]
toggle_dc = [sg.Button(f"{serial_ports[i]}", key=TOGGLE_DC_KEYS[i]) for i in range(len(serial_ports))]


layout = [[sg.Column(column, element_justification='l', vertical_alignment='t')],
        [sg.Input(key='SerialInput', do_not_clear=True)],
        [sg.Button('[Enter] Send', bind_return_key=True), sg.Button('Clear Figures')],
        [sg.Text("Toggle Reference/Lux Measurements")] + toggle_rl,
        [sg.Text("Toggle Tracking Error Measurements")] + toggle_te,
        [sg.Text("Toggle Integral Error Measurements")] + toggle_ie,
        [sg.Text("Toggle Duty-Cycle Measurements")] + toggle_dc,
        [sg.Button('Exit')]
]

window = sg.Window('Serial monitor', layout)

_iter = 0
window.read(timeout=1)
while True:
    for idx, buffer_queue in enumerate(queues_out):
        try:
            while not buffer_queue.empty():
                data_read_buffers[idx] += buffer_queue.get(block=False)
            data_string = data_read_buffers[idx].decode('latin-1')
            data_read_buffers[idx] = b''
            if data_string:
                for line in data_string.split('\n'):
                    if line:
                        update_str_lst(outputs_str[idx], line)
                        if idx != 0: # skip all but the master
                            continue 
                        for command in commands:
                            if line.startswith(command[0]):
                                args = (line.split(command[0])[1]).split(' ')
                                flargs = [float(arg) for arg in args[1:]]
                                time_of_command = float(args[-1])/1000.0
                                if time_of_command < artist.latest_time - 0.3: # ignore old samples
                                    break
                                print(f"{command[1]}({args[0]}): {flargs}")
                                command[2](int(args[0]), *flargs[:-1], time_of_command)

                window[serial_ports[idx]].update('\n'.join(outputs_str[idx]))
        except queue.Empty:
            pass
    
    if _iter % 100 == 0:
        artist.update_figures()
        event, values = window.read(timeout=1)
        if event == sg.TIMEOUT_KEY:
            continue
        if event == sg.WIN_CLOSED or event == 'Exit':
            break
        if event == '[Enter] Send':
            update_str_lst(inputs_str, values['SerialInput'])
            window['input_win'].update('\n'.join(inputs_str))
            queues_in[0].put(f"{values['SerialInput']}\n".encode('latin-1'))
            window['SerialInput'].update('')

        if event == 'Clear Figures':
            artist.clear_figures()
        
        for idx, toggle_info_key in enumerate(TOGGLE_RL_KEYS):
            if event == toggle_info_key:
                queues_in[0].put(f"s l {idx}\n".encode('latin-1'))
                queues_in[0].put(f"s r {idx}\n".encode('latin-1'))

        for idx, toggle_info_key in enumerate(TOGGLE_IE_KEYS):
            if event == toggle_info_key:
                queues_in[0].put(f"s i {idx}\n".encode('latin-1'))

        for idx, toggle_info_key in enumerate(TOGGLE_TE_KEYS):
            if event == toggle_info_key:
                queues_in[0].put(f"s t {idx}\n".encode('latin-1'))

        for idx, toggle_info_key in enumerate(TOGGLE_DC_KEYS):
            if event == toggle_info_key:
                queues_in[0].put(f"s d {idx}\n".encode('latin-1'))
        
    _iter += 1


window.close()