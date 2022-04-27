import threading
import time
from traceback import print_tb
import numpy as np
import serial.tools.list_ports
import serial
import queue
import PySimpleGUI as sg
import matplotlib.figure as figure
import matplotlib.pyplot as plt


number_of_network_nodes = 3

monitor_serial = 'All' # 'Single' or 'All'

class Artist:
    MAX_SAMPLES = 5000
    def __init__(self, nodes) -> None:
        self.nodes = nodes

        self.rows = 2
        self.cols = 2
        plt.ion()
        self.subplots = [plt.subplots(self.rows, self.cols, num=i, sharex=True) for i in range(len(nodes))]

        self.figures:list[figure.Figure] = [subplot[0] for subplot in self.subplots]
        self.axes: list[tuple[tuple[plt.Axes]]] = [subplot[1] for subplot in self.subplots] # self.rows by self.cols

        self.latest_time = [0] * len(nodes)


        # illuminance
        self.l = [[]] * len(nodes)
        self.l_t = [[]] * len(nodes)
        self.l_lines : list[plt.Line2D] = [ax[0][0].plot([], label='illuminance')[0] for ax in self.axes]

        # reference
        self.r = [[]] * len(nodes)
        self.r_t = [[]] * len(nodes)
        self.r_lines : list[plt.Line2D] = [ax[0][0].plot([], label='reference')[0] for ax in self.axes]

        # duty cycle
        self.DC = [[]] * len(nodes)
        self.DC_t = [[]] * len(nodes)
        self.DC_lines : list[plt.Line2D] = [ax[0][1].plot([], label='duty cycle')[0] for ax in self.axes]

        # integral error
        self.IE = [[]] * len(nodes)
        self.IE_t = [[]] * len(nodes)
        self.IE_lines : list[plt.Line2D] = [ax[1][0].plot([], label='integral error')[0] for ax in self.axes]
        
        # tracking error
        self.TE = [[]] * len(nodes)
        self.TE_t = [[]] * len(nodes)
        self.TE_lines : list[plt.Line2D] = [ax[1][1].plot([], label='tracking error')[0] for ax in self.axes]

    def setup_figures(self):
        for idx, figure in enumerate(self.figures):
            figure.suptitle(f"Node '{self.nodes[idx]}'")
            self.axes[idx][1][0].set_xlabel('Time (s)')
            self.axes[idx][1][1].set_xlabel('Time (s)')

    def clear_figures(self):
        for idx in range(len(self.nodes)):
            for row in range(self.rows):
                for col in range(self.cols):
                    ax = self.axes[idx][row][col]
                    for line in ax.get_lines():
                        line.set_data([], [])
        self.update_figures()

        # illuminance
        self.l = [[]] * len(self.nodes)
        self.l_t = [[]] * len(self.nodes)

        # reference
        self.r = [[]] * len(self.nodes)
        self.r_t = [[]] * len(self.nodes)

        # duty cycle
        self.DC = [[]] * len(self.nodes)
        self.DC_t = [[]] * len(self.nodes)

        # integral error
        self.IE = [[]] * len(self.nodes)
        self.IE_t = [[]] * len(self.nodes)
        
        # tracking error
        self.TE = [[]] * len(self.nodes)
        self.TE_t = [[]] * len(self.nodes)

    def update_figures(self):
        for idx in range(len(self.nodes)):
            for row in range(self.rows):
                for col in range(self.cols):
                    ax = self.axes[idx][row][col]
                    ax.set_xlim((self.latest_time[idx] - 5, self.latest_time[idx] + 0.5))
                    ax.legend()
            node_axes = self.axes[idx]
            lux_ax = node_axes[0][0]
            dc_ax = node_axes[0][1]
            ie_ax = node_axes[1][0]
            te_ax = node_axes[1][1]
            
            dc_ax.set_ylim((-0.05, 1.05))
            if self.l[idx]:
                lux_ax.set_ylim((-2, max(max(max(self.l[idx]), max(self.r[idx])) * 1.1, 10)))
            else:
                lux_ax.set_ylim((-2, 10))
            if self.IE[idx]:
                ie_ax.set_ylim((min(-1, min(self.IE[idx]) - 0.3), max(1, max(self.IE[idx]) + 0.3) ) )
            else:
                ie_ax.set_ylim((-1, 1))
            if self.TE[idx]:
                te_ax.set_ylim((min(-1, min(self.TE[idx]) - 0.3), max(1, max(self.TE[idx]) + 0.3) ) )
            else:
                te_ax.set_ylim((-1, 1))

    @staticmethod 
    def wrap(t: list, vals: list):
        vals = vals[-Artist.MAX_SAMPLES:]

        tmin = t[-1] - 6.0
        tmax = t[-1] + 0.5
        t_arr = np.array(t)
        tmin_idx = np.searchsorted(t_arr, tmin)
        tmax_idx = np.searchsorted(t_arr, tmax)
        return t[tmin_idx:tmax_idx], vals[tmin_idx:tmax_idx]

    def func_TE(self, idx, tracking_error, time):
        self.TE_t[idx].append(time)
        self.TE[idx].append(tracking_error)
        self.TE_t[idx], self.TE[idx] = self.wrap(self.TE_t[idx], self.TE[idx])
        self.TE_lines[idx].set_data(self.TE_t[idx], self.TE[idx])

    def func_l(self, idx, illuminance, time):
        self.l_t[idx].append(time)
        self.l[idx].append(illuminance)
        self.l_t[idx], self.l[idx] = self.wrap(self.l_t[idx], self.l[idx])
        self.l_lines[idx].set_data(self.l_t[idx], self.l[idx])

    def func_r(self, idx, reference, time):
        self.r_t[idx].append(time)
        self.r[idx].append(reference)
        self.r_t[idx], self.r[idx] = self.wrap(self.r_t[idx], self.r[idx])
        self.r_lines[idx].set_data(self.r_t[idx], self.r[idx])

    def func_IE(self, idx, integral_error, time):
        self.IE_t[idx].append(time)
        self.IE[idx].append(integral_error)
        self.IE_t[idx], self.IE[idx] = self.wrap(self.IE_t[idx], self.IE[idx])
        self.IE_lines[idx].set_data(self.IE_t[idx], self.IE[idx])

    def func_DC(self, idx, duty_cycle, time):
        self.DC_t[idx].append(time)
        self.DC[idx].append(duty_cycle)
        self.DC_t[idx], self.DC[idx] = self.wrap(self.DC_t[idx], self.DC[idx])
        self.DC_lines[idx].set_data(self.DC_t[idx], self.DC[idx])

ports = serial.tools.list_ports.comports()

serial_ports = []
for port, desc, hwid in sorted(ports):
    print(f"{port}: {desc} [{hwid}]")
for port, desc, hwid in sorted(ports):
    if 'Serial' in desc:
        print(f"Added {port}")
        serial_ports.append(port)

if monitor_serial == 'Single':
    ports_to_monitor = [serial_ports[0]]
else:
    ports_to_monitor = serial_ports

names = [f"Node {i+1}" for i in range(number_of_network_nodes)]
artist = Artist(names)
artist.setup_figures()

commands = (
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
outputs_str = [[""]*output_height for _ in range(number_of_network_nodes)]

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

queues_out = [queue.Queue(QUEUE_SIZE) for _ in range(len(ports_to_monitor))]
queues_in = [queue.Queue(QUEUE_SIZE) for _ in range(len(ports_to_monitor))]
serial_objects = []
for port in ports_to_monitor:
    serial_objects += [serial.Serial(port, timeout=None)]
    time.sleep(0.2)
    
threads = [threading.Thread(target=serial_read, args=(serial_objects[i], queues_out[i], queues_in[i]), daemon=True) for i in range(len(serial_objects))]

[thread.start() for thread in threads]

data_read_buffers = [b'' for _ in range(len(ports_to_monitor))]

sg.theme('Python')
column = [[sg.Text(f'{port}:'), sg.Text(size=(80,output_height), key=port)] for port in ports_to_monitor]

column += [[sg.Text(f'[{ports_to_monitor[0]}] Input:'), sg.Text(size=(80, input_height), key='input_win')]]

TOGGLE_RL_KEYS = [f"{node}_toggle_rl" for node in range(number_of_network_nodes)]
toggle_rl = [sg.Button(f"Node {i+1}", key=TOGGLE_RL_KEYS[i]) for i in range(number_of_network_nodes)]

TOGGLE_TE_KEYS = [f"{node}_toggle_te" for node in range(number_of_network_nodes)]
toggle_te = [sg.Button(f"Node {i+1}", key=TOGGLE_TE_KEYS[i]) for i in range(number_of_network_nodes)]

TOGGLE_IE_KEYS = [f"{node}_toggle_ie" for node in range(number_of_network_nodes)]
toggle_ie = [sg.Button(f"Node {i+1}", key=TOGGLE_IE_KEYS[i]) for i in range(number_of_network_nodes)]

TOGGLE_DC_KEYS = [f"{node}_toggle_dc" for node in range(number_of_network_nodes)]
toggle_dc = [sg.Button(f"Node {i+1}", key=TOGGLE_DC_KEYS[i]) for i in range(number_of_network_nodes)]


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
            if _iter == 0:
                break
            if data_string:
                for line in data_string.split('\n'):
                    if line:
                        update_str_lst(outputs_str[idx], line)
                        for command in commands:
                            if line.startswith(command[0]):
                                args = (line.split(command[0])[1]).split(' ')
                                flargs = [float(arg) for arg in args[1:]]
                                time_of_command = float(args[-1])/1000.0
                                if artist.latest_time[int(args[0])] == 0.0:
                                    artist.latest_time[int(args[0])] = time_of_command
                                if time_of_command < artist.latest_time[int(args[0])] - 1.0: # ignore old samples
                                    continue
                                if time_of_command > artist.latest_time[int(args[0])] + 6000.0: # ignore probably corrupted samples
                                    continue
                                artist.latest_time[int(args[0])] = max(artist.latest_time[int(args[0])], time_of_command)
                                command[2](int(args[0]), *flargs[:-1], time_of_command)

                window[serial_ports[idx]].update('\n'.join(outputs_str[idx]))
        except queue.Empty:
            pass
    
    _iter += 1
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
        


window.close()