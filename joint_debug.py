import asyncio
import PySimpleGUI as sg
from serial_com_interface import SerialComms, SerialDataInput, SerialOutputAtomic, SerialInputStdin, SerialOutputStdout

def update_str_lst(lst, new_str):
    lst[:-1] = lst[1:]
    lst[-1] = new_str

class SerialOutputToWindow(SerialOutputAtomic):
    def __init__(self, window, window_key):
        super().__init__()
        self.my_output = [""]*output_height
        self.window = window
        self.window_key = window_key

    def handle_output_line(self, line: str):
        update_str_lst(self.my_output, line)
        self.window[self.window_key].update(self.my_output)
        
    def handle_broadcast(self, msg):
        print(f"Message: {msg}")

class JointNodes:
    def __init__(self, comms: list[SerialComms]) -> None:
        self.comms = comms

    def start(self):
        for comm in self.comms: # send 
            comm.start()

    def stop(self):
        for comm in self.comms:
            comm.stop()
if __name__ == "__main__":
    sg.theme('Python')
    input_height = 5
    output_height = 5
    column = [
        [sg.Text('node1:'), sg.Text(size=(30,output_height), key='node1')],
        [sg.Text('node2:'), sg.Text(size=(30,output_height), key='node2')],
        [sg.Text('node3:'), sg.Text(size=(30,output_height), key='node3')],
        [sg.Text('Input:'), sg.Text(size=(30, input_height), key='input_win')]
    ]
    layout = [[sg.Column(column, element_justification='l', vertical_alignment='t')],
            [sg.Input(key='SerialInput', do_not_clear=False)],
            [sg.Button('Send', bind_return_key=True)],
            [sg.Button('Restart'), sg.Button('Exit')]]

    window = sg.Window('Serial monitor', layout)


    output2 = [""]*output_height
    output3 = [""]*output_height
    input = [""]*input_height

    comms1 = SerialComms('COM3', SerialInputStdin(), SerialOutputToWindow(window=window, window_key='node1'))
    comms2 = SerialComms('COM4', SerialInputStdin(), SerialOutputToWindow(window=window, window_key='node2'))
    comms3 = SerialComms('COM5', SerialInputStdin(), SerialOutputToWindow(window=window, window_key='node3'))

    comms = JointNodes([comms1, comms2, comms3])

    try:
        comms.start()
        while True:  # Event Loop
            event, values = window.read()
            print(event, values)
            update_str_lst(input, values['SerialInput'])
            if event == sg.WIN_CLOSED or event == 'Exit':
                break
            if event == 'Send':
                window['input_win'].update('\n'.join(input))
            if event == 'Restart':
                window['COM4'].update("We are restarting")
    except KeyboardInterrupt:
        comms.stop()

    window.close()