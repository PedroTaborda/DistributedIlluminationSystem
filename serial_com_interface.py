import serial
import logging
import multiprocessing
import time
import os
import asyncio

from abc import ABC, abstractmethod

logs_folder = 'logs'
read_size = 1024

if not os.path.isdir(logs_folder):
    os.mkdir(logs_folder)

class SerialDataInput(ABC, multiprocessing.Process):
    def __init__(self):
        super().__init__()

    def set_process_intercom(self, run_event: multiprocessing.Event, data_input_event: multiprocessing.Event, shared_namespace, broadcast_event: multiprocessing.Event):
        self.run_event = run_event
        self.data_input_event = data_input_event
        self.shared_namespace = shared_namespace
        self.broadcast_event = broadcast_event

    def run(self):
        self.send_data(self.run_event, self.data_input_event, self.shared_namespace)

    @abstractmethod
    def send_data(self, run_event: multiprocessing.Event, data_input_event: multiprocessing.Event, shared_namespace):
        pass

class SerialDataOutput(ABC, multiprocessing.Process):
    def __init__(self):
        super().__init__()
        
    def set_process_intercom(self, run_event: multiprocessing.Event, data_output_event: multiprocessing.Event, data_output_lock: multiprocessing.Lock, shared_namespace, broadcast_event: multiprocessing.Event):
        self.run_event = run_event
        self.data_output_event = data_output_event
        self.data_output_lock = data_output_lock
        self.shared_namespace = shared_namespace
        self.broadcast_event = broadcast_event

    def run(self):
        self.handle_data(self.run_event, self.data_output_event, self.data_output_lock, self.shared_namespace)

    @abstractmethod
    def handle_data(self, run_event: multiprocessing.Event, data_output_event: multiprocessing.Event, data_output_lock: multiprocessing.Lock, shared_namespace):
        pass

class SerialOutputAtomic(SerialDataOutput):
    def __init__(self) -> None:
        super().__init__()
        self.output_lines = []

        self.broadcast_reacted = False
    
    def handle_data(self, run_event: multiprocessing.Event, data_output_event: multiprocessing.Event, data_output_lock: multiprocessing.Lock, shared_namespace):
        internal_buf = ''
        while run_event.is_set():
            if data_output_event.is_set():
                with data_output_lock:
                    data_output_event.clear()
                    if shared_namespace.data_out_buffer:
                        internal_buf += shared_namespace.data_out_buffer
                        shared_namespace.data_out_buffer = ''
                    logging.info(shared_namespace.data_out_buffer)

            if internal_buf:
                lines = internal_buf.split('\n')
                internal_buf = lines[-1] # incomplete lines go back to the buffer
                lines = lines[:-1]
                for line in lines:
                    self.handle_output_line(line)

            if self.broadcast_event.is_set() and self.broadcast_reacted == False:
                self.handle_broadcast(self.shared_namespace.msg)
                self.broadcast_reacted = True

            if (not self.broadcast_event.is_set()) and self.broadcast_reacted == True:
                self.broadcast_reacted = False

    @abstractmethod
    def handle_broadcast(self, msg):
        pass

    @abstractmethod
    def handle_output_line(self, line: str):
        pass

class SerialInputStdin(SerialDataInput):
    def send_data(self, run_event: multiprocessing.Event, input_event: multiprocessing.Event, shared_namespace):
        import sys
        sys.stdin = open(0)
        while run_event.is_set():
            try:
                if not input_event.is_set():
                    data_buffer = input()
                    shared_namespace.data_in_buffer += data_buffer + '\n'
                    input_event.set()
            except EOFError:
                raise KeyboardInterrupt

class SerialOutputStdout(SerialDataOutput):
    def handle_data(self, run_event: multiprocessing.Event, data_output_event: multiprocessing.Event, data_output_lock: multiprocessing.Lock, shared_namespace):
        while run_event.is_set():
            if data_output_event.is_set():
                with data_output_lock:
                    data_output_event.clear()
                    if shared_namespace.data_out_buffer:
                        print(shared_namespace.data_out_buffer, end='')
                        shared_namespace.data_out_buffer = ''
                    logging.info(shared_namespace.data_out_buffer)

class SerialComms:
    def __init__(self, port, data_input_handler: SerialDataInput, data_output_handler: SerialDataOutput):
        serial_com_mgr = multiprocessing.Manager()
        input_ready = serial_com_mgr.Event()
        output_ready = serial_com_mgr.Event()
        output_lock = serial_com_mgr.Lock()
        shared_namespace = serial_com_mgr.Namespace()
        shared_namespace.data_in_buffer = ''
        shared_namespace.data_out_buffer = ''

        self.broadcast_event = multiprocessing.Event()
        shared_namespace.msg = ''

        self.namespace = shared_namespace

        self.run = multiprocessing.Event()
        self.run.set()

        self.do_comms = multiprocessing.Event()
        self.do_comms.set()

        serial_handler = multiprocessing.Process(target=SerialComms.serial_comms, args=(port, self.do_comms, input_ready, output_ready, output_lock, shared_namespace))
        data_input_handler.set_process_intercom(self.run, input_ready, shared_namespace, self.broadcast_event)
        data_output_handler.set_process_intercom(self.run, output_ready, output_lock, shared_namespace, self.broadcast_event)

        self.serial_handler = serial_handler
        self.data_input_handler = data_input_handler
        self.data_output_handler = data_output_handler

    def start(self):
        self.serial_handler.start()
        self.data_input_handler.start()
        self.data_output_handler.start()

    def stop(self):
        self.run.clear()
        self.data_input_handler.join()
        self.data_output_handler.join()
        self.do_comms.clear()
        self.serial_handler.join()

    async def broadcast_msg(self, msg):
        self.namespace.msg = msg
        self.broadcast_event.set()
        await asyncio.sleep(1)
        self.broadcast_event.clear()
        self.namespace.msg = None

    @staticmethod
    def serial_comms(port: str, run_event: multiprocessing.Event, write_event: multiprocessing.Event, 
                        read_event: multiprocessing.Event, output_lock: multiprocessing.Lock, shared_namespace):
        serial_obj = serial.Serial(port, 115200, timeout=0.1)
        while run_event.is_set():
            res = serial_obj.read(size=read_size).decode('utf-8')
            if res:
                with output_lock:
                    shared_namespace.data_out_buffer += res
                    read_event.set()
                    #print(res, end='\n')

            if write_event.is_set():
                if shared_namespace.data_in_buffer:
                    commands = shared_namespace.data_in_buffer.split('\n')
                    shared_namespace.data_in_buffer = commands[-1]
                    commands = commands[:-1]
                    for command in commands:
                        serial_obj.write(command.encode('utf-8'))
                write_event.clear()

        serial_obj.close()


if __name__ == '__main__':
    comms = SerialComms('COM3', SerialInputStdin(), SerialOutputStdout())
    comms.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        comms.stop()


