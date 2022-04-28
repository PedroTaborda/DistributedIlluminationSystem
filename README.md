# SCDTR

This directory contains the source code for an implementation of the 2021/2022 (2nd semester) SCDTR class project.

The main code, to be run in a Raspberry Pi Pico, lies in the `_main/` directory as an Arduino sketch. 

The available commands are presented by entering `h <id>` in the Serial interface of a Pico (such as `h 0` if only one is connected). 

If the main sketch is compiled with `-DDEBUG` as a compile flag, descriptive messages will be sent to the Serial interface.

The `serial_com_async.py` file implements a GUI for visualizing data on the network nodes. However, it is very limited and if configuration changes are required, the code may need to be altered. The required libraries for this Python script can be installed via  `pip3 install -r requirements.txt`
