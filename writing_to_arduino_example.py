# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 14:30:22 2023

@author: eh711
"""

import serial
import time


board = serial.Serial('COM3', 9600, timeout=0) #include timeout so code will stop if nothing is read/immediately return values, port is opened upon creation
#print(board.name) #cneck the correct port is being used
while True:
    try:
        board.write(b'H') # write an example string to Arduino - b is for bytes
        time.sleep(1)
        board.write(b'L')
        time.sleep(1)
    except KeyboardInterrupt:
        break

board.close() #prevents 'access denied' error when you rerun the script