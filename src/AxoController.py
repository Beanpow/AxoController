# -*- encoding: utf-8 -*-
'''
@File    :   AxoController.py
@Time    :   2022/10/07 16:24:32
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   AxoController
'''
from signal import SIGINT
import serial
import numpy as np
import signal

class AxoController:

    def __init__(self, port: str, byterate: int = 38400, timeout: int = 0.01):
        self.ser = serial.Serial(port, byterate, timeout=timeout)

    def _send_message(self, msg: bytes):
        self.ser.write(msg)

    def enter_control_mode(self):
        control_command = np.array([0x55,5,0xA0,self.validation_message,0xBB])
        return control_command
    
    def exit_control_mode(self):
        control_command = np.array([0x55,5,0xA1,self.validation_message,0xBB])
        return control_command

    def validation_message(self):
        c = "VS"
        return c
    
    def emergency_stop(self,signal_received,steps):
        
        exit(0)


if __name__ == "__main__":
    a = bytearray([0x1, 0x2, 0x3, 0x4])
    a[0] = 10
    ac = AxoController("com3")
    signal(SIGINT,ac.emergency_stop)
