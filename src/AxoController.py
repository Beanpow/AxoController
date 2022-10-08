# -*- encoding: utf-8 -*-
'''
@File    :   AxoController.py
@Time    :   2022/10/07 16:24:32
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   AxoController
'''
import serial


class AxoController:

    def __init__(self, port: str, byterate: int = 38400, timeout: int = 0.01):
        self.ser = serial.Serial(port, byterate, timeout=timeout)

    def _send_message(self, msg: bytes):
        self.ser.write(msg)

    def enter_control_mode(self):
        pass

    def validation_message(self):
        c = "VS"
        return c


if __name__ == "__main__":
    a = bytearray([0x1, 0x2, 0x3, 0x4])
    a[0] = 10
    ac = AxoController("com3")
