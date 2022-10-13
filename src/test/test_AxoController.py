# -*- encoding: utf-8 -*-
'''
@File    :   test_enter_exit_control_mode.py
@Time    :   2022/10/13 00:24:16
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :
'''
import unittest
import time
import sys
sys.path.append("..")
from AxoController import AxoController  # noqa: E402


class TestAxoController(unittest.TestCase):
    # port = 'com3'
    port = '/dev/tty.usbserial-CHAIb11A920'

    def test_axo_init(self):
        axo_ctrl = AxoController(port=self.port)  # noqa

    def test_axo_get_info_thread(self):
        axo_ctrl = AxoController(port=self.port)
        axo_ctrl.open_receive_info()
        time.sleep(2)
        axo_ctrl.close_receive_info()
        print(axo_ctrl.info_stack)


    # def test_enter_exit_control_mode(self):
    #     axo_ctrl = AxoController(port=self.port)
    #     axo_ctrl.enter_control_mode()
    #     axo_ctrl.exit_control_mode()

    # def test_set_single_motor_pos(self):
    #     axo_ctrl = AxoController(port=self.port)
    #     axo_ctrl.set_one_motor_pos(0, 5)
    #     axo_ctrl.set_one_motor_pos(1, 5)


if __name__ == "__main__":
    unittest.main()
