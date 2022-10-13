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
import sys
sys.path.append("..")
from AxoController import AxoController  # noqa: E402


class TestAxoController(unittest.TestCase):
    port = 'com3'

    def test_enter_control_mode(self):
        axo_ctrl = AxoController(port=self.port)
        self.assertIsNone(axo_ctrl.enter_control_mode())

    def test_exit_control_mode(self):
        axo_ctrl = AxoController(port=self.port)
        self.assertIsNone(axo_ctrl.exit_control_mode())

    def test_set_single_motor_pos(self):
        axo_ctrl = AxoController(port=self.port)
        self.assertIsNone(axo_ctrl.set_one_motor_pos(0, 5))
        self.assertIsNone(axo_ctrl.set_one_motor_pos(1, 5))

    def test_close_communication(self):
        axo_ctrl = AxoController(port=self.port)


if __name__ == "__main__":
    unittest.main()
