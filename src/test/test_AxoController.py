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
import time

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


class TestAxoController(unittest.TestCase):
    port = 'com3'
    # port = '/dev/tty.usbserial-CHAIb11A920'

    def test_axo_init(self):
        axo_ctrl = AxoController(port=self.port)  # noqa
        axo_ctrl.close_controller()

    def test_axo_get_info_thread(self):
        axo_ctrl = AxoController(port=self.port)
        time.sleep(2)
        self.assertGreater(len(axo_ctrl.info_stack), 0)
        axo_ctrl.close_controller()

    def test_enter_exit_control_mode(self):
        axo_ctrl = AxoController(port=self.port)
        axo_ctrl.enter_control_mode()
        self.assertTrue(axo_ctrl.in_control_mode)
        axo_ctrl.exit_control_mode()
        self.assertFalse(axo_ctrl.in_control_mode)
        axo_ctrl.close_controller()

    def test_set_all_motor_pos(self):
        axo_ctrl = AxoController(port=self.port)
        axo_ctrl.enter_control_mode()

        print(start_time := time.time())
        for i in range(20):
            axo_ctrl.set_all_motors_pos_sync([i, 0, i, 0])
            print(i, time.time() - start_time)

        axo_ctrl.exit_control_mode()
        axo_ctrl.close_controller()

    def test_get_leg_info(self):
        axo_ctrl = AxoController(port=self.port)

        time.sleep(1)
        print(axo_ctrl.get_leg_info())

        axo_ctrl.close_controller()

    def test_angle_detection(self):
        axo_ctrl = AxoController(port=self.port, angle_telorance=[20, 60, 80, 5])

        axo_ctrl.set_all_motors_pos_sync([10, 0, 10, 0])

        with self.assertRaises(Exception):
            axo_ctrl.set_all_motors_pos_sync([40, -30, 40, -30])


if __name__ == "__main__":
    unittest.main()
