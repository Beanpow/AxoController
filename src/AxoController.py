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
from utils import check_sum, get_16bit


class AxoController:

    def __init__(self, port: str, byterate: int = 38400, timeout: int = 0.01):
        # Constant
        self._hip_limit = [-25, 95]      # degree
        self._knee_limit = [-100, 8]     # degree
        self._vel_limit = [-3000, 3000]  # rpm
        self._current_limit = [-15, 15]  # A

        self.ser = serial.Serial(port, byterate, timeout=timeout)

    def _send_message(self, msg: bytearray):
        assert msg[-2] == check_sum(msg)
        self.ser.write(msg)

    def enter_control_mode(self):
        msg = bytearray([0x55, 0x5, 0xA0, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def exit_control_mode(self):
        msg = bytearray([0x55, 0x5, 0xA1, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def set_one_motor_pos(self, motor_id: int, pos: float):
        """set motor position singly

        Args:
            motor_id (int): left hip = 0, left knee = 1, right hip = 3, right knee = 4
            pos (float): motor's angle, unit: degree
        """
        assert motor_id in [0, 1, 3, 4]
        assert self._hip_limit[0] <= pos <= self._hip_limit[1] if motor_id in [0, 3] else self._knee_limit[0] <= pos <= self._knee_limit[1]

        high_byte, low_byte = get_16bit(pos, 100)

        msg = bytearray([0x55, 0x8, 0xA6, motor_id, high_byte, low_byte, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def set_all_motors_pos(self, pos: list) -> None:
        """set all motor position

        Args:
            pos (list): the desire position of all motors, unit: degree,
                        order: [left hip, left knee, right hip, right knee]
        """
        assert len(pos) == 4
        assert self._hip_limit[0] <= pos[0] <= self._hip_limit[1]
        assert self._knee_limit[0] <= pos[1] <= self._knee_limit[1]
        assert self._hip_limit[0] <= pos[2] <= self._hip_limit[1]
        assert self._knee_limit[0] <= pos[3] <= self._knee_limit[1]

        byte_tuple = [get_16bit(i, 100) for i in pos]

        msg = bytearray([0x55, 13, 0xA7])
        for high_byte, low_byte in byte_tuple:
            msg += bytearray([high_byte, low_byte])
        msg += bytearray([0x00, 0xBB])
        msg[-2] = check_sum(msg)

    def _set_single_motor_vel(self, motor_id: int, vel: int):
        pass

    def _set_all_motors_vel(self, vel: list):
        pass

    def _set_single_motor_current(self, motor_id: int, current: int):
        pass

    def _set_all_motor_current(self, current: list):
        pass

    def change_control_mode(self, mode: str):
        assert mode in ["pos", "vel", "current"]

        mode_idx = -1
        if mode == "pos":
            mode_idx = 0
        elif mode == "vel":
            mode_idx = 1
        elif mode == "current":
            mode_idx = 2
        assert mode_idx in [0, 1, 2]

        msg = bytearray([0xAA, 0x6, 0xA8, mode_idx, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def change_communication_state(self, state: str):
        assert state in ["close", "open"]

        state_id = -1
        if state == "close":
            state_id = 0
        elif state == "open":
            state_id = 1
        assert state_id in [0, 1]

        msg = bytearray([0xAA, 0x6, 0xA9, state_id, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def query(self):
        pass

    def test_commuintation(self):
        pass
