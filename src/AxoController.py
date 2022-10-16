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
import threading
import time
import numpy as np
from typing import Union, List, Tuple
from utils import check_sum, from_16bit_to_int, from_int_to_16bit


LegInfoType = Tuple[float, float, float, float]


class AxoController:
    def __init__(self, port: str, byterate: int = 38400, timeout: int = 0, angle_telorance: List[int] = [10, 50, 50, 5], verbose: bool = False):
        # Constant
        self._hip_limit = [-25, 95]  # degree
        self._knee_limit = [-100, 8]  # degree
        self._hip_limit[0] += angle_telorance[0]
        self._hip_limit[1] -= angle_telorance[1]
        self._knee_limit[0] += angle_telorance[2]
        self._knee_limit[1] -= angle_telorance[3]
        assert self._hip_limit[0] < self._hip_limit[1] and self._knee_limit[0] < self._knee_limit[1], "Angle telorance is too large."
        self._vel_limit = [-3000, 3000]  # rpm
        self._current_limit = [-15, 15]  # A
        self._pos_factor = 100  # the pos will be multiplied by this factor
        self._current_factor = 100  # the current will be multiplied by this factor
        self._vel_download_factor = 1  # the vel will be multiplied by this factor when download to the robot
        self._vel_upload_factor = 10  # the vel will be multiplied by this factor when upload from the robot

        # Variable for communication
        self.info_stack = []
        self.info_stacksize = 10000
        self.is_get_info = False

        # Variable for angle detection
        self.is_angle_detection = False

        # Variable for safe control
        self.dangerous_cmds = [0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7]

        # Variable for control
        self.control_mode = "position"
        self.control_target = "all"
        self.in_control_mode = False

        # Establish serial connection
        try:
            self.ser = serial.Serial(port, byterate, timeout=timeout)
            assert self.ser.is_open is True
        except Exception as e:
            print(f"[error]: {e}")
            exit(1)
        self.verbose = verbose

        # Initial check
        if self.verbose:
            print("[info]: Checking the connection and robot state...")
        self.initial_check()
        if self.verbose:
            print("[info]: The connection and robot state is fine.")

        # update in_control_mode
        self.update_in_control_mode()
        while self.in_control_mode is not False:
            self.exit_control_mode()
            self.update_in_control_mode()
        assert self.in_control_mode is False, "The robot is in control mode, exit control mode first."

        # Receive info thread
        self.open_receive_info()

        # Angle detection thread
        time.sleep(0.05)  # wait for the info thread to start
        self.open_angle_detection()

    def __del__(self):
        self.close_controller()
        print("[info]: The controller is closed.")

    def open_angle_detection(self) -> None:
        self.is_angle_detection = True
        self.angle_detection_thread = threading.Thread(target=self._angle_detection)
        self.angle_detection_thread.setDaemon(True)
        self.angle_detection_thread.start()

    def close_angle_detection(self) -> None:
        self.is_angle_detection = False
        time.sleep(0.3)
        assert self.angle_detection_thread.is_alive() is False

    def _angle_detection(self) -> None:
        while self.is_angle_detection:
            left_hip, left_knee, right_hip, right_knee = self.get_leg_pos()
            if self.verbose:
                print(f"[info]: left_hip: {left_hip}, left_knee: {left_knee}, right_hip: {right_hip}, right_knee: {right_knee}")

            if left_hip < self._hip_limit[0] or left_hip > self._hip_limit[1]:
                print(f"[fatal]: left_hip: {left_hip} is out of limit: {self._hip_limit}")
                self.exit_control_mode()

            if left_knee < self._knee_limit[0] or left_knee > self._knee_limit[1]:
                print(f"[fatal]: left_knee: {left_knee} is out of limit: {self._knee_limit}")
                self.exit_control_mode()

            if right_hip < self._hip_limit[0] or right_hip > self._hip_limit[1]:
                print(f"[fatal]: right_hip: {right_hip} is out of limit: {self._hip_limit}")
                self.exit_control_mode()

            if right_knee < self._knee_limit[0] or right_knee > self._knee_limit[1]:
                print(f"[fatal]: right_knee: {right_knee} is out of limit: {self._knee_limit}")
                self.exit_control_mode()

            time.sleep(0.1)

    def close_controller(self):
        if self.in_control_mode:
            self.exit_control_mode()
        if self.is_get_info:
            self.close_receive_info()
        if self.is_angle_detection:
            self.close_angle_detection()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

    def _send_message(self, msg: bytearray):
        if self.verbose:
            print(f"[info]: Sending message: {msg}")
        assert msg[-2] == check_sum(msg)

        if msg[2] in self.dangerous_cmds:
            self._cmd_check(msg)

        time.sleep(0.005)
        write_num = self.ser.write(msg)
        assert write_num == len(msg)

    def _cmd_check(self, msg):
        assert (cmd := msg[2]) in self.dangerous_cmds

        if cmd in self.dangerous_cmds[0:6:2]:  # single control
            motor_name = self.motor_id2name(msg[3])
            assert motor_name in ["left_hip", "left_knee", "right_hip", "right_knee"]

            if cmd == 0xA2:  # current
                cmd_value = from_16bit_to_int(msg[4], msg[5], 1 / self._current_factor)
                assert self._current_limit[0] <= cmd_value <= self._current_limit[1]

            elif cmd == 0xA4:  # velocity
                cmd_value = from_16bit_to_int(msg[4], msg[5], 1 / self._vel_upload_factor)
                assert self._vel_limit[0] <= cmd_value <= self._vel_limit[1]

            elif cmd == 0xA6:  # position
                cmd_value = from_16bit_to_int(msg[4], msg[5], 1 / self._pos_factor)
                if motor_name.endswith("hip"):
                    assert self._hip_limit[0] <= cmd_value <= self._hip_limit[1]
                elif motor_name.endswith("knee"):
                    assert self._knee_limit[0] <= cmd_value <= self._knee_limit[1]

        elif cmd in self.dangerous_cmds[1:6:2]:  # all motor control

            if cmd == 0xA3:  # current
                for byte in zip(msg[3:11:2], msg[4:11:2]):
                    cmd_value = from_16bit_to_int(*byte, 1 / self._current_factor)
                    assert self._current_limit[0] <= cmd_value <= self._current_limit[1]

            elif cmd == 0xA5:  # velocity control
                for byte in zip(msg[3:11:2], msg[4:11:2]):
                    cmd_value = from_16bit_to_int(*byte, 1 / self._vel_upload_factor)
                    assert self._vel_limit[0] <= cmd_value <= self._vel_limit[1]

            elif cmd == 0xA7:  # position control
                hip_l = from_16bit_to_int(msg[3], msg[4], 1 / self._pos_factor)
                hip_r = from_16bit_to_int(msg[7], msg[8], 1 / self._pos_factor)

                knee_l = from_16bit_to_int(msg[5], msg[6], 1 / self._pos_factor)
                knee_r = from_16bit_to_int(msg[9], msg[10], 1 / self._pos_factor)

                assert self._hip_limit[0] <= hip_l <= self._hip_limit[1]
                assert self._hip_limit[0] <= hip_r <= self._hip_limit[1]
                assert self._knee_limit[0] <= knee_l <= self._knee_limit[1]
                assert self._knee_limit[0] <= knee_r <= self._knee_limit[1]

        else:  # other impossible cmd
            raise Exception("Impossible cmd.")

    def update_in_control_mode(self) -> None:
        robot_state = self.get_robot_state()
        assert robot_state[5] == robot_state[8] == robot_state[11] == robot_state[14]

        self.in_control_mode = robot_state[5] == 1

    def enter_control_mode(self) -> None:
        self.change_control_mode("position")
        if self.control_target == "all":
            self.set_all_motors_pos_async(pos=[0, 0, 0, 0])
        else:
            self.set_one_motor_pos(motor_id=self.motor_name2id(self.control_target), pos=0)

        msg = bytearray([0xAA, 0x5, 0xA0, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

        self.update_in_control_mode()
        if self.in_control_mode is not True:
            raise Exception("Enter control mode failed.")

    def motor_name2id(self, motor_name: str) -> int:
        assert motor_name in ["left_hip", "left_knee", "right_hip", "right_knee"]
        motor_name2id = {"left_hip": 0, "left_knee": 1, "right_hip": 3, "right_knee": 4}
        return motor_name2id[motor_name]

    def motor_id2name(self, motor_id: int) -> str:
        assert motor_id in [0, 1, 3, 4]
        motor_id2name = {0: "left_hip", 1: "left_knee", 3: "right_hip", 4: "right_knee"}
        return motor_id2name[motor_id]

    def exit_control_mode(self):
        msg = bytearray([0xAA, 0x5, 0xA1, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        for i in range(3):
            self._send_message(msg)

        self.update_in_control_mode()
        if self.in_control_mode is not False:
            raise Exception("Exit control mode failed.")

    def set_one_motor_pos(self, motor_id: int, pos: float):
        """set motor position singly

        Args:
            motor_id (int): left hip = 0, left knee = 1, right hip = 3, right knee = 4
            pos (float): motor's angle, unit: degree
        """
        assert motor_id in [0, 1, 3, 4]
        assert self.control_target == self.motor_id2name(motor_id)
        assert self._hip_limit[0] <= pos <= self._hip_limit[1] if motor_id in [0, 3] else self._knee_limit[0] <= pos <= self._knee_limit[1]
        assert self.control_mode == "position"

        high_byte, low_byte = from_int_to_16bit(pos, self._pos_factor)

        msg = bytearray([0xAA, 0x8, 0xA6, motor_id, high_byte, low_byte, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def set_all_motors_pos_async(self, pos: list) -> None:
        """set all motor position

        Args:
            pos (list): the desire position of all motors, unit: degree,
                        order: [left hip, left knee, right hip, right knee]
        """
        assert self.control_target == "all"
        assert len(pos) == 4
        assert self._hip_limit[0] <= pos[0] <= self._hip_limit[1]
        assert self._knee_limit[0] <= pos[1] <= self._knee_limit[1]
        assert self._hip_limit[0] <= pos[2] <= self._hip_limit[1]
        assert self._knee_limit[0] <= pos[3] <= self._knee_limit[1]
        assert self.control_mode == "position"

        byte_tuple = [from_int_to_16bit(i, self._pos_factor) for i in pos]

        msg = bytearray([0xAA, 13, 0xA7])
        for high_byte, low_byte in byte_tuple:
            msg += bytearray([high_byte, low_byte])
        msg += bytearray([0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def set_all_motors_pos_vel_based(self, pos: List[float]) -> None:
        pass

    def set_all_motors_pos_sync(self, pos: List[float]) -> None:
        self.set_all_motors_pos_async(pos)

        while self._angle_norm(self.get_leg_pos(), pos) > 0.1:
            time.sleep(0.05)

    def get_leg_pos(self) -> LegInfoType:
        leg_info = self.get_leg_info()
        left_hip = leg_info["left"]["hip_pos"]
        left_knee = leg_info["left"]["knee_pos"]
        right_hip = leg_info["right"]["hip_pos"]
        right_knee = leg_info["right"]["knee_pos"]
        return left_hip, left_knee, right_hip, right_knee

    def get_leg_vel(self) -> LegInfoType:
        leg_info = self.get_leg_info()
        left_hip = leg_info["left"]["hip_vel"]
        left_knee = leg_info["left"]["knee_vel"]
        right_hip = leg_info["right"]["hip_vel"]
        right_knee = leg_info["right"]["knee_vel"]
        return left_hip, left_knee, right_hip, right_knee

    def _angle_norm(self, pos1: Union[List[float], LegInfoType], pos2: Union[List[float], LegInfoType]) -> float:
        return np.linalg.norm(np.array(pos1) - np.array(pos2))  # type: ignore

    def set_one_motor_vel(self, motor_id: int, vel: int):
        assert self.control_mode == "velocity"
        raise NotImplementedError

    def set_all_motors_vel(self, vel: list):
        """set all motor position

        Args:
            vel (list): the desire position of all motors, unit: rpm,
                        order: [left hip, left knee, right hip, right knee]
        """
        assert self.control_target == "all"
        assert self.control_mode == "velocity"
        assert len(vel) == 4
        for v in vel:
            assert self._vel_limit[0] <= v <= self._vel_limit[1]

        byte_tuple = [from_int_to_16bit(i, self._vel_download_factor) for i in vel]

        msg = bytearray([0xAA, 13, 0xA5])
        for high_byte, low_byte in byte_tuple:
            msg += bytearray([high_byte, low_byte])
        msg += bytearray([0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

    def set_one_motor_current(self, motor_id: int, current: int):
        raise NotImplementedError

    def set_all_motor_current(self, current: list):
        raise NotImplementedError

    def change_control_mode(self, mode: str):
        assert mode in ["position", "velocity", "current"]

        mode_idx = -1
        if mode == "position":
            mode_idx = 0
        elif mode == "velocity":
            mode_idx = 1
        elif mode == "current":
            mode_idx = 2
        assert mode_idx in [0, 1, 2]

        msg = bytearray([0xAA, 0x6, 0xA8, mode_idx, 0x00, 0xBB])
        msg[-2] = check_sum(msg)
        self._send_message(msg)

        self.control_mode = mode
        robot_state = self.get_robot_state()
        assert robot_state[6] == mode_idx and robot_state[9] == mode_idx and robot_state[12] == mode_idx and robot_state[15] == mode_idx

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

    def open_receive_info(self):
        self.is_get_info = True
        self.receive_info_thread = threading.Thread(target=self._recevice_info)
        self.receive_info_thread.setDaemon(True)
        self.receive_info_thread.start()

    def close_receive_info(self):
        self.is_get_info = False
        time.sleep(0.3)
        assert self.receive_info_thread.is_alive() is False

    def _recevice_info(self):
        while self.is_get_info:
            # Prevent the msg is too short, which will cannot be decoded.
            time.sleep(0.005)

            self.info_stack.extend(self._get_info())

            if len(self.info_stack) > self.info_stacksize:
                self.info_stack = self.info_stack[-self.info_stacksize :]

    def initial_check(self):
        self._check_commuintation()
        self._check_robot_state()

    def _process_leg_info(self, leg_info: bytes) -> dict:
        hip_current = from_16bit_to_int(leg_info[3], leg_info[4], 1 / self._current_factor)
        hip_vel = from_16bit_to_int(leg_info[5], leg_info[6], 1 / self._vel_upload_factor)
        hip_pos_incre = from_16bit_to_int(leg_info[7], leg_info[8], 1 / self._pos_factor)  # the increamental encoder value
        hip_pos_abs = from_16bit_to_int(leg_info[9], leg_info[10], 1 / self._pos_factor)  # the absolute encoder value
        assert abs(hip_pos_incre - hip_pos_abs) < 3, f"hip_pos_incre: {hip_pos_incre}, hip_pos_abs: {hip_pos_abs}"

        knee_current = from_16bit_to_int(leg_info[11], leg_info[12], 1 / self._current_factor)
        knee_vel = from_16bit_to_int(leg_info[13], leg_info[14], 1 / self._vel_upload_factor)
        knee_pos_incre = from_16bit_to_int(leg_info[15], leg_info[16], 1 / self._pos_factor)
        knee_pos_abs = from_16bit_to_int(leg_info[17], leg_info[18], 1 / self._pos_factor)
        assert abs(knee_pos_incre - knee_pos_abs) < 3, f"knee_pos_incre: {knee_pos_incre}, knee_pos_abs: {knee_pos_abs}"

        return {
            "hip_current": hip_current,
            "hip_vel": hip_vel if leg_info[2] == 0 else -hip_vel,  # Due to the encoder direction, right leg is opposite to left leg
            "hip_pos": hip_pos_incre,
            "knee_current": knee_current,
            "knee_vel": knee_vel if leg_info[2] == 0 else -knee_vel,  # Due to the encoder direction, right leg is opposite to left leg
            "knee_pos": knee_pos_incre,
        }

    def get_leg_info(self) -> dict:
        has_left_leg_info = False
        has_right_leg_info = False
        left_info = None
        right_info = None

        assert len(self.info_stack) > 0, "The info stack is empty, please open the receive info thread first."

        for info in reversed(self.info_stack):
            if info[2] == 0x01:
                has_right_leg_info = True
                right_info = self._process_leg_info(info)
            elif info[2] == 0x00:
                has_left_leg_info = True
                left_info = self._process_leg_info(info)

            if has_left_leg_info and has_right_leg_info:
                break

        else:  # Be Careful, this branch will be executed if the for loop is not breaked.
            raise Exception(f"Cannot get leg info. has_left_leg_info: {has_left_leg_info}, has_right_leg_info: {has_right_leg_info}")

        # TODO: should i clear the info stack?

        return {"left": left_info, "right": right_info}

    def get_robot_state(self, timeout: float = 0.1) -> bytes:
        self.change_communication_state("open")

        time.sleep(timeout)
        if self.is_get_info:
            robot_state = self.info_stack[-1]
        else:
            robot_state = self._get_info()[-1]

        assert robot_state[2] == 2, f"Cannot get robot state, current info_num is {robot_state[2]}, is_get_info: {self.is_get_info}, info_stack: {self.info_stack[-10:]}"

        self.change_communication_state("close")

        time.sleep(timeout)
        if self.is_get_info:
            tmp = self.info_stack[-1]
        else:
            tmp = self._get_info()[-1]
        assert tmp[2] in [0, 1], f"Cannnot get leg state, current info_num is {tmp[2]}, is_get_info: {self.is_get_info}"

        return robot_state

    def _check_robot_state(self):
        robot_state = self.get_robot_state()[3]
        if robot_state != 0:
            raise Exception(f"robot state is abnormal, state: {robot_state}")

    def _check_commuintation(self):
        time.sleep(0.5)

        info = self._get_info()
        if len(info) == 0:
            raise Exception("Communication Error, there is no valid message in serial buffer")

    def _get_info(self):
        prefix = b'\xaa'
        suffix = b'\xbb'
        info_size = 21

        info = self.ser.read_all()
        assert info is not None and len(info) > 0, "Cannot get info from serial port"
        info = info.split(prefix)
        info = [prefix + i for i in info if len(i) == info_size - 1]
        info = [i for i in info if i[-2] == check_sum(i)]

        valid_info = [i for i in info if i.endswith(suffix)]

        return valid_info
