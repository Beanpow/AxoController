# -*- encoding: utf-8 -*-
'''
@File    :   SafetyController.py
@Time    :   2022/10/16 22:22:39
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''

import time
from tqdm import tqdm
from typing import Callable
import numpy as np
from scipy import signal

from InfoPlottor import InfoPlottor
from AxoController import AxoController
from MomentManager import MomentManager


class SafetyController:
    def __init__(self, axo_port: str, moment_port: str, trajectory: tuple[list[list[float]], list[list[float]]], isPlot: bool = True) -> None:
        self.trajectory = trajectory
        self.one_cycle_time = 4  # unit: second

        self.moment_menager = MomentManager(port=moment_port)
        self.axo_controller = AxoController(port=axo_port)
        self.isPlot = isPlot
        if self.isPlot:
            self.info_plottor = InfoPlottor(axo_controller=self.axo_controller)
            self.info_plottor.open_plot_info()

        self.safe_moment_raw = []
        self.safe_moment = None

    def record_safe_info(self, cycle_num: int):
        self.axo_controller.enter_control_mode()
        time.sleep(4)
        self.axo_controller.exit_control_mode()

        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_vel([0, 0, 0, 0])
        self.axo_controller.enter_control_mode(isChangeMode=False)
        self.axo_controller.set_all_motors_pos_vel_based_sync(self.trajectory[0][0])

        with tqdm(total=cycle_num) as pbar:
            for _ in range(cycle_num):
                self.run_one_cycle(callback_func=self.record_callback)
                pbar.update(1)

        self.process_raw_data()

        if self.axo_controller.in_control_mode:
            self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def record_callback(self, indx: int) -> bool:
        moments = self.moment_menager.get_all_moments()[0]
        moments = np.append(indx, moments)

        assert isinstance(self.safe_moment_raw, list)
        self.safe_moment_raw.append(moments)

        return True

    def process_raw_data(self):
        self.safe_moment_raw = np.array(self.safe_moment_raw)
        assert self.safe_moment_raw.shape[0] > 0

        max_indx = int(max(self.safe_moment_raw[:, 0]) + 1)
        indx_set = [[] for i in range(max_indx)]
        for i in range(self.safe_moment_raw.shape[0]):
            indx_set[int(self.safe_moment_raw[i, 0])].append(i)

        safe_moment = []
        for indx in indx_set:
            if len(indx) == 0:
                print("[error]: exist empty moment in some indx")

            raw_moment = self.safe_moment_raw[indx, 1:]  # type: ignore
            mean = np.mean(raw_moment, axis=0)
            std = np.std(raw_moment, axis=0)

            safe_moment.append(np.hstack((mean, std)))

        safe_moment = np.array(safe_moment)
        for i in range(safe_moment.shape[1]):
            safe_moment[:, i] = signal.savgol_filter(safe_moment[:, i], 51, 3)

        self.safe_moment = safe_moment

        np.save("safe_moment.npy", self.safe_moment)

    def run_one_cycle(self, callback_func: Callable):
        traj_pos, traj_vel = self.trajectory
        one_step_time = self.one_cycle_time / len(traj_pos)
        for idx, (target_pos, target_vel) in enumerate(zip(traj_pos, traj_vel)):
            start_time = time.time()
            if not self.axo_controller.in_control_mode:
                break

            if self.isPlot:
                self.info_plottor.set_target_pos(target_pos)

            if not callback_func(idx):
                break

            self.axo_controller.set_all_motors_pos_vel_based(target_pos, target_vel)

            assert time.time() - start_time <= one_step_time, "one step time is too short"
            time.sleep(one_step_time - (time.time() - start_time))

    def detect_callback(self, indx: int) -> bool:
        moments = self.moment_menager.get_all_moments()[0]
        assert self.safe_moment is not None

        factor = 4

        low_limitation = self.safe_moment[indx, 0:4] - factor * self.safe_moment[indx, 4:8]
        high_limitation = self.safe_moment[indx, 0:4] + factor * self.safe_moment[indx, 4:8]

        if np.any(moments < low_limitation) or np.any(moments > high_limitation):
            which_motor = np.where((moments < low_limitation) | (moments > high_limitation))
            which_motor = int(which_motor[0])
            print(
                f"[error]: moment is unsafe, motor_id: {which_motor}, moments: {moments[which_motor]}, low_limitation: {low_limitation[which_motor]}, high_limitation: {high_limitation[which_motor]}"
            )
            self.axo_controller.exit_control_mode()
            return False

        return True

    def run_with_detection(self, cycle_num: int):
        self.axo_controller.enter_control_mode()
        time.sleep(4)  # wait for the robot reset to the initial position

        self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("velocity")
        self.axo_controller.set_all_motors_vel([0, 0, 0, 0])
        self.axo_controller.enter_control_mode(isChangeMode=False)
        self.axo_controller.set_all_motors_pos_vel_based_sync(self.trajectory[0][0])

        with tqdm(total=cycle_num) as pbar:
            for _ in range(cycle_num):
                if not self.axo_controller.in_control_mode:
                    break

                self.run_one_cycle(self.detect_callback)
                pbar.update(1)

        if self.axo_controller.in_control_mode:
            self.axo_controller.exit_control_mode()
        self.axo_controller.change_control_mode("position")

    def close(self):
        if self.isPlot:
            self.info_plottor.close_plot_info()

        self.axo_controller.close_controller()
