# -*- encoding: utf-8 -*-
'''
@File    :   utils.py
@Time    :   2022/10/12 22:02:59
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   The utils for the project
             The livefilter is from https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/
'''
from typing import Union
import time
from scipy import signal
from collections import deque
import numpy as np


class LiveFilter:
    """Base class for live filters."""

    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")


class LiveLFilter(LiveFilter):
    def __init__(self, b, a):
        """Initialize live filter based on difference equation.

        Args:
            b (array-like): numerator coefficients obtained from scipy.
            a (array-like): denominator coefficients obtained from scipy.
        """
        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a) - 1)

    def _process(self, x):
        """Filter incoming data with standard difference equations."""
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y


class LiveSosFilter(LiveFilter):
    """Live implementation of digital filter with second-order sections."""

    def __init__(self, sos):
        """Initialize live second-order sections filter.

        Args:
            sos (array-like): second-order sections obtained from scipy
                filter design (with output="sos").
        """
        self.sos = sos

        self.n_sections = sos.shape[0]
        self.state = np.zeros((self.n_sections, 2))

    def _process(self, x):
        """Filter incoming data with cascaded second-order sections."""
        for s in range(self.n_sections):  # apply filter sections in sequence
            b0, b1, b2, a0, a1, a2 = self.sos[s, :]

            # compute difference equations of transposed direct form II
            y = b0 * x + self.state[s, 0]
            self.state[s, 0] = b1 * x - a1 * y + self.state[s, 1]
            self.state[s, 1] = b2 * x - a2 * y
            x = y  # set biquad output as input of next filter section.

        return y


class PID:
    def __init__(self, kp: float = 1.0, ki: float = 0, kd: float = 0, setpoint: float = 0, output_limits: tuple[float, float] = (-float("inf"), float("inf"))) -> None:
        self.kp = kp
        self.ki = ki
        if self.ki != 0:
            raise NotImplementedError("ki is not implemented")
        self.kd = kd
        self.setpoint = setpoint

        self.output_limits = output_limits

        self._proportional = 0
        self._derivative = 0

        self.last_input = None
        self.last_error = None
        self.last_time = time.time()

        sos = signal.iirfilter(2, Wn=1, fs=30, btype="low", ftype="butter", output="sos")
        self.kp_filter = LiveSosFilter(sos)

        sos1 = signal.iirfilter(2, Wn=1, fs=30, btype="low", ftype="butter", output="sos")
        self.kd_filter = LiveSosFilter(sos1)

    def __call__(self, input_: float, vel_: float) -> float:
        now = time.time()
        dt = now - self.last_time if now - self.last_time else 1e-16

        error = self.setpoint - input_
        error = self.kp_filter(error)

        self._proportional = self.kp * error
        self.current_vel = (input_ - self.last_input) / dt if self.last_input else 0
        self.current_vel = self.kd_filter(self.current_vel)

        self._derivative = self.kd * (vel_ - self.current_vel)

        self.last_error = error
        self.last_input = input_
        self.last_time = now

        return max(min(self._proportional + self._derivative, self.output_limits[1]), self.output_limits[0])


def check_sum(msg: Union[bytearray, bytes]) -> int:
    assert len(msg) >= 5

    sum = msg[1]
    for i in range(2, len(msg) - 2):
        sum ^= msg[i]

    return sum


def from_int_to_16bit(value: Union[int, float], factor: int = 1) -> tuple[int, int]:
    """get value * factor's high byte and low byte

    Args:
        value (int | float): the value to be converted
        factor (int, optional): factor. Defaults to 1.

    Returns:
        tuple[int, int]: high byte, low byte
    """
    value = int(value * factor)

    value &= 0xFFFF
    high_byte = value >> 8
    low_byte = value & 0xFF

    return high_byte, low_byte


def from_16bit_to_int(high_byte: int, low_byte: int, factor: float) -> float:
    """get int from high byte and low byte

    Args:
        high_byte (int): high byte
        low_byte (int): low byte
        factor (int): factor

    Returns:
        int: int value
    """
    byte16 = (high_byte << 8) | low_byte
    if byte16 & 0x8000:
        byte16 ^= 0xFFFF
        byte16 += 1
        byte16 *= -1

    return byte16 * factor


def load_trj(file_path):
    trj = []
    with open(file_path, "r") as f:
        for line in f:
            line = line.split(',')
            trj.append([float(i) for i in line])
    trj_pos, trj_vel = trj[: len(trj) // 2], trj[len(trj) // 2 :]
    return trj_pos, trj_vel


def accurate_delay(delay):
    '''Function to provide accurate time delay in millisecond'''
    _ = time.perf_counter() + delay / 1000
    while time.perf_counter() < _:
        pass
