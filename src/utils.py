# -*- encoding: utf-8 -*-
'''
@File    :   utils.py
@Time    :   2022/10/12 22:02:59
@Author  :   Beanpow
@Version :   1.0
@Contact :   beanpow@gmail.com
@Desc    :   None
'''
from typing import Union


def check_sum(msg: bytearray) -> int:
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
