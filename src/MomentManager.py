import serial
import binascii
import sys
import numpy as np


class MomentManager:
    def __init__(self, port: str, byterate: int = 38400, timeout: float = 10) -> None:
        self.__reqListForPortList = [
            bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A]),
            bytes([0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39]),
            bytes([0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x85, 0xE8]),
            bytes([0x04, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x5F]),
        ]

        self.reqListForAllPort = bytes([0x01, 0x03, 0x01, 0xF4, 0x00, 0x08, 0x04, 0x02])

        try:
            self.ser = serial.Serial(port, byterate, timeout=timeout)
        except Exception as e:
            print(f"[error]: {e}")
            sys.exit()

        self.init_check()

    def init_check(self):
        assert self.get_all_moments()[0].shape == (4,)
        assert np.any(np.isnan(self.get_all_moments()[0])) is False

    def __del__(self):
        self.ser.close()

    def _SendMessage(self, req):
        self.ser.write(req)

        rawDataHex = self.ser.read(21)

        # parse received data
        parsedDataHex = binascii.b2a_hex(rawDataHex)
        parsedDataHex = str(parsedDataHex)
        parsedDataHex = parsedDataHex.split('\'')[1]
        return rawDataHex, parsedDataHex

    def get_all_moments(self):
        req = self.reqListForAllPort
        rawDataHex, parsedDataHex = self._SendMessage(req)

        try:
            assert len(parsedDataHex) == 42
        except AssertionError:
            print(rawDataHex)
            sys.exit()

        momentList = []
        momentList.append(int(parsedDataHex[6:14], 16))
        momentList.append(int(parsedDataHex[14:22], 16))
        momentList.append(int(parsedDataHex[22:30], 16))
        momentList.append(int(parsedDataHex[30:38], 16))

        for i in range(4):
            if momentList[i] > 2147483648:
                momentList[i] = momentList[i] - 4294967296
            momentList[i] /= 100

        return np.array(momentList), [1, 2, 3, 4], parsedDataHex
