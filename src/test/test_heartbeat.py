import sys
import time

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


def main():
    axo = AxoController(port="com3")
    axo.enter_control_mode()
    time.sleep(10)


if __name__ == "__main__":
    main()
