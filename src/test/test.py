import sys

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


def main():
    port = "COM3"
    axo_ctrl = AxoController(port=port, angle_telorance=[20, 60, 80, 5])

    axo_ctrl.set_all_motors_pos_sync([10, 0, 10, 0])

    axo_ctrl.set_all_motors_pos_sync([40, -30, 40, -30])


if __name__ == "__main__":
    main()
