import sys

sys.path.append("..")
from AxoController import AxoController  # noqa: E402


def main():
    axo = AxoController(port="/dev/tty.usbserial-CHAIb11A920")


if __name__ == "__main__":
    main()
