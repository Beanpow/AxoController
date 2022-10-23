import sys
import time

sys.path.append("..")

from MomentManager import MomentManager  # noqa


def main():
    m = MomentManager(port="COM5")
    while 1:
        start = time.time()
        print(m.get_all_moments(), time.time() - start)


if __name__ == "__main__":
    main()
