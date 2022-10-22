import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

sys.path.append("..")
from utils import LiveLFilter, LiveSosFilter


data = np.load('pid_res1.npy')[:, 2]

b, a = signal.butter(1, 0.1, "lowpass")

sos = signal.iirfilter(2, Wn=1.5, fs=30, btype="low", ftype="butter", output="sos")
print(sos.shape)

live_sosfilter = LiveSosFilter(sos)

# live_lfilter = LiveLFilter(b, a)
filted = [live_sosfilter(d) for d in data]

lf = LiveLFilter(b, a)
fil = [lf(d) for d in data]

fi = [data[0]]
a = 0.05
for i in data[1:]:
    fi.append(a * i + (1 - a) * fi[-1])


plt.plot(data)
plt.plot(filted)
plt.plot(fi)
plt.plot(fil)
plt.legend(['raw', 'sos', 'exp', 'lfilter'])
plt.show()
