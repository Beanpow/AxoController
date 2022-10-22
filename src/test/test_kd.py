from scipy import signal
import numpy as np
import matplotlib.pyplot as plt


data = np.load('pid_res.npy')[:, 2]

print(data.shape)

b, a = signal.butter(8, 0.1, "lowpass")

his = []
for i in range(0, data.shape[0] - 100):
    fi = data[i : i + 100]
    fi = signal.filtfilt(b, a, fi)
    data[i : i + 100] = fi
    his.append(fi[-1])

plt.plot(his)
plt.plot(data[100:])


plt.show()
