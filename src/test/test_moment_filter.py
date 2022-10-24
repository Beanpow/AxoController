import matplotlib.pyplot as plt
import numpy as np

from scipy import signal


data = np.load("./safe_moment.npy")
data = data[:, 4]

filted = signal.savgol_filter(data, 51, 3)
print(type(filted))


plt.plot(data, label="M1")
plt.plot(filted, label="M1_filted")
plt.show()
