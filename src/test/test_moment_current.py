import numpy as np

import matplotlib.pyplot as plt

data = np.load("../moment_current.npy")
print(data.shape)

# current: left hip left knee, right hip, right knee


plt.plot(data[:, 1])
plt.plot(data[:, 7] * 4 + 18)
plt.show()
