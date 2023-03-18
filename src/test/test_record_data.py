import numpy as np
import matplotlib.pyplot as plt


data_path = "../logs/dyj_2023-03-16_15-48-02"

detected_info = np.load(data_path + "/detected_info.npy")

print(detected_info.shape)

for i in range(4):
    plt.plot(detected_info[:, 5 + i])
plt.show()
