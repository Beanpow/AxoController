import numpy as np
import matplotlib.pyplot as plt

data = np.load("../safe_moment.npy")


print(data.shape)

mean_nums = data.shape[1] // 2
motor_name = ["Left Hip", "Left Knee", "Right Hip", "Right Knee"]

for i in range(mean_nums):
    plt.subplot(241 + i)
    plt.plot(data[:, i], "--")
    plt.fill_between(range(data.shape[0]), data[:, i] - data[:, i + mean_nums], data[:, i] + data[:, i + mean_nums], alpha=0.2)
    plt.title(motor_name[i % 4])

    if i == 0:
        plt.ylabel("Moment")

    if i == 4:
        plt.ylabel("Current")

plt.show()
