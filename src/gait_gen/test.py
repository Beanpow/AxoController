import numpy as np
import matplotlib.pyplot as plt


angleList = []

with open('./gait.csv', mode='r', encoding='utf-8-sig') as f:
    for line in f:
        line = line.split('\n')[0].split(',')
        line = [float(i) for i in line]
        angleList.append(line)

angleList = np.array(angleList)
print(angleList.shape)

if 0:
    plt.plot(angleList[:, 0], label='hl')
    plt.plot(angleList[:, 1], label='kl')
    # plt.plot(angleList[:, 2], label='hr')
    # plt.plot(angleList[:, 3], label='kr')
    plt.legend()
    plt.show()

with open('./gait_predict.csv', mode='w', encoding='utf-8-sig') as f:
    for i in range(angleList.shape[0]):
        f.write(str(angleList[i, 1] / 2) + ', ')
    f.write('\n')
