import numpy as np
import matplotlib.pyplot as plt


# exo = r"C:\Users\thewa\Desktop\AxoController\src\scs\raw_data\exo\data1668326196.902254.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\raw_data\scs\data1668326201.7407806.npy"

# exo = r"C:\Users\thewa\Desktop\AxoController\src\scs\raw_data\exo\data1668327240.648813.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\raw_data\scs\data1668327248.5605605.npy"

# exo = r"C:\Users\thewa\Desktop\AxoController\src\scs\raw_data\exo\data1668327385.7948885.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\raw_data\scs\data1668327391.717494.npy"

# exo = r"C:\Users\thewa\Desktop\AxoController\src\data1668330185.2697563.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\stimulate1668330236.3102915.npy"

# 5
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\stimulate1668330578.5892212.npy"
# exo = r"C:\Users\thewa\Desktop\AxoController\src\data1668330576.065527.npy"

# 4
# exo = r"C:\Users\thewa\Desktop\AxoController\src\data1668330490.1906183.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\stimulate1668330494.4346995.npy"

# 3
# exo = r"C:\Users\thewa\Desktop\AxoController\src\data1668330407.6536098.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\stimulate1668330414.0447302.npy"

# 2
# exo = r"C:\Users\thewa\Desktop\AxoController\src\data1668330296.1072989.npy"
# scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\stimulate1668330302.1949542.npy"

exo = r"C:\Users\thewa\Desktop\AxoController\src\data1668330185.2697563.npy"
scs = r"C:\Users\thewa\Desktop\AxoController\src\scs\stimulate1668330236.3102915.npy"


exo = np.load(exo)
scs = np.load(scs)
scs[:, 1] = scs[:, 1] * 30

exo_precents = exo.shape[0] // 1
scs_precents = scs.shape[0] // 1

plt.plot(scs[-scs_precents:, 0], scs[-scs_precents:, 1], label="scs")
plt.plot(exo[-exo_precents:, 0], exo[-exo_precents:, 3], label="exo")

plt.vlines(1668330179.2191021, -50, 50, colors="r", linestyles="dashed")
# plt.vlines(1668330571.88, -50, 50, colors="b")
# plt.vlines(1668330571.79, -50, 50, colors="b", linestyles="dashed")

plt.xlabel("time / s")
plt.ylabel("Amplitude / deg for exo, 30*V for scs")

plt.ylim(-50, 50)
plt.legend()
plt.show()
