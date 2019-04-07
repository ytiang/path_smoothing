#!/usr/bin/env python
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

cwd = os.getcwd()

print
cwd

data = pd.read_csv(cwd + "/path.csv")
x = data["x"]
y = data["y"]

data2 = pd.read_csv(cwd + "/smooth_result.csv")
x2 = data2["x"]
y2 = data2["y"]

data3= pd.read_csv(cwd + "/smooth_result1.csv")
x3 = data3["x"]
y3 = data3["y"]
plt.plot(x,y, "r", x2, y2, "g", x3, y3)

# plt.plot(x, y, x2, y2)
plt.show()
