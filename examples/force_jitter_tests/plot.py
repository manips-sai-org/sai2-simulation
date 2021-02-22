import numpy
import pandas as pd
from matplotlib import pyplot as plt

data = pd.read_csv('/Users/shameekg/src/sai2-simulation/bin/log.csv')
data = data.rename(columns=lambda x: x.strip())

plt.figure()
# plt.plot(data['time'], data['q0'], 'r')
# plt.plot(data['time'], data['q1'], 'g')
# plt.plot(data['time'], data['q2'], 'b')
# plt.figure()
# plt.plot(data['time'], data['dq0'], 'r')
# plt.plot(data['time'], data['dq1'], 'g')
# plt.plot(data['time'], data['dq2'], 'b')
# plt.figure()
# plt.plot(data['time'], data['ddq0'], 'r')
# plt.plot(data['time'], data['ddq1'], 'g')
# plt.plot(data['time'], data['ddq2'], 'b')
# plt.figure()
plt.plot(data['time'], data['force'])
# plt.figure()
# plt.plot(data['time'], data['contacts'])
plt.show()

