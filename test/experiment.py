from __future__ import division, print_function
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import time

def logfunc(x, alog, alin, b):
    return alog*x*np.log(x) + alin*x + b

def linfunc(x, alin, b):
    return alin*x + b

N = 10000
ns = np.array(range(1,N))

def f1(n):
    test = np.random.randn(n)

def f2(n):
    test = np.random.randn(n).sort()

t1 = []
t2 = []

runs = 1

for n in ns:
    print(n)
    ta = time.time()
    for i in range(runs):
        f1(n)
    tb = time.time()
    t1.append((tb-ta)*1000.0/runs)

    ta = time.time()
    for i in range(runs):
        f2(n)
    tb = time.time()
    t2.append((tb-ta)*1000.0/runs)

u = N
popt1, pcov = curve_fit(logfunc, ns[:u], t1[:u])
print("Standard deviation of logarithmic fit to f1:", np.sqrt(np.diag(pcov)))
popt1l, pcov = curve_fit(linfunc, ns[:u], t2[:u])
print("Standard deviation of linear fit to f1:", np.sqrt(np.diag(pcov)))
popt2, pcov = curve_fit(logfunc, ns[:u], t2[:u])
print("Standard deviation of logarithmic fit to f2:", np.sqrt(np.diag(pcov)))
popt2l, pcov = curve_fit(linfunc, ns[:u], t2[:u])
print("Standard deviation of linear fit to f2:", np.sqrt(np.diag(pcov)))
print(popt1)
print(popt2)

if(10*popt1[0]<popt1[1]):
    print('f1 is O(n)')
else:
    print('f1 is O(n*log n)')

if(10*popt2[0]<popt2[1]):
    print('f2 is O(n)')
else:
    print('f2 is O(n*log n)')

# plt.plot(ns, t1, 'o', label='f1')
plt.plot(ns, t2, '-', label='f2')
# plt.plot(ns, logfunc(ns, *popt1), label='Log curve fit of f1')
plt.plot(ns, logfunc(ns, *popt2), ':', label='Log curve fit of f2')
# plt.plot(ns, linfunc(ns, *popt1l), label='Linear curve fit of f1')
plt.plot(ns, linfunc(ns, *popt2l), '--', label='Linear curve fit of f2')
plt.xlabel('Sample size')
plt.ylabel('Execution time [ms]')
plt.title('Numerical measurement experiment')
plt.legend(loc='upper left')
plt.grid()
plt.show()
