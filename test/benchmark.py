from __future__ import division, print_function
from convert import *
from itertools import count
import time
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit

def func(x, alog, alin, b):
    return alog*x*np.log(x) + alin*x + b

epsilon = 1e-10

sizes = 0.1*np.array([(1./i)**(1./3.) for i in range(1,6)])

# sizes = [0.3, 0.2, 0.1, 0.03, 0.04, 0.045, 0.048, 0.049, 0.0495, 0.05]
nNodes = []
timeVoropp = []
timeFEdensity = []

for sz in sizes:

    fname = generateMesh("../mesh/basic.geo",sz)
    lower, upper = gmshToVoro(fname,"voro.txt")

    if(upper[2]-lower[2]<epsilon):
        # 2D
        lower[2] = 0
        upper[2] = 1

    lower -= epsilon
    upper += epsilon


    t0 = time.time()
    N = 1
    for i in range(N):
        execVoropp("voro.txt", lower, upper)
    t1 = time.time()
    t = 1000.0*(t1-t0)/N
    print("voropp executed in    %7.2f ms"%t)
    timeVoropp.append(t)
    volVoropp = readVolume("voro.txt.vol")

    t0 = time.time()
    execFEdensity(fname, "fedensity.txt")
    t1 = time.time()
    t = 1000.0*(t1-t0)
    print("FEdensity executed in %7.2f ms"%t)
    timeFEdensity.append(t)
    volFEdensity = readVolume("fedensity.txt")

    maxnorm = np.max(np.abs(volVoropp-volFEdensity)/volVoropp)
    print("Maximum deviation: %5.2f%%"%(100*maxnorm))

    nNodes.append(len(volVoropp))

print(sizes)
print(nNodes)

# poptVoropp, pcov = curve_fit(func, nNodes, timeVoropp)
# poptFEdensity, pcov = curve_fit(func, nNodes, timeFEdensity)
#
# if(10*poptVoropp[0]<poptVoropp[1]):
#     print('Voro++ is O(n)')
# else:
#     print('Voro++ is O(n*log n)')
#
# if(10*poptFEdensity[0]<poptFEdensity[1]):
#     print('FEdensity is O(n)')
# else:
#     print('FEdensity is O(n*log n)')

plt.plot(nNodes, timeVoropp, 'o', label='Voro++')
plt.plot(nNodes, timeFEdensity, 's', label='FEdensity')
plt.grid()
plt.xlabel('Number of nodes')
plt.ylabel('Execution time [ms]')
plt.title('Computation of Voronoi volumes')
plt.legend(loc='upper left')
plt.show()
