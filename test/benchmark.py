from __future__ import division, print_function
from convert import *
from itertools import count
import time
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit

def func(x, alog, alin, b):
    return alog*x*np.log(x) + alin*x + b

epsilon = 1e-10

sizes = 0.1*np.array([(1./i)**(1./3.) for i in range(1,17)])

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

    # t0 = time.time()
    # execFEdensity(fname, "fedensity.txt")
    # t1 = time.time()
    # t = 1000.0*(t1-t0)
    # print("FEdensity executed in %7.2f ms"%t)
    # timeFEdensity.append(t)
    # volFEdensity = readVolume("fedensity.txt")

    # maxnorm = np.max(np.abs(volVoropp-volFEdensity)/volVoropp)
    # print("Maximum deviation: %5.2f%%"%(100*maxnorm))

    nNodes.append(len(volVoropp))

print(sizes)
print(nNodes)

popt, pcov = curve_fit(func, nNodes, timeVoropp)
print(popt)

alin = (timeVoropp[1]-timeVoropp[0])/(nNodes[1]-nNodes[0])
alog = (timeVoropp[1]-timeVoropp[0])/(nNodes[1]*np.log(nNodes[1])-nNodes[0]*np.log(nNodes[0]))
blin = timeVoropp[0]-alin*nNodes[0];
blog = timeVoropp[0]-alog*nNodes[0]*np.log(nNodes[0]);
x = np.linspace(nNodes[0],nNodes[-1],100)
ylin = alin*x+blin;
ylog = alog*x*np.log(x)+blog;

plt.plot(nNodes, timeVoropp, 'o', label='Voro++')
plt.plot(x,ylin, label='O(n)')
plt.plot(x,ylog, label='O(n*log n)')
plt.plot(x,func(x,*popt), label='curve fit')
# plt.plot(nNodes, timeFEdensity, 's', label='FEdensity')
plt.grid()
plt.xlabel('Number of nodes')
plt.ylabel('Execution time [ms]')
plt.title('Computation of Voronoi volumes')
plt.legend(loc='upper left')
plt.show()
