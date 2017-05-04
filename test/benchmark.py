from __future__ import division, print_function
from convert import *
from itertools import count
import time


epsilon = 1e-10

generateMesh("../mesh/basic.geo",0.1)

fname = "../mesh/basic.msh"
lower, upper = gmshToVoro(fname,"voro.txt")

if(upper[2]-lower[2]<epsilon):
    # 2D
    lower[2] = 0
    upper[2] = 1

lower -= epsilon
upper += epsilon


t0 = time.time()
execVoropp("voro.txt", lower, upper)
t1 = time.time()
print("voropp executed in    %7.2f ms"%(1000.0*(t1-t0)))
volVoropp = readVolume("voro.txt.vol")

t0 = time.time()
execFEdensity(fname, "fedensity.txt")
t1 = time.time()
print("FEdensity executed in %7.2f ms"%(1000.0*(t1-t0)))
volFEdensity = readVolume("fedensity.txt")

maxnorm = np.max(np.abs(volVoropp-volFEdensity)/volVoropp)
print("Maximum deviation: %5.2f%%"%(100*maxnorm))
# assert eq(volVoropp, volFEdensity, epsilon), "Correctness violated"
# for i, v, f in zip(count(), volVoropp, volFEdensity):
#     if abs(v-f)>epsilon:
#         print("Node %d deviates: %f vs. %f"%(i,v,f))
