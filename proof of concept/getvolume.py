from dolfin import *
import time

mesh = Mesh("../mesh/sphere.xml")

t0 = time.time()
volume = assemble(1*dx(mesh))
print volume
print time.time()-t0
