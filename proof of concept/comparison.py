from clipping import *
from clipping4 import *
from timeit import timeit

print(timeit("method1()", setup="from clipping import method1", number=1000))
print(timeit("method2()", setup="from clipping import method2", number=1000))
print(timeit("method3()", setup="from clipping4 import method3", number=1000))
