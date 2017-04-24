"""
Copyright 2017 Sigvald Marholm <marholm@marebakken.com>

This file is part of FEdensity.

FEdensity is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

FEdensity is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
FEdensity. If not, see <http://www.gnu.org/licenses/>.
"""

from clipping import *
from clipping4 import *
from timeit import timeit

print(timeit("method1()", setup="from clipping import method1", number=1000))
print(timeit("method2()", setup="from clipping import method2", number=1000))
print(timeit("method3()", setup="from clipping4 import method3", number=1000))
