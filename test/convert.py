from __future__ import division, print_function
import numpy as np
import subprocess as sp
import os

def gmshToVoro(fInName, fOutName):
    """
    Takes a gmsh file named fInName in and translates it to a Voro++ file named
    fOutName. Return lower and upper coordinate of a rectangular bounding box
    exactly encapsulating the whole domain. If the domain is rectangular this
    will specify the size of the domain.
    """

    with open(fInName, 'r') as fIn:
        with open(fOutName, 'w') as fOut:

            lower = 3*np.array([float("inf")])
            upper = 3*np.array([float("-inf")])

            while fIn.readline().strip() != "$Nodes": pass
            nNodes = int(fIn.readline().strip())

            for i in range(nNodes):
                line = fIn.readline()
                fOut.write(line)

                coords = np.fromstring(line, sep=' ')[1:]
                lower = np.minimum(lower, coords)
                upper = np.maximum(upper, coords)

            return lower, upper

def execVoropp(fInName, lower, upper):
    cmd = os.path.join(os.path.dirname(os.path.realpath(__file__)),"voro++hex")
    cmd += " -o"
    for i in range(3): cmd += " %s %s"%(lower[i].hex(),upper[i].hex())
    cmd += " voro.txt"
    sp.call(cmd, shell=True)

def execGiraffe(fInName, fOutName):
    cmd = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../giraffe")
    cmd += " " + fInName + " " + fOutName
    devnull = open(os.devnull, 'w')
    sp.call(cmd, shell=True, stdout=devnull)

def readVolume(fInName):
    with open(fInName, 'r') as fIn:
        return np.array([float.fromhex(line.split()[-1]) for line in fIn])

def eq(a, b, tol):
    return all(np.abs(np.array(a)-np.array(b))<tol)

def generateMesh(fName, sz, cached=True):
    pathOfThisFile = os.path.dirname(os.path.realpath(__file__))
    cachePath = os.path.join(pathOfThisFile, "cache")

    fOutName = os.path.join(cachePath, "%e.msh"%sz)

    if (not os.path.isfile(fOutName)) or cached==False:
        cmd = "gmsh -setnumber sz %e"%sz
        cmd += " -o %s"%fOutName
        cmd += " -3 %s"%os.path.join(pathOfThisFile, fName)
        sp.call(cmd, shell=True)

    return fOutName
