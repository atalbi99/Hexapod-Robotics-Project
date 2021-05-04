import numpy as np
import matplotlib.pyplot as plt

def affine(x1,x2,y1,y2):
    a = (y2 - y1) / (x2 - x1)
    b = y1 - a * x1
    return lambda x : a * x + b

def find(x, l):
    s = len(l)
    if s == 0:
        return -1
    for i in range (s):
        if x <  l[i]:
            return i - 1
    return s - 1

def interpolation(xs, ys, x):
    l = len(xs)
    if x < xs[0]:
        return ys[0]
    elif x > xs[l-1]:
        return ys[l-1]
    else:
        i = find(x, xs)
        f = affine(xs[i], xs[i+1], ys[i], ys[i+1])
        return f(x)

class LinearSpline:
    def __init__(self):
        self.T = []
        self.X = []

    def add_entry(self, t, x):
        i = find(t, self.T)
        self.T.insert(i+1,t)
        self.X.insert(i+1,x)
        
    def interpolate(self, t):
        return interpolation(self.T, self.X, t)


class LinearSpline3D:
    def __init__(self):
        self.X = LinearSpline()
        self.Y = LinearSpline()
        self.Z = LinearSpline()

    def add_entry(self, t, x, y ,z):
        self.X.add_entry(t, x)
        self.Y.add_entry(t, y)
        self.Z.add_entry(t, z)

    def interpolate(self, t):
        return self.X.interpolate(t), self.Y.interpolate(t), self.Z.interpolate(t)