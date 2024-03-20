import math
import random as rd
pi=math.pi
rand=rd.random


def a(x) :
    return math.atan(x)/pi + 1/2

def b(x) :
    return 2 * pi / (pi + 2*math.atan(x))

def c(x) :
    return math.tan(pi * x/2)

def d(x) :
    return math.tan(pi * (x-0.5))

def fx(t) :
    return math.atan(t)/pi + 1/2

def test (n) :
    print(a(n))
    print(b(n))
    print(c(n))
    print(d(n))
#print(fx(1))

test(rand())



