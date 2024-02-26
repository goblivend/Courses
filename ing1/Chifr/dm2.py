import numpy as np
from numpy.polynomial import Polynomial as Pol

g = Pol([ 1, -1,  0,  0,  1, -1, -1,  1,])
r = g
h = Pol([-184,   42, -106,   99, -100,  273,  221,  152,   96,   11,   49,])
N = Pol([-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,])
p = 29
q = 553


f = Pol([-1,  1,  1,  0,  1, -1, -1,  0,  1,])
Fp = Pol([24,  7, 24, 17, 11,  7, 19, 19,  3,  8,  7,])
### Centered version of Polynomial modulo p

def centerPol(a,p):
    """ Centers a polynomail a around ]-p/2 , p/2] """
    b=[]

        ### FIX ME
    for i in range(len(a.coef)):
        val = a.coef[i]%p
        b.append(val if val <= p/2 else val-p)


    return Pol(b)

### Decentering a polynomial

def deCenterPol(a,p):
    """ Decenters a centered polynomial a,
    returns a polynomail with coefficients in [0,p[ """
    b=[]

    ### DONE
    for i in range(len(a.coef)):
        val = a.coef[i]%p
        b.append(val if val >= 0 else val + p)

    return Pol(b)

### Recovering a text message from a centered polynomial

def recoverTextFromPol(a,p)->str:
    """ Recovers a text message from a centered polynomial a,
    p is the number around which the coefficients are centered """

    m=""
    newA = deCenterPol(a, p)
    for c in newA.coef :
        m += chr(ord('A') + int(c) - 1)

    ### FIXME
    return m

### Creates a centered polynomial around p from a text message

def createPolFromText(s:str,p:int):
    """ Creates a centered polynomial p from a string s of capital letters
        Each coefficient corresponds to the alphabetical order of a letter
        A -> 1, B ->2 etc
        Start with a non centered version :
        For example OP corresponds to the polynomial 15+16X
        Then return the centered version, for 29 that would be -14-13X  """

    ### FIXME
    pol = []
    for c in s :
        pol.append(ord(c) - ord('A') + 1)

    return centerPol(Pol(pol), p)

### NTRU encrypt
def encrypt(m:str,r,h,N,p,q):
    """ encrypts a text message m using NTRU
        The following arguments are already defined
        r is the temporary polynomial
        h the public key
        N is the polynomial X^N-1
        p, q integers
    """

    ### FIXME

    pol = createPolFromText(m, p) % N

    pol += (((p% N) * r) % N) * h
    res = centerPol(pol % N, q)
    # print(res.coef)
    return res


### NTRU decrypt
def decrypt(c,f,Fp,N,p,q):
    """ Recovers a text message from a polynomail c
    representing the cyphered message """

    ### FIXME
    pol = f*c

    pol = centerPol(pol % N, q)
    m = Fp * pol
    m %= N

    m = deCenterPol(m, p) % N

    size = len(m.coef)
    for i in range(len(m.coef) - 1, -1, -1) :
        if m.coef[i] != 0 :
            break
        size -= 1

    return "" if size == 0 else recoverTextFromPol(m, p )[:size]


print(decrypt(encrypt("EPITA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("EPLTA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("EPITA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("NPITA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("EAITA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("EPSTA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("EPIKA",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("EPITO",r,h,N,p,q),f,Fp,N,p,q))
print(decrypt(encrypt("NASKO",r,h,N,p,q),f,Fp,N,p,q))
