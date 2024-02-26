def inv_mod(a, n):
    u1, v1 = 1, 0
    u2, v2 = 0, 1
    r1, r2 = a, n

    q, r = r1 // r2, r1 % r2

    while r>0:
        u1, v1, u2, v2 = u2, v2, u1 - q*u2, v1 - q*v2

        r1, r2 = r2, r
        q, r = r1 // r2, r1 % r2
    if r2==1:

        return u2%n

    raise ValueError("The modular inverse does not exist")

class EllipticCurve():

    def __init__(self, a, b, p):
        self.a = a
        self.b = b
        self.p = p

    def __eq__(self, C):
        return (self.a, self.b) == (C.a, C.b)

    def has_point(self, x, y):
        return (y ** 2) % self.p == (x ** 3 + self.a * x + self.b) % self.p

    def __str__(self):
        return 'y^2 = x^3 + {}x + {}'.format(self.a, self.b)

    def __len__(self):
        """ Number of points including infinity """
        return len(self.points())+1

    def points(self) :
       points = []

       # DONE
       for x in range(self.p):
            for y in range(self.p):
                if self.has_point(x,y):
                    points.append((x,y))

       return points

    def print_points(self):
       for point in self.points():
           print(f"({point[0]},{point[1]})")

    def enc(self,P,sP,k,M):
        """ ElGamal encrypting over elliptic curve
            P point on the elliptic curve,
            sP=B point, public key
            k private key
            M message to encode
            returns the encrypted message :a list of coordinates of two points on the curve
        """

        #FIXME
        c1 = k*P
        c2 = M + k*sP
        return [(c1.x, c1.y), (c2.x, c2.y)]


    def dec(self,M1,M2,s):
        """ ElGamal decrypting
            M1,M2 : encrypted message
            s private key
        """
        #FIXME
        return M2 - s*M1

class Point(object):
    def __init__(self, curve, x, y):
        self.curve = curve
        self.x = x % curve.p
        self.y = y % curve.p

        if not self.curve.has_point(x, y):
            raise ValueError('{} is not on curve {}'.format(self, self.curve))

    def __str__(self):
        return '({}, {})'.format(self.x, self.y)

    def __getitem__(self, index):
        return [self.x, self.y][index]

    def __eq__(self, Q):
        return (self.curve, self.x, self.y) == (Q.curve, Q.x, Q.y)

    def __neg__(self):
        return Point(self.curve, self.x, -self.y)

    def __add__(self, Q):
        # DONE
        # Case 0.0: if Q is the point at infinity, return self
        if isinstance(Q, Inf) :
            return self

        if isinstance(self, Inf) :
            return Q

        m = 0
        if self.x == Q.x and self.y != Q.y:
            return Inf(self.curve)

        if self == Q :
            if self.y == 0 :
                return Inf(self.curve)
            m = (3 * self.x ** 2 + self.curve.a) * inv_mod(2 * self.y, self.curve.p)
        else :
            m = (Q.y - self.y) * inv_mod(Q.x - self.x, self.curve.p)

        x = m ** 2 - self.x - Q.x
        y = m * (self.x - x) - self.y
        return Point(self.curve, int(x), int(y))

    def __sub__(self,Q):
        # DONE
        # If Q is the point at infinity, call add on Q
        if isinstance(Q, Inf):
            return Q + self
        return self + Point(self.curve, Q.x, -Q.y)

    def __str__(self):
        if self != Inf(self.curve):
            return '({}, {})'.format(self.x, self.y)

        else:
            return "O"


    def __mul__(self, n):
        # DONE
        if n == 0 :
            return Inf(self.curve)
        res = self
        for _ in range(n-1):
            res = res + self
        return res


    def __rmul__(self, n):
        return self * n



class Inf(Point):
    """The custom infinity point."""
    def __init__(self, curve):
        self.curve = curve

    def __eq__(self, Q):
        return isinstance(Q, Inf)

    def __neg__(self):
        """-0 = 0"""
        return self

    def __add__(self, Q):
        """P + 0 = P"""
        return Q
