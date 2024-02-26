import numpy as np
import math

def getAngles(x, y, z, a = 53, b = 79.5, c = 30.5):
    gamma = round(math.atan2(y, x) * 180 / np.pi, 2)

    v = math.sqrt(x**2 + y**2) - c
    d = math.sqrt(v**2 + z**2)

    alpha1 = math.atan(z/v)
    alpha2 = math.acos((a**2 + d**2 - b**2)/(2*a*d))
    alpha = round(np.rad2deg(alpha1 + alpha2), 2)

    beta = round(np.rad2deg(math.acos((a**2 + b**2 - d**2)/(2*a*b))), 2)

    return gamma, alpha, beta



print(getAngles(73, 25, -20))
#print(getAngles(100, 70, 15))
