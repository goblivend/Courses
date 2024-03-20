import math

e = math.exp(1)
floor = math.floor
ln = math.log

def test(x) :
    a = e * (math.exp(floor(ln(x))) - 1)/ (e - 1)
    print(f'a : {a}')
    b = floor(x * ln(x) - x)
    print(f'b : {b}')
    c = math.exp(x-floor(x)) + (math.exp(floor(x)) - 1)/(e-1)
    print(f'c : {c}')
    d = x * floor(ln(x)) - e * (math.exp(floor(ln(x))) - 1) / (e-1)
    print(f'd : {d}')

test(3)
