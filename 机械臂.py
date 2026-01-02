import math

def ik(x, y, L1=10, L2=10):
    d = math.sqrt(x*x + y*y)
    a = math.acos((L1*L1 + d*d - L2*L2) / (2*L1*d))
    b = math.atan2(y, x)
    t1 = b - a
    t2 = math.acos((L1*L1 + L2*L2 - d*d) / (2*L1*L2))
    return math.degrees(t1), math.degrees(t2)

print(ik(12, 8))
