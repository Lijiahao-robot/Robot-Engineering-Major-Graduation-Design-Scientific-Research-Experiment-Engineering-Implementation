import numpy as np

def complementary_filter(acc_angle, gyro_rate, dt, alpha=0.98):
    return alpha*(gyro_rate*dt) + (1-alpha)*acc_angle

angle = complementary_filter(30, 0.5, 0.01)
print(angle)
