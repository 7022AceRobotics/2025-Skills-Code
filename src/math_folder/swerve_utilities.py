from constants import *
import math

def mps_to_rpm(mps):
    return mps / (VexSwerveModuleConstants.wheel_radius * math.pi * 2)

def rpm_to_mps(rpm):
    return rpm * (VexSwerveModuleConstants.wheel_radius * math.pi * 2)

def rpm_to_degps(rpm):
    return rpm * 360/60 # rpm/60 = rps, rps x 360 = deg/s

def degps_to_rpm(degps):
    return degps * 1/6

def make_rotations_universal(rotation):
    rotation = rotation % 360
    if rotation < 0:
        rotation = 360 + rotation
    return rotation

def optimize_rotation(current_position, desired_position, clockwise):
    current_position = make_rotations_universal(current_position)
    desired_position = make_rotations_universal(desired_position)

    if clockwise == 'forward':
        if abs(current_position - desired_position) > 180:
            return 'forward'
        else:
            return 'backward'
    else:
        if abs(current_position - desired_position) > 180:
            return 'backward'
        else:
            return 'forward'
        
def multiply_matrices(m1, m2):
    m3 = []
    for i in range(len(m1)):
        total = 0
        for j in range(len(m1[i])):
            total += m1[i][j] * m2[j][i]
        m3.append(total)
    return m3

def get_swerve_angle(x, y):
    hyp = (x**2 + y**2)**(1/2)
    if hyp > 0:
        x = x/ hyp
        y = y / hyp
    else:
        x = 1
        y = 0
    return math.atan2(x, y)