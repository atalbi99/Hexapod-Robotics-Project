import math
from constants import *
from scipy.optimize import minimize
import numpy as np
import interpolation
import time

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=-1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def computeDKDetailed(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = computeDK(
        theta1_verif, theta2_verif, theta3_verif, l1, l2, l3, use_rads, use_mm
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=-1,
    use_mm=USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        None
    return result


def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle

#return the vector (x,y,z) with a teta rotation in the x,y plan
def rotaton_2D(x, y, z, teta):
    vect = np.dot(np.array([[np.cos(teta), -np.sin(teta)],
                            [np.sin(teta), np.cos(teta)]]), np.array([x, y]))
    return [vect[0], vect[1], z]


# Computes the inverse kinematics of the leg leg_id in the body's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# and an angle teta that chage the direction of the movement
# returns the angles to apply to the 3 axes
def computeIKOriented(
    x,
    y,
    z,
    leg_id,
    params,
    teta,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=-1,
    use_mm=USE_MM_INPUT,
):


    [x,y,z] = rotaton_2D(x, y, z, modulopi(params.legAngles[leg_id-1] + teta))
    x += params.initLeg[leg_id-1][0]
    y += params.initLeg[leg_id-1][1]
    z += params.z

    return computeIK(
        x,
        y,
        z,
        l1,
        l2,
        l3,
        verbose,
        use_rads,
        sign,
        use_mm,
        )

def computeIKCenter(x, y, z, legID, verbose=False):
    x -= LEG_CENTER_POS[legID - 1][0]
    y -= LEG_CENTER_POS[legID - 1][1]
    z -= LEG_CENTER_POS[legID - 1][2]

    [x,y,z] = rotaton_2D(x, y, z, -LEG_ANGLES_2[legID - 1])

    return computeIK(x, y, z, verbose=verbose, use_rads=True)


def set_leg_angles_2(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]

#used to compute the position
#TODO: change names (horrible), and use integers insteads of arrays
position = [0,0]
triPos1 = [0,0,0]
triPos2 = [0,0,0]


def walk(freq, params, targets, teta, length, height, body):
    global triPos1
    global triPos2
    global position
    t = time.time()
    if freq == 0:
        tri1 = [0,0,0]
        tri2 = [0,0,0]
    else:
        T = 1 / freq
        triangle = interpolation.LinearSpline3D()
        triangle.add_entry(0, length / 2, 0,  0)
        triangle.add_entry(T / 2, -length / 2, 0, 0)
        triangle.add_entry(3 * T / 4, 0, 0, height)
        triangle.add_entry(T, length / 2, 0,  0)

        tri1 = triangle.interpolate(t % T)
        tri2 = triangle.interpolate((t + T/2) % T)

        #Update Position
        if tri1[2] == 0:
            triPos2 = [length / 2, 0, 0]
            previousTri = triPos1
            triPos1 = tri1
            position[0] -= (triPos1[0] - previousTri[0]) * np.cos(teta)
            position[1] -= (triPos1[0] - previousTri[0]) * np.sin(teta)
        else:
            triPos1 = [length / 2, 0, 0]
            previousTri = triPos2
            triPos2 = tri2
            position[0] -= (triPos2[0] - previousTri[0]) * np.cos(teta)
            position[1] -= (triPos2[0] - previousTri[0]) * np.sin(teta)



    for leg_id in [1,3,5]:
        alphas = computeIKOriented(tri1[0], tri1[1], tri1[2] - body, leg_id, params, teta, verbose=True)
        set_leg_angles_2(alphas, leg_id, targets, params)
    for leg_id in [2,4,6]:
        alphas = computeIKOriented(tri2[0], tri2[1], tri2[2] - body, leg_id, params, teta, verbose=True)
        set_leg_angles_2(alphas, leg_id, targets, params)

def toupie(t, freq, d_x, d_y, height, z):

    if freq == 0:
        tri1 = [0,0,0]
        tri2 = [0,0,0]

    else:
        T = 1 / freq
        triangle = interpolation.LinearSpline3D()

        triangle.add_entry(0, d_x, 0, z/2)
        triangle.add_entry(T / 2, d_x, 0, z)
        triangle.add_entry((3 * T) / 4, d_x, -d_y, z)
        triangle.add_entry(T, d_x, 0, z+height)



        tri1 = triangle.interpolate(t % T)
        tri2 = triangle.interpolate((t + T/2) % T)

    return tri1, tri2

def getPosition():
    return (round(position[0], 1), round(position[1], 1))

