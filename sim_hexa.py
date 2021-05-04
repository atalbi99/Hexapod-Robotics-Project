#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation

import kinematics
from constants import *

from operator import add

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


class Parameters:
    def __init__(
        self,
        z=-0.06,
    ):
        self.z = z
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[6] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[5] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[2] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        self.legs[3] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[4] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4
params = Parameters()


if args.mode == "frozen-direct":
    crosses = []
    for i in range(4*6):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

elif args.mode == "walk":
    controls["teta"] = p.addUserDebugParameter("Direction", -math.pi, math.pi, 0)
    controls["freq"] = p.addUserDebugParameter("Frequence", 0, 5, 1)
    controls["length"] = p.addUserDebugParameter("Longueur", 0, 0.15, 0.1)
    controls["height"] = p.addUserDebugParameter("Hauteur des pas", 0, 0.2, 0.07)
    controls["body"] = p.addUserDebugParameter("Hauteur du corps", 0, 0.2, 0)

elif args.mode == "move-center":
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.3, 0.3, 0)
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.3, 0.3, 0)
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.3, 0.3, 0)

elif args.mode == "rotate":
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, -0.1)
    controls["rayon"] = p.addUserDebugParameter("rayon", 0.2, 1, 0.3)
    controls["angle"] = p.addUserDebugParameter("angle", 5, 55, 15)

elif args.mode == "toupie":
    controls["d_x"] = p.addUserDebugParameter("d_x", 0.15, 0.2, 0.16)
    controls["d_y"] = p.addUserDebugParameter("d_y", 0.001, 0.12, 0.05)
    controls["freq"] = p.addUserDebugParameter("frequence", -5 , 5, 1.3)
    controls["height"] = p.addUserDebugParameter("Hauteur des pas", 0, 0.2, 0.057)


while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    ###############################################################################
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])

        pts, T, steps, num_legs = [], [], 4, 6

        for i in range(num_legs):
            pts.extend(kinematics.computeDKDetailed(targets[params.legs[i+1][0]],
                                                    targets[params.legs[i+1][1]],
                                                    targets[params.legs[i+1][2]],
                                                    use_rads=True))
            for pt in range(steps):
                T.append(kinematics.rotaton_2D( pts[i*4+pt][0], pts[i*4+pt][1],
                                                pts[i*4+pt][2], -LEG_ANGLES[i]))
                T[-1] = list(map(   lambda a,b,c : a+b+c, T[-1], LEG_CENTER_POS[i],
                                    [0., 0., 0.5]))
                p.resetBasePositionAndOrientation(crosses[i*4+pt], T[-1],
                to_pybullet_quaternion(0, 0, -LEG_ANGLES[i]))

        sim.setRobotPose([0,0,0.5], to_pybullet_quaternion(0, 0, 0))
        state = sim.setJoints(targets)
    ###############################################################################
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        # Use your own IK function
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )
    elif args.mode == "robot-ik":
        None
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                leg_id,
                params,
                verbose=True,
            )
            # alphas = kinematics.computeIKOriented(
            #     0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
            #     0*0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
            #     0*0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
            #     leg_id,
            #     params,
            #     verbose=True,
            # )
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)
    elif args.mode == "walk":
        None
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        teta = p.readUserDebugParameter(controls["teta"])
        freq = p.readUserDebugParameter(controls["freq"])
        length =  p.readUserDebugParameter(controls["length"])
        height =  p.readUserDebugParameter(controls["height"])
        body =  p.readUserDebugParameter(controls["body"])


        kinematics.walk(freq, params, targets, teta, length, height, body)

        #print("\rPosition : ", kinematics.getPosition())
        s = "Position = " + str(kinematics.getPosition())                  # string for output
        print('{0}\r'.format(s), end='')

        state = sim.setJoints(targets)
    elif args.mode == "move-center":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])

        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                x,
                y,
                z,
                leg_id,
                params,
                0,
                verbose=True,
            )
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)

    elif args.mode == "rotate" :
            angle = math.pi/p.readUserDebugParameter(controls["angle"])
            z = p.readUserDebugParameter(controls["target_z"])
            radius = p.readUserDebugParameter(controls["rayon"])
            for leg_id in range (1,7):
                alpha = angle + LEG_ANGLES_2[leg_id - 1]
                x = radius * math.cos(alpha)
                y = radius * math.sin(alpha)
                tetas = kinematics.computeIKCenter(x, y, z, leg_id)
                set_leg_angles(tetas, leg_id, targets, params)

            state = sim.setJoints(targets)

    elif args.mode == "toupie":
        t = time.time()
        freq = p.readUserDebugParameter(controls["freq"])
        height = p.readUserDebugParameter(controls["height"])
        d_x = p.readUserDebugParameter(controls["d_x"])
        d_y = p.readUserDebugParameter(controls["d_y"])
        z = params.z


        tri1, tri2 = kinematics.toupie(t, freq, d_x, d_y, height, z)

        for leg_id in [1,3,5]:
            tetas = kinematics.computeIK(tri1[0], tri1[1], tri1[2])
            set_leg_angles(tetas, leg_id, targets, params)

        for leg_id in [2,4,6]:
            tetas = kinematics.computeIK(tri2[0], tri2[1], tri2[2])
            set_leg_angles(tetas, leg_id, targets, params)


        state = sim.setJoints(targets)

    #not usfull
    elif args.mode == "walk-input":
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        teta = 0
        freq = 0
        length =  0.1
        height =  0.07
        body =  0

        pressed_keys = []
        events = p.getKeyboardEvents()
        key_codes = events.keys()

        for key in key_codes:
            pressed_keys.append(key)

        D_KEY, Q_KEY, S_KEY, Z_KEY = 100, 113, 115, 122
        SHIFT_KEY = 65306
        SPACE_KEY = 32
        MOV_KEY_LIST = [D_KEY, Q_KEY, S_KEY, Z_KEY]

        dirv = [0,0]

        mov_keys = list(set(MOV_KEY_LIST) & set(pressed_keys))

        #if len(pressed_keys) > 0:
        #    print(pressed_keys)  

        if Z_KEY in pressed_keys:
            dirv[0] += 1
        if Q_KEY in pressed_keys:
            dirv[1] += 1
        if S_KEY in pressed_keys:
            dirv[0] -= 1
        if D_KEY in pressed_keys:
            dirv[1] -= 1
        if len(mov_keys) < 3 and mov_keys:
            freq = 3 if SHIFT_KEY in pressed_keys else 1  
            norm = np.sqrt(dirv[0]**2 + dirv[1]**2)
            if norm > 0: 
                dirv[0] /= norm
                dirv[1] /= norm
                teta = -np.pi if dirv == [-1,0] else np.arccos(np.dot([1,0], dirv)) * np.sign(dirv[1])
            else: freq = 0

        if SPACE_KEY in pressed_keys:
            body = 0.1
            

        kinematics.walk(freq, params, targets, teta, length, height, body)


        state = sim.setJoints(targets)

    #not usfull
    elif args.mode == "toupie-input":
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        t = time.time()

        height =  0.07


        pressed_keys = []
        events = p.getKeyboardEvents()
        key_codes = events.keys()

        for key in key_codes:
            pressed_keys.append(key)

        D_KEY, Q_KEY, S_KEY, Z_KEY = 100, 113, 115, 122
        SHIFT_KEY = 65306
        SPACE_KEY = 32
        MOV_KEY_LIST = [D_KEY, Q_KEY, S_KEY, Z_KEY]

        dirv = [0,0]

        mov_keys = list(set(MOV_KEY_LIST) & set(pressed_keys))

        freq = 0
        if Q_KEY in pressed_keys:
            freq = -1
        if D_KEY in pressed_keys:
            freq = 1

        z = params.z


        tri1, tri2 = kinematics.toupie(t, freq, 0.16, 0.05, height, z)

        if freq != 0:
            for leg_id in [1,3,5]:
                tetas = kinematics.computeIK(tri1[0], tri1[1], tri1[2])
                set_leg_angles(tetas, leg_id, targets, params)

            for leg_id in [2,4,6]:
                tetas = kinematics.computeIK(tri2[0], tri2[1], tri2[2])
                set_leg_angles(tetas, leg_id, targets, params)
        else: #horrible
            for leg_id in [1,3,5]:
                alphas = kinematics.computeIKOriented(tri1[0], tri1[1], tri1[2], leg_id, params, 0, verbose=True)
                set_leg_angles(alphas, leg_id, targets, params)
            for leg_id in [2,4,6]:
                alphas = kinematics.computeIKOriented(tri2[0], tri2[1], tri2[2], leg_id, params, 0, verbose=True)
                set_leg_angles(alphas, leg_id, targets, params)

        state = sim.setJoints(targets)

    
    elif args.mode == "input":
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        pressed_keys = []
        events = p.getKeyboardEvents()
        key_codes = events.keys()

        for key in key_codes:
            pressed_keys.append(key)

        D_KEY, Q_KEY, S_KEY, Z_KEY, E_KEY, A_KEY = 100, 113, 115, 122, 101, 97
        SHIFT_KEY = 65306
        SPACE_KEY = 32
        MOV_KEY_LIST = [D_KEY, Q_KEY, S_KEY, Z_KEY]


        dirv = [0,0]

        mov_keys = list(set(MOV_KEY_LIST) & set(pressed_keys))

        if A_KEY in pressed_keys or E_KEY in pressed_keys:

            
            t = time.time()

            height =  0.07
            
            freq = 0
            if A_KEY in pressed_keys:
                freq = 1
            if E_KEY in pressed_keys:
                freq = -1
            if SHIFT_KEY in pressed_keys:
                freq *= 3

            z = params.z


            tri1, tri2 = kinematics.toupie(t, freq, 0.16, 0.05, height, z)

            if freq != 0:
                for leg_id in [1,3,5]:
                    tetas = kinematics.computeIK(tri1[0], tri1[1], tri1[2])
                    set_leg_angles(tetas, leg_id, targets, params)

                for leg_id in [2,4,6]:
                    tetas = kinematics.computeIK(tri2[0], tri2[1], tri2[2])
                    set_leg_angles(tetas, leg_id, targets, params)
            else: #horrible, à changer 
                for leg_id in [1,3,5]:
                    alphas = kinematics.computeIKOriented(tri1[0], tri1[1], tri1[2], leg_id, params, 0, verbose=True)
                    set_leg_angles(alphas, leg_id, targets, params)
                for leg_id in [2,4,6]:
                    alphas = kinematics.computeIKOriented(tri2[0], tri2[1], tri2[2], leg_id, params, 0, verbose=True)
                    set_leg_angles(alphas, leg_id, targets, params)

            state = sim.setJoints(targets)
        else: 
            teta = 0
            freq = 0
            length =  0.1
            height =  0.07
            body =  0
            if Z_KEY in pressed_keys:
                dirv[0] += 1
            if Q_KEY in pressed_keys:
                dirv[1] += 1
            if S_KEY in pressed_keys:
                dirv[0] -= 1
            if D_KEY in pressed_keys:
                dirv[1] -= 1
            if len(mov_keys) < 3 and mov_keys:
                freq = 3 if SHIFT_KEY in pressed_keys else 1  
                norm = np.sqrt(dirv[0]**2 + dirv[1]**2)
                if norm > 0: 
                    dirv[0] /= norm
                    dirv[1] /= norm
                    teta = -np.pi if dirv == [-1,0] else np.arccos(np.dot([1,0], dirv)) * np.sign(dirv[1])
                else: freq = 0

            if SPACE_KEY in pressed_keys:
                body = 0.1
                
            kinematics.walk(freq, params, targets, teta, length, height, body)


            state = sim.setJoints(targets)

    sim.tick()
