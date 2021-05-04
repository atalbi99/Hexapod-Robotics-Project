# from kinematics import *
from constants import *
import time


class Parameters:
    def __init__(
        self,
        freq=50,
        speed=1,
        z=-60,
        travelDistancePerStep=80,
        lateralDistance=90,
        frontDistance=87,
        frontStart=32,
        method="constantSpeed",
        maxAccel=6000,
        maxSpeed=500,
        startFromInit=True,
        endToInit=False,
        up=False,
        down=False,
        left=False,
        right=False,
        walkMagnitudeX=0,
        walkMagnitudeY=0,
        activateWalk=False,
    ):
        self.freq = freq
        self.speed = speed
        self.z = z
        self.travelDistancePerStep = travelDistancePerStep
        self.lateralDistance = lateralDistance
        self.frontDistance = frontDistance
        self.frontStart = frontStart
        self.method = method
        self.maxAccel = maxAccel
        self.maxSpeed = maxSpeed
        self.startFromInit = startFromInit
        self.endToInit = endToInit
        self.up = up
        self.down = down
        self.left = left
        self.right = right
        self.walkMagnitudeX = walkMagnitudeX
        self.walkMagnitudeY = walkMagnitudeY
        self.activateWalk = activateWalk
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        if ROBOT_TYPE == BIOLOID:
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, -self.frontDistance]
            )
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, self.frontDistance]
            )
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, -self.frontDistance]
            )
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, self.frontDistance]
            )
        elif ROBOT_TYPE == PHANTOMX or ROBOT_TYPE == PHANTOMX_SIMULATION:
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])


# Classes used to use dxl_io directly and by-pass Pypot's Robot class
class SimpleMotor:
    def __init__(self, id):
        self.id = id
        self.present_position = 0
        self.goal_position = 0
        self.smooth_start_position = 0
        self.smooth_final_position = 0

    def __repr__(self):
        return "id {}, goal_position {}, present_position {}".format(
            self.id, self.goal_position, self.present_position
        )


class SimpleRobot:
    def __init__(self, dxl_io):
        self.dxl_io = dxl_io
        self.legs = {
            1: [SimpleMotor(8), SimpleMotor(10), SimpleMotor(12)],
            2: [SimpleMotor(14), SimpleMotor(16), SimpleMotor(18)],
            3: [SimpleMotor(13), SimpleMotor(15), SimpleMotor(17)],
            4: [SimpleMotor(7), SimpleMotor(9), SimpleMotor(11)],
            5: [SimpleMotor(1), SimpleMotor(3), SimpleMotor(5)],
            6: [SimpleMotor(2), SimpleMotor(4), SimpleMotor(6)],
        }
        self.delay_after_write = 0.01
        self.params = None

    def __repr__(self):
        output = "##### Robot #####\n"
        for k, v in self.legs.items():
            output += "# Leg{}: [{:.2f}] [{:.2f}] [{:.2f}]\n".format(
                k, v[0], v[1], v[2]
            )
        return output

    # def print_dk(self):
    #     output = "##### Robot #####\n"

    #     for k, v in self.legs.items():
    #         p0, p1, p2, p3 = computeDKDetailed(
    #             v[0].present_position,
    #             v[1].present_position,
    #             v[2].present_position,
    #         )
    #         output += "# Leg{}. Angles: [{:.2f}] [{:.2f}] [{:.2f}]. DK P3: x={:.2f}, y={:.2f}, z={:.2f} DK P2: x={:.2f}, y={:.2f}, z={:.2f}\n".format(
    #             k,
    #             v[0].present_position,
    #             v[1].present_position,
    #             v[2].present_position,
    #             p3[0],
    #             p3[1],
    #             p3[2],
    #             p2[0],
    #             p2[1],
    #             p2[2],
    #         )
    #     print(output)

    def init(self):
        """Sets the goal_position to the present_position"""
        self.tick_read(verbose=True)
        for k, v in self.legs.items():
            v[0].goal_position = v[0].present_position
            v[1].goal_position = v[1].present_position
            v[2].goal_position = v[2].present_position

    def motors(self):
        list_of_motors = []
        for k, v in self.legs.items():
            list_of_motors.append(v[0])
            list_of_motors.append(v[1])
            list_of_motors.append(v[2])
        return list_of_motors

    def enable_torque(self, list_of_ids=None):
        to_send = []
        if list_of_ids == None:
            for k, v in self.legs.items():
                to_send.append(v[0].id)
                to_send.append(v[1].id)
                to_send.append(v[2].id)
        else:
            to_send = list_of_ids
        self.dxl_io.enable_torque(to_send)
        time.sleep(self.delay_after_write)

    def disable_torque(self, list_of_ids=None):
        to_send = []
        if list_of_ids == None:
            for k, v in self.legs.items():
                to_send.append(v[0].id)
                to_send.append(v[1].id)
                to_send.append(v[2].id)
        else:
            to_send = list_of_ids
        self.dxl_io.disable_torque(to_send)
        time.sleep(self.delay_after_write)

    def tick_read(self, verbose=False):
        # Creating a list for a read request
        to_read = []
        for k, v in self.legs.items():
            to_read.append(v[0].id)
            to_read.append(v[1].id)
            to_read.append(v[2].id)

        if verbose:
            print("Sending read command '{}'".format(to_read))
        result = self.dxl_io.get_present_position(to_read)
        for i in range(len(to_read)):
            id = to_read[i]
            value = result[i]
            # Meh
            for m in self.motors():
                if m.id == id:
                    m.present_position = value
        if verbose:
            print("Read tick done")

    def tick_write(self, verbose=False):
        # Creating a dict for a write request
        to_write = {}
        for k, v in self.legs.items():
            for i in range(3):
                if MOTOR_TYPE == AX12:
                    # Not sending values out of motor range
                    if v[i].goal_position <= -150 or v[i].goal_position >= 150:
                        continue
                to_write[v[i].id] = v[i].goal_position

        if verbose:
            print("Sending write command '{}'".format(to_write))
        self.dxl_io.set_goal_position(to_write)
        if verbose:
            print("Write tick done")
        time.sleep(self.delay_after_write)

    def tick_read_and_write(self, verbose=False):
        # Creating a list for a read request and a dict for a write request
        self.tick_read()
        self.tick_write()
        if verbose:
            print("IO tick done")

    def smooth_tick_read_and_write(self, delay, verbose=False):
        # Reads the current state of the robot and applies the write positions smoothly over 'time'
        self.tick_read()
        # Setting the start and end positions
        t0 = time.time()
        for m in self.motors():
            m.smooth_start_position = m.present_position
            m.smooth_final_position = m.goal_position
            if verbose:
                print(
                    "m.smooth_start_position {}, m.smooth_final_position {}".format(
                        m.smooth_start_position,
                        m.smooth_final_position,
                    )
                )
        t = time.time() - t0
        while t < delay:
            for m in self.motors():
                m.goal_position = (t / delay) * (
                    m.smooth_final_position - m.smooth_start_position
                ) + m.smooth_start_position
            self.tick_write(verbose=verbose)
            t = time.time() - t0
        for m in self.motors():
            m.goal_position = m.smooth_final_position
        self.tick_write(verbose=verbose)
        if verbose:
            print("IO smooth tick done")


# Class used to simulate the robot in PyBullet's (actually onshape_to_robot) environnment
class SimpleRobotSimulation:
    def __init__(self, sim):
        self.sim = sim  # e.g onshape_to_robot.simulation.Simulation("phantomx_description/urdf/phantomx.urdf", gui=True, panels=True, useUrdfInertia=False)
        self.legs = {
            1: [
                SimpleMotor("j_c1_rf"),
                SimpleMotor("j_thigh_rf"),
                SimpleMotor("j_tibia_rf"),
            ],
            6: [
                SimpleMotor("j_c1_rm"),
                SimpleMotor("j_thigh_rm"),
                SimpleMotor("j_tibia_rm"),
            ],
            5: [
                SimpleMotor("j_c1_rr"),
                SimpleMotor("j_thigh_rr"),
                SimpleMotor("j_tibia_rr"),
            ],
            2: [
                SimpleMotor("j_c1_lf"),
                SimpleMotor("j_thigh_lf"),
                SimpleMotor("j_tibia_lf"),
            ],
            3: [
                SimpleMotor("j_c1_lm"),
                SimpleMotor("j_thigh_lm"),
                SimpleMotor("j_tibia_lm"),
            ],
            4: [
                SimpleMotor("j_c1_lr"),
                SimpleMotor("j_thigh_lr"),
                SimpleMotor("j_tibia_lr"),
            ],
        }
        self.delay_after_write = 0.00
        self.drawOn = True
        self.params = None
        self.centerCamera = False

    def __repr__(self):
        output = "##### Robot #####\n"
        for k, v in self.legs.items():
            output += "# Leg{}: [{}] [{}] [{}]\n".format(k, v[0], v[1], v[2])
        return output

    def init(self):
        """Sets the goal_position to the present_position"""
        self.tick_read(verbose=True)
        for k, v in self.legs.items():
            v[0].goal_position = v[0].present_position
            v[1].goal_position = v[1].present_position
            v[2].goal_position = v[2].present_position

    def motors(self):
        list_of_motors = []
        for k, v in self.legs.items():
            list_of_motors.append(v[0])
            list_of_motors.append(v[1])
            list_of_motors.append(v[2])
        return list_of_motors

    def enable_torque(self, list_of_ids=None):
        if list_of_ids == None:
            # Enabling torque for all motors (it's weird I know)
            self.sim.maxTorques = {}
        else:
            new_torques = {}
            for k, v in self.sim.maxTorques.items():
                if not (k in list_of_ids):
                    # Copying previous IDs and removing the ones that need their torque reactivated
                    new_torques[k] = v
            self.sim.maxTorques = new_torques

    def disable_torque(self, list_of_ids=None):
        if list_of_ids == None:
            # Disabling torque for all motors
            for m in self.motors():
                self.sim.maxTorques[m.id] = 0
        else:
            new_torques = self.sim.maxTorques
            for id in list_of_ids:
                if not (id in new_torques):
                    # Adding new IDs and avoiding duplicates
                    new_torques[id] = 0
            self.sim.maxTorques = new_torques

    def tick_read(self, verbose=False):
        # Read and write are not separated for now in the simulator
        self.tick_read_and_write()

    def tick_write(self, verbose=False):
        # Read and write are not separated for now in the simulator
        self.tick_read_and_write()

    def tick_read_and_write(self, verbose=False):
        # Creating a list for a read request and a dict for a write request
        to_write = {}
        for k, v in self.legs.items():
            for i in range(3):
                to_write[v[i].id] = v[i].goal_position

        if verbose:
            print("Sending write command '{}'".format(to_write))
        state = self.sim.setJoints(to_write)
        for m in self.motors():
            id = m.id
            position = state[id][
                0
            ]  # contains [position, speed, (3 forces and 3 torques)
            m.present_position = position
        if self.drawOn:
            self.drawLegTips()

        if verbose:
            print("IO tick done")

    def smooth_tick_read_and_write(self, delay, verbose=False):
        # Reads the current state of the robot and applies the write positions smoothly over 'time'
        self.tick_read()
        # Setting the start and end positions
        t0 = time.time()
        for m in self.motors():
            m.smooth_start_position = m.present_position
            m.smooth_final_position = m.goal_position
            if verbose:
                print(
                    "m.smooth_start_position {}, m.smooth_final_position {}".format(
                        m.smooth_start_position,
                        m.smooth_final_position,
                    )
                )
        t = time.time() - t0
        while t < delay:
            for m in self.motors():
                m.goal_position = (t / delay) * (
                    m.smooth_final_position - m.smooth_start_position
                ) + m.smooth_start_position
            self.tick_write(verbose=verbose)
            t = time.time() - t0
            # Blocking call has to tick the simulation here
            self.sim.tick()
        for m in self.motors():
            m.goal_position = m.smooth_final_position
        self.tick_write(verbose=verbose)
        if verbose:
            print("IO smooth tick done")

    def drawLegTips(self, duration=2):
        # TODO folks!
        None

    def tickSim(self):
        if self.centerCamera:
            self.centerCameraOnRobot()
        self.sim.tick()

    def centerCameraOnRobot(self):
        robot_pose = (
            self.sim.getRobotPose()
        )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        self.sim.lookAt(robot_pose[0])
