import pypot.dynamixel
import time
import math
from utils import *
import time

from kinematics import *
import math
import sys
from signal import signal, SIGINT
import traceback
import os

# import display


def main():
    ports = pypot.dynamixel.get_available_ports()
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

    robot = SimpleRobot(dxl_io)
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:

        params = Parameters(
            freq=50,
            speed=1,
            z=-60,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
            v[0].goal_position = 0
            v[1].goal_position = 0
            v[2].goal_position = 0

        # TODO create this function instead:
        # setPositionToRobot(0, 0, 0, robot, params)
        robot.smooth_tick_read_and_write(3, verbose=False)
        print("Init position reached")
        time.sleep(2)
        print("Closing")
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
