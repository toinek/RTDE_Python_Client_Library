import argparse
import logging
import sys
sys.path.append('..')

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time
import numpy as np


class Pose:
    """The class Pose is used to define the position of the robot in 3D space.
    The position is defined by the x, y, z, rx, ry, rz coordinates.
    The class also has two boolean variables to_cut and to_release
    which are used to define if the robot should cut or release the object.
    When cutting at a position, the robot will first grip, as this is a coupled motion"""
    def __init__(self, x, y, z, rx, ry, rz, to_cut=False, to_release=False):
        self.position = [x, y, z, rx, ry, rz]
        self.to_cut = to_cut
        self.to_release = to_release

class Robot:
    """The class Robot is used to control the robot. First the connection with the RTDE client is set up and
    the recipes are loaded from the configuration file. The class has methods to move the robot to a waypoint,
    move the robot in the y and z direction, control the scissors and the gripper. The class also has a method
    to disconnect the robot"""
    def __init__(self, args):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

        self.logger.info("Connecting to robot")
        self.host = args.host
        self.port = args.port
        self.config = rtde_config.ConfigFile(args.config)
        self.con = rtde.RTDE(self.host, self.port)
        self.con.connect()
        self.con.get_controller_version()
        self.logger.info("Connected to robot")

        self.output_names, self.output_types = self.config.get_recipe('out')
        self.waypoint_names, self.waypoint_types = self.config.get_recipe('waypoint')
        self.scissors_names, self.scissors_types = self.config.get_recipe('scissors')
        self.y_belt_names, self.y_belt_types = self.config.get_recipe('y_belt')
        self.z_belt_names, self.z_belt_types = self.config.get_recipe('z_belt')
        self.gripper_names, self.gripper_types = self.config.get_recipe('gripper')
        self.watchdog_names, self.watchdog_types = self.config.get_recipe("watchdog")
        self.con.send_output_setup(self.output_names, self.output_types, frequency=args.frequency)

        self.waypoint = self.con.send_input_setup(self.waypoint_names, self.waypoint_types)
        self.scissors = self.con.send_input_setup(self.scissors_names, self.scissors_types)
        self.watchdog = self.con.send_input_setup(self.watchdog_names, self.watchdog_types)
        self.y_belt = self.con.send_input_setup(self.y_belt_names, self.y_belt_types)
        self.z_belt = self.con.send_input_setup(self.z_belt_names, self.z_belt_types)
        self.gripper = self.con.send_input_setup(self.gripper_names, self.gripper_types)
        self.con.send_start()
        self.logger.info("Robot setup complete")

    def list_to_setp(self, pose):
        """The method list_to_setp is used to fill the waypoint register to be sent to the robot
        with a list of 6 values. The values are the x, y, z, rx, ry, rz coordinates of the robot."""
        for i in range(0, 6):
            self.waypoint.__dict__["input_double_register_%i" % i] = pose[i]

    def euclidean_distance(self, p1, p2):
        """Calculate the Euclidean distance between two points in 3D space"""
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**0.5

    def send_waypoint(self, pose):
        """Sets the waypoint registers with a list of poses and sends these to the robot"""
        self.list_to_setp(pose)
        self.con.send(self.waypoint)

    def move_y(self, distance):
        """Sets and sends the y_belt registers to move the robot scissors in the y direction"""
        self.y_belt.input_double_register_6 = distance
        self.con.send(self.y_belt)
        time.sleep(1)

    def move_z(self, distance):
        """Sets and sends the z_belt registers to move the robot scissors in the z direction"""
        self.z_belt.input_double_register_7 = distance
        self.con.send(self.z_belt)

    def control_scissors(self, close=False):
        """Sets and sends the scissors registers to control closing & opening of the scissors"""
        self.scissors.input_bit_register_73 = close
        self.con.send(self.scissors)
        # print("Scissors closed")
        time.sleep(0.5)
        self.scissors.input_bit_register_73 = not close
        self.con.send(self.scissors)
        # print("Scissors opened")

    def control_gripper(self, width, force):
        """Sets and sends the gripper registers to control the width and force of the gripping motion"""
        self.gripper.input_double_register_8 = width
        self.gripper.input_double_register_9 = force
        self.con.send(self.gripper)

    def disconnect(self):
        """Disconnect the robot"""
        self.con.send_pause()
        self.con.disconnect()

    def follow_path(self, path):
        """Follow a path of poses. Each pose is a waypoint the robot should move to.
        If the pose has the to_cut or to_release variable set to True, the robot will perform
        the cutting or releasing action. The robot will grip the object before cutting."""
        for pose in path:
            """"TODO: Use logger instead of print, info, warning, error, critical"""
            self.logger.info("Moving to pose: %s", pose.position)
            self.send_waypoint(pose.position)
            pose_reached = False
            while not pose_reached:
                state = self.con.receive(args.binary)
                if state is not None:
                    distance_to_pose = np.round(self.euclidean_distance(pose.position, state.actual_TCP_pose),3)
                    if distance_to_pose == 0:
                        pose_reached = True
                        if pose.to_cut:
                            """TODO: add while loops for cutting, gripping, releasing based on RTDE output"""
                            """"Try to find signal when cutting/grasping is done"""
                            self.control_gripper(width=50, force=120)
                            self.move_y(-0.11)
                            self.control_scissors(close=True)
                        if pose.to_release:
                            grip = 145
                            self.control_gripper(width=grip, force=120)
                            time.sleep(0.5)
                            self.move_y(-0.039)


if __name__ == "__main__":
    # parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='192.168.1.10', help='name of host to connect to (localhost)')
    parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
    parser.add_argument('--samples', type=int, default=0, help='number of samples to record')
    parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Hertz')
    parser.add_argument('--config', default='basic_cutting_configuration.xml',
                        help='data configuration file to use (record_configuration.xml)')
    parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
    parser.add_argument("--binary", help="save the data in binary format", action="store_true")

    args = parser.parse_args()

    # Waypoints to move the robot to
    home_pos = Pose(-0.475, 0.580, -0.07, 0.0, np.pi, 0.0)
    boven_pick = Pose(-0.475, 0.580, -0.165, 0.0, np.pi, 0.0)
    knip_pos = Pose(-0.475, 0.500, -0.165, 0.0, np.pi, 0.0, to_cut=True)
    after_knip = Pose(-0.475, 0.500, -0.095, 0.00, 3.032, 0.812)
    pre_place = Pose(-0.800, 0.500, -0.035, 0.00, 3.032, 0.812)
    place = Pose(-0.800, 0.500, -0.06, 0.00, 3.032, 0.812, to_release=True)

    poses = [home_pos, boven_pick, knip_pos, after_knip, pre_place, place, home_pos]

    neutral_pose = Pose(-0.475, 0.580, -0.165, 0.0, np.pi, 0.0)
    # y_belt: -110 mm
    # z_belt: 59 mm

    # Initializing the robot
    robot = Robot(args)
    robot.follow_path(poses)
    robot.disconnect()


