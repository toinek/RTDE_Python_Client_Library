import argparse
import logging
import sys
sys.path.append('..')

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time
import numpy as np


class Pose:
    def __init__(self, x, y, z, rx, ry, rz, to_cut=False, to_release=False):
        self.position = [x, y, z, rx, ry, rz]
        self.to_cut = to_cut # Cutting is a coupled motion with gripping
        self.to_release = to_release

# class Robot:
#     def __init__(self, host, port, config):
#         self.host = host
#         self.port = port
#         self.config = config
#         self.con = rtde.RTDE(self.host, self.port)
#         self.con.connect()
#         self.con.get_controller_version()
#
#         self.output_names, self.output_types = self.config.get_recipe('out')
#         self.waypoint_names, self.waypoint_types = self.config.get_recipe('waypoint')
#         self.scissors_names, self.scissors_types = self.config.get_recipe('scissors')
#         self.y_belt_names, self.y_belt_types = self.config.get_recipe('y_belt')
#         self.z_belt_names, self.z_belt_types = self.config.get_recipe('z_belt')
#         self.gripper_names, self.gripper_types = self.config.get_recipe('gripper')
#         self.watchdog_names, self.watchdog_types = self.config.get_recipe("watchdog")
#
#         self.waypoint = self.con.send_input_setup(self.waypoint_names, self.waypoint_types)
#         self.scissors = self.con.send_input_setup(self.scissors_names, self.scissors_types)
#         self.watchdog = self.con.send_input_setup(self.watchdog_names, self.watchdog_types)
#         self.y_belt = self.con.send_input_setup(self.y_belt_names, self.y_belt_types)
#         self.z_belt = self.con.send_input_setup(self.z_belt_names, self.z_belt_types)
#         self.gripper = self.con.send_input_setup(self.gripper_names, self.gripper_types)
#         self.con.send_start()

def list_to_setp(waypoint, list):
    for i in range(0, 6):
        waypoint.__dict__["input_double_register_%i" % i] = list[i]
    return waypoint

def euclidean_distance(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**0.5

def send_waypoint(con, waypoint, list):
    waypoint_command = list_to_setp(waypoint, list)
    con.send(waypoint_command)

def move_y(con, y_belt, distance):
    y_belt.input_double_register_6 = distance
    con.send(y_belt)
    time.sleep(1)

def move_z(con, z_belt, distance):
    z_belt.input_double_register_7 = distance
    con.send(z_belt)

def control_scissors(con, scissors, close=False):
    scissors.input_bit_register_73 = close
    con.send(scissors)
    # print("Scissors closed")
    time.sleep(0.5)
    scissors.input_bit_register_73 = not close
    con.send(scissors)
    # print("Scissors opened")

def control_gripper(con, gripper, width, force):
    gripper.input_double_register_8 = width
    gripper.input_double_register_9 = force
    con.send(gripper)

""""TODO: create variables at the top of the script"""
def follow_path(con, scissors, waypoint, path):
    grip = 145
    for pose in path:
        """"TODO: Use logger instead of print, info, warning, error, critical"""
        # print("Moving to pose: ", pose.position)
        send_waypoint(con, waypoint, pose.position)
        pose_reached = False
        while not pose_reached:
            state = con.receive(args.binary)
            if state is not None:
                X, Y, Z, RX, RY, RZ = state.actual_TCP_pose
                # print("X: ", X, "Y: ", Y, "Z: ", Z, "RX: ", RX, "RY: ", RY, "RZ: ", RZ)
                distance_to_pose = np.round(euclidean_distance(pose.position, state.actual_TCP_pose),3)
                # print("Distance to pose: ", distance_to_pose)
                if distance_to_pose == 0:
                    pose_reached = True
                    if pose.to_cut:
                        """TODO: add while loops for cutting, gripping, releasing based on RTDE output"""
                        """"Try to find signal when cutting/grasping is done"""
                        grip = np.random.randint(50, 120)
                        control_gripper(con, gripper, 50, force=120)
                        move_y(con, y_belt, -0.12)
                        control_scissors(con, scissors, close=True)
                    if pose.to_release:
                        grip = 145
                        control_gripper(con, gripper, grip, force=120)
                        time.sleep(0.5)
                        move_y(con, y_belt, -0.039)
# parameters
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.1.10',help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Hertz')
parser.add_argument('--config', default='basic_cutting_configuration.xml', help='data configuration file to use (record_configuration.xml)')
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")

args = parser.parse_args()
print(args.binary)
logging.basicConfig(level=logging.INFO)

# Connecting to the robot
con = rtde.RTDE(args.host, args.port)
con.connect()
con.get_controller_version()
con.send_start()

# Reading configuration file recipes
conf = rtde_config.ConfigFile(args.config)
output_names, output_types = conf.get_recipe('out')
waypoint_names, waypoint_types = conf.get_recipe('waypoint')
scissors_names, scissors_types = conf.get_recipe('scissors')
y_belt_names, y_belt_types = conf.get_recipe('y_belt')
z_belt_names, z_belt_types = conf.get_recipe('z_belt')
gripper_names, gripper_types = conf.get_recipe('gripper')
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

# Sending reading, waypoint, belts and scissors input setup
con.send_output_setup(output_names, output_types, frequency=args.frequency)
"""TODO: make attributes for robot class"""
waypoint = con.send_input_setup(waypoint_names, waypoint_types)
scissors = con.send_input_setup(scissors_names, scissors_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)
y_belt = con.send_input_setup(y_belt_names, y_belt_types)
z_belt = con.send_input_setup(z_belt_names, z_belt_types)
gripper = con.send_input_setup(gripper_names, gripper_types)

con.send_start()


"""TODO: make code modular for moving and cutting"""
# Waypoints to move the robot to

home_pos = Pose(-0.470, 0.580, -0.073, 0.106, 3.120, 0.013)
boven_pick = Pose(-0.470, 0.580, -0.160, 0.106, 3.120, 0.013)
knip_pos = Pose(-0.470, 0.500, -0.160, 0.106, 3.120, 0.013, to_cut=True)
after_knip = Pose(-0.470, 0.500, -0.095, 0.00, 3.032, 0.812)
pre_place = Pose(-0.800, 0.500, -0.035, 0.00, 3.032, 0.812)
place = Pose(-0.800, 0.500, -0.06, 0.00, 3.032, 0.812, to_release=True)

poses = [home_pos, boven_pick, knip_pos, after_knip, pre_place, place, home_pos]

# Main loop to run for 2 hours
start_time = time.time()
two_hours_later = 3600
print(start_time, two_hours_later)
runs = 0
run_lengths = []
last_run = time.time()
while time.time() - start_time < two_hours_later:
    print(f'seconds passed: {time.time() - start_time}')
    follow_path(con, scissors, waypoint, poses)
    run_length = time.time() - last_run
    run_lengths.append(run_length)
    last_run = time.time()
    runs += 1
    print(f'runs completed: {runs}')
print(f'average run length: {np.mean(run_lengths)}')
print(f'runs completed: {runs}')
con.send_pause()
con.disconnect()

