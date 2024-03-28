#!/usr/bin/env python

import sys

sys.path.append('..')
import logging
import time

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '192.168.1.10'
ROBOT_PORT = 30004
config_filename = 'example_boolreg_configuration.xml'
keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
output_names, output_types = conf.get_recipe('outputs')
input_names, input_types = conf.get_recipe('inputs')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(output_names, output_types)
inputs = con.send_input_setup(input_names, input_types)

# start data synchronization
if not con.send_start():
    sys.exit()

# inputs.standard_digital_output_mask = 1

print("running now")
# control loop
while keep_running:
    # receive the current state
    outputs = con.receive()

    if outputs is None:
        break;

    reg_value = 1
    print("reg70: " + str(reg_value))
    inputs.input_bit_register_73 = True
    print(inputs.__dict__)

    con.send(inputs)

con.send_pause()
con.disconnect()