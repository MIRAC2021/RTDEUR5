#!/usr/bin/env python

import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "192.168.1.102"
ROBOT_PORT = 30004
# ROBOT_PORT = 50002
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")
mode_names, mode_types = conf.get_recipe("mode")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)
mode = con.send_input_setup(mode_names, mode_types)

# Setpoints to move the robot to
setp1 = [206.33, 580.78, -99.85, 2.7, -1.8, -0.037]
setp2 = [221.60, 716.24, -82.40, 2.792, -1.867, -0.134]
setp3 = [-21.15, 744.18, -58.86, 3.065, -1.386, -0.226]
setp4 = [206.33, 580.78, -99.85, 2.7, -1.8, -0.037]
# setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
# setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0
mode.input_int_register_1 = 1

def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp


# start data synchronization
if not con.send_start():
    sys.exit()

# control loop
move_completed = True
while keep_running:
    # receive the current state
    state = con.receive()

    if state is None:
        break

    # do something...
    if move_completed and state.output_int_register_0 == 1:
        move_completed = False
        new_setp = setp1 if setp_to_list(setp) == setp2 else setp2
        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        # send new setpoint
        con.send(setp)
        watchdog.input_int_register_0 = 1
    elif not move_completed and state.output_int_register_0 == 0:
        print("Move to confirmed pose = " + str(state.target_q))
        move_completed = True
        watchdog.input_int_register_0 = 0

    # kick watchdog
    con.send(mode)
    con.send(watchdog)

con.send_pause()

con.disconnect()
