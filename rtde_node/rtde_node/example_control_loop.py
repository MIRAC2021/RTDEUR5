#!/usr/bin/env python

import sys
import std_msgs.msg

sys.path.append(".")

import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import threading
import time
import rclpy
import std_msgs
from rclpy.node import Node

class RTDE_Node(Node):
    def __init__(self):
        self.init_ur5()
        self.init_ros()

    def init_ros(self):
        super().__init__("Robot RTDE Node")
        # a custom message format would be a whole lot better...
        self.subscription = self.create_subscription(std_msgs.msg.Float64MultiArray, "path_planning/out_path", self.ros_message_callback, 10)
        self.ros_lock = threading.Lock()

    def init_ur5(self):
        # logging.basicConfig(level=logging.INFO)
        ROBOT_HOST = "192.168.1.102"
        ROBOT_PORT = 30004 # ROBOT_PORT = 50002
        config_filename = "control_loop_configuration.xml"

        self.keep_running = True

        logging.getLogger().setLevel(logging.INFO)

        conf = rtde_config.ConfigFile(config_filename)
        self.state_names, self.state_types = conf.get_recipe("state")
        self.setp_names, self.setp_types = conf.get_recipe("setp")
        self.watchdog_names, self.watchdog_types = conf.get_recipe("watchdog")
        self.mode_names, self.mode_types = conf.get_recipe("mode")

        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con_lock = threading.Lock()


    def transform(msg): # can be redone as long as you return a 2d list from the ros message
        rows = msg.layout.dim[0].size if msg.layout.dim else 0
        cols = msg.layout.dim[1].size if len(msg.layout.dim) > 1 else 6

        if rows * cols != len(msg.data):
            print('Layout size does not match data length!')
            return None

        return [msg.data[i:i+cols] for i in range(0, len(msg.data), cols)]

    def ros_message_callback(self, msg):
        # a custom message would be a whole lot better...
        if not self.running: # don't process messages early
            return

        with self.ros_lock: # this one is probably overkill, but i'm paranoid :)
            path = RTDE_Node.transform(msg)
            with self.con_lock:
                state = self.con.receive()

                if path is None:
                    sys.exit()

                if state is None:
                    # this probably doesn't work since it is in a thread, so... find a better way
                    sys.exit()

                for setp_list in path:
                    move_completed = True

                    while True:
                        if move_completed and state.output_int_register_0 == 1:
                            move_completed = False
                            #new_setp = setp1 if RTDE_Node.setp_to_list(setp) == setp2 else setp2
                            RTDE_Node.list_to_setp(self.setp, setp_list)
                            print("New pose = " + str(self.setp))
                            # send new setpoint
                            self.con.send(self.setp)
                            self.watchdog.input_int_register_0 = 1
                        elif not move_completed and state.output_int_register_0 == 0:
                            print("Move to confirmed pose = " + str(state.target_q))
                            move_completed = True
                            self.watchdog.input_int_register_0 = 0
                            break

    def setp_to_list(sp):
        sp_list = []
        for i in range(0, 6):
            sp_list.append(sp.__dict__["input_double_register_%i" % i])
        return sp_list

    def list_to_setp(sp, list):
        for i in range(0, 6):
            sp.__dict__["input_double_register_%i" % i] = list[i]
        return sp
    
    def run(self):
        self.con.connect()

        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(self.state_names, self.state_types)
        self.setp = self.con.send_input_setup(self.setp_names, self.setp_types)
        self.watchdog = self.con.send_input_setup(self.watchdog_names, self.watchdog_types)
        self.mode = self.con.send_input_setup(self.mode_names, self.mode_types)

        # Setpoints to move the robot to
        # setp1 = [206.33, 580.78, -99.85, 2.7, -1.8, -0.037]
        # setp2 = [221.60, 716.24, -82.40, 2.792, -1.867, -0.134]
        # setp3 = [-21.15, 744.18, -58.86, 3.065, -1.386, -0.226]
        # setp4 = [206.33, 580.78, -99.85, 2.7, -1.8, -0.037]
        # setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
        # setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]

        state = self.con.receive()
        self.setp.input_double_register_0 = state.actual_TCP_pose[0]
        self.setp.input_double_register_1 = state.actual_TCP_pose[1]
        self.setp.input_double_register_2 = state.actual_TCP_pose[2]
        self.setp.input_double_register_3 = state.actual_TCP_pose[3]
        self.setp.input_double_register_4 = state.actual_TCP_pose[4]
        self.setp.input_double_register_5 = state.actual_TCP_pose[5]

        # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
        self.watchdog.input_int_register_0 = 0
        self.mode.input_int_register_1 = 1

        # start data synchronization
        if not self.con.send_start():
            sys.exit()

        self.watchdog_thread = threading.Thread(target=self.loop_watchdog, daemon=True)
        self.running = True

    def loop_watch(self):
        while True:
            with self.con_lock:
                self.con.send(self.mode)
                self.con.send(self.watchdog)
                time.sleep(0.005)


    def temp(self):
        # control loop
        move_completed = True
        while self.keep_running:
            # receive the current state

            # do something...
            break
            

        #self.con.send_pause()
        #self.con.disconnect()

if __name__ == "main":
    node = RTDE_Node()
    node.run()
