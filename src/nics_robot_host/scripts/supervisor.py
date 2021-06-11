#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import math
from vrpn_client_ros.msg import global_state
import struct
import PyKDL as kdl
import threading
import argparse

def callback(msg,arg):
    car_num = arg
    pass

    #rospy.loginfo("RigidBody0%d[coordinate]: Header seq = %d; Position x = %f, y = %f, z = %f; Orientation roll = %f, pitch = %f, yaw = %f.", car_num+2, seq, x, y, z, roll, pitch, yaw)

def check_collision():
    pass

def handle_function(req):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--node_name", help="Name of node", type=str)
    parser.add_argument('args', nargs=argparse.REMAINDER)
    config = parser.parse_args()
    agent_num = rospy.get_param('/supervisor/agent_num')
    rospy.init_node(config.node_name, anonymous=True)
    rospy.Subscriber('global_state', global_state, callback,agent_num)
    #rospy.Service("send_obs", obs, handle_function)

    
    car_poses_x = [0.0 for i in range(agent_num)]
    car_poses_y = [0.0 for i in range(agent_num)]
    car_poses_yaw = [0.0 for i in range(agent_num)]

    rospy.spin()