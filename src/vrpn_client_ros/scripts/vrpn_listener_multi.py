#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from vrpn_client_ros.msg import global_state
import struct
import PyKDL as kdl
import threading
import argparse
import scipy.io as scio
import numpy as np

car_poses = []

car_record_x = []
car_record_y = []
car_record_z = []
car_record_yaw = []
car_record_stamp = []

# car_poses = [(0.0, 0.0, 0.0) for i in range(agent_num)]

car_target = []

# car_target = [(3.84, 1.17), (1.57, 3.98)]

LASER_RANGE = 3.5

ROBOT_LENGTH = 0.25

count = 0
# tcp_ip = '172.16.0.15'

# tcp_port = [8888, 9999]

def callback(msg, arg):
    car_num = arg[0]
    pub = arg[1]
    global car_poses
    global count
    global car_record_x
    global car_record_y
    global car_record_z
    global car_record_yaw
    global car_record_stamp
    seq = msg.header.seq
    stamp = msg.header.stamp
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    rotation_matrix = kdl.Rotation.Quaternion(qx, qy, qz, qw)

    yaw, pitch, roll = rotation_matrix.GetEulerZYX()

    if(yaw > math.pi):
        yaw = yaw - 2 * math.pi
    elif(yaw <= -math.pi):
        yaw = yaw + 2 * math.pi

    car_poses[car_num] = (x, y, yaw)

    car_poses_x = [p[0] for p in car_poses]
    car_poses_y = [p[1] for p in car_poses]
    car_poses_yaw = [p[2] for p in car_poses]

    pub.publish(global_state(car_poses_x, car_poses_y, car_poses_yaw))
    count += 1
    rospy.loginfo("RigidBody0%d[coordinate]: Header seq = %d; Position x = %f, y = %f, z = %f; Orientation roll = %f, pitch = %f, yaw = %f.", car_num+2, seq, x, y, z, roll, pitch, yaw)
    rospy.loginfo(count)
    #if count % 5 == 0 and count < 2000:
    if True:
        car_record_x.append(x)
        car_record_y.append(y)
        car_record_z.append(z)
        car_record_yaw.append(yaw)
        car_record_stamp.append(int(stamp.sec))
        #rospy.loginfo(stamp.to_sec())
    if count == 300:
        scio.savemat("/home/opttrack/catkin_ws/src/vrpn_client_ros/scripts/data_v2_fix/c2_back_data_v1_fix.mat", {'x':np.array(car_record_x),'y':np.array(car_record_y),'z':np.array(car_record_z),'yaw':np.array(car_record_yaw), 'stamp':np.array(car_record_stamp)})

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--node_name", help="Name of node", type=str)
    parser.add_argument('args', nargs=argparse.REMAINDER)
    config = parser.parse_args()

    rospy.init_node(config.node_name, anonymous=True)
    pub = rospy.Publisher('global_state', global_state , queue_size=10)

    agent_num = rospy.get_param('~agent_num')
    car_poses = [(0.0, 0.0, 0.0) for i in range(agent_num)]
    LASER_RANGE = rospy.get_param('~LASER_RANGE')
    ROBOT_LENGTH = rospy.get_param('~ROBOT_LENGTH')

    #checking_list = rospy.get_published_topics()
    #for i in range(agent_num):
    #    if not ["/c"+str(i)+"/voltage","std_msgs/Float32"] in checking_list:
    #        print("Car"+str(i)+"not ready!")

    for i in range(agent_num):
        rospy.Subscriber('/vrpn_client_node/c' + str(i+2) + '/pose', PoseStamped, callback, (i, pub))
    rospy.spin()
