#!/usr/bin/python3
# -*- coding:utf-8 -*-

from typing import *
import numpy as np
import pickle
import time
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nics_ros_host.msg import global_state
from nics_ros_host.msg import vehicle_state
from nics_ros_host.msg import human_cmd
from nics_ros_host.srv import *
import struct
import threading
import argparse
from MultiVehicleEnv.environment import MultiVehicleEnv
import MultiVehicleEnv.scenarios as scenarios

#parser = argparse.ArgumentParser(description="GUI for Multi-VehicleEnv")
#parser.add_argument('--guiport',type=str,default='/dev/shm/gui_port')
#parser.add_argument('--usegui', action='store_true', default=False)
#parser.add_argument('--step-t',type=float,default=1.0)
#parser.add_argument('--sim-step',type=int,default=100)
#parser.add_argument('--direction_alpha', type=float, default=1.0)
#parser.add_argument('--add_direction_encoder',type=str, default='')
#args = parser.parse_args()

class RosWorld(object):
    def __init__(self,scenario_name, args):
        RosScenario = scenarios.load(scenario_name + ".py").Scenario()
        self.world = RosScenario.make_world(args)
        self.env = MultiVehicleEnv(self.world, RosScenario.reset_world, RosScenario.reward, RosScenario.observation,RosScenario.info)
        
        agent_num = len(self.world.vehicles)

        rospy.set_param('/All_Ready',0)
        rospy.set_param('/car_id_list',[])

        while True:
            car_id_list = rospy.get_param('/car_id_list')
            for i in range(agent_num):
                if i not in car_id_list:
                    print("Car %i not ready. Plz Check again.",i)
                    break
            if len(car_id_list) == agent_num:
                print("All Cars ready. Let's go.")
                rospy.set_param('/All_Ready',1)
                break
            time.sleep(0.5)

        rospy.init_node("RosWorld")
        pub = rospy.Publisher('global_state', global_state , queue_size=10)
        #service list
        command_service_list = []

        ####### spin Parts #########
        obs_messenger = rospy.Service('get_obs',obs,self.obs_calculate)
        cmd_sub = rospy.Subscriber('human_cmd', human_cmd, self.cmd_callback)
        for i in range(agent_num):
            command_service_list.append(rospy.ServiceProxy("supervisor_c"+str(car_id_list[i]),sup))
            rospy.Subscriber('/vrpn_client_node/c' + str(i+1) + '/pose', PoseStamped, self.pose_callback, (i, pub, command_service_list))
        rospy.spin()

    def pose_callback(self, msg, arg):
        car_id = arg[0]
        pub = arg[1]
        global command_list
        command_list = arg[2]
        vehicle_list = self.world.vehicles
        agent_num = len(vehicle_list)

        seq = msg.header.seq
        stamp = msg.header.stamp
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll = math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
        pitch = math.asin(2*(qw*qy-qz*qx))
        yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qz*qz+qy*qy))

        #rospy.loginfo("yaw:%f,pitch:%f,roll:%f"%(yaw,pitch,roll))
        #if(yaw > math.pi):
        #    yaw = yaw - 2 * math.pi
        #elif(yaw <= -math.pi):
        #    yaw = yaw + 2 * math.pi

        vehicle_list[car_id].state.coordinate = [x, z]
        vehicle_list[car_id].state.theta = pitch

        self.world._check_collision()
        for i in range(agent_num):
            #print("car",i)
            #print("movable",world.vehicles[i].state.movable)
            #print("crashed",world.vehicles[i].state.crashed)
            if self.world.vehicles[i].state.movable == False or self.world.vehicles[i].state.crashed == True:
                result = command_list[i].call(i,self.world.vehicles[i].state.movable,self.world.vehicles[i].state.crashed)

        pub.publish(global_state([vehicle_state(
            self.world.vehicles[i].state.coordinate,
            self.world.vehicles[i].state.theta,
            self.world.vehicles[i].state.vel_b,
            self.world.vehicles[i].state.phi,
            self.world.vehicles[i].state.ctrl_vel_b,
            self.world.vehicles[i].state.ctrl_phi,
            self.world.vehicles[i].state.movable,
            self.world.vehicles[i].state.crashed
        ) for i in range(agent_num)]))
        # maybe put world.dataslot in the topic

        #rospy.loginfo("RigidBody0%d[coordinate]: Header seq = %d; Position x = %f, y = %f, z = %f; Orientation roll = %f, pitch = %f, yaw = %f.", car_id+1, seq, x, y, z, roll, pitch, yaw)
        #if count % 5 == 0 and count < 2000:

    def obs_calculate(self,req):
        car_id = req.car_id
        self.world.dumpGUI()
        rospy.loginfo("Calculate obs for car %i",car_id)
        return obsResponse([2],[2],[2])

    def cmd_callback(self,msg):
        #add how to deal with dataslot
        rospy.loginfo("Cmdline get ", msg.cmd)

class rosarg(object):
    pass

if __name__ == '__main__':
    args = rosarg()
    args.guiport='/dev/shm/gui_port'
    args.usegui=False
    args.step_t=1.0
    args.sim_step=100
    args.direction_alpha=1.0
    args.add_direction_encoder=''


    ros_env = RosWorld('3p2t2f',args)