#!/usr/bin/python3

import threading
import time

import rospy

import rosnode
import copy
import threading

from nics_robot_host.srv import *


class pos_data(object):
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

class DummyHost(object):
    def __init__(self, args, env=None):
        # get agent number from env
        self.agent_num = 1
        # init host node
        rospy.init_node("robot_host")

        # TODO get this param from launch file
        self.core_fps = 10

        # check the number of agent client 
        All_ready = False
        while not All_ready:
            node_name_list:list[str] = rosnode.get_node_names()
            self.car_id_list = []
            for node_name in node_name_list:
                if node_name.endswith('robot_client'):
                    # assume all robot_client note named as '/XXXX/car_id/robot_client'
                    self.car_id_list.append(node_name.split('/')[-2])
            if len(self.car_id_list) == self.agent_num:
                All_ready = True
                break
            print(self.car_id_list)
            time.sleep(0.5)
        #build observation services
        self.obs_server_list = []
        for car_id in self.car_id_list:
            handle = lambda req: self.obs_calculate(car_id,req)
            obs_messenger = rospy.Service('/'+car_id+'/get_obs', obs, handle)
        
        self.env = None
        self.vrpn_list = [pos_data(0,0,0) for _ in range(self.agent_num)]

        self.core_thread = threading.Thread(target=self.core_function)
        rospy.spin()

    def obs_calculate(self,car_id,req):
        obs_result = [0.0,0.0,1.0,0.0,0.0,0.5]
        rospy.loginfo("Calculate obs for car %s",car_id)
        print(obs_result)
        return obsResponse(obs_result)

    def core_function(self):
        c = 0
        while True:
            if c == 0:
              rospy.loginfo("dummy host alive")
            c = (c + 1)%100
            time.sleep(self.core_fps)
