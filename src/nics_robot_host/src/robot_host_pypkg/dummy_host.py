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

class FakeHost(object):
    def __init__(self, args, env):
        # get agent number from env
        self.agent_num = len(env.vehicle_list)
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
        
        self.env = env
        self.vrpn_list = [pos_data(0,0,0) for _ in range(self.agent_num)]



        self.core_thread = threading.Thread(target=self.core_function)
        rospy.spin()

    def obs_calculate(self,car_id,req):
        rospy.loginfo("Calculate obs for car %s",car_id)
        car_index = self.car_id_list.index(car_id)
        agent = self.env.vehicle_list[car_index]
        obs_result = self.env._get_obs(agent)
        print(obs_result) 
        return obsResponse(obs_result)

    def core_function(self):
        while True:
            old_movable_list = copy.deepcopy([v.state.movable for v in self.env.vehicle_list])
            self._update_world()
            for v_idx in range(self.agent_num):
                v = self.env.vehicle_list[v_idx]
                if not(v.state.movable == old_movable_list[v_idx]):
                    #TODO call v_idx vehicle set state service
                    pass
            time.sleep(self.core_fps)

    def _update_world(self):
        self._set_state_callback()
        #　TODO set correct total time
        total_time = 0.0
        self.env.ros_step(total_time)

    def _set_state_callback(self):
        for vehicle_idx in range(self.agent_num):
            state = self.env.vehicle_list[vehicle_idx].state
            vrpn_data = self.vrpn_list[vehicle_idx]
            state.coordinate[0] = vrpn_data.x
            state.coordinate[1] = vrpn_data.y
            state.theta = vrpn_data.theta
