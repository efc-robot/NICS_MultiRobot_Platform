#!/usr/bin/python3

import time
import rospy
import numpy as np
import argparse 
import rosnode

from fake_host.srv import *

class FakeHost(object):
    def __init__(self, args):
        self.agent_num = args.agent_num
        rospy.init_node("Fake_ros_host")
        All_ready = False
        while not All_ready:
            node_name_list:str = rosnode.get_node_names()
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
        
        self.obs_server_list = []
        for car_id in self.car_id_list:
            handle = lambda req: self.obs_calculate(car_id,req)
            obs_messenger = rospy.Service(car_id+'/get_obs', obs, handle)
        
        rospy.spin()

    def obs_calculate(self,car_id,req):
        rospy.loginfo("Calculate obs for car %s",car_id)
        
        dummy_obs = np.random()
        return obsResponse([2],[2],[2])

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Fake vrpn")
    parser.add_argument('--agent-num',type=int,default=1)
    try:
        args = parser.parse_args()
    except:
        args = parser.parse_args([])
    else:
        args = parser.parse_args()
    
    fake_host = FakeHost(args)