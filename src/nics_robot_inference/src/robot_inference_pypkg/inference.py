import rospy

import numpy as np
import time,threading
import torch
from gym.spaces import Box, Discrete
from onpolicy.config import get_config

from nics_robot_inference.srv import *



def naive_inference(target_x:float, target_y:float, theta:float,
                    dist:float=0.0, min_r:float=0.0) :
    r = (target_x**2+target_y**2)**0.5
    relate_theta = np.arctan2(target_y,target_x)-theta
    yt = np.sin(relate_theta)*r
    xt = np.cos(relate_theta)*r
    d = abs(np.tan(relate_theta)*r)
    if d < dist * 0.5:
        vel = np.sign(xt)
        phi = 0
    else:
        in_min_r = (xt**2+(abs(yt)-min_r)**2)< min_r**2
        vel = -1 if (bool(in_min_r) ^ bool(xt<0)) else 1
        phi = -1 if (bool(in_min_r) ^ bool(yt<0)) else 1
        if  d < dist :
            phi = phi *(2*d/dist-1)
    vel = vel*1.0
    return vel,phi


class RobotInference(object):
    def __init__(self,id_str):
        self.discrete_table = {0:[ 0.0, 0.0],
                               1:[ 1.0, 0.0], 2:[ 1.0, 1.0], 3:[ 1.0, -1.0],
                               4:[-1.0, 0.0], 5:[-1.0, 1.0], 6:[-1.0, -1.0]}

        self.car_id=id_str
        rospy.init_node('robot_inference')

        
        rospy.loginfo('robot inference load model')
        self.load_model()
        self.inference_service_name = '/' + self.car_id + '/inference'
        self.inference_service = rospy.Service(self.inference_service_name, infer, self.inference)
        rospy.loginfo("client_serv setup")

        self.get_obs_service_name = '/'+ self.car_id +'/get_obs'
        rospy.wait_for_service(self.get_obs_service_name)
        self.get_obs_client=rospy.ServiceProxy(self.get_obs_service_name, obs)


        self.inference_service.spin()


    def load_model(self):
        pass

    def inference(self, req):
        obs = self.get_obs_client().obs_vector
        tx = obs[4] - obs[0]
        ty = obs[5] - obs[1]
        theta =  np.arctan2(obs[3],obs[2])
        c_action = naive_inference(tx,ty,theta,dist= 0.4,min_r = 0.3)
        return inferResponse(c_action)