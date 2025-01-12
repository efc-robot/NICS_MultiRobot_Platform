import rospy

import numpy as np
import time,threading
import torch
from gym.spaces import Box, Discrete
from onpolicy.config import get_config

from nics_robot_inference.srv import *

import sys
sys.path.append('/home/ubuntu/project/FastSimulatorExp')
from src.Algorithm import SAC


class RobotInference(object):
    def __init__(self,id_str, args = None):
        self.discrete_table = {0:[ 0.0, 0.0],
                               1:[ 1.0, 0.0], 2:[ 1.0, 1.0], 3:[ 1.0, -1.0],
                               4:[-1.0, 0.0], 5:[-1.0, 1.0], 6:[-1.0, -1.0]}
        self.args = args
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
        self.agent = SAC(self.args)
        self.agent.setup(726, 2)
        actor = torch.load('/home/ubuntu/project/FastSimulatorExp/wandb/run-20210903_032937-2h64wlx6/files/_actor.pkl')
        self.agent.policy.load_state_dict(actor.state_dict(), strict=True)

    def inference(self, req):

        try:
            obs = self.get_obs_client().obs_vector
            obs = torch.tensor(data=[obs], dtype=torch.float32).cuda()
            with torch.no_grad():
                print('-----obs-----',obs)
                action = self.agent.select_action(obs, eval=True)
                action = action.cpu().squeeze(0).numpy()
                print('-----action-----',action)
        except rospy.ServiceException as exc:
            print("Failed to get obs vector, " + str(exc))
            action = [0.0, 0.0]
        return inferResponse(action)