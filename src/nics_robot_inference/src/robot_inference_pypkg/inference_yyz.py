import rospy

import numpy as np
import time,threading
import torch
from gym.spaces import Box, Discrete
from onpolicy.config import get_config

from nics_robot_inference.srv import *


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

    def parse_args(self, parser):
        parser.add_argument('--scenario_name', type=str,
                            default='relative_formation', help="Which scenario to run on")
        parser.add_argument("--num_landmarks", type=int, default=1)
        parser.add_argument('--num_agents', type=int,
                            default=3, help="number of players")
        parser.add_argument('--num_obstacles', type=int, default=0)
        parser.add_argument('--guiport',type=str,default='/dev/shm/gui_port')
        parser.add_argument('--usegui', action='store_true', default=False)
        parser.add_argument('--step-t',type=float,default=1.0)
        parser.add_argument('--sim-step',type=int,default=100)
        parser.add_argument('--direction_alpha', type=float, default=0.1)
        parser.add_argument('--add_direction_encoder',type=str, default='train')

        parser.add_argument('--ideal_side_len', type=float, default=5.0)
        parser.add_argument("--nav-rew-weight", type=float, default=1.0)
        all_args = parser.parse_known_args(None)[0]

        return all_args

    def restore(self):
        policy_actor_state_dict = torch.load(str(self.all_args.model_dir) + '/actor.pt')#, map_location='cpu')
        self.policy.actor.load_state_dict(policy_actor_state_dict)
        if not self.all_args.use_render:
            policy_critic_state_dict = torch.load(str(self.all_args.model_dir) + '/critic.pt')
            self.policy.critic.load_state_dict(policy_critic_state_dict)


    def load_model(self):
        from onpolicy.algorithms.r_mappo.r_mappo import R_MAPPO as TrainAlgo
        from onpolicy.algorithms.r_mappo.algorithm.rMAPPOPolicy import R_MAPPOPolicy as Policy
        parser = get_config()
        
        self.all_args = self.parse_args(parser)
        self.all_args.model_dir = '/home/ubuntu/project/NICS_MultiRobot_Platform/src/nics_robot_inference/src/robot_inference_pypkg/models'
        self.all_args.use_recurrent_policy = False
        self.rnn_states = np.zeros((self.all_args.num_agents, 1, 64), dtype=np.float32)
        
        self.policy = Policy(self.all_args,
                            obs_space = Box(low=-np.inf, high=+np.inf, shape=(7,), dtype=np.float32),
                            share_obs_space = Box(low=-np.inf, high=+np.inf, shape=(21,), dtype=np.float32),
                            act_space = Discrete(len(self.discrete_table)),
                            device = torch.device('cpu') )
        
        self.restore()
        self.trainer = TrainAlgo(self.all_args, self.policy, device = torch.device('cpu'))

    def inference(self, req):
        obs = self.get_obs_client()
        action = self.inference_core(obs)
        return inferResponse(action)

    def inference_core(self,obs):
        obs = np.zeros((self.all_args.num_agents, 7), dtype=np.float32)
        masks = np.ones((self.all_args.num_agents, 1), dtype=np.float32)
        action, self.rnn_states = self.trainer.policy.act(obs,                                    # 3,7
                                                          self.rnn_states,        # 3,1,64
                                                          masks,                                  # 3,1
                                                          deterministic=True)
        # action: torch tensor, torch.Size = [3,1] 取值范围0-6
        action = action.numpy() # [3,1]
        action = [self.discrete_table[action[i][0]] for i in range(self.all_args.num_agents)]
        # K_vel = 0.707
        # target_vel_b = [ action[i][0] * K_vel for i in range(self.args.num_agents) ] 
        # K_phi = 0.298
        # target_phi = [ action[i][1] * K_phi for i in range(self.args.num_agents) ]
        action = tuple(action[0])
        return action