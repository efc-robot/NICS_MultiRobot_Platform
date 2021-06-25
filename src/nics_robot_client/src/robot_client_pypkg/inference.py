import torch
import numpy as np

def naive_path(agent_x, agent_y, ct, st,
               target_x, target_y,
               dist:float=0.0, min_r:float=0.5):
    relate_target_x = target_x-agent_x
    relate_target_y = target_y-agent_y
    r = (relate_target_x**2+relate_target_y**2)**0.5
    relate_theta = np.arctan2(relate_target_y,relate_target_x)-np.arctan2(st,ct)
    yt = np.sin(relate_theta)*r
    xt = np.cos(relate_theta)*r
    if abs(np.tan(relate_theta)*r) < dist * 0.5:
        vel = np.sign(xt)
        phi = 0
    else:
        in_min_r = (xt**2+(abs(yt)-min_r)**2)< min_r**2
        vel = -1 if (bool(in_min_r) ^ bool(xt<0)) else 1
        phi = -1 if (bool(in_min_r) ^ bool(yt<0)) else 1
    return vel,phi

def naive_inference(obs_vec,dist:float=0.0, min_r:float=0.5):
    vel, phi = naive_path(obs_vec[0], obs_vec[1], obs_vec[2], obs_vec[3],
                         obs_vec[4], obs_vec[5],
                         dist, min_r)
    return vel, phi

class inference_handle(object):
    def __init__(self):
        self.dummy_action = torch.tensor([1.0,1.0])
        self.model = None
    
    def load_model(self):
        pass
    
    def inferece(self, obs):
        vel, phi = naive_inference(obs.obs_vector, 0.1, 0.5)
        action = [vel, phi]
        return action