import torch

class inference_handle(object):
    def __init__(self):
        self.dummy_action = torch.tensor([1.0,1.0])
        self.model = None
    
    def load_model(self):
        pass
    
    def inferece(self, obs):
        action = self.dummy_action.numpy()
        return action