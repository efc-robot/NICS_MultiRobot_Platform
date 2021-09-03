import rospy

import numpy as np

from nics_robot_inference.srv import *


class RobotInference(object):
    def __init__(self,id_str, args = None):
        self.args = args
        self.car_id=id_str
        self.time_record = None
        rospy.init_node('robot_inference')

        
        rospy.loginfo('robot inference load model')
        self.load_model()
        self.inference_service_name = '/' + self.car_id + '/inference'
        self.inference_service = rospy.Service(self.inference_service_name, infer, self.inference)
        rospy.loginfo("client_serv setup")

        self.inference_service.spin()


    def load_model(self):
        pass

    def inference(self, req):
        if self.time_record is None:
            self.time_record = rospy.get_time()
            action = [1.0, 0.0]
            self.last_action = action
        else:
            if rospy.get_time()-self.time_record > 2.0:
                action = [self.last_action[0]*-1, 0.0]
                self.last_action = action
                self.time_record = rospy.get_time()
            else:
                action = self.last_action
        action = [1.0,1.0]
        return inferResponse(action)
