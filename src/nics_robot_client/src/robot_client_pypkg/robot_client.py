import rospy

import numpy as np
import time,threading
from geometry_msgs.msg import Twist
from nics_robot_client.srv import *
from .inference import inference_handle


class RobotClient(object):
    def __init__(self,id_str):
        self.car_id=id_str
        rospy.init_node('robot_client')
        self.pre_twist = Twist()
        self.twist = Twist()
        self.movable=1
        #self.register()
        self.inference_handle = inference_handle()
        self.car_allready=rospy.Service('supervisor',sup,self.allready_callback)
        self.get_obs_service_name = '/'+id_str+'/get_obs'
        self.get_obs_client=rospy.ServiceProxy(self.get_obs_service_name, obs)
        self.send_velocity_thread = threading.Thread(target=self.send_velocity,name='send_velocity_thread')
        self.deside_action_thread = threading.Thread(target=self.deside_action,name='deside_action_thread')
        self.send_velocity_thread.setDaemon(True)
        self.deside_action_thread.setDaemon(True)
        self.send_velocity_thread.start()
        self.deside_action_thread.start()
        rospy.spin()

    def deside_action(self):        
        self.inference_handle.load_model()
        while True:
            rospy.wait_for_service(self.get_obs_service_name)
            self.car_obs=self.get_obs_client.call()
            action = self.inference_handle.inferece(self.car_obs)
            self.action_to_velocity(action)
            time.sleep(1.0)

    def send_velocity(self):
        """下端所接串口发送频率50Hz，此线程不要有太多计算"""
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        while True:
            self.twist.linear.x=self.pre_twist.linear.x*self.movable
            self.twist.linear.y=self.pre_twist.linear.y*self.movable
            self.twist.linear.z=self.pre_twist.linear.z*self.movable   
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = self.pre_twist.angular.z*self.movable
            self.pub.publish(self.twist)
            time.sleep(0.02)
    
    def register(self):
        pass

    def action_to_velocity(self, action):
        self.pre_twist.linear.x =action[0]
        self.pre_twist.linear.y =0
        self.pre_twist.linear.z =0
        self.pre_twist.angular.z = action[1]

    def allready_callback(self,req):
        if req.collision:
            self.movable=0
        else:
            self.movable=req.movable