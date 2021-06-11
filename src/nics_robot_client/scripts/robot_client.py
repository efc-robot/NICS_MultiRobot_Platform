#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
import time,threading
from geometry_msgs.msg import Twist
from rospy import service
from robot_client.srv import *
from .mac2id import mac2id,get_mac_address
from .inference import inference_handle
class RobotClient:
    def __init__(self):
        id_str = mac2id(get_mac_address())
        self.car_id=np.int32(id_str)
        rospy.init_node('robot_client_c'+str(self.car_id))
        self.pre_twist = Twist()
        self.twist = Twist()
        self.movable=1
        self.register()
        self.inference_handle = inference_handle()
        self.car_allready=rospy.Service('supervisor_c'+str(self.car_id),sup,self.allready_callback)
        
        self.get_obs_client=rospy.ServiceProxy('get_obs',obs)
        self.send_velocity_thread = threading.Thread(target=self.send_velocity,name='send_velocity_thread')
        self.deside_action_thread = threading.Thread(target=self.deside_action,name='deside_action_thread')
        self.send_velocity_thread.start()
        self.deside_action_thread.start()
        rospy.spin()

    def deside_action(self):        
        self.inference_handle.load_model()
        while True:
            rospy.wait_for_service('get_obs')
            self.car_obs=self.get_obs_client.call(self.car_id)
            action = self.inference_handle.inferece(self.car_obs)
            self.action_to_velocity(action)
            time.sleep(1.0)

    def send_velocity(self):
        """下端所接串口发送频率50Hz，此线程不要有太多计算"""
        self.pub = rospy.Publisher('c0/cmd_vel', Twist, queue_size = 1)
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
        if rospy.has_param('/car_id_list'):
            car_id_list=rospy.get_param('/car_id_list')
            car_id_list.append(int(self.car_id))
            rospy.set_param('/car_id_list',car_id_list)
        else:
            rospy.set_param('/car_id_list',[int(self.car_id)])

    def action_to_velocity(self, action):
        self.pre_twist.linear.x =action[0]
        self.pre_twist.linear.y =0
        self.pre_twist.linear.z =0
        self.pre_twist.angular.z = action[1]

    def allready_callback(self,req):
        if req.car_id==self.car_id:
            if req.collision:
                self.movable=0
            else:
                self.movable=req.movable
            return supResponse(1)
                      
        else:
            return supResponse(0)

if __name__=="__main__":
    l=RobotClient()