import rospy

import numpy as np
import time,threading
from geometry_msgs.msg import Twist
from nics_robot_client.srv import *



class RobotClient(object):
    def __init__(self,id_str):
        self.car_id=id_str
        rospy.init_node('robot_client')   ### Edit by tjh, 31/8/2021

        self.inference_fps = 5.0
        self.send_velocity_fps = 20.0

        self.inference_twist = Twist()
        
        self.movable = 0
        self.run_state = 0

        self.client_control_thread = threading.Thread(target=self.client_control_target,name='client_control_thread',daemon=True) 
        self.client_control_thread.start()
        
        while self.run_state == 0:
            time.sleep(0.1)
        rospy.loginfo('robot receive start signal')

        self.send_velocity_thread = threading.Thread(target=self.send_velocity_target,name='send_velocity_thread',daemon=True)
        self.inference_thread = threading.Thread(target=self.inference_target,name='inference_thread',daemon=True)
        self.send_velocity_thread.start()
        self.inference_thread.start()

        while not rospy.is_shutdown():
            time.sleep(1)

    def inference_target(self):

        self.inference_service_name = '/'+ self.car_id +'/inference'

        rospy.wait_for_service(self.inference_service_name)
        self.inference_client=rospy.ServiceProxy(self.inference_service_name, infer)

        rate = rospy.Rate(self.inference_fps)
        while True:
            if self.run_state:
                try:
                    action = self.inference_client()
                    self.action_to_velocity(action.act_vector)
                    rate.sleep()
                except rospy.ServiceException as exc:
                    print("Can not get action: " + str(exc))
            else:
                rospy.sleep(0.1)

    def action_to_velocity(self, action):
        self.inference_twist.linear.x = action[0]
        self.inference_twist.linear.y = 0
        self.inference_twist.linear.z = 0
        self.inference_twist.angular.x = 0
        self.inference_twist.angular.y = 0
        self.inference_twist.angular.z = action[1]

    def send_velocity_target(self):
        """下端所接串口发送频率50Hz，此线程不要有太多计算"""
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rate = rospy.Rate(self.send_velocity_fps)
        while True:
            send_twist = Twist()
            send_twist.linear.x = self.inference_twist.linear.x * self.movable
            send_twist.linear.y = self.inference_twist.linear.y * self.movable
            send_twist.linear.z = self.inference_twist.linear.z * self.movable   
            send_twist.angular.x = self.inference_twist.angular.x * self.movable
            send_twist.angular.y = self.inference_twist.angular.y * self.movable
            send_twist.angular.z = self.inference_twist.angular.z * self.movable
            self.pub.publish(send_twist)
            rate.sleep()

    def client_control_target(self):
        self.car_allready=rospy.Service('client_control', sup, self.client_control)
        rospy.loginfo("client_serv setup")
        self.car_allready.spin()

    def client_control(self,req):
        self.run_state = req.start
        rospy.loginfo("client_req")
        if req.collision:
            self.movable=0
        else:
            self.movable=req.movable
        rep = supResponse()
        rep.result = True
        rep.twist = self.inference_twist
        return rep