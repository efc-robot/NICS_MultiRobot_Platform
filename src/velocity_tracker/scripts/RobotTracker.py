#!/usr/bin/env python
#coding=utf-8
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
import csv
import time
from geometry_msgs.msg import Twist
import threading
from geometry_msgs.msg import PoseStamped
from random import *


class RobotTracker():
    def __init__(self):
        self.start_time = time.time()
        self.lvel = {}
        self.rvel = {}
        self.acc = {}
        self.move_cmd = Twist()
        self.vrpn_data = {}
        self.lock = [threading.Lock()]*5
        self.starting_vrpn_stamp = None
        self.cmd_record = {}

        # ROS节点初始化
        rospy.init_node('RobotTracker', anonymous=False)

        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        # 创建一个Subscriber，注册回调函数
        rospy.Subscriber("/imu", Imu, self.poseCallback_imu)
        rospy.Subscriber("/xtark/lvel", Int32, self.poseCallback_lvel)
        rospy.Subscriber("/xtark/rvel", Int32, self.poseCallback_rvel)
        rospy.Subscriber("/vrpn_client_node/AKM_5/pose", PoseStamped, self.poseCallback_vrpn)


        # rospy.on_shutdown(self.export_data)

        t = threading.Thread(target=self.publish_data,args=()) 
        t.setDaemon(True)
        t.start()

        t1 = threading.Thread(target=self.export_data,args=()) 
        t1.setDaemon(True)
        t1.start()

        # 循环等待回调函数
        rospy.spin()

    def current_time(self, time_stamp):
        return time_stamp - self.start_time

    def poseCallback_imu(self, msg):
        self.lock[0].acquire()
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        # rospy.loginfo("IMU data: Linear Accelaration x, y, z")
        # rospy.loginfo("%0.6f, %0.6f, %0.6f", acc_x, acc_y, acc_z)
        
        self.acc[self.current_time(time.time())]=[acc_x, acc_y, acc_z]
        self.lock[0].release()

    def poseCallback_rvel(self, msg):
        # rospy.loginfo("Rvel data: %s", msg)
        self.lock[1].acquire()
        self.lvel[self.current_time(time.time())] = [msg.data]
        self.lock[1].release()

    def poseCallback_lvel(self, msg):
        # rospy.loginfo("Lvel data: %s", msg)
        self.lock[2].acquire()
        self.rvel[self.current_time(time.time())] = [msg.data]
        self.lock[2].release()

    def poseCallback_vrpn(self, msg):
        if self.starting_vrpn_stamp is None:
            self.starting_vrpn_stamp = msg.header.stamp
        self.lock[3].acquire()
        self.vrpn_data[msg.header.stamp-self.starting_vrpn_stamp] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.lock[3].release()

    def write_dict(self, data_dict, csv_writer, data_file):
        for key in data_dict:
            entry_list = []
            for index in range(len(data_dict[key])):
                entry_list.append(data_dict[key][index])
            entry_list.append(key)
            csv_writer.writerow(entry_list)
        data_file.close()
        
    def export_data(self):
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            file_acc = open("acceleration.csv", "w")
            csv_write_acc = csv.writer(file_acc,dialect='excel')
            file_L_wheel = open("L_wheel.csv", "w")
            csv_write_L_wheel = csv.writer(file_L_wheel,dialect='excel')
            file_R_wheel = open("R_wheel.csv", "w")
            csv_write_R_wheel = csv.writer(file_R_wheel,dialect='excel')
            file_vrpn = open("vrpn.csv", "w")
            csv_write_vrpn = csv.writer(file_vrpn,dialect='excel')
            file_cmd = open("cmd.csv", "w")
            csv_write_cmd = csv.writer(file_cmd, dialect = 'excel')
            
            self.lock[0].acquire()
            lvel_buffer = self.lvel.copy()
            self.lock[0].release()
            self.lock[1].acquire()            
            rvel_buffer = self.rvel.copy()
            self.lock[1].release()
            self.lock[2].acquire()
            acc_buffer = self.acc.copy()
            self.lock[2].release()
            self.lock[3].acquire()
            vrpn_buffer = self.vrpn_data.copy()
            self.lock[3].release()
            self.lock[4].acquire()
            cmd_buffer = self.cmd_record.copy()
            self.lock[4].release()

            
            
            self.write_dict(lvel_buffer, csv_write_L_wheel, file_L_wheel)
            self.write_dict(rvel_buffer, csv_write_R_wheel, file_R_wheel)
            self.write_dict(acc_buffer, csv_write_acc, file_acc)
            self.write_dict(vrpn_buffer, csv_write_vrpn, file_vrpn)
            self.write_dict(cmd_buffer, csv_write_cmd, file_cmd)
            rate.sleep()


    def publish_data(self):
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            [x_vel, y_vel, z_vel, x_ang, y_ang, z_ang] = [0.1, 0, 0, 0, 0.5, -0.5]
            # [x_vel, y_vel, z_vel, x_ang, y_ang, z_ang] = [random(), 0, 0, 0, 0, -0.5]
            self.move_cmd.linear.x = x_vel
            self.move_cmd.linear.y = y_vel
            self.move_cmd.linear.z = z_vel
            self.move_cmd.angular.x = x_ang
            self.move_cmd.angular.y = y_ang
            self.move_cmd.angular.z = z_ang
            self.vel_pub.publish(self.move_cmd)
            self.lock[4].acquire()
            self.cmd_record[self.current_time(time.time())] = [self.move_cmd.linear.x, self.move_cmd.angular.z]
            self.lock[4].release()
            rate.sleep()

        
   


if __name__ == '__main__':
        RobotTracker()