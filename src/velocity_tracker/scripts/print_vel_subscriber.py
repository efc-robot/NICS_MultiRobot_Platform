#!/usr/bin/env python
#coding=utf-8
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
import csv
import time

start_time = time.time()
lvel = {}
rvel = {}
acc = {}

data_already_exported = False
def current_time(time_stamp):
    return time_stamp - start_time
def poseCallback_imu(msg):
    global data_already_exported
    acc_x = msg.linear_acceleration.x
    acc_y = msg.linear_acceleration.y
    acc_z = msg.linear_acceleration.z
    rospy.loginfo("IMU data: Linear Accelaration x, y, z")
    rospy.loginfo("%0.6f, %0.6f, %0.6f", acc_x, acc_y, acc_z)
    
    acc[current_time(time.time())]=[acc_x, acc_y, acc_z]
    # rospy.loginfo("current time is %f", time.time())
    # duration = 1
    # if time.time() > start_time + duration and not data_already_exported:
    #     export_data()
        
    #     data_already_exported = True
    #     exit()
        



def poseCallback_rvel(msg):
    # rospy.loginfo("Rvel data: %s", msg)
    lvel[current_time(time.time())] = [msg.data]

def poseCallback_lvel(msg):
    # rospy.loginfo("Lvel data: %s", msg)
    rvel[current_time(time.time())] = [msg.data]

def write_dict(data_dict, csv_writer, data_file):
    for key in data_dict:
        entry_list = []
        for index in range(len(data_dict[key])):
            entry_list.append(data_dict[key][index])
        entry_list.append(key)
        csv_writer.writerow(entry_list)
    data_file.close()
    
def export_data():
    file_acc = open("acceleration.csv", "w")
    csv_write_acc = csv.writer(file_acc,dialect='excel')
    file_L_wheel = open("L_wheel.csv", "w")
    csv_write_L_wheel = csv.writer(file_L_wheel,dialect='excel')
    file_R_wheel = open("R_wheel.csv", "w")
    csv_write_R_wheel = csv.writer(file_R_wheel,dialect='excel')

    write_dict(lvel, csv_write_L_wheel, file_L_wheel)
    write_dict(rvel, csv_write_R_wheel, file_R_wheel)
    write_dict(acc, csv_write_acc, file_acc)


def pose_subscriber():
	# ROS节点初始化
    rospy.init_node('pose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/imu", Imu, poseCallback_imu)
    rospy.Subscriber("/xtark/lvel", Int32, poseCallback_lvel)
    rospy.Subscriber("/xtark/rvel", Int32, poseCallback_rvel)

    rospy.on_shutdown(export_data)

	# 循环等待回调函数
    rospy.spin()

pose_subscriber()

# import csv
 
# a = ['sdvvd','vdvd']   # 要写入的数据
# b = ['dcd','dcdvd']
 
# out = open(filename,'a',newline='')    # 打开要写入的文件
 
# csv_write = csv.writer(out,dialect='excel')   # 设定写入的模式，dialect就是定义文件的类型
#                                               # 此处将其定义为了excel
 
# csv_write.writerow(a)        # 写入
# csv_write.writerow(b)
 
# out.close()      # 关闭文件
