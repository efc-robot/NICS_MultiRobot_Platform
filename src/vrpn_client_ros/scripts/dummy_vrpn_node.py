#!/usr/bin/python3
from math import pi
import time
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import threading
import argparse
import sys
class FakeVrpn(object):
    def __init__(self,name='car_0'):
        rospy.init_node("dummpy_vrpn")
        self.pub = rospy.Publisher('/vrpn_client_node/'+name+'/pose', PoseStamped , queue_size=10)
    
    def pubmsg(self, msg):
        self.pub.publish(msg)

def euler2Q(pitch,yaw,roll):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    qw = cy * cp * cr + sy * sp * sr
    return (qx,qy,qz,qw)

def Q2euler(qx,qy,qz,qw):
    roll = np.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
    pitch = np.asin(2*(qw*qy-qz*qx))
    yaw = np.atan2(2*(qw*qz+qx*qy),1-2*(qz*qz+qy*qy))
    
    return pitch,yaw,roll


def run_fake_object(fake_vrpn):
    '''
    WARNING!!! 
    In this file we use:
        x as the first axis which is horizontal,
        y as the second axis  which is vertical,
        z as the third axis which is horizontal,
        pitch twist about x,
        yaw twist about y,
        roll twist about z,
    '''
    
    Radius = 1.0
    seq = 0
    roll = 0
    pitch = 0
    yaw = 3.1415/2
    x = Radius
    y = 0
    z = 0
    while True:
        time.sleep(1)
        msg = PoseStamped()
        msg.header.seq = seq

        x = Radius * np.cos(yaw-3.1415/2)
        z = -Radius * np.sin(yaw-3.1415/2)
        msg.pose.position.x = x
        msg.pose.position.y = 0
        msg.pose.position.z = z
        
        qx,qy,qz,qw = euler2Q(pitch, yaw, roll)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        fake_vrpn.pubmsg(msg)

        yaw+=0.05
        seq+=1

if __name__ == '__main__' :
    parser = argparse.ArgumentParser(description="Fake vrpn")
    parser.add_argument('--name',type=str,default='car_0')
    try:
        args = parser.parse_args()
    except:
        args = parser.parse_args([])
    else:
        args = parser.parse_args()
        
    fake_vrpn = FakeVrpn(args.name)
    t = threading.Thread(target=run_fake_object,args=(fake_vrpn,))
    t.setDaemon(True)
    t.start()
    while True:
        cmd = input('waiting for cmd: ')
        if cmd == 'x':
            break
