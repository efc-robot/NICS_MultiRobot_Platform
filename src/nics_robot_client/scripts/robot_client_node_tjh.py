#!/usr/bin/python3

from robot_client_pypkg.robot_client import RobotClient
import rospy 

if __name__ == '__main__':
    rospy.init_node("nics_robot_client")
    car_id = rospy.get_param("~CAR_ID")
    client = RobotClient(car_id)