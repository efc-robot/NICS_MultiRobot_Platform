#!/usr/bin/python3

from robot_inference_pypkg.inference import RobotInference
import rospy

if __name__ == '__main__':
    rospy.init_node("robot_inference")
    car_id = rospy.get_param("~CAR_ID")
    client = RobotInference(car_id)