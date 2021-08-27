#!/usr/bin/python3

from robot_inference_pypkg.inference import RobotInference
import os 

if __name__ == '__main__':
    car_id = os.getenv('CAR_ID')
    client = RobotInference(car_id)