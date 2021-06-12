#!/usr/bin/python3

from robot_client_pypkg.robot_client import RobotClient
import os 

if __name__ == '__main__':
    car_id = os.getenv('CAR_ID')
    client = RobotClient(car_id)