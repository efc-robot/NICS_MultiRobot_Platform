#!/usr/bin/python3

from robot_inference_pypkg.inference import RobotInference
import os 
import pickle

if __name__ == '__main__':
    car_id = os.getenv('CAR_ID')
    with open('/home/ubuntu/project/FastSimulatorExp/args.pkl','rb') as f:
        args_dict_= pickle.load(f)
    class ARGS:
        def __init__(self):
            pass
    args = ARGS()
    args.__dict__.update(args_dict_)
    client = RobotInference(car_id,args)