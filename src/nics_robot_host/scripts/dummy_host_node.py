#!/usr/bin/python3
import argparse 
from MultiVehicleEnv.environment import MultiVehicleEnv
import MultiVehicleEnv.scenarios as scenarios
from robot_host_pypkg.dummy_host import DummyHost

if __name__ == '__main__':


    parser = argparse.ArgumentParser(description="GUI for Multi-VehicleEnv")
    parser.add_argument('--gui-port',type=str,default='/dev/shm/gui_port')
    parser.add_argument('--usegui', action='store_true', default=False)
    parser.add_argument('--step-t',type=float,default=1.0)
    parser.add_argument('--sim-step',type=int,default=100)
    parser.add_argument('--direction_alpha', type=float, default=1.0)
    parser.add_argument('--add_direction_encoder',type=str, default='train')

 
    try:
        args = parser.parse_args()
    except:
        args = parser.parse_args([])
    else:
        args = parser.parse_args()


    fake_host = DummyHost(args)