#!/usr/bin/python3
import argparse 
from MultiVehicleEnv.environment import MultiVehicleEnv
import MultiVehicleEnv.scenarios as scenarios
from robot_host_pypkg.robot_host import RobotHost

def make_env(scenario_name, args):
    # load scenario from script
    scenario = scenarios.load(scenario_name + ".py").Scenario()
    # create world
    world = scenario.make_world(args)
    # create multiagent environment

    env = MultiVehicleEnv(world, scenario.reset_world, scenario.reward, scenario.observation,scenario.info,None,None)
    return env


if __name__ == '__main__':



    parser = argparse.ArgumentParser(description="GUI for Multi-VehicleEnv")
    parser.add_argument('--guiport',type=str,default='/dev/shm/gui_port')
    parser.add_argument('--fps',type=int,default=24)
    parser.add_argument('--usegui', action='store_true', default=False)
    parser.add_argument('--step-t',type=float,default=0.1)
    parser.add_argument('--sim-step',type=int,default=100)
    parser.add_argument('--direction_alpha', type=float, default=1.0)
    parser.add_argument('--num_agents', type=int, default=1)
    parser.add_argument('--ideal_side_len', type=float, default=5.0)
    parser.add_argument('--num_landmarks', type=int, default=1)
    parser.add_argument('--num_obstacles', type=int, default=0)
    
    parser.add_argument('--add_direction_encoder',type=str, default='train')

 
    try:
        args = parser.parse_args()
    except:
        args = parser.parse_args(['--usegui'])
    else:
        args = parser.parse_args()


    env = make_env('yyz', args)
    RobotHost = RobotHost(args, env)