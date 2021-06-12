#!/usr/bin/python3
import argparse 
from MultiVehicleEnv.environment import MultiVehicleEnv
import MultiVehicleEnv.scenarios as scenarios
from robot_host_pypkg.dummy_host import FakeHost

def make_env(scenario_name, args):
    # load scenario from script
    scenario = scenarios.load(scenario_name + ".py").Scenario()
    # create world
    world = scenario.make_world(args)
    # create multiagent environment

    env = MultiVehicleEnv(world, scenario.reset_world, scenario.reward, scenario.observation,scenario.info)
    return env


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


    env = make_env('1p', args)
    fake_host = FakeHost(args, env)