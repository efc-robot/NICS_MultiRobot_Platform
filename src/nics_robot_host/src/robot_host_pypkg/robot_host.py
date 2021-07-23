#!/usr/bin/python3
import time
import math
import rospy
import rosnode
import copy
import threading
from geometry_msgs.msg import PoseStamped
from nics_robot_host.srv import *


class pos_data(object):
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

class RobotHost(object):
    def __init__(self, args, env):
        # get agent number from env
        self.agent_num = len(env.vehicle_list)
        # init host node
        rospy.init_node("robot_host")

        # TODO get this param from launch file
        self.core_fps = 10

        # check the number of agent client 
        All_ready = False
        while not All_ready:
            node_name_list:list[str] = rosnode.get_node_names()
            self.car_id_list = []
            for node_name in node_name_list:
                if node_name.endswith('robot_client'):
                    # assume all robot_client note named as '/XXXX/car_id/robot_client'
                    self.car_id_list.append(node_name.split('/')[-2])
            if len(self.car_id_list) == self.agent_num:
                All_ready = True
                break
            print(self.car_id_list)
            time.sleep(0.5)
        #build observation services
        self.obs_server_list = []
        for car_id in self.car_id_list:
            handle = lambda req: self.obs_calculate(car_id,req)
            obs_messenger = rospy.Service('/'+car_id+'/get_obs', obs, handle)
            self.obs_server_list.append(obs_messenger)
        
        self.env = env
        self.vrpn_list = [pos_data(0,0,0) for _ in range(self.agent_num)]

        self.client_ctrl_srv = []
        for car_id in self.car_id_list:
            client_ctrl_name = '/'+car_id+'/client_control'
            rospy.wait_for_service(client_ctrl_name)
            self.client_ctrl_srv.append(rospy.ServiceProxy(client_ctrl_name,sup))
        
        for vehicle_idx,car_id in enumerate(self.car_id_list):
            vrpn_pose_name = '/vrpn_client_node/'+car_id+'/pose'
            pose_call_back = lambda msg: self.update_vrpn_pose(msg, vehicle_idx)
            rospy.Subscriber(vrpn_pose_name, PoseStamped, pose_call_back)
        

        self.ros_spin_thread = threading.Thread(target=rospy.spin)
        self.ros_spin_thread.setDaemon(True)
        self.ros_spin_thread.start()

        state_flag = 'wait for pos'
        while True:
            cmd = input('state is %s, waiting for cmd '%state_flag)
            
            if state_flag == 'wait for pos':
                if cmd == 'pos':
                    self.env.reset()
                    while True:
                        result = self.waiting_for_vehicle()
                        if result is True:
                            rospy.loginfo('all agent is ready')
                            break
                        else:
                            rospy.loginfo(result)
                            time.sleep(1.0)
                    state_flag = 'wait for start'
                    rospy.loginfo('pos mode reset')
                
                if cmd == 'random':
                    self.env.reset()
                    self.random_set_vehicle()
                    state_flag = 'wait for start'
                    rospy.loginfo('random mode reset')

            if cmd == 'start' and state_flag == 'wait for start':
                state_flag = 'start'
                rospy.loginfo('start!')
                
                self.core_thread = threading.Thread(target=self.core_function)
                self.core_thread.setDaemon(True)
                self.core_thread.start()

                for idx in range(len(self.car_id_list)):
                    sup_arg = supRequest()
                sup_arg.movable = True
                sup_arg.collision = False
                self.client_ctrl_srv[idx](sup_arg)
            if cmd == 'exit':
                rospy.signal_shutdown('exit')
                break
            
    def random_set_vehicle(self):
        for agent, pos in zip(self.env.vehicle_list, self.vrpn_list):
            agent.state.coordinate[0] = pos.x
            agent.state.coordinate[1] = pos.y
            agent.state.theta = pos.theta

    def waiting_for_vehicle(self):
        def near_enough(x, y, yaw, x_t, y_t, yaw_t):
            #not near_enough distance of agent and the reset agent larger than 0.01m
            if ((x-x_t)**2 + (y-y_t)**2)**0.5 > 0.01:
                return False
            #if yaw and yaw_t distance is larger than 5 degree
            sin_con = math.sin(abs(yaw - yaw_t))<math.sin(5/180*3.1415)
            cos_con = math.cos(abs(yaw - yaw_t))>math.cos(5/180*3.1415)
            if not(sin_con and cos_con):
                return False
            return True

        for idx, agent, pos in zip(range(len(self.env.vehicle_list), self.env.vehicle_list, self.vrpn_list)):
            x_t = agent.state.coordinate[0]
            y_t = agent.state.coordinate[1]
            yaw_t = agent.state.theta
            x = pos.x
            y = pos.y
            yaw = pos.theta
            if not near_enough(x,y,yaw,x_t,y_t,yaw_t):
                info_str = "%s pos is (%f, %f, %f) but (%f, %f, %f) is required" %(self.car_id_list[idx], x,y,yaw, x_t,y_t,yaw_t)
                return info_str

        return True


    def update_vrpn_pose(self, msg, vehicle_idx):

        #seq = msg.header.seq
        #stamp = msg.header.stamp
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = -msg.pose.orientation.z
        qz = msg.pose.orientation.y
        qw = msg.pose.orientation.w

        #roll = math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
        #pitch = math.asin(2*(qw*qy-qz*qx))
        yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qz*qz+qy*qy))
        self.vrpn_list[vehicle_idx].x = x
        self.vrpn_list[vehicle_idx].y = z
        self.vrpn_list[vehicle_idx].theta = yaw


    def obs_calculate(self,car_id,req):
        rospy.loginfo("Calculate obs for car %s",car_id)
        car_index = self.car_id_list.index(car_id)
        agent = self.env.vehicle_list[car_index]
        obs_result = self.env._get_obs(agent)
        print(obs_result) 
        return obsResponse(obs_result)

    def core_function(self):
        self.start_time = time.time()
        while True:
            old_movable_list = copy.deepcopy([v.state.movable for v in self.env.vehicle_list])
            self._update_world()
            for v_idx in range(self.agent_num):
                v = self.env.vehicle_list[v_idx]
                if not(v.state.movable == old_movable_list[v_idx]):
                    sup_arg = supRequest()
                    sup_arg.movable = v.state.movable
                    sup_arg.collision = v.state.collision
                    self.client_ctrl_srv[v_idx](sup_arg)
            time.sleep(1.0/self.core_fps)

    def _update_world(self):
        total_time = time.time() - self.start_time
        self._set_state_callback()
        self.env.ros_step(total_time)

    def _set_state_callback(self):
        for vehicle_idx in range(self.agent_num):
            state = self.env.vehicle_list[vehicle_idx].state
            vrpn_data = self.vrpn_list[vehicle_idx]
            state.coordinate[0] = vrpn_data.x
            state.coordinate[1] = vrpn_data.y
            state.theta = vrpn_data.theta
