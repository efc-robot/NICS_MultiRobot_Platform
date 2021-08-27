#!/usr/bin/python3
import math
import rospy
import rostopic
import rosnode
import copy
import threading
from geometry_msgs.msg import PoseStamped
from nics_robot_host.srv import *
from rospy.core import rospyinfo


class RobotHost(object):
    def __init__(self, args, env):
        # get agent number from env
        self.agent_num = len(env.world.vehicle_list)
        # init host node
        rospy.init_node("robot_host")

        # TODO get this param from launch file
        self.core_fps = 10

        # check the number of agent client 
        All_ready = False
        while not All_ready:
            node_name_list:list[str] = rosnode.get_node_names()
            self.vehicle_id_list = []
            for node_name in node_name_list:
                if node_name.endswith('robot_client'):
                    # assume all robot_client note named as '/XXXX/vehicle_id/robot_client'
                    self.vehicle_id_list.append(node_name.split('/')[-2])
            if len(self.vehicle_id_list) == self.agent_num:
                All_ready = True
                break
            print(self.vehicle_id_list)
            rospy.sleep(0.5)


        #build observation services
        self.obs_server_list = []
        for vehicle_id in self.vehicle_id_list:
            handle = lambda req: self.obs_calculate(vehicle_id,req)
            obs_messenger = rospy.Service('/'+vehicle_id+'/get_obs', obs, handle)
            self.obs_server_list.append(obs_messenger)

        # update the agent data_interface
        self.env = env
        for vehicle_idx in range(self.agent_num):
            vehicle_id = self.vehicle_id_list[vehicle_idx]
            vehicle = self.env.world.vehicle_list[vehicle_idx]
            old_id = vehicle.vehicle_id
            vehicle.vehicle_id = vehicle_id
            interface = self.env.world.data_interface
            interface[vehicle_id] = interface.pop(old_id)
        
        # waiting for all topic
        self.ros_data_interface = {}
        for vehicle_id in self.vehicle_id_list:
            ros_data = {}
            for data_name in ['pose']:
                ros_data[data_name] = False
            self.ros_data_interface[vehicle_id] = ros_data
        
        #check for all topic
        rospyinfo('check for all topic ready')
        all_data_interface_ready = False
        while not all_data_interface_ready:
            all_data_interface_ready = True
            for v_id, inter in self.ros_data_interface.items():
                for data_name,state in inter.items():
                    if state is False:
                        print('/'+v_id+'/'+ data_name + ' is not found')
                        all_data_interface_ready = False
                   
            topic_list = rospy.get_published_topics()
            for topic in topic_list:
                topic_name:str = topic[0]
                topic_name_split = topic_name.split('/')
                v_id = topic_name_split[1]
                if v_id in self.ros_data_interface.keys():
                    data_name = topic_name_split[2]
                    if data_name in self.ros_data_interface[v_id]:
                        self.ros_data_interface[v_id][data_name] = True
            rospy.sleep(1.0)


        self.ros_data_interface_sub = []
        #subscribe all ros data interface
        for vehicle_id in self.vehicle_id_list:
            for data_name in self.ros_data_interface[vehicle_id].keys():
                handle = lambda msg: self.store_data(msg, vehicle_id, data_name)
                topic_name = '/'+vehicle_id+'/'+data_name
                data_class = rostopic.get_topic_class(topic_name)[0]
                sub = rospy.Subscriber('/'+vehicle_id+'/'+data_name, data_class, handle)
                self.ros_data_interface_sub.append(sub)


        #check for all client control services
        self.client_ctrl_srv = []
        for vehicle_id in self.vehicle_id_list:
            client_ctrl_name = '/'+vehicle_id+'/client_control'
            rospy.wait_for_service(client_ctrl_name)
            self.client_ctrl_srv.append(rospy.ServiceProxy(client_ctrl_name,sup))
        
        

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
                            rospy.sleep(0.1)
                    state_flag = 'wait for start'
                    rospy.loginfo('pos mode reset')
                
                if cmd == 'random':
                    self.env.reset()
                    for agent in self.env.world.vehicle_list:
                        ros_data_interface = self.ros_data_interface[agent.vehicle_id]
                        agent.state.coordinate[0] = ros_data_interface['pose'].twist.linear.x
                        agent.state.coordinate[1] = ros_data_interface['pose'].twist.linear.y
                        agent.state.theta = ros_data_interface['pose'].twist.angular.z
                    state_flag = 'wait for start'
                    rospy.loginfo('random mode reset')

            if cmd == 'start' and state_flag == 'wait for start':
                state_flag = 'start'
                rospy.loginfo('start!')
                
                self.core_thread = threading.Thread(target=self.core_function)
                self.core_thread.setDaemon(True)
                self.core_thread.start()

                for idx in range(len(self.vehicle_id_list)):
                    sup_arg = supRequest()
                sup_arg.movable = True
                sup_arg.collision = False
                self.client_ctrl_srv[idx](sup_arg)
            if cmd == 'exit':
                rospy.signal_shutdown('exit')
                break
    
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


        for agent in self.env.world.vehicle_list:
            ros_data_interface = self.ros_data_interface[agent.vehicle_id]
            x_t = agent.state.coordinate[0]
            y_t = agent.state.coordinate[1]
            yaw_t = agent.state.theta
            x = ros_data_interface['pose'].twist.linear.x
            y = ros_data_interface['pose'].twist.linear.y
            yaw = ros_data_interface['pose'].twist.angular.z
            if not near_enough(x,y,yaw,x_t,y_t,yaw_t):
                info_str = "%s pos is (%f, %f, %f) but (%f, %f, %f) is required" %(agent.vehicle_id, x,y,yaw, x_t,y_t,yaw_t)
                return info_str
        return True

    def store_data(self, msg, v_id, data_name):
        self.ros_data_interface[v_id][data_name] = copy.deepcopy(msg)

    def obs_calculate(self,vehicle_id,req):
        rospy.loginfo("Calculate obs for car %s",vehicle_id)
        car_index = self.vehicle_id_list.index(vehicle_id)
        agent = self.env.world.vehicle_list[car_index]
        obs_result = self.env._get_obs(agent)
        print(obs_result) 
        return obsResponse(obs_result)

    def core_function(self):
        self.start_time = rospy.get_time()
        rate = rospy.Rate(self.core_fps)
        while True:
            old_movable_list = copy.deepcopy([v.state.movable for v in self.env.world.vehicle_list])
            total_time = rospy.get_time() - self.start_time
            self._update_data_interface()
            self.env.ros_step(total_time)
            for v_idx in range(self.agent_num):
                v = self.env.world.vehicle_list[v_idx]
                if not(v.state.movable == old_movable_list[v_idx]):
                    sup_arg = supRequest()
                    sup_arg.movable = v.state.movable
                    sup_arg.collision = v.state.crashed
                    self.client_ctrl_srv[v_idx](sup_arg)
            rate.sleep()

    def _update_data_interface(self):
        for vehicle_idx in range(self.agent_num):
            vehicle_id = self.vehicle_id_list[vehicle_idx]
            data_interface = self.env.world.data_interface[vehicle_id]
            ros_data_interface = self.ros_data_interface[vehicle_id]
            data_interface['x'] = ros_data_interface['pose'].twist.linear.x
            data_interface['y'] = ros_data_interface['pose'].twist.linear.y
            data_interface['theta'] = ros_data_interface['pose'].twist.angular.z