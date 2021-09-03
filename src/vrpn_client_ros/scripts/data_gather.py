#!/usr/bin/env python
import rospy
import pickle
from geometry_msgs.msg import TwistStamped


class data_gather(object):
    def __init__(self):
        rospy.init_node('record')
        self.record_data = {}
        while not rospy.is_shutdown():
            topic_list = rospy.get_published_topics()
            for topic in topic_list:
                vrpn_topic_name:str = topic[0]
                vrpn_topic_split = vrpn_topic_name.split('/')
                
                if len(vrpn_topic_split)==3 and vrpn_topic_split[-1] == 'pose':
                    car_id = vrpn_topic_split[1]
                    if car_id not in self.record_data.keys():
                        self.record_data[car_id] = {'data':[]}
                        sub = rospy.Subscriber(vrpn_topic_name, TwistStamped, self.record_data_callback, (car_id,))
                        self.record_data[car_id]['sub'] = sub
                        self.record_data[car_id]['file'] = '/home/ubuntu/project/NICS_MultiRobot_Platform/'+car_id+'_left_forward.pkl'
                        print('subscribe ' + car_id)
            
            rospy.sleep(1.0)
    
    def record_data_callback(self, msg:TwistStamped, args):
        car_id = args[0]
        stamp = msg.header.stamp.to_nsec()
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        theta = msg.twist.angular.z

        self.record_data[car_id]['data'].append([stamp, x, y, theta])
        if (len(self.record_data[car_id]['data'])+1)%100 == 0:
            print(self.record_data[car_id]['file'])
            with open(self.record_data[car_id]['file'], 'wb') as f:
                pickle.dump(self.record_data[car_id]['data'], f)

        

if __name__ == '__main__':
    gather = data_gather()
