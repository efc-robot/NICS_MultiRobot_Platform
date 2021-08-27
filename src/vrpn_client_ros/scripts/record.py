#!/usr/bin/env python
import rospy
import pickle
from geometry_msgs.msg import PoseStamped

car_data_dict = {'c17816':[],
                 'c4491':[],
                 'c6563':[],
                 'c6438':[],
                 'c4641':[]}

car_count_dict = {'c17816':1,
                  'c4491':1,
                  'c6563':1,
                  'c6438':1,
                  'c4641':1}
def callback(msg, arg):
    car_name = arg[0]

    stamp = msg.header.stamp
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    car_data_dict[car_name].append((int(1000*(stamp.secs + stamp.nsecs *1e-9)), x, y, z, qx,qy,qz,qw))
    car_count_dict[car_name] += 1
    rospy.loginfo('record %s'%'/vrpn_client_node/' + str(car_name) + '/pose')
    if car_count_dict[car_name] % 100 == 0:
        f=open('/home/ubuntu/project/NICS_MultiRobot_Platform/opti_Data/%s.pkl' % car_name,'wb+')
        data = car_data_dict[car_name]
        pickle.dump(data, f)  
        f.close()  
        rospy.loginfo('dump %s'%'/vrpn_client_node/' + str(car_name) + '/pose')
        car_count_dict[car_name]=1



if __name__ == '__main__':
    rospy.init_node('record')
    for car_name in car_data_dict.keys():
        rospy.Subscriber('/vrpn_client_node/' + str(car_name) + '/pose', PoseStamped, callback, (car_name,) )
        rospy.loginfo('Subscribe %s'%'/vrpn_client_node/' + str(car_name) + '/pose')
    rospy.spin()