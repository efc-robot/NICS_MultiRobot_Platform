import rospy
import time
import math
import threading
from geometry_msgs.msg import PoseStamped,TwistStamped

class VrpnPose(object):
    def __init__(self):
        prefix = 'vrpn_client_node'
        postfix = 'pose'
        rospy.init_node('VrpnPose')
        self.transfer = {}

        self.ros_spin_thread = threading.Thread(target=rospy.spin)
        self.ros_spin_thread.setDaemon(True)
        self.ros_spin_thread.start()

        while not rospy.is_shutdown():
            topic_list = rospy.get_published_topics()
            for topic in topic_list:
                vrpn_topic_name:str = topic[0]
                vrpn_topic_split = vrpn_topic_name.split('/')
                if vrpn_topic_split[1] == prefix and vrpn_topic_split[-1] == postfix:
                    car_id = vrpn_topic_split[2]
                    if car_id not in self.transfer.keys():
                        sub = rospy.Subscriber(vrpn_topic_name, PoseStamped, self.transfer_callback,(car_id,))
                        pub = rospy.Publisher('/'+car_id+'/pose',TwistStamped)
                        self.transfer[car_id] = {'sub':sub, 'pub':pub}
            print(self.transfer)
            time.sleep(1)

    def transfer_callback(self, msg:PoseStamped, args):
        car_id = args[0]
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qw = msg.pose.orientation.w
        qz = msg.pose.orientation.z
        roll = math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
        pitch = math.asin(2*(qw*qy-qz*qx))
        yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qz*qz+qy*qy))
        
        twist = TwistStamped()
        twist.header = msg.header
        twist.twist.linear.x = msg.pose.position.x
        twist.twist.linear.y = msg.pose.position.y
        twist.twist.linear.z = msg.pose.position.z
        twist.twist.angular.x = roll
        twist.twist.angular.y = pitch
        twist.twist.angular.z = yaw
        pub:rospy.Publisher = self.transfer[car_id]['pub']
        pub.publish(twist)