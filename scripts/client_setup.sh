export ROS_MASTER_URI=http://172.16.1.11:11311
catkin_make --only-pkg-with-deps nics_robot_client xtark_driver
source devel/setup.sh
export CAR_ID=`python src/mac2id.py`
chmod +x src/nics_robot_client/scripts/robot_client_node.py
roslaunch src/xtark_driver/launch/xtark_driver.launch &
roslaunch src/nics_robot_client/launch/robot_client.launch &
