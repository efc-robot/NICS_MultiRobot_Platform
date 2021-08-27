catkin_make --only-pkg-with-deps nics_robot_client xtark_driver
source devel/setup.sh
export ROS_MASTER_URI=http://192.168.50.129:11311
export CAR_ID=`python src/mac2id.py`
chmod +x src/nics_robot_client/scripts/robot_client_node.py
roslaunch scripts/client.launch
