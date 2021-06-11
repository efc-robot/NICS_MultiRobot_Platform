export ROS_MASTER_URI=http://172.16.1.11:11311
catkin_make --only-pkg-with-deps robot_client xtark_driver
source devel/setup.sh
export CAR_ID=`python src/mac2id.py`
chmod +x src/robot_client/scripts/robot_client.py
roslaunch xtark_driver xtark_driver.launch &
roslaunch robot_client robot_client.launch &