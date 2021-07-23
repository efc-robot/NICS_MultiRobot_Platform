catkin_make --only-pkg-with-deps nics_robot_client xtark_driver
source devel/setup.sh
export CAR_ID=`python src/mac2id.py`
chmod +x src/nics_robot_client/scripts/robot_client.py
roslaunch nics_robot_client robot_client.launch &
roslaunch xtark_driver xtark_driver.launch &

