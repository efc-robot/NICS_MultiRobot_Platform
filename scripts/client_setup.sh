catkin_make  -DCATKIN_WHITELIST_PACKAGES=xtark_driver
catkin_make  -DCATKIN_WHITELIST_PACKAGES=robot_client
source devel/setup.sh
export CAR_ID=`python src/mac2id.py`
roslaunch xtark_driver xtark_driver.launch