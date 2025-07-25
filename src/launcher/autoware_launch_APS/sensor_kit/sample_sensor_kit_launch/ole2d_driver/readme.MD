# install required dependencies
sudo apt-get install ros-humble-pcl-ros
sudo apt-get install libcurlpp-dev

colcon build --symlink-install
source install/setup.bash

# launch file 
ros2 launch olei_driver vf.launch.py​​

# visualize
rviz2 -f scanner_link 
