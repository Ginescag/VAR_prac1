source catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-turtlebot3-msgs pip 
sudo pip install numpy==1.24.3
sudo pip install tensorflow ultralytics opencv-python cv_bridge