# set -e
# set -x
realsense_path="/home/pi/codefield/realsense-ros"
config_path="/home/pi/codefield/ros2/config/d435.yaml"
cd ${realsense_path}
source ./install/setup.bash
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x240x30
ros2 launch realsense2_camera rs_launch.py config_file:="'${config_path}'"