# set -e
# set -x
realsense_path="/home/pi/codefield/realsense-ros"
cd ${realsense_path}
source ./install/setup.bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x240x30