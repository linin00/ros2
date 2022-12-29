source ./install/setup.bash
bash ./scripts/utils/setup_can2usb.bash
while true; do ros2 run robot robot; done;