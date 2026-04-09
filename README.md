source /opt/ros/jazzy/setup.bash
ros2 pkg create --build-type ament_cmake --dependencies rclcpp -- yathra


colcon build --packages-select yathra --symlink-install


colcon build --packages-select yathra --symlink-install
source install/setup.bash


sudo chmod 666 /dev/ttyUSB0
ros2 run yathra yathra_serial --ros-args -p port:=/dev/ttyUSB0

ros2 launch foxglove_bridge foxglove_bridge_launch.xml

ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:57600 gcs_url:=udp://@127.0.0.1:14550


ros2 run yathra dummy_depth


TODO
Feature,Your Old Code,New Solution,Benefit
Baud Rate,115200,921600,8x faster data transfer
Architecture,Polling Loop,Interrupt/Event Queue,Zero CPU waste waiting for bytes
Control Sync,Blocked by UART,Mailbox (xQueueOverwrite),Control loop runs at 1kHz freely
Robustness,Fragile (String tag),Framing (0xA5 Header),Auto-recovers from noise