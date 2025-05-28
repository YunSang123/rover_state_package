```
# make rover_workspace and src folder
mkdir -p osr_ws/src
cd osr_ws/src
git clone https://github.com/YunSang123/rover_state_package.git
cd rover_state_package

# Download zed-ros2 package
# zed-ros2-examples와 zed-ros2-wrapper 폴더로 각각 들어가서 둘 다 git checkout humble-v4.2.5 수행 ㄱㄱ
git clone https://github.com/stereolabs/zed-ros2-examples.git
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

# Install dependencies package
# For jetson nano developer kit, jetpack4.6.2, ubuntu20.04, ros2-humble
cd osr_ws/src
mkdir dependencies
cd dependencies
# git clone all of the following packages and use git checkout to switch branches
# angles -> humble-devel
# cob_common-2.7.10
# diagnostics -> ros2-humble
# geographic_info -> ros2
# nmea_msgs -> ros2
# point_cloud_transport -> humble
# robot_localization -> humble-devel
# xacro -> ros2
# zed-ros2-interfaces -> master

# Source install/setup.bash
# Building package in jetson nano is so slow that I set parallel-workers option to 2
cd osr_ws
colcon build --symlink-install --parallel-workers 2

# 
```