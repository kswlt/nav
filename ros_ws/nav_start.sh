# !/bin/bash


cd ~/ros_ws


# 运行第一个 ROS 2 启动文件
gnome-terminal -- bash -c "sudo chmod 777 /dev/ttyUSB0; exec bash"

sleep 5

# 运行第二个 ROS 2 启动文件，带参数
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
#     world:=zl2 \
#     slam:=False \
#     use_robot_state_pub:=False; exec bash"
# sleep 4



# # 运行第四个 ROS 2 启动文件
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch my_serial_py serial.launch.py; exec bash"
# sleep 3


# # 运行第三个 ROS 2 启动文件
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch my_nav_controller navigation_client.launch.py; exec bash"

gnome-terminal -- bash -c "source install/setup.bash && ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py; exec bash"

