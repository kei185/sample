.PHONY: build visual init clean run stack display preview model nodes topics imu scan odom ekf map tf

init:
	colcon build --symlink-install --packages-select my_node --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-16


build:
	colcon build --packages-select my_node --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-16

visual:
	colcon build --packages-select my_visualization

clean:
	rm -rf build

run:
	ros2 launch my_node my_node.launch.py

stack:
	ros2 launch my_node stack.launch.py

display:
	ros2 launch my_visualization visualization.launch.py

preview:
	ros2 launch my_visualization visualization.launch.py fixed_frame:=base_footprint

model:
	ros2 launch my_node description.launch.py

nodes:
	ros2 node list --no-daemon --spin-time 10

topics:
	ros2 topic list --no-daemon --spin-time 10 -t

imu:
	ros2 topic echo /imu --once --qos-reliability best_effort

scan:
	ros2 topic echo /scan --once --qos-reliability best_effort

odom:
	ros2 topic echo /odom --once --qos-reliability best_effort

ekf:
	ros2 topic echo /odometry/filtered --once

map:
	ros2 topic echo /map --once --qos-durability transient_local --field info

tf:
	ros2 run tf2_ros tf2_echo map base_link
