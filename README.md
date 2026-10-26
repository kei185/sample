# 探査機 毛玉 Probe KEDAMA

LiDAR: YDLIDAR T-mini Plus  
IMU: STMicroelectronics LSM6DSV16X  
Motor: DFRobot FIT0483  
Motor DriverTexas Instruments DRV8835  
MCU: STM32 NUCLEO-F446RE  
Computing: Raspberry Pi 5 8Gb RAM, 128Gb Storage

Including a submodule [sensor-flow](https://github.com/kei185/sensor-flow) at my_node/src/sensor-flow  
Supposed [sensor-echo](https://github.com/kei185/sensor-echo) to be run on the above MCU

`my_node` runs the sensor publishers, Commander, and Governor in one process.
`my_visualization` launches `rviz2` on the visualization PC.
Topic names below use the default launch files without a namespace.

## Nodes

| Node        | Package   | Role                                       |
| ----------- | --------- | ------------------------------------------ |
| `lidar`     | `my_node` | Publish LiDAR scans                        |
| `imu`       | `my_node` | Publish angular velocity and acceleration  |
| `encoder`   | `my_node` | Publish wheel-based body velocity          |
| `commander` | `my_node` | Convert velocity commands to motor PWM     |
| `governor`  | `my_node` | Find frontier goals and request navigation |

## Published Topics

| Node      | Topic   | Type                        | Description                                                          |
| --------- | ------- | --------------------------- | -------------------------------------------------------------------- |
| `lidar`   | `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR ranges                                                         |
| `imu`     | `/imu`  | `sensor_msgs/msg/Imu`       | Angular velocity and linear acceleration; orientation is unavailable |
| `encoder` | `/odom` | `nav_msgs/msg/Odometry`     | Body velocity from wheel encoders; pose is not integrated            |

Sensor input comes from sensor-flow queues populated from the MCU serial link.

## Subscribed Topics

| Node                     | Topic        | Type                         | Publisher package                                             | Description                                   |
| ------------------------ | ------------ | ---------------------------- | ------------------------------------------------------------- | --------------------------------------------- |
| `commander`              | `/cmd_vel`   | `geometry_msgs/msg/Twist`    | `nav2_collision_monitor`                                      | Forward velocity and yaw rate                 |
| `governor`               | `/map`       | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox`                                                | Map used for frontier detection               |
| `governor` (TF listener) | `/tf`        | `tf2_msgs/msg/TFMessage`     | `robot_localization`, `slam_toolbox`, `robot_state_publisher` | Dynamic transforms for the current robot pose |
| `governor` (TF listener) | `/tf_static` | `tf2_msgs/msg/TFMessage`     | `robot_state_publisher`                                       | Fixed model transforms                        |

The `/cmd_vel` publisher above is the final stage of the default Nav2 command path.
Commander sends the resulting PWM values through the serial command queue.

## Actions

| Client node | Action              | Type                              | Server package      | Description                 |
| ----------- | ------------------- | --------------------------------- | ------------------- | --------------------------- |
| `governor`  | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `nav2_bt_navigator` | Navigate to a frontier goal |

## References

- [Frontier Based Exploration for Autonomous Robot](https://arxiv.org/pdf/1806.03581)
- [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/README.md)
- [ROS2 Navigation2 Action Clientを使用してゴールを送信する](https://qiita.com/porizou1/items/cb9382bb2955c144d168)
