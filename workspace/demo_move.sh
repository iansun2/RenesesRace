# forward
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
# backward
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: -0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
# left turn
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
sleep 2
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
# right turn
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
sleep 2
ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
# forward, right turn
# ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# sleep 2
# ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
# sleep 2
# ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# sleep 2
# ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
# sleep 2
# ros2 topic pub --once /motor/main geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"



# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser