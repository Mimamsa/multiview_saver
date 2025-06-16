# Multiview Saver

This ROS2 node subscribes data from a RGBD camera and a robot arm periodically, and saves data to files if request received.

The node subscribes messages the following topics:
1. "/camera/color/image_raw" (sensor_msgs/msg/Image)
2. "/camera/depth_registered/points" (sensor_msgs/msg/PointCloud2)
3. "/tcp_pose_broadcaster/pose" (geometry_msgs/msg/PoseStamped)
4. "/joint_states" (sensor_msgs/msg/JointState)


## Dependencies

This ROS2 node cooperates with the following tools:
1. [Universal Robots ROS2 Driver (humble branch) - github.com](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
2. [OrbbecSDK ROS2 (v2-main branch) - github.com](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)


## Environment

- Ubuntu 22.04
- Python 3.10
- ROS2 Humble


## Build

### 1. Create directories

For my example, all ROS2 package are in ```~/ros2_ws/src``` directory.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Mimamsa/multiview_saver
```

### 2. Build individual package

```
cd ~/ros2_ws
colcon build --packages-select multiview_saver
```


## Usage

### Launch node

```
ros2 launch multiview_saver save_three_view.launch.py
```


### Save data of the view

This will trigger the node to save:
1. RGB image (rgb_img_{save_count}.png)
2. Robot states (robot_state_{save_count}.yaml)
3. Colored point clouds (pcd_{save_count}.glb)

The files will be saved to the working directory where the node launched.

```
ros2 service call /multiview_saver/capture_point std_srvs/srv/Trigger {}
```


### Restart save count

This will trigger the node to reset ```save_count``` to 1. This effects the file name to be used in the next data saving.
```
ros2 service call /multiview_saver/restart_count std_srvs/srv/Trigger {}
```


## Information

- Author: Yu-Hsien Chen (mike_chen@wistron.com)
- Latest update: 2025/6/11

