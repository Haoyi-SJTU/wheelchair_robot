# Mobile Robot ROS Operation Guide

## Overview
This document provides an ROS (Robot Operating System) operation guide for the mobile robot system, covering two main parts: mapping and navigation. Mapping supports two methods: gmapping and cartographer; please choose one based on actual needs. The navigation section provides a complete startup process and operation instructions.

![](https://github.com/Haoyi-SJTU/wheelchair_robot/blob/main/fig/Snipaste_2026-03-20_14-51-13.png)
![](https://github.com/Haoyi-SJTU/laser_data_fusion/blob/main/rviz.jpg)

## Table of Contents
- [Mapping](#mapping)
  - [Mapping with gmapping](#mapping-with-gmapping)
  - [Mapping with cartographer](#mapping-with-cartographer)
- [Navigation](#navigation)
- [Notes](#notes)

## Mapping
Choose either of the two mapping methods.

### Mapping with gmapping
**Steps:**

1. **Set USB device permissions** (may need to be executed after every reboot):
   ```bash
   sudo chmod 777 /dev/ttyUSB0
   sudo chmod 777 /dev/ttyUSB1
   sudo chmod 777 /dev/ttyUSB2
   ```

2. **Start nodes in order** (recommended to execute in separate terminals):
   ```bash
   roscore
   rosrun N_Robot_Topic NMotionCtrl_X64_Topic /dev/ttyUSB2
   rosrun robot_base_odometry new_robot_base_odometry
   # Wait 5 seconds until data output appears in the terminal before proceeding
   roslaunch rplidar_ros test0_1.launch
   roslaunch robot_base_mapping mapping.launch
   ```

3. **Visualization (Optional):**
   ```bash
   rosrun rviz rviz
   ```
   In RViz, add the `Map` module and select the corresponding topic to view real-time mapping.

4. **Save Map:**
   After mapping is complete, run the following command to save the map:
   ```bash
   rosrun map_server map_saver -f <save_path/filename>
   ```
   **Example:**
   ```bash
   rosrun map_server map_saver -f ~/map_1
   ```
   This will generate `map_1.pgm` (map image) and `map_1.yaml` (map info file). After saving, close the mapping process (`mapping.launch`).

### Mapping with cartographer
**Applicable Scenario:** Recommended when remote control experiences latency.

**Steps:**

1. **Start LiDAR node:**
   ```bash
   roslaunch rplidar_ros test0_1.launch
   ```

2. **Start cartographer mapping:**
   ```bash
   roslaunch cartographer_ros demo_revo_lds_rplidar.launch
   ```

3. **Visualization (Optional):**
   ```bash
   rosrun rviz rviz
   ```
   If TF errors occur in RViz, they can be ignored as long as the map displays correctly.

4. **Save Map:**
   ```bash
   rosservice call /finish_trajectory 0
   rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"
   rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05
   ```

## Navigation
**Steps:**

1. **Set USB device permissions:**
   ```bash
   sudo chmod 777 /dev/ttyUSB0
   sudo chmod 777 /dev/ttyUSB1
   sudo chmod 777 /dev/ttyUSB2
   ```

2. **Start nodes:**
   ```bash
   rosrun N_Robot_Topic NMotionCtrl_X64_Topic /dev/ttyUSB2
   rosrun robot_base_odometry new_robot_base_odometry
   ```

3. **Choose one navigation launch file to start (select one of three):**
   ```bash
   roslaunch robot_base_navigation new.launch
   # OR
   roslaunch robot_base_navigation teb_nav.launch
   # OR
   roslaunch robot_base_navigation nav_with_people.launch
   ```

4. **Visualization (Optional):**
   ```bash
   rosrun rviz rviz
   ```

5. **Localization Initialization:**
   - Use the `2D Pose Estimate` tool in RViz to mark the robot's initial pose on the map.
   - Visualize the `/particlecloud` topic of type `PoseArray`.
   - Move the robot a short distance using the remote control. If the red arrows gradually converge and align with the actual pose, localization is successful.

6. **Start Navigation:**
   - **Method 1:** Use the `2D Nav Goal` tool in RViz to directly specify the target pose.
   - **Method 2:** Publish action commands via a node.

## Notes
- **Device Ports:** The default configuration in launch files is: LiDAR near the manipulator — `/dev/ttyUSB0`, other LiDAR — `/dev/ttyUSB1`, robot chassis — `/dev/ttyUSB2`. If devices do not match ports, it is recommended to re-plug USB devices and connect them in this order.
- **Startup Sequence:** When mapping or navigating, strictly follow the documented node startup sequence. Especially after starting the odometry node, wait 5 seconds before moving the robot.
- **Map Saving:** Save the map promptly after gmapping is complete; for cartographer, follow the steps to save and convert formats.
- **Navigation Configuration:** Before starting navigation, ensure the map file path and name loaded by the `map_server` node are correct (modify in the launch file).
- **Coordinate Frame Alignment:** During mapping, it is recommended to align the robot's initial pose with the map origin to simplify subsequent coordinate transformations.


---

# 移动机器人ROS操作指南

## 概述
本文档提供了移动机器人系统的ROS（Robot Operating System）操作指南，包括建图与导航两大部分。建图支持两种方式：gmapping和cartographer，请根据实际需求选择其一。导航部分提供了完整的启动流程与操作说明。

![](https://github.com/Haoyi-SJTU/wheelchair_robot/blob/main/fig/Snipaste_2026-03-20_14-51-13.png)
![](https://github.com/Haoyi-SJTU/laser_data_fusion/blob/main/rviz.jpg)

## 目录
- #建图
  - #使用gmapping建图
  - #使用cartographer建图
- #导航
- #注意事项


## 建图
两种建图方式任选其一即可。

### 使用gmapping建图
**步骤：**
1. **设置USB设备权限**（每次重启后可能需要执行）：
   ```bash
   sudo chmod 777 /dev/ttyUSB0
   sudo chmod 777 /dev/ttyUSB1
   sudo chmod 777 /dev/ttyUSB2
   ```

2. **按顺序启动节点**（建议在多个终端中分别执行）：
   ```bash
   roscore
   rosrun N_Robot_Topic NMotionCtrl_X64_Topic /dev/ttyUSB2
   rosrun robot_base_odometry new_robot_base_odometry
   # 等待5秒，直到终端有数据输出后再继续
   roslaunch rplidar_ros test0_1.launch
   roslaunch robot_base_mapping mapping.launch
   ```

3. **可视化**（可选）：
   ```bash
   rosrun rviz rviz
   ```
   在RViz中添加`Map`模块，并选择对应的topic即可查看实时建图。

4. **保存地图**：
   建图完成后，运行以下命令保存地图：
   ```bash
   rosrun map_server map_saver -f <保存路径/文件名>
   ```
   **示例**：
   ```bash
   rosrun map_server map_saver -f ~/map_1
   ```
   将生成`map_1.pgm`（地图图片）和`map_1.yaml`（地图信息文件）。保存后关闭建图进程（`mapping.launch`）。


### 使用cartographer建图
**适用场景**：当遥控器控制出现延迟时，推荐使用此方法。

**步骤：**
1. **启动激光节点**：
   ```bash
   roslaunch rplidar_ros test0_1.launch
   ```

2. **启动cartographer建图**：
   ```bash
   roslaunch cartographer_ros demo_revo_lds_rplidar.launch
   ```

3. **可视化**（可选）：
   ```bash
   rosrun rviz rviz
   ```
   若RViz中tf报错，可忽略，只要地图正常显示即可。

4. **保存地图**：
   ```bash
   rosservice call /finish_trajectory 0
   rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"
   rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05
   ```

---

## 导航
**步骤：**
1. **设置USB设备权限**：
   ```bash
   sudo chmod 777 /dev/ttyUSB0
   sudo chmod 777 /dev/ttyUSB1
   sudo chmod 777 /dev/ttyUSB2
   ```

2. **启动节点**：
   ```bash
   rosrun N_Robot_Topic NMotionCtrl_X64_Topic /dev/ttyUSB2
   rosrun robot_base_odometry new_robot_base_odometry
   ```
   选择一种导航launch文件启动（三选一）：
   ```bash
   roslaunch robot_base_navigation new.launch
   roslaunch robot_base_navigation teb_nav.launch
   roslaunch robot_base_navigation nav_with_people.launch
   ```

3. **可视化**（可选）：
   ```bash
   rosrun rviz rviz
   ```

4. **定位初始化**：
   - 在RViz中使用`2D Pose Estimate`工具，在地图上标记机器人初始位姿。
   - 可视化`PoseArray`类型的`/particlecloud`话题。
   - 使用遥控器移动机器人一小段距离，若红色箭头逐渐收敛并与实际位姿一致，则定位成功。

5. **开始导航**：
   - **方式一**：在RViz中使用`2D Nav Goal`工具直接指定目标位姿。
   - **方式二**：通过节点发布action命令。

---

## 注意事项
1. **设备端口**：launch文件中默认配置为：靠近机械臂的激光—`/dev/ttyUSB0`，另一激光—`/dev/ttyUSB1`，小车底盘—`/dev/ttyUSB2`。若设备与端口不对应，建议重新拔插USB设备并按此顺序连接。
2. **启动顺序**：建图或导航时，务必按文档顺序启动节点，尤其是里程计节点启动后需等待5秒再移动小车。
3. **地图保存**：gmapping建图完成后及时保存地图；cartographer建图需按步骤保存并转换格式。
4. **导航配置**：启动导航前，请确保`map_server`节点加载的地图文件路径与名称正确（在launch文件中修改）。
5. **坐标系对齐**：建图时建议将小车初始位姿与地图原点对齐，以简化后续坐标变换。


