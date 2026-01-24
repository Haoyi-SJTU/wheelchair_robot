# 移动机器人ROS操作指南

## 概述
本文档提供了移动机器人系统的ROS（Robot Operating System）操作指南，包括建图与导航两大部分。建图支持两种方式：gmapping和cartographer，请根据实际需求选择其一。导航部分提供了完整的启动流程与操作说明。



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

