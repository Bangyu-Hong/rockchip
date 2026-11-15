# 基于 RK3588 的智能物料拾取机器人

**RK3588 Mobile Manipulation Robot — ROS 2 / SLAM / Vision / Manipulation**

> 2026 年全国大学生嵌入式芯片与系统设计竞赛团队项目。

 ## 🎬 项目演示 | Project Demo

[▶ 查看机器人实机演示视频](./演示视频.mp4)

[📄 查看完整技术文档](./技术文档.pdf)

> 本项目为团队竞赛作品，实际演示内容与
> 功能完成情况以视频和实验记录为准。


## 1. 项目简介

项目面向室内球体拾取与物料递送场景，采用 **RK3588 + STM32F407 + ESP32** 分层控制架构，包含视觉目标检测、地图构建与导航、底盘运动控制、机械臂抓取和任务状态管理等模块。

- **RK3588**：ROS 2 节点、视觉推理、导航与任务调度。
- **STM32F407**：麦克纳姆轮控制、编码器反馈、里程计及通信接口。
- **ESP32 / 机械臂控制侧**：抓取执行机构（完整固件与接口仍需按实际仓库核对）。


## 
## 2. 核心代码模块 | Core Modules

本项目采用 ROS 2 模块化架构，包含机器人导航与控制、
视觉感知、机械臂执行三个主要部分。

| 模块 | 核心功能 | 源代码 |
|---|---|---|
| 机器人导航与控制 | 任务状态机、导航调度、二维 EKF、底盘通信 | [my_robot_core](./src/my_robot_core/) |
| 机器人视觉感知 | RKNNLite/YOLO 推理、目标三维定位、RGB-D 视觉里程计 | [robot_vision](./src/robot_vision/) |
| 机械臂控制 | 机械臂运动学、驱动和抓取执行 | [jibot1_driver](./src/jibot1_driver/) |

### 主要技术实现

**机器人导航与控制**
- 基于 ROS 2 的节点通信与任务调度
- 搜索、导航接近、视觉对位与抓取状态机
- 轮式里程计及多传感器状态估计

**视觉感知**
- 基于 RKNNLite 的端侧目标检测
- 结合颜色、形状和深度信息进行目标定位
- RGB-D 视觉里程计

**机械臂执行**
- 机械臂运动学计算
- 抓取任务节点与动作执行

- 
### 关键算法实现 | Key Implementations

以下为项目中值得重点查看的算法与控制代码。

| 核心代码 | 技术内容 |
|---|---|
| [main_control.py](./src/my_robot_core/my_robot_core/main_control.py) | ROS 2 任务状态机、Nav2 目标调度、视觉对位与抓取流程 |
| [npu_3d_tracker.py](./src/robot_vision/robot_vision/npu_3d_tracker.py) | YOLO/RKNN 端侧推理、目标筛选、RGB-D 三维定位 |
| [planar_ekf_node.py](./src/my_robot_core/my_robot_core/planar_ekf_node.py) | 二维 EKF、轮式里程计与 IMU 融合、可选视觉里程计融合 |
| [rgbd_visual_odometry_node.py](./src/robot_vision/robot_vision/rgbd_visual_odometry_node.py) | RGB-D 特征匹配与视觉里程计 |

### 算法验证代码

项目还包含部分模块测试代码：

- [任务控制测试](./src/my_robot_core/test/test_main_control.py)
- [EKF 测试](./src/my_robot_core/test/test_planar_ekf_node.py)
- [RGB-D 视觉里程计测试](./src/robot_vision/test/test_rgbd_visual_odometry_node.py)

说明：以上列出了仓库中的算法实现与测试文件。
实际运行结果以实验记录与演示视频为准。


## 3. 个人贡献 | My Contributions

本项目为团队合作完成的智能移动操作机器人，采用 **RK3588 + STM32F407 + ESP32** 多控制器协同架构。

本人主要负责 **ROS 2 导航与任务控制、机器人视觉感知、多传感器状态估计及 STM32 底盘控制**，同时参与机械臂控制和整机系统集成联调。

### 3.1 主导开发 | Primary Contributions

**1. ROS 2 导航与任务状态机**

- 主导机器人 ROS 2 导航与任务控制模块的开发，基于 Nav2 `NavigateToPose` Action 接口实现导航目标管理。
- 设计目标搜索、导航接近、视觉对位与抓取等任务状态的切换逻辑。
- 实现导航超时、目标丢失、障碍物检测及恢复处理等功能。

核心代码：[main_control.py](./src/my_robot_core/my_robot_core/main_control.py)

**2. 机器人视觉感知与端侧推理**

- 主导基于 RK3588 NPU 的视觉感知模块开发，使用 RKNNLite 加载和运行 YOLO 检测模型。
- 结合颜色筛选、轮廓形状分析及深度信息，提高目标筛选与位置估计的适用性。
- 实现相机图像处理、目标深度获取、像素坐标到三维空间坐标转换及 ROS 目标信息发布。

核心代码：[npu_3d_tracker.py](./src/robot_vision/robot_vision/npu_3d_tracker.py)

**3. 多传感器状态估计与视觉里程计**

- 主导二维扩展卡尔曼滤波（EKF）节点开发，实现状态预测与观测更新。
- 融合轮式里程计、IMU 以及可选 RGB-D 视觉里程计信息，为机器人导航提供状态估计。
- 开发基于 ORB 特征提取、特征匹配与 PnP 位姿估计的 RGB-D 视觉里程计节点。

核心代码：

- [planar_ekf_node.py](./src/my_robot_core/my_robot_core/planar_ekf_node.py)
- [rgbd_visual_odometry_node.py](./src/robot_vision/robot_vision/rgbd_visual_odometry_node.py)

**4. STM32F407 麦克纳姆轮底盘控制**

- 主导 STM32F407 底盘控制模块开发，涉及麦克纳姆轮运动学、编码器反馈及电机速度闭环控制。
- 开发和对接 RK3588 与 STM32F407 之间的串口通信链路，实现车体速度指令传输和底盘状态回传。
- 参与底盘运动调试、里程计数据处理及通信安全保护机制的实现。

相关 ROS 接口代码：[base_driver.py](./src/my_robot_core/my_robot_core/base_driver.py)

STM32 固件代码将另行整理并标注。

### 3.2 参与开发与系统联调 | Collaborative Contributions

**机械臂控制**

- 参与机械臂运动学、抓取节点和执行控制接口的开发联调。
- 配合完成视觉目标位置向机械臂抓取指令的传递及动作执行流程调试。

**系统架构与实机联调**

- 参与 RK3588、STM32F407 与机械臂控制器之间的模块集成。
- 参与 ROS 2 节点通信、传感器数据流、导航控制与机器人执行链路的联调。
- 协助排查多控制器通信和机器人整机运行过程中的问题。

### 3.3 代码说明 | Implementation Notes

- 本项目为团队合作成果，以上内容按本人实际工作分工整理。
- Nav2、ROS 2、相机驱动及其他第三方开源组件不作为个人独立原创算法成果。
- 代码实现、仿真验证及实机验收属于不同阶段，具体实验结果以仓库公开的运行记录与演示材料为准。
- 项目仍在整理实验数据、测试记录和相关技术文档。

## 4. 实验与验证

目前整理中的仓库包含实现代码、启动配置和技术文档。

- SLAM 地图与机器人实际运动轨迹；
- 底盘里程计、定位和导航实验记录；
- YOLO/RKNN 实际推理结果和性能测量；
- 机械臂抓取与完整任务流程演示。

## 5. 我的工作（待按实际贡献填写）

- [x] 机器人 ROS 2 软件架构与节点集成：说明本人负责的文件与功能。
- [x] 视觉检测、模型部署和目标定位：说明训练、量化、调参或集成工作。
- [x] 建图、导航及底盘通信：说明本人实现或调试的模块。

团队项目应将本人贡献与队友贡献明确区分。

## 6. 环境与运行

源码包含 ROS 2 Python 功能包与部分硬件相关路径。完整依赖版本、设备接口、标定参数及可复现启动流程仍需核验后补充。请勿直接将仓库中现有参数用于无人看护的真实机器人运动。

## 7. 第三方依赖与致谢

项目源码目录还包含 `costmap_converter`、`OrbbecSDK_ROS2-master`、`sllidar_ros2`、`teb_local_planner`、`vision_opencv` 等第三方组件。这些组件并非本项目独立原创成果。**Project status:** Repository documentation in progress. 


