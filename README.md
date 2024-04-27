# VINS-Fusion-UWB-PHY

## Tight Fusion of Odometry and Kinematic Constraints for Multiple Aerial Vehicles in Physical Interconnection 
论文题目 紧耦合物理连接下的多飞行器动力学约束里程计融合

**特性:**
- 支持双目相机,IMU,UWB传感器
- 支持多机物理约束
  
## 准备安装工作
### **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### **OpenCV**

Follow [OpenCV Installation](https://github.com/opencv/opencv).

### **nlink_parser**
Follow [nlink_parser Installation](https://github.com/nooploop-dev/nlink_parser).


### **GTSAM**
Follow [GTSAM Installation](https://github.com/borglab/gtsam).
## 配置文件与参数新增内容
```
agent_number:子机编号id(1到3)

USE_LOOSE:使用松耦合优化的结果

USE_UWB:使用UWB紧耦合

USE_KIN使用物理约束紧耦合

imu_delta_fre两帧图像之间使用新约束的帧数

HINGE:球铰偏置量

KIN_LENGTH:球铰之间直线距离

para_tag:UWB偏置量
```

## 启动程序
roslaunch vins xxx.launch(包括启动三个子机节点,数据播放bag)

### 仿真数据集
```
roslaunch vins ue_lab*.launch
```

对应topic
```
1号机相机 camera1 , camera2 IMU imu1
2号机相机 camera5 , camera6 IMU imu3
3号机相机 camera3 , camera4 IMU imu2
相机配置文件 config/ue/
```
对应偏置与配置常量
```
para_tag=(0,0,0)
HINGE=(0,0,0)
KIN_LENGTH=0.957
```
### realsense数据集
```
roslaunch vins realsense_lab*.launch
```

对应topic
```
1号机相机 camera1 , camera2 IMU imu1
2号机相机 camera3 , camera4 IMU imu2
3号机相机 camera5 , camera6 IMU imu3
相机配置文件 config/realsense_d435i_*/
```
对应偏置与配置常量
```
para_tag=(-0.1,0,0.0)
HINGE=(-0.1,0,-0.03)
KIN_LENGTH=0.841
```

## 实验运动轨迹
### realsense数据运动轨迹
<img src="support_files/image/Figure_2.png" width = 100% height = 100% div align=left />

## 实验结果
### realsense实验结果
### SIAP与LIAP误差四分位图
<img src="support_files/image/siapBox.png" width = 45% height = 100% div align=left />
<img src="support_files/image/liapBox.png" width = 45% height = 100% div align=center />

### 实验序列结果对比
<img src="support_files/image/res1.png" width = 50% height = 100% div align=left />
<img src="support_files/image/res2.png" width = 50% height = 100% div align=center />





