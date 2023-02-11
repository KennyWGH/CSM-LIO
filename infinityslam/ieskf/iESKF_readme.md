# ESKF/SwiftLocalizer
融合轮速Odom、IMU、RTK的误差状态卡尔曼滤波器。   
输入：Odom、IMU、RTK观测；   
输出：融合后的位姿信息；   

## 初始化
代码： Initializer::InitializeEKFState()   
要求：
- IMU数据不少于100个
- Odom数据不少于2个
- 用最新IMU的时间戳更新 State 时间戳，把最新IMU数据和最新Odom数据保存到 State中；
- 用最新IMU的时间戳更新 DatumState 时间戳；

SO3 G_R_I   
Mat3 I_R_G   


Wheel/Odometer, Imu, Gnss/RTK, LiDAR

GIO-Fusion

IESKF-Fusion(GNSS/Imu/Odometer-GIO)   
IESKF-Fusion(LiDAR/Imu/Odometer-LIO)