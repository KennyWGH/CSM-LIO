/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_UTILS_UTILITY_H_
#define INFINITYSLAM_UTILS_UTILITY_H_

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <deque>

#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/sensor/timed_point_cloud_data.h"
#include "infinityslam/transform/timed_pose.h"

namespace infinityslam {
namespace utils {


/**
 * 以已知位姿队列为主，以IMU数据为辅，在指定时间段内，以指定时间步长进行位姿插值，输出插值位姿表。
 * 输出插值位姿表中的位姿可以为绝对位姿 —— 也即与已知位姿为同一坐标基准；也可以为相对位姿，此时
 * 所有插值位姿将再进行一次转换，表达在“以end_time时的绝对位姿为基准”的坐标系下，是为“相对”位姿。
 * @tparam PoseType 模板类型必须提供【Rigid3d位姿、double时间戳】信息。
 * @param start_time 
 * @param end_time 
 * @param source_pose_queue 
 * @param source_imu_queue 
 * @param time_step 
 * @param output_pose_queue 
 * @param relative_to_end 是否输出相对于end_time时刻的相对位姿
 * @param time_error 允许的插值时间误差 
 * @return bool-插值是否成功
 * @note 本函数具有借助IMU旋转积分进行精确姿态插值的特点，放在utils模块（而非transform模块）更合理。
*/
bool InterpolatePoseQueueWithIMUs( // template<typename PoseType>
    const double& start_time, 
    const double& end_time,
    const std::vector<transform::TimedPose>& source_pose_queue,
    const std::vector<sensor::ImuData>& source_imu_queue,
    const double& time_step,
    std::vector<transform::TimedPose>& output_pose_queue,
    const bool& relative_to_end = false,
    const double& time_error = 0.1);


} // namespace utils
} // namespace infinityslam


#endif // INFINITYSLAM_UTILS_UTILITY_H_