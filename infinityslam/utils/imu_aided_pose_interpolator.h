/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_UTILS_UTILITY_H_
#define INFINITYSLAM_UTILS_UTILITY_H_

#include <cmath>
#include <vector>
#include <deque>
#include <mutex>

#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/common/optional.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/transform/timed_pose.h"
#include "infinityslam/utils/imu_integration.h"

namespace infinityslam {
namespace utils {


/**
 * 基于TimedPose队列和IMU积分做高精度的位姿插值。
 * 所有的时间类型都用double表示。
 * 
 * 要求：
 * -- 所有接口保证【多线程安全】
 * -- -- 1). 所有Add接口必须加all锁，在需要时加interp锁；
 * -- -- 2). 所有LookUp接口只需要加interp锁；
 * -- -- 3). 其它public接口按需加锁；
 * -- -- 4). 所有private接口均不加锁，以防死锁；
*/
class ImuAidedPoseInterpolator {
   public:
    ImuAidedPoseInterpolator(double time_range_limit = 60.0);
    ~ImuAidedPoseInterpolator() {}

    // 不允许时间回退
    void AddPose(const transform::TimedPose& pose);
    void AddPose(const common::Time& time, 
        const transform::Rigid3d& local_pose);
    void AddImu(const sensor::ImuData& imu_data);

    // 支持单时刻内插查询
    bool LookUp(const double& timepoint, transform::TimedPose& pose);

    // 支持时间段内按时间步长内插查询
    bool LookUp(
        const double& start_time, 
        const double& end_time,
        const double& time_step,
        std::vector<transform::TimedPose>& output_pose_queue,
        const bool& relative_to_end = false,
        const double& time_error = 0.1);

    // 支持多个连续时刻的内插查询
    bool LookUp(
        const std::vector<double>& interp_times, 
        std::vector<transform::TimedPose>& output_pose_queue,
        const bool& relative_to_end = false,
        const double& time_error = 0.1);

    std::deque<transform::TimedPose> GetInternalPoseQueue();

    void Clear();                       //清空所有数据队列

    bool Has(double timepoint) const;   //当前内插位姿队列是否涵盖该时刻

    double earliest_time() const;       //当前内插位姿队列的最早时刻
    double latest_time() const;         //当前内插位姿队列的最新时刻

    bool empty() const;                 //当前内插位姿队列是否为空
    size_t size() const;                //当前内插位姿队列的size

    double time_range_limit() const;    //查询固定参数
    double time_range() const;          //当前内插位姿队列的时间跨度

    void SetDebugMode(bool value) { 
        kDebugMode = value; 
    }

   private:

    /**
     * 一次advance操作的对象，满足如下时间规律：
     * source pose queue: ---------------|--- pose_index，也即将要推进到的时刻
     * source imu  queue: -----|---------|--- imu_st_index ~ imu_ed_index，左开右闭
     * interpolated time: ----|-------------- 当前最新插值时刻
    */
   template<typename PoseType, typename ImuIteratorType>
    struct TaskUnit {
        const PoseType curr_pose;
        const PoseType next_pose;
        const ImuIteratorType imu_st_it;
        const ImuIteratorType imu_ed_it;
        // PoseType curr_pose;
        // PoseType next_pose;
        // ImuIteratorType imu_st_it;
        // ImuIteratorType imu_ed_it;
    };

    using InterpTask = 
        TaskUnit<transform::TimedPose, 
            std::deque<sensor::ImuData>::iterator>;


    // 推进内部的“高精度IMU同频插值表”到尽可能的最新时刻【所有private函数不得加锁】
    void AdvanceInterpolation();
    void AdvanceOneTask(const InterpTask& task,
        std::deque<transform::TimedPose>& target_pose_queue);

    // 丢掉太老的pose【所有private函数不得加锁】
    void TrimSourcePoseQueueIfNeeded();
    void TrimInterpPoseQueueIfNeeded();

    // 多线程访问保护
    using UniqLock = std::unique_lock<std::mutex>;
    mutable std::mutex all_data_mutex_; //全局数据保护（除了内插位姿队列）
    mutable std::mutex interp_data_mutex_; //内插位姿队列保护

    // 参数
    const double kTimeRangeLimit = 60.0;
    bool kDebugMode = false;

    // source数据队列
    std::deque<transform::TimedPose> source_pose_queue_;
    std::deque<sensor::ImuData> source_imu_queue_;

    // 内部的“高精度IMU同频插值表”
    std::deque<transform::TimedPose> interpolated_pose_queue_;
    common::optional<double> lastest_interp_timestamp;

};


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