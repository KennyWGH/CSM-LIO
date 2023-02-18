/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#include "infinityslam/utils/utility.h"
#include "infinityslam/transform/timed_pose.h"
#include "glog/logging.h"

#include <cmath>

namespace infinityslam {
namespace utils {

namespace {

} // namespace



bool InterpolatePoseQueueWithIMUs(  // template<typename PoseType>
    const double& start_time, 
    const double& end_time,
    const std::vector<transform::TimedPose>& source_pose_queue,
    const std::vector<sensor::ImuData>& source_imu_queue,
    const double& time_step,
    std::vector<transform::TimedPose>& output_pose_queue,
    const bool& relative_to_end,
    const double& time_error)
{
    output_pose_queue.clear();
    if (source_pose_queue.empty()) {
        LOG(ERROR) << "Interpolate pose queue failed: source pose queue is EMPTY!";
        return false;
    }
    if (start_time >= end_time ) { 
        LOG(ERROR) << "Interpolate pose queue failed: start_time BIGGER than end_time!";
        return false; 
    }
    if (start_time < (source_pose_queue.front().timestamp - time_error)
        || end_time > (source_pose_queue.back().timestamp + time_error)) {
        LOG(ERROR) << "Interpolate pose queue failed: time error too large!";
        return false; 
    }

    /**
     * 插值条件：
     * -- source pose 队列非空；
     * -- 开始～结束时间为顺序；
     * -- source pose 队列覆盖开始～结束时间段；
     * -- （IMU队列不强制要求，可为空或部分缺失）
     * 
     * 点云去畸变原理（时间对齐）：
     * time point: **************** t1 ********** t2 ********** tn *****
     * pose queue: **************** T1 ********** T2 ********** Tn *****
     * pose queue: **************** |  ********** |  ********** |  *****
     * coordinate of point 'P': *** p1 ********** p2 ********** pn *****
     * For point P sampled at t1 timepoint with coordinate p1, we try to
     * get the coordinate of P represented at t2 timepoint, i.e., p2.
     * Note that "T1, T2, ... Tn" are already given -- from interpolation!
     * we have T1 * p1 = T2 * p2, then p2 = T2^(-1) * T1 * p1,
     * so the element in output pose queue should be: {T_e^(-1) * T_i},
     * in which "T_e" means pose of end time.
    */

    const double duration = end_time - start_time;
    const int steps = std::ceil(duration / time_step) + 0.1;
    if (steps <= 0) {
        LOG(ERROR) << "Interpolate pose queue failed: "
            << "interp steps should be >= 1, which is " << steps;
        return false; 
    }
    output_pose_queue.reserve(steps);

    for (int i=0; i<steps; ++i) {
        //
    }

    // 首先得到绝对位姿插值表。
    transform::Rigid3d st_pose;
    transform::Rigid3d ed_pose;
    bool st_pose_found = false;
    bool ed_pose_found = false;
    for (auto& timed_pose : source_pose_queue) {
        if (timed_pose.timestamp == start_time) {
            st_pose = timed_pose.pose;
            st_pose_found = true;
        }
        if (timed_pose.timestamp == end_time) {
            ed_pose = timed_pose.pose;
            ed_pose_found = true;
        }
    }
    if (!st_pose_found) {
        for (int i=0; i<source_pose_queue.size()-1; ++i) {
            if (source_pose_queue[i].timestamp < start_time
                && source_pose_queue[i+1].timestamp > start_time) 
            {
                //
            }
        }
    }

    for (int i=0; i<steps; ++i) {
        //
    }



    // 如果指定了需要输出相对位姿，我们再做一次位姿转换。
    if (relative_to_end) {
        //
    }


    return true;
}


} // namespace utils
} // namespace infinityslam