/**
 * Copyright 2023 WANG Guanhua (wangxxx@gmail.com)
*/

#include "csmlio/lio/utility.h"

namespace csmlio {

namespace mapping {

// using TimedPointCloudData = ::csmlio::sensor::TimedPointCloudData;

bool InterpolatePoseTable(const common::Time& start_time, 
                        const common::Time& end_time,
                        const std::vector<TimedPose>& timed_pose_queue,
                        const std::vector<sensor::ImuData>& imu_queue,
                        const double& interp_step,
                        std::vector<TimedPose>& pose_table,
                        bool based_on_end)
{
    if (timed_pose_queue.empty() || imu_queue.empty()) return false;
    if (start_time >= end_time || start_time < timed_pose_queue.front().time 
        || end_time > timed_pose_queue.back().time) 
    { return false; }

    // 插值原理：
    // pose queue: **************** T1 ********** T2 ************
    // pose queue: **************** | *********** | *************
    // coordinate of point P: ***** p1 ********** p2 ************
    // For point P sampled at T1 time with coordinate p1, we try to
    // get the coordinate of P represented at T2, i.e., p2.
    // we have T1 * p1 = T2 * p2, then p2 = T2^(-1) * T1 * p1,
    // so the element in output pose queue should be: {T_e^(-1) * T_i},
    // in which "T_e" means pose of end time.
    pose_table.clear();

    // TODO.
    // ...

    return false;
}


}  // namespace mapping
}  // namespace csmlio