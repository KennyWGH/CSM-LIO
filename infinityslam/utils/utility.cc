#include "infinityslam/utils/utility.h"

namespace infinityslam {

namespace utils {

namespace {

} // namespace


bool InterpolatePose(const TimedPose& pose1, 
    const TimedPose& pose2, 
    const common::Time& time_point, 
    TimedPose& pose_)
{
    if (time_point < pose1.time || time_point > pose2.time) {
        return false;
    }
    return false;
}

bool InterpolatePoseTable(const common::Time& start_time, 
                        const common::Time& end_time,
                        const std::vector<TimedPose>& timed_pose_queue,
                        const std::vector<sensor::ImuData>& imu_queue,
                        const double& interp_step,
                        std::vector<TimedPose>& pose_table,
                        bool based_on_end)
{
    pose_table.clear();

    // 保证：
    // 开始/结束时间为顺序；
    // pose队列非空；
    // pose队列覆盖开始～结束时间段；
    // （IMU队列不强制要求，可为空或部分缺失）
    if (timed_pose_queue.empty()) return false;
    if (start_time >= end_time 
        || start_time < timed_pose_queue.front().time 
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

    // TODO.
    // ...
    const double duration = common::ToSeconds(end_time - start_time);
    const int steps = std::ceil(duration / interp_step) + 0.1;
    assert(steps >= 0);
    pose_table.resize(steps);

    // 首先得到绝对位姿插值表。
    transform::Rigid3d st_pose;
    transform::Rigid3d ed_pose;
    bool st_pose_found = false;
    bool ed_pose_found = false;
    for (auto& timed_pose : timed_pose_queue) {
        if (timed_pose.time == start_time) {
            st_pose = timed_pose.pose;
            st_pose_found = true;
        }
        if (timed_pose.time == end_time) {
            ed_pose = timed_pose.pose;
            ed_pose_found = true;
        }
    }
    if (!st_pose_found) {
        for (int i=0; i<timed_pose_queue.size()-1; ++i) {
            if (timed_pose_queue[i].time < start_time
                && timed_pose_queue[i+1].time > start_time) 
            {
                //
            }
        }
    }

    for (int i=0; i<steps; ++i) {
        //
    }

    // TimedPose ed_pose;
    // ed_pose.time = common::Time::min();
    // ed_pose.pose = transform::Rigid3d::Identity();

    // 获得相对位姿插值表。


    return false;
}


} // namespace utils

} // namespace infinityslam