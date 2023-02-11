#ifndef CSMLIO_UTILS_UTILITY_H_
#define CSMLIO_UTILS_UTILITY_H_

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <deque>

#include "infinityslam/common/port.h"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/sensor/timed_point_cloud_data.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {

namespace utils {

namespace {

} // namespace

struct TimedPose {
    double timestamp = 0;
    common::Time time;
    transform::Rigid3d pose;

    TimedPose () {}
    TimedPose (const double& timestamp_in, 
        const common::Time& time_in, 
        const transform::Rigid3d& pose_in)
        :timestamp(timestamp_in), time(time_in), pose(pose_in) {}
};

inline TimedPose CreateFullTimedPose(const TimedPose& t_pose) {
    return TimedPose(common::ToSeconds(t_pose.time), t_pose.time, t_pose.pose);
}

bool InterpolatePose(const TimedPose& pose1, 
    const TimedPose& pose2, 
    const common::Time& time_point, 
    TimedPose& pose_);

// 以配准位姿队列为主，以IMU数据为辅，在指定时间段内，以指定时间步长进行位姿插值，得到插值位姿表；
// 插值位姿表中的位姿为相对位姿 —— 我们规定以 end_time 时的位姿为基准来计算“相对”；
// 因为每次计算相对位姿的“相对基准”不一样，所以本函数无法对【插值位姿表】做增量式更新。
bool InterpolatePoseTable(const common::Time& start_time, 
                        const common::Time& end_time,
                        const std::vector<TimedPose>& timed_pose_queue,
                        const std::vector<sensor::ImuData>& imu_queue,
                        const double& interp_step,
                        std::vector<TimedPose>& pose_table,
                        bool based_on_end = true);


} // namespace utils

} // namespace infinityslam


#endif // CSMLIO_UTILS_UTILITY_H_