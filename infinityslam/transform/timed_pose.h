/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_TRANSFORM_TIMED_POSE_H_
#define INFINITYSLAM_TRANSFORM_TIMED_POSE_H_

#include "infinityslam/common/time.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {
namespace transform {

struct TimedPose {
    common::Time time;
    double timestamp;
    transform::Rigid3d pose;

    TimedPose () {}
    TimedPose (const common::Time& time_in, 
        const double& timestamp_in, 
        const transform::Rigid3d& pose_in)
        :time(time_in), timestamp(timestamp_in), pose(pose_in) {}
};

inline TimedPose CreateTimedPoseFully(const TimedPose& other) {
    return TimedPose(other.time, common::ToSeconds(other.time), other.pose);
}

inline bool InterpolateTimedPose(
    const TimedPose& pose1, 
    const TimedPose& pose2, 
    const common::Time& time_point, 
    TimedPose& pose_)
{
    /// TODO: @wgh 实现插值
    if (time_point < pose1.time || time_point > pose2.time) {
        return false;
    }
    return false;
}

inline bool InterpolateTimedPose(
    const TimedPose& pose1, 
    const TimedPose& pose2, 
    const double& timestamp, 
    TimedPose& pose_)
{
    /// TODO: @wgh 实现插值
    if (timestamp < pose1.timestamp || timestamp > pose2.timestamp) {
        return false;
    }
    return false;
}


}  // namespace transform
}  // namespace infinityslam

#endif  // INFINITYSLAM_TRANSFORM_TIMED_POSE_H_
