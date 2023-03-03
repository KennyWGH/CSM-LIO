/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_UTILS_INTERNAL_MOTION_COMPENSATOR_H_
#define INFINITYSLAM_UTILS_INTERNAL_MOTION_COMPENSATOR_H_

// #include <limits>
// #include <iomanip>

#include "infinityslam/transform/timed_pose.h"
#include "infinityslam/utils/imu_aided_pose_interpolator.h"

namespace infinityslam {
namespace utils {


/**
 * 运动畸变补偿器：提供点云去畸变功能。
 * 支持所有的点云类型，包括PointCloud, PointsBatch, (Multi)TimedPointCloudData等。
*/
class MotionCompensator {
   public:
    explicit MotionCompensator() = default;
    ~MotionCompensator() {}



   private:
    //
};

}  // namespace utils
}  // namespace infinityslam

#endif  // INFINITYSLAM_UTILS_INTERNAL_MOTION_COMPENSATOR_H_
