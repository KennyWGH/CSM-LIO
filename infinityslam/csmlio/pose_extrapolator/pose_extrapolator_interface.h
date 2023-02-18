/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_CSMLIO_POSE_EXTRAPOLATOR_INTERFACE_H_
#define INFINITYSLAM_CSMLIO_POSE_EXTRAPOLATOR_INTERFACE_H_

#include <memory>
#include <tuple>
#include <sstream>

#include "infinityslam/common/time.h"
// #include "infinityslam/csmlio/proto/pose_extrapolator_options.pb.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/timestamped_transform.h"

namespace infinityslam {
namespace csmlio {

struct PoseExtrapolatorFullOptions {
    double imu_gravity_time_constant = 10.;             // 位姿外推器
    bool   use_imu_based_pose_extrapolator = false;     // 位姿外推器

    double pose_extrap1_imu_gravity_time_constant = 10.; // 基于匀速模型和ImuTracker的位姿外推器
    double pose_extrap1_pose_queue_duration = 0.001;     // 基于匀速模型和ImuTracker的位姿外推器

    double pose_extrap2_pose_queue_duration = 5.;           // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_gravity_constant = 9.806;           // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_pose_translation_weight = 1.;       // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_pose_rotation_weight = 1.;          // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_imu_acceleration_weight = 1.;       // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_imu_rotation_weight = 1.;           // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_odometry_translation_weight = 1.;   // 基于IMU积分和bias矫正的位姿外推器
    double pose_extrap2_odometry_rotation_weight = 1.;      // 基于IMU积分和bias矫正的位姿外推器
    bool pose_extrap2_solver_use_nonmonotonic_steps = false;// 基于IMU积分和bias矫正的位姿外推器
    int  pose_extrap2_solver_max_num_iterations = 10;       // 基于IMU积分和bias矫正的位姿外推器
    int  pose_extrap2_solver_num_threads = 1;               // 基于IMU积分和bias矫正的位姿外推器

    // 在一些地方需要调用默认构造函数
    PoseExtrapolatorFullOptions() {}

    std::string DebugString() const {
        //
        std::ostringstream oss;
        oss << "PoseExtrapolatorFullOptions Info: \n";
        oss << "    imu_gravity_time_constant : " << imu_gravity_time_constant << "\n"
            << "    use_imu_based_pose_extrapolator : " << use_imu_based_pose_extrapolator << "\n"
            << "    ... \n";
        // TODO: print all variables.
        return oss.str();
    }
};

// 从common::options中读取参数
PoseExtrapolatorFullOptions ReadPoseExtrapolatorFullOptions();

class PoseExtrapolatorInterface {
  public:
    struct ExtrapolationResult {
        // The poses for the requested times at index 0 to N-1.
        std::vector<transform::Rigid3f> previous_poses;
        // The pose for the requested time at index N.
        transform::Rigid3d current_pose;
        Eigen::Vector3d current_velocity;
        Eigen::Quaterniond gravity_from_tracking;
    };

    PoseExtrapolatorInterface(const PoseExtrapolatorInterface&) = delete;
    PoseExtrapolatorInterface& operator=(const PoseExtrapolatorInterface&) =
        delete;
    virtual ~PoseExtrapolatorInterface() {}

    static std::unique_ptr<PoseExtrapolatorInterface> CreateWithImuData(
        const std::vector<sensor::ImuData>& imu_data,
        const std::vector<transform::TimestampedTransform>& initial_poses,
        bool init_with_common_options = true);

    // Returns the time of the last added pose or Time::min() if no pose was added
    // yet.
    virtual common::Time GetLastPoseTime() const = 0;
    virtual common::Time GetLastExtrapolatedTime() const = 0;

    virtual void AddPose(common::Time time, const transform::Rigid3d& pose) = 0;
    virtual void AddImuData(const sensor::ImuData& imu_data) = 0;
    virtual void AddOdometryData(const sensor::OdometryData& odometry_data) = 0;
    virtual transform::Rigid3d ExtrapolatePose(common::Time time) = 0;

    virtual ExtrapolationResult ExtrapolatePosesWithGravity(
        const std::vector<common::Time>& times) = 0;

    // Returns the current gravity alignment estimate as a rotation from
    // the tracking frame into a gravity aligned frame.
    virtual Eigen::Quaterniond EstimateGravityOrientation(common::Time time) = 0;

  protected:
    PoseExtrapolatorInterface() {}
};

}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_POSE_EXTRAPOLATOR_INTERFACE_H_
