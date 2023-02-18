/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_CSMLIO_POSE_EXTRAPOLATOR_H_
#define INFINITYSLAM_CSMLIO_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "infinityslam/common/time.h"
#include "infinityslam/common/system_options.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/timed_pose.h"
#include "infinityslam/csmlio/csm_lio_type_def.h"
#include "infinityslam/utils/imu_tracker.h"
#include "infinityslam/csmlio/pose_extrapolator/pose_extrapolator_interface.h"


namespace infinityslam {
namespace csmlio {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class PoseExtrapolator : public PoseExtrapolatorInterface {
    using TimedPose = infinityslam::transform::TimedPose;
    using ImuTracker = infinityslam::utils::ImuTracker;
 public:
    explicit PoseExtrapolator(common::Duration pose_queue_duration,
                                double imu_gravity_time_constant);

    explicit PoseExtrapolator(const PoseExtrapolatorFullOptions& options);

    PoseExtrapolator(const PoseExtrapolator&) = delete;
    PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

    static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
        common::Duration pose_queue_duration, double imu_gravity_time_constant,
        const sensor::ImuData& imu_data);

    static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
        const PoseExtrapolatorFullOptions& options,
        const sensor::ImuData& imu_data);

    // Returns the time of the last added pose or Time::min() if no pose was added
    // yet.
    common::Time GetLastPoseTime() const override;
    common::Time GetLastExtrapolatedTime() const override;

    void AddPose(common::Time time, const transform::Rigid3d& pose) override;
    void AddImuData(const sensor::ImuData& imu_data) override;
    void AddOdometryData(const sensor::OdometryData& odometry_data) override;
    transform::Rigid3d ExtrapolatePose(common::Time time) override;

    ExtrapolationResult ExtrapolatePosesWithGravity(
        const std::vector<common::Time>& times) override;

    // Returns the current gravity alignment estimate as a rotation from
    // the tracking frame into a gravity aligned frame.
    Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
    void UpdateVelocitiesFromPoses();
    void TrimImuData();
    void TrimOdometryData();
    void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
    Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                            ImuTracker* imu_tracker) const;
    Eigen::Vector3d ExtrapolateTranslation(common::Time time);

    const PoseExtrapolatorFullOptions full_options_;

    const common::Duration pose_queue_duration_;

    std::deque<TimedPose> timed_pose_queue_;
    Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

    const double gravity_time_constant_;
    std::deque<sensor::ImuData> imu_data_;
    std::unique_ptr<ImuTracker> imu_tracker_;
    std::unique_ptr<ImuTracker> odometry_imu_tracker_;
    std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
    TimedPose cached_extrapolated_pose_;

    std::deque<sensor::OdometryData> odometry_data_;
    Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_POSE_EXTRAPOLATOR_H_
