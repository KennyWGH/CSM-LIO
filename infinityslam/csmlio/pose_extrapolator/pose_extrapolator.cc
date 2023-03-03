/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include <algorithm>
#include "boost/make_unique.hpp"
#include "glog/logging.h"

#include "infinityslam/csmlio/pose_extrapolator/pose_extrapolator.h"
#include "infinityslam/transform/transform.h"

namespace infinityslam {
namespace csmlio {

PoseExtrapolator::PoseExtrapolator(
    const common::Duration pose_queue_duration,
    double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration)
    , gravity_time_constant_(imu_gravity_time_constant)
    , cached_extrapolated_pose_{
        common::Time::min(), 
        common::ToSeconds(common::Time::min()), 
        transform::Rigid3d::Identity()} 
{
    LOG(INFO) << "Constructed PoseExtrapolator with lua options.";
    LOG(INFO) << " ** pose_queue_duration_ = " << common::ToSeconds(pose_queue_duration_);
    LOG(INFO) << " ** gravity_time_constant_ = " << gravity_time_constant_;
}

PoseExtrapolator::PoseExtrapolator(
    const PoseExtrapolatorFullOptions& options)
    : full_options_(options)
    , pose_queue_duration_(
        common::FromSeconds(full_options_.pose_extrap1_pose_queue_duration))
    , gravity_time_constant_(full_options_.pose_extrap1_imu_gravity_time_constant)
    , cached_extrapolated_pose_{
        common::Time::min(), 
        common::ToSeconds(common::Time::min()), 
        transform::Rigid3d::Identity()} 
{
    LOG(INFO) << "Constructed PoseExtrapolator with common options.";
    LOG(INFO) << " [PARAM] pose_queue_duration_ = " << common::ToSeconds(pose_queue_duration_);
    LOG(INFO) << " [PARAM] gravity_time_constant_ = " << gravity_time_constant_;
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, 
    const sensor::ImuData& imu_data) 
{
    auto extrapolator = boost::make_unique<PoseExtrapolator>(
        pose_queue_duration, imu_gravity_time_constant);
    extrapolator->AddImuData(imu_data);
    extrapolator->imu_tracker_ =
        boost::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
    extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
        imu_data.linear_acceleration);
    extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
        imu_data.angular_velocity);
    extrapolator->imu_tracker_->Advance(imu_data.time);
    extrapolator->AddPose(
        imu_data.time,
        transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
    return extrapolator;
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const PoseExtrapolatorFullOptions& options,
    const sensor::ImuData& imu_data) 
{
    auto extrapolator = boost::make_unique<PoseExtrapolator>(options);
    extrapolator->AddImuData(imu_data);
    extrapolator->imu_tracker_ = 
        boost::make_unique<ImuTracker>(
            options.pose_extrap1_imu_gravity_time_constant, imu_data.time);
    extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
        imu_data.linear_acceleration);
    extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
        imu_data.angular_velocity);
    extrapolator->imu_tracker_->Advance(imu_data.time);
    extrapolator->AddPose(
        imu_data.time,
        transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
    return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const 
{
    if (timed_pose_queue_.empty()) {
        return common::Time::min();
    }
    return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const 
{
    if (!extrapolation_imu_tracker_) {
        return common::Time::min();
    }
    return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) 
{
    if (imu_tracker_ == nullptr) {
        common::Time tracker_start = time;
        if (!imu_data_.empty()) {
        tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        imu_tracker_ =
            boost::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
    }
    timed_pose_queue_.push_back(TimedPose{time, common::ToSeconds(time), pose});
    while (timed_pose_queue_.size() > 2 &&
            timed_pose_queue_[1].time <= time - pose_queue_duration_) {
        timed_pose_queue_.pop_front();
    }
    UpdateVelocitiesFromPoses();
    AdvanceImuTracker(time, imu_tracker_.get());
    TrimImuData();
    TrimOdometryData();
    odometry_imu_tracker_ = boost::make_unique<ImuTracker>(*imu_tracker_);
    extrapolation_imu_tracker_ = boost::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) 
{
    CHECK(timed_pose_queue_.empty() ||
            imu_data.time >= timed_pose_queue_.back().time);
    imu_data_.push_back(imu_data);
    TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
    CHECK(timed_pose_queue_.empty() ||
            odometry_data.time >= timed_pose_queue_.back().time);
    odometry_data_.push_back(odometry_data);
    TrimOdometryData();
    if (odometry_data_.size() < 2) {
        return;
    }
    // TODO(whess): Improve by using more than just the last two odometry poses.
    // Compute extrapolation in the tracking frame.
    const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
    const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
    const double odometry_time_delta =
        common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
    const transform::Rigid3d odometry_pose_delta =
        odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
    angular_velocity_from_odometry_ =
        transform::RotationQuaternionToAngleAxisVector(
            odometry_pose_delta.rotation()) /
        odometry_time_delta;
    if (timed_pose_queue_.empty()) {
        return;
    }
    const Eigen::Vector3d
        linear_velocity_in_tracking_frame_at_newest_odometry_time =
            odometry_pose_delta.translation() / odometry_time_delta;
    const Eigen::Quaterniond orientation_at_newest_odometry_time =
        timed_pose_queue_.back().pose.rotation() *
        ExtrapolateRotation(odometry_data_newest.time,
                            odometry_imu_tracker_.get());
    linear_velocity_from_odometry_ =
        orientation_at_newest_odometry_time *
        linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) 
{
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    CHECK_GE(time, newest_timed_pose.time);
    if (cached_extrapolated_pose_.time != time) {
        const Eigen::Vector3d translation =
            ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
        const Eigen::Quaterniond rotation =
            newest_timed_pose.pose.rotation() *
            ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
        cached_extrapolated_pose_ =
            TimedPose{time, common::ToSeconds(time), 
                transform::Rigid3d{translation, rotation}};
    }
    return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) 
{
    ImuTracker imu_tracker = *imu_tracker_;
    AdvanceImuTracker(time, &imu_tracker);
    return imu_tracker.orientation();
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() 
{
    if (timed_pose_queue_.size() < 2) {
        // We need two poses to estimate velocities.
        return;
    }
    CHECK(!timed_pose_queue_.empty());
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const auto newest_time = newest_timed_pose.time;
    const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
    const auto oldest_time = oldest_timed_pose.time;
    const double queue_delta = common::ToSeconds(newest_time - oldest_time);
    if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
        LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                    << queue_delta << " s";
        return;
    }
    const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
    const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
    linear_velocity_from_poses_ =
        (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
    angular_velocity_from_poses_ =
        transform::RotationQuaternionToAngleAxisVector(
            oldest_pose.rotation().inverse() * newest_pose.rotation()) /
        queue_delta;
}

void PoseExtrapolator::TrimImuData() 
{
    while (imu_data_.size() > 1 
            && !timed_pose_queue_.empty() 
            && imu_data_[1].time <= timed_pose_queue_.back().time) 
    { imu_data_.pop_front(); }
}

void PoseExtrapolator::TrimOdometryData() 
{
    while (odometry_data_.size() > 2 
            && !timed_pose_queue_.empty() 
            && odometry_data_[1].time <= timed_pose_queue_.back().time) 
    { odometry_data_.pop_front(); }
}

void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const 
{
    CHECK_GE(time, imu_tracker->time());
    if (imu_data_.empty() || time < imu_data_.front().time) {
        // There is no IMU data until 'time', so we advance the ImuTracker and use
        // the angular velocities from poses and fake gravity to help 2D stability.
        imu_tracker->Advance(time);
        imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
        imu_tracker->AddImuAngularVelocityObservation(
            odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                    : angular_velocity_from_odometry_);
        return;
    }
    if (imu_tracker->time() < imu_data_.front().time) {
        // Advance to the beginning of 'imu_data_'.
        imu_tracker->Advance(imu_data_.front().time);
    }
    auto it = std::lower_bound(
        imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
        [](const sensor::ImuData& imu_data, const common::Time& time) {
            return imu_data.time < time;
        });
    while (it != imu_data_.end() && it->time < time) {
        imu_tracker->Advance(it->time);
        imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
        imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
        ++it;
    }
    imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const 
{
    CHECK_GE(time, imu_tracker->time());
    AdvanceImuTracker(time, imu_tracker);
    const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
    return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) 
{
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =
        common::ToSeconds(time - newest_timed_pose.time);
    if (odometry_data_.size() < 2) {
        return extrapolation_delta * linear_velocity_from_poses_;
    }
    return extrapolation_delta * linear_velocity_from_odometry_;
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) 
{
    std::vector<transform::Rigid3f> poses;
    for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
        poses.push_back(ExtrapolatePose(*it).cast<float>());
    }

    // // debug issue.
    // LOG(INFO) << "Print ExtrapolatePosesWithGravity info: \n"
    //     << "    times.front() is " << times.front() << "\n"
    //     << "    times.back() is " << times.back() << "\n"
    //     << "    linear_velocity_from_poses_ is " << linear_velocity_from_poses_ << "\n"
    //     << "    pose queue duration (allowed) is " << common::ToSeconds(pose_queue_duration_) << "s \n"
    //     << "    pose queue size is " << timed_pose_queue_.size() << "\n"
    //     << "    pose queue duration (actually) is " 
    //         << common::ToSeconds(timed_pose_queue_.back().time - timed_pose_queue_.front().time) << "s \n"
    //     << "    pose queue front timestamp is " << timed_pose_queue_.front().time << "\n"
    //     << "    pose queue back timestamp is " << timed_pose_queue_.back().time << "\n"
    //     << "    pose queue back position is " 
    //         << timed_pose_queue_.back().pose.translation().transpose() << ".";

    const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                                ? linear_velocity_from_poses_
                                                : linear_velocity_from_odometry_;
    return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                                current_velocity,
                                EstimateGravityOrientation(times.back())};
}

}  // namespace csmlio
}  // namespace infinityslam
