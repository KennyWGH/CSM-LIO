/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "infinityslam/csmlio/pose_extrapolator/imu_based_pose_extrapolator.h"

#include <algorithm>
#include "boost/make_unique.hpp"

#include "infinityslam/utils/imu_integration.h"
#include "infinityslam/utils/eigen_quaternion_from_two_vectors.h"
#include "infinityslam/csmlio/optimization/rotation_parameterization.h"
#include "infinityslam/csmlio/optimization/ceres_pose.h"
#include "infinityslam/csmlio/optimization/cost_functions/acceleration_cost_function_3d.h"
#include "infinityslam/csmlio/optimization/cost_functions/rotation_cost_function_3d.h"
#include "infinityslam/csmlio/optimization/cost_functions/spa_cost_function_3d.h"
#include "infinityslam/csmlio/optimization/optimization_type_def.h"
#include "infinityslam/transform/transform.h"
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {

using ::infinityslam::transform::TimestampedTransform;

namespace {

ceres::Solver::Options CreateCeresSolverOptions (
    const PoseExtrapolatorFullOptions& full_options) {
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = full_options.pose_extrap2_solver_use_nonmonotonic_steps;
    options.max_num_iterations = full_options.pose_extrap2_solver_max_num_iterations;
    options.num_threads = full_options.pose_extrap2_solver_num_threads;
    return options;
}

} // namespace

ImuBasedPoseExtrapolator::ImuBasedPoseExtrapolator(
    const PoseExtrapolatorFullOptions& options)
    : full_Options_(options),
      solver_Options_(CreateCeresSolverOptions(options)) {
    LOG(INFO) << "Constructed ImuBasedPoseExtrapolator with common options.";
} 

ImuBasedPoseExtrapolator::~ImuBasedPoseExtrapolator() {
    LOG(INFO) << "Number of iterations for pose extrapolation:";
    LOG(INFO) << num_iterations_hist_.ToString(10);
}

std::unique_ptr<PoseExtrapolatorInterface>
ImuBasedPoseExtrapolator::InitializeWithImu(
    const PoseExtrapolatorFullOptions& full_options,
    const std::vector<sensor::ImuData>& imu_data,
    const std::vector<transform::TimestampedTransform>& initial_poses) 
{
    CHECK(!imu_data.empty());
    LOG(INFO) << full_options.DebugString();
    auto extrapolator = boost::make_unique<ImuBasedPoseExtrapolator>(full_options);
    std::copy(imu_data.begin(), imu_data.end(),
                std::back_inserter(extrapolator->imu_data_));
    if (!initial_poses.empty()) {
        for (const auto& pose : initial_poses) {
        if (pose.time > imu_data.front().time) {
            extrapolator->AddPose(pose.time, pose.transform);
        }
        }
    } else {
        extrapolator->AddPose(
            imu_data.back().time,
            transform::Rigid3d::Rotation(utils::FromTwoVectors(
                imu_data.back().linear_acceleration, Eigen::Vector3d::UnitZ())));
    }
    return extrapolator;
}

common::Time ImuBasedPoseExtrapolator::GetLastPoseTime() const 
{
    if (timed_pose_queue_.empty()) {
        return common::Time::min();
    }
    return timed_pose_queue_.back().time;
}

common::Time ImuBasedPoseExtrapolator::GetLastExtrapolatedTime() const 
{
    return last_extrapolated_time_;
}

void ImuBasedPoseExtrapolator::AddPose(const common::Time time,
                                       const transform::Rigid3d& pose) 
{
    timed_pose_queue_.push_back(TimestampedTransform{time, pose});
    while (timed_pose_queue_.size() > 3 &&
            timed_pose_queue_[1].time <=
                time - common::FromSeconds(full_Options_.pose_extrap2_pose_queue_duration)) {
        if (!previous_solution_.empty()) {
        CHECK_EQ(timed_pose_queue_.front().time, previous_solution_.front().time);
        previous_solution_.pop_front();
        }
        timed_pose_queue_.pop_front();
    }
    TrimImuData();
}

void ImuBasedPoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) 
{
    CHECK(timed_pose_queue_.empty() ||
            imu_data.time >= timed_pose_queue_.back().time);
    imu_data_.push_back(imu_data);
    TrimImuData();
}

void ImuBasedPoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
    CHECK(timed_pose_queue_.empty() ||
            odometry_data.time >= timed_pose_queue_.back().time);
    odometry_data_.push_back(odometry_data);
    TrimOdometryData();
}

ImuBasedPoseExtrapolator::ExtrapolationResult
ImuBasedPoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) 
{
    const auto& time = times.back();
    const auto& newest_timed_pose = timed_pose_queue_.back();
    CHECK_GE(time, newest_timed_pose.time);
    CHECK_GE(times.size(), 1);
    last_extrapolated_time_ = time;

    if (timed_pose_queue_.size() < 3 ||
        common::ToSeconds(time - newest_timed_pose.time) < 1e-6) {
        return ExtrapolationResult{
            std::vector<transform::Rigid3f>(
                times.size() - 1, timed_pose_queue_.back().transform.cast<float>()),
            timed_pose_queue_.back().transform, Eigen::Vector3d::Zero(),
            timed_pose_queue_.back().transform.rotation()};
    }

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // Track gravity alignment over time and use this as a frame here so that
    // we can estimate the gravity alignment of the current pose.
    optimization::CeresPose gravity_from_local(
        gravity_from_local_, nullptr,
        boost::make_unique<ceres::QuaternionParameterization>(), &problem);
    // Use deque so addresses stay constant during problem formulation.
    std::deque<optimization::CeresPose> nodes;
    std::vector<common::Time> node_times;

    for (size_t i = 0; i < timed_pose_queue_.size(); i++) {
        const bool is_last = (i == (timed_pose_queue_.size() - 1));
        const auto& timed_pose = timed_pose_queue_[i];
        node_times.push_back(timed_pose.time);

        transform::Rigid3d gravity_from_node;
        // Use the last scan match result (timed_pose_queue_.back()) for
        // initialization here instead of the last result from the optimization.
        // This keeps poses from slowly drifting apart due to lack of feedback
        // from the scan matching here.
        if (!previous_solution_.empty() && !is_last) {
        CHECK_GT(previous_solution_.size(), i);
        CHECK_EQ(timed_pose.time, previous_solution_[i].time);
        gravity_from_node = previous_solution_[i].transform;
        } else {
        gravity_from_node = gravity_from_local_ * timed_pose.transform;
        }

        if (is_last) {
        nodes.emplace_back(gravity_from_node, nullptr,
                            boost::make_unique<ceres::AutoDiffLocalParameterization<
                                ConstantYawQuaternionPlus, 4, 2>>(),
                            &problem);
        problem.SetParameterBlockConstant(nodes.back().translation());
        } else {
        nodes.emplace_back(gravity_from_node, nullptr,
                            boost::make_unique<ceres::QuaternionParameterization>(),
                            &problem);
        }
    }

    double gravity_constant = 9.8;
    bool fix_gravity = false;
    if (full_Options_.pose_extrap2_gravity_constant > 0) {
        fix_gravity = true;
        gravity_constant = full_Options_.pose_extrap2_gravity_constant;
    }

    auto imu_it_prev_prev = imu_data_.begin();
    while (imu_it_prev_prev != std::prev(imu_data_.end()) &&
            std::next(imu_it_prev_prev)->time <= timed_pose_queue_.back().time) {
        ++imu_it_prev_prev;
    }

    const TimestampedTransform prev_gravity_from_tracking =
        TimestampedTransform{node_times.back(), nodes.back().ToRigid()};
    const TimestampedTransform prev_prev_gravity_from_tracking =
        TimestampedTransform{node_times.at(node_times.size() - 2),
                            nodes.at(nodes.size() - 2).ToRigid()};
    const transform::Rigid3d initial_estimate =
        utils::ExtrapolatePoseWithImu<double>(
            prev_gravity_from_tracking.transform, prev_gravity_from_tracking.time,
            prev_prev_gravity_from_tracking.transform,
            prev_prev_gravity_from_tracking.time,
            gravity_constant * Eigen::Vector3d::UnitZ(), time, imu_data_,
            &imu_it_prev_prev)
            .pose;
    nodes.emplace_back(initial_estimate, nullptr,
                        boost::make_unique<ceres::QuaternionParameterization>(),
                        &problem);
    node_times.push_back(time);

    // Add cost functions for node constraints.
    for (size_t i = 0; i < timed_pose_queue_.size(); i++) {
        const auto& timed_pose = timed_pose_queue_[i];
        problem.AddResidualBlock(
            optimization::SpaCostFunction3D::CreateAutoDiffCostFunction(
                optimization::Constraint::Pose{
                    timed_pose.transform, full_Options_.pose_extrap2_pose_translation_weight,
                    full_Options_.pose_extrap2_pose_rotation_weight}),
            nullptr /* loss function */, gravity_from_local.rotation(),
            gravity_from_local.translation(), nodes.at(i).rotation(),
            nodes.at(i).translation());
    }

    CHECK(!imu_data_.empty());
    CHECK_LE(imu_data_.front().time, timed_pose_queue_.front().time);

    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};

    problem.AddParameterBlock(imu_calibration.data(), 4,
                                new ceres::QuaternionParameterization());
    problem.SetParameterBlockConstant(imu_calibration.data());

    auto imu_it = imu_data_.begin();
    CHECK(imu_data_.size() == 1 ||
            std::next(imu_it)->time > timed_pose_queue_.front().time);

    transform::Rigid3d last_node_odometry;
    common::Time last_node_odometry_time;

    for (size_t i = 1; i < nodes.size(); i++) {
        const common::Time first_time = node_times[i - 1];
        const common::Time second_time = node_times[i];

        auto imu_it2 = imu_it;
        const utils::IntegrateImuResult<double> result =
            utils::IntegrateImu(imu_data_, first_time, second_time, &imu_it);
        if ((i + 1) < nodes.size()) {
        const common::Time third_time = node_times[i + 1];
        const common::Duration first_duration = second_time - first_time;
        const common::Duration second_duration = third_time - second_time;
        const common::Time first_center = first_time + first_duration / 2;
        const common::Time second_center = second_time + second_duration / 2;
        const utils::IntegrateImuResult<double> result_to_first_center =
            utils::IntegrateImu(imu_data_, first_time, first_center, &imu_it2);
        const utils::IntegrateImuResult<double> result_center_to_center =
            utils::IntegrateImu(imu_data_, first_center, second_center, &imu_it2);
        // 'delta_velocity' is the change in velocity from the point in time
        // halfway between the first and second poses to halfway between
        // second and third pose. It is computed from IMU data and still
        // contains a delta due to gravity. The orientation of this vector is
        // in the IMU frame at the second pose.
        const Eigen::Vector3d delta_velocity =
            (result.delta_rotation.inverse() *
            result_to_first_center.delta_rotation) *
            result_center_to_center.delta_velocity;
        problem.AddResidualBlock(
            AccelerationCostFunction3D::CreateAutoDiffCostFunction(
                full_Options_.pose_extrap2_imu_acceleration_weight, delta_velocity,
                common::ToSeconds(first_duration),
                common::ToSeconds(second_duration)),
            nullptr /* loss function */, nodes.at(i).rotation(),
            nodes.at(i - 1).translation(), nodes.at(i).translation(),
            nodes.at(i + 1).translation(), &gravity_constant,
            imu_calibration.data());
        // TODO(danielsievers): Fix gravity in CostFunction.
        if (fix_gravity) {
            problem.SetParameterBlockConstant(&gravity_constant);
        } else {
            // Force gravity constant to be positive.
            problem.SetParameterLowerBound(&gravity_constant, 0, 0.0);
        }
        }
        problem.AddResidualBlock(
            RotationCostFunction3D::CreateAutoDiffCostFunction(
                full_Options_.pose_extrap2_imu_rotation_weight, result.delta_rotation),
            nullptr /* loss function */, nodes.at(i - 1).rotation(),
            nodes.at(i).rotation(), imu_calibration.data());

        // Add a relative pose constraint based on the odometry (if available).
        if (HasOdometryDataForTime(first_time) &&
            HasOdometryDataForTime(second_time)) {
        // Here keep track of last node odometry to avoid double computation.
        // Do this if first loop, or if there were some missing odometry nodes
        // then recalculate.
        if (i == 1 || last_node_odometry_time != first_time) {
            last_node_odometry = InterpolateOdometry(first_time);
            last_node_odometry_time = first_time;
        }
        const transform::Rigid3d current_node_odometry =
            InterpolateOdometry(second_time);
        const transform::Rigid3d relative_odometry =
            CalculateOdometryBetweenNodes(last_node_odometry,
                                            current_node_odometry);

        problem.AddResidualBlock(
            optimization::SpaCostFunction3D::CreateAutoDiffCostFunction(
                optimization::Constraint::Pose{
                    relative_odometry, full_Options_.pose_extrap2_odometry_translation_weight,
                    full_Options_.pose_extrap2_odometry_rotation_weight}),
            nullptr /* loss function */, nodes.at(i - 1).rotation(),
            nodes.at(i - 1).translation(), nodes.at(i).rotation(),
            nodes.at(i).translation());
        // Use the current node odometry in the next iteration
        last_node_odometry = current_node_odometry;
        last_node_odometry_time = second_time;
        }
    }

    // Solve.
    ceres::Solver::Summary summary;
    ceres::Solve(solver_Options_, &problem, &summary);
    LOG_IF_EVERY_N(INFO, !fix_gravity, 20) << "Gravity was: " << gravity_constant;

    const auto gravity_estimate = nodes.back().ToRigid().rotation();

    const auto& last_pose = timed_pose_queue_.back();
    const auto extrapolated_pose = TimestampedTransform{
        time, last_pose.transform *
                    nodes.at(nodes.size() - 2).ToRigid().inverse() *
                    nodes.back().ToRigid()};

    num_iterations_hist_.Add(summary.iterations.size());

    gravity_from_local_ = gravity_from_local.ToRigid();

    previous_solution_.clear();
    for (size_t i = 0; i < nodes.size(); ++i) {
        previous_solution_.push_back(
            TimestampedTransform{node_times.at(i), nodes.at(i).ToRigid()});
    }

    const Eigen::Vector3d current_velocity =
        (extrapolated_pose.transform.translation() -
        last_pose.transform.translation()) /
        common::ToSeconds(time - last_pose.time);

    return ExtrapolationResult{
        InterpolatePoses(last_pose, extrapolated_pose, times.begin(),
                        std::prev(times.end())),
        extrapolated_pose.transform, current_velocity, gravity_estimate};
}

std::vector<transform::Rigid3f> ImuBasedPoseExtrapolator::InterpolatePoses(
    const TimestampedTransform& start, const TimestampedTransform& end,
    const std::vector<common::Time>::const_iterator times_begin,
    const std::vector<common::Time>::const_iterator times_end) 
{
    std::vector<transform::Rigid3f> poses;
    poses.reserve(std::distance(times_begin, times_end));
    const float duration_scale = 1. / common::ToSeconds(end.time - start.time);

    const Eigen::Quaternionf start_rotation =
        Eigen::Quaternionf(start.transform.rotation());
    const Eigen::Quaternionf end_rotation =
        Eigen::Quaternionf(end.transform.rotation());
    const Eigen::Vector3f start_translation =
        start.transform.translation().cast<float>();
    const Eigen::Vector3f end_translation =
        end.transform.translation().cast<float>();

    for (auto it = times_begin; it != times_end; ++it) {
        const float factor = common::ToSeconds(*it - start.time) * duration_scale;
        const Eigen::Vector3f origin =
            start_translation + (end_translation - start_translation) * factor;
        const Eigen::Quaternionf rotation =
            start_rotation.slerp(factor, end_rotation);
        poses.emplace_back(origin, rotation);
    }
    return poses;
}

transform::Rigid3d 
ImuBasedPoseExtrapolator::ExtrapolatePose(const common::Time time) 
{
    return ExtrapolatePosesWithGravity(std::vector<common::Time>{time})
        .current_pose;
}

Eigen::Quaterniond 
ImuBasedPoseExtrapolator::EstimateGravityOrientation(const common::Time time) 
{
    return ExtrapolatePosesWithGravity(std::vector<common::Time>{time})
        .gravity_from_tracking;
}

template <typename T>
void ImuBasedPoseExtrapolator::TrimDequeData(std::deque<T>* data) 
{
    while (data->size() > 1 && !timed_pose_queue_.empty() &&
            data->at(1).time <= timed_pose_queue_.front().time) {
        data->pop_front();
    }
}

void ImuBasedPoseExtrapolator::TrimImuData() 
{
    TrimDequeData<sensor::ImuData>(&imu_data_);
}

void ImuBasedPoseExtrapolator::TrimOdometryData() 
{
    TrimDequeData<sensor::OdometryData>(&odometry_data_);
}

// Odometry methods
bool ImuBasedPoseExtrapolator::HasOdometryData() const 
{
    return odometry_data_.size() >= 2;
}

bool ImuBasedPoseExtrapolator::HasOdometryDataForTime(
    const common::Time& time) const 
{
    return HasOdometryData() && odometry_data_.front().time < time &&
            time < odometry_data_.back().time;
}

transform::Rigid3d 
ImuBasedPoseExtrapolator::InterpolateOdometry(
    const common::Time& time) const 
{
    // Only interpolate if time is within odometry data range.
    CHECK(HasOdometryDataForTime(time))
        << "Odometry data range does not include time " << time;
    std::deque<sensor::OdometryData>::const_iterator data = std::upper_bound(
        odometry_data_.begin(), odometry_data_.end(), time,
        [](const common::Time& time, const sensor::OdometryData& odometry_data) {
            return time < odometry_data.time;
        });
    const TimestampedTransform first{std::prev(data)->time,
                                    std::prev(data)->pose};
    const TimestampedTransform second{data->time, data->pose};
    return Interpolate(first, second, time).transform;
}

transform::Rigid3d 
ImuBasedPoseExtrapolator::CalculateOdometryBetweenNodes(
    const transform::Rigid3d& first_node_odometry,
    const transform::Rigid3d& second_node_odometry) const 
{
    return first_node_odometry.inverse() * second_node_odometry;
}

}  // namespace csmlio
}  // namespace infinityslam
