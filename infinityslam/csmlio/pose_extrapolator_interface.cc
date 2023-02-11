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

#include "infinityslam/csmlio/pose_extrapolator_interface.h"

#include "infinityslam/common/time.h"
#include "infinityslam/common/system_options.h"
#include "infinityslam/csmlio/internal/imu_based_pose_extrapolator.h"
#include "infinityslam/csmlio/pose_extrapolator.h"

namespace infinityslam {
namespace mapping {

PoseExtrapolatorFullOptions ReadPoseExtrapolatorFullOptions() {
    PoseExtrapolatorFullOptions options;

    options.imu_gravity_time_constant = infinityslam::common::options::csmlio::imu_gravity_time_constant;
    options.use_imu_based_pose_extrapolator = infinityslam::common::options::csmlio::use_imu_based_pose_extrapolator;

    options.pose_extrap1_imu_gravity_time_constant = infinityslam::common::options::csmlio::pose_extrap1_imu_gravity_time_constant;
    options.pose_extrap1_pose_queue_duration = infinityslam::common::options::csmlio::pose_extrap1_pose_queue_duration;

    options.pose_extrap2_pose_queue_duration = infinityslam::common::options::csmlio::pose_extrap2_pose_queue_duration;    
    options.pose_extrap2_gravity_constant = infinityslam::common::options::csmlio::pose_extrap2_gravity_constant;
    options.pose_extrap2_pose_translation_weight = infinityslam::common::options::csmlio::pose_extrap2_pose_translation_weight;
    options.pose_extrap2_pose_rotation_weight = infinityslam::common::options::csmlio::pose_extrap2_pose_rotation_weight;
    options.pose_extrap2_imu_acceleration_weight = infinityslam::common::options::csmlio::pose_extrap2_imu_acceleration_weight;
    options.pose_extrap2_imu_rotation_weight = infinityslam::common::options::csmlio::pose_extrap2_imu_rotation_weight;
    options.pose_extrap2_odometry_translation_weight = infinityslam::common::options::csmlio::pose_extrap2_odometry_translation_weight;
    options.pose_extrap2_odometry_rotation_weight = infinityslam::common::options::csmlio::pose_extrap2_odometry_rotation_weight;
    options.pose_extrap2_solver_use_nonmonotonic_steps = infinityslam::common::options::csmlio::pose_extrap2_solver_use_nonmonotonic_steps;
    options.pose_extrap2_solver_max_num_iterations = infinityslam::common::options::csmlio::pose_extrap2_solver_max_num_iterations;
    options.pose_extrap2_solver_num_threads = infinityslam::common::options::csmlio::pose_extrap2_solver_num_threads;

    return options;
}

std::unique_ptr<PoseExtrapolatorInterface>
PoseExtrapolatorInterface::CreateWithImuData(
    const std::vector<sensor::ImuData>& imu_data,
    const std::vector<transform::TimestampedTransform>& initial_poses,
    bool init_with_common_options)
{
    CHECK(!imu_data.empty());

    PoseExtrapolatorFullOptions full_options = init_with_common_options ?
        ReadPoseExtrapolatorFullOptions() : PoseExtrapolatorFullOptions();

    if (full_options.use_imu_based_pose_extrapolator) {
        return ImuBasedPoseExtrapolator::InitializeWithImu(full_options, 
                                                        imu_data, initial_poses);
    } else {
        return PoseExtrapolator::InitializeWithImu(full_options, imu_data.back());
    }
}

}  // namespace mapping
}  // namespace infinityslam
