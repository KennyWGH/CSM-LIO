/*
 * Copyright 2016 The Cartographer Authors
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

#include "infinityslam/csmlio/scan_matching/ceres_scan_matcher_3d.h"

#include <string>
#include <utility>
#include <vector>
#include <boost/make_unique.hpp>

#include "infinityslam/common/system_options.h"
#include "infinityslam/csmlio/optimization/rotation_parameterization.h"
#include "infinityslam/csmlio/scan_matching/intensity_cost_function_3d.h"
#include "infinityslam/csmlio/scan_matching/occupied_space_cost_function_3d.h"
#include "infinityslam/csmlio/scan_matching/rotation_delta_cost_functor_3d.h"
#include "infinityslam/csmlio/scan_matching/translation_delta_cost_functor_3d.h"
#include "infinityslam/csmlio/optimization/ceres_pose.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {
namespace scan_matching {

namespace {

ceres::Solver::Options CreateCeresSolverOptions (
    const CeresScanMatcher3DOptions& full_options) {
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = full_options.csm_solver_use_nonmonotonic_steps;
    options.max_num_iterations = full_options.csm_solver_max_num_iterations;
    options.num_threads = full_options.csm_solver_num_threads;
    return options;
}

} // namespace

CeresScanMatcher3DOptions ReadCeresScanMatcher3DOptions() {
    CeresScanMatcher3DOptions options;
    options.csm_occupied_space_weight_0 = common::options::csmlio::csm_occupied_space_weight_0;
    options.csm_occupied_space_weight_1 = common::options::csmlio::csm_occupied_space_weight_1;
    options.csm_translation_weight = common::options::csmlio::csm_translation_weight;
    options.csm_rotation_weight = common::options::csmlio::csm_rotation_weight;
    options.csm_only_optimize_yaw = common::options::csmlio::csm_only_optimize_yaw;
    options.csm_intensity_0_weight = common::options::csmlio::csm_intensity_0_weight;
    options.csm_intensity_0_huber_scale = common::options::csmlio::csm_intensity_0_huber_scale;
    options.csm_intensity_0_threshold = common::options::csmlio::csm_intensity_0_threshold;
    options.csm_solver_use_nonmonotonic_steps = common::options::csmlio::csm_solver_use_nonmonotonic_steps;
    options.csm_solver_max_num_iterations = common::options::csmlio::csm_solver_max_num_iterations;
    options.csm_solver_num_threads = common::options::csmlio::csm_solver_num_threads;
    return options;
}

CeresScanMatcher3D::CeresScanMatcher3D(
    const CeresScanMatcher3DOptions& full_options) 
    : full_Options_(full_options)
    , ceres_solver_Options_(CreateCeresSolverOptions(full_options)) {
    ceres_solver_Options_.linear_solver_type = ceres::DENSE_QR;
}

void CeresScanMatcher3D::Match(
    const Eigen::Vector3d& target_translation,
    const transform::Rigid3d& initial_pose_estimate,
    const std::vector<PointCloudAndHybridGridsPointers>&
        point_clouds_and_hybrid_grids,
    transform::Rigid3d* const pose_estimate,
    ceres::Solver::Summary* const summary) const 
{
    ceres::Problem problem;
    optimization::CeresPose ceres_pose(
        initial_pose_estimate, nullptr /* translation_parameterization */,
        full_Options_.csm_only_optimize_yaw
            ? std::unique_ptr<ceres::LocalParameterization>(
                    boost::make_unique<ceres::AutoDiffLocalParameterization<
                        YawOnlyQuaternionPlus, 4, 1>>())
            : std::unique_ptr<ceres::LocalParameterization>(
                    boost::make_unique<ceres::QuaternionParameterization>()),
        &problem);

    CHECK_EQ(/*高+低分辨率共两个*/ 2, point_clouds_and_hybrid_grids.size());
    for (size_t i = 0; i != point_clouds_and_hybrid_grids.size(); ++i) {
        CHECK_GT(full_Options_.occupied_space_weight(i), 0.);
        const sensor::PointCloud& point_cloud =
            *point_clouds_and_hybrid_grids[i].point_cloud;
        const HybridGrid& hybrid_grid =
            *point_clouds_and_hybrid_grids[i].hybrid_grid;
        problem.AddResidualBlock(
            OccupiedSpaceCostFunction3D::CreateAutoDiffCostFunction(
                full_Options_.occupied_space_weight(i) /
                    std::sqrt(static_cast<double>(point_cloud.size())),
                point_cloud, hybrid_grid),
            nullptr /* loss function */, ceres_pose.translation(),
            ceres_pose.rotation());
        if (point_clouds_and_hybrid_grids[i].intensity_hybrid_grid) {
            LOG(INFO) << " ************* add intensity cost function with i=" << i;
            CHECK_GT(full_Options_.csm_intensity_0_huber_scale, 0.);
            CHECK_GT(full_Options_.csm_intensity_0_weight, 0.);
            CHECK_GT(full_Options_.csm_intensity_0_threshold, 0);
            const IntensityHybridGrid& intensity_hybrid_grid =
                *point_clouds_and_hybrid_grids[i].intensity_hybrid_grid;
            problem.AddResidualBlock(
                IntensityCostFunction3D::CreateAutoDiffCostFunction(
                    full_Options_.csm_intensity_0_weight /
                        std::sqrt(static_cast<double>(point_cloud.size())),
                    full_Options_.csm_intensity_0_threshold,
                    point_cloud, intensity_hybrid_grid),
                new ceres::HuberLoss(full_Options_.csm_intensity_0_huber_scale),
                ceres_pose.translation(), ceres_pose.rotation());
        }
    }

    CHECK_GT(full_Options_.csm_translation_weight, 0.);
    problem.AddResidualBlock(
        TranslationDeltaCostFunctor3D::CreateAutoDiffCostFunction(
            full_Options_.csm_translation_weight, 
            target_translation),
        nullptr /* loss function */, ceres_pose.translation());
    CHECK_GT(full_Options_.csm_rotation_weight, 0.);
    problem.AddResidualBlock(
        RotationDeltaCostFunctor3D::CreateAutoDiffCostFunction(
            full_Options_.csm_rotation_weight,
            initial_pose_estimate.rotation()),
        nullptr /* loss function */, ceres_pose.rotation());

    ceres::Solve(ceres_solver_Options_, &problem, summary);
    *pose_estimate = ceres_pose.ToRigid();
}

std::unique_ptr<CeresScanMatcher3D> CreateCeresScanMatcher3D() {
    return boost::make_unique<CeresScanMatcher3D>(ReadCeresScanMatcher3DOptions());
}

}  // namespace scan_matching
}  // namespace csmlio
}  // namespace infinityslam
