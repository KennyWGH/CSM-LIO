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

#ifndef INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H_
#define INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H_

#include <vector>
#include <memory>
#include <boost/make_unique.hpp>
#include "Eigen/Core"

#include "infinityslam/common/system_options.h"
#include "infinityslam/csmlio/submap/hybrid_grid.h"
#include "infinityslam/sensor/point_cloud.h"

namespace infinityslam {
namespace csmlio {
namespace scan_matching {

struct RTCorrelativeScanMatcher3DOptions {
 public:
    explicit RTCorrelativeScanMatcher3DOptions(
        const double& linear_search_window = 0.15,
        const double& angular_search_window = 0.035,
        const double& trans_delta_cost_weight = 1e-1, 
        const double& rot_delta_cost_weight = 1e-1)
        : rt_csm_linear_search_window(linear_search_window)
        , rt_csm_angular_search_window(angular_search_window)
        , rt_csm_translation_delta_cost_weight(trans_delta_cost_weight)
        , rt_csm_rotation_delta_cost_weight(rot_delta_cost_weight) {}

    double linear_search_window() const {return rt_csm_linear_search_window;}
    double angular_search_window() const {return rt_csm_angular_search_window;}
    double translation_delta_cost_weight() const {return rt_csm_translation_delta_cost_weight;}
    double rotation_delta_cost_weight() const {return rt_csm_rotation_delta_cost_weight;}

    // std::string DebugString() {/*TODO*/}

 private:
    double rt_csm_linear_search_window = 0.15;
    double rt_csm_angular_search_window = 0.035; // math.rad(1.)   0.0175
    double rt_csm_translation_delta_cost_weight = 1e-1;
    double rt_csm_rotation_delta_cost_weight = 1e-1;
};

// 从common::options加载参数
RTCorrelativeScanMatcher3DOptions ReadRTCorrelativeScanMatcher3DOptions();

// A voxel accurate scan matcher, exhaustively evaluating the scan matching
// search space.
class RealTimeCorrelativeScanMatcher3D {
 public:

    explicit RealTimeCorrelativeScanMatcher3D(
        const RTCorrelativeScanMatcher3DOptions& options);

    RealTimeCorrelativeScanMatcher3D(const RealTimeCorrelativeScanMatcher3D&) = delete;
    RealTimeCorrelativeScanMatcher3D& operator=(const RealTimeCorrelativeScanMatcher3D&) = delete;

    // Aligns 'point_cloud' within the 'hybrid_grid' given an
    // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
    // returns the score.
    float Match(const transform::Rigid3d& initial_pose_estimate,
                const sensor::PointCloud& point_cloud,
                const HybridGrid& hybrid_grid,
                transform::Rigid3d* pose_estimate) const;

 private:
    std::vector<transform::Rigid3f> GenerateExhaustiveSearchTransforms(
        float resolution, const sensor::PointCloud& point_cloud) const;
    float ScoreCandidate(const HybridGrid& hybrid_grid,
                        const sensor::PointCloud& transformed_point_cloud,
                        const transform::Rigid3f& transform) const;

    const RTCorrelativeScanMatcher3DOptions oPtIons_;
};

// 基于自定义参数来构造对象
std::unique_ptr<RealTimeCorrelativeScanMatcher3D> 
CreateRealTimeCorrelativeScanMatcher3D (
    const RTCorrelativeScanMatcher3DOptions& options);

// 基于common::options参数(系统默认参数)来构造对象
std::unique_ptr<RealTimeCorrelativeScanMatcher3D> 
CreateRealTimeCorrelativeScanMatcher3D ();

}  // namespace scan_matching
}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H_
