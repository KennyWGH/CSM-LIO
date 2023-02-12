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

#ifndef INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_
#define INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "infinityslam/common/system_options.h"
#include "infinityslam/csmlio/submap/hybrid_grid.h"
#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/transform/rigid_transform.h"
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {
namespace scan_matching {

// 自定义Options结构体
struct CeresScanMatcher3DOptions {
    double csm_occupied_space_weight_0 = 6.; // wgh 高分辨率栅格&点云匹配
    double csm_occupied_space_weight_1 = 6.; // wgh 低分辨率栅格&点云匹配
    double csm_translation_weight = 5.;
    double csm_rotation_weight = 4e2;
    bool   csm_only_optimize_yaw = true;
    double csm_intensity_0_weight = 0.5;
    double csm_intensity_0_huber_scale = 0.3;
    double csm_intensity_0_threshold = 20;
    bool csm_solver_use_nonmonotonic_steps = false;
    int  csm_solver_max_num_iterations = 12;
    int  csm_solver_num_threads = 1;

    CeresScanMatcher3DOptions() {}

    double occupied_space_weight(int i) const {
        if (i==0) return csm_occupied_space_weight_0;
        if (i==1) return csm_occupied_space_weight_1;
        LOG(ERROR) << "Input index must be 0 or 1, which is " << i << " actually.";
        return 1e-6; //异常处理
    }

    std::string DebugString() const {
        std::ostringstream oss;
        oss << "CeresScanMatcher3DOptions Info: \n";
        oss << "    csm_occupied_space_weight_0 : " << csm_occupied_space_weight_0 << "\n"
            << "    csm_occupied_space_weight_1 : " << csm_occupied_space_weight_1 << "\n"
            << "    csm_translation_weight      : " << csm_translation_weight << "\n"
            << "    csm_rotation_weight         : " << csm_rotation_weight << "\n"
            << "    csm_only_optimize_yaw       : " << csm_only_optimize_yaw << "\n"
            << "    csm_intensity_0_weight      : " << csm_intensity_0_weight << "\n"
            << "    csm_intensity_0_huber_scale : " << csm_intensity_0_huber_scale << "\n"
            << "    csm_intensity_0_threshold   : " << csm_intensity_0_threshold << "\n"
            << "    csm_solver_use_nonmonotonic_steps : " << csm_solver_use_nonmonotonic_steps << "\n"
            << "    csm_solver_max_num_iterations     : " << csm_solver_max_num_iterations << "\n"
            << "    csm_solver_num_threads            : " << csm_solver_num_threads;
        return oss.str();
    }
};

// 根据common参数构造CeresScanMatcher3DOptions对象。
CeresScanMatcher3DOptions ReadCeresScanMatcher3DOptions();

struct PointCloudAndHybridGridsPointers {
    const sensor::PointCloud* point_cloud;
    const HybridGrid* hybrid_grid;
    const IntensityHybridGrid* intensity_hybrid_grid;  // optional
};

// This scan matcher uses Ceres to align scans with an existing map.
class CeresScanMatcher3D {
 public:
    explicit CeresScanMatcher3D(const CeresScanMatcher3DOptions& full_options);

    CeresScanMatcher3D(const CeresScanMatcher3D&) = delete;
    CeresScanMatcher3D& operator=(const CeresScanMatcher3D&) = delete;

    // Aligns 'point_clouds' within the 'hybrid_grids' given an
    // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
    // 'summary'.
    void Match(const Eigen::Vector3d& target_translation,
                const transform::Rigid3d& initial_pose_estimate,
                const std::vector<PointCloudAndHybridGridsPointers>&
                    point_clouds_and_hybrid_grids,
                transform::Rigid3d* pose_estimate,
                ceres::Solver::Summary* summary) const;

 private:
    const CeresScanMatcher3DOptions full_Options_;
    ceres::Solver::Options ceres_solver_Options_;
};

// 根据common参数构造CeresScanMatcher3D对象。
std::unique_ptr<CeresScanMatcher3D> CreateCeresScanMatcher3D();

}  // namespace scan_matching
}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_
