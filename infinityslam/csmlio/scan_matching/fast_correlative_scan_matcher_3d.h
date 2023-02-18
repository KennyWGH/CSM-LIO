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

// This is an implementation of a 3D branch-and-bound algorithm.

#ifndef INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_3D_H_
#define INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_3D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "infinityslam/common/numeric_types.h"
#include "infinityslam/csmlio/submap/hybrid_grid.h"
#include "infinityslam/csmlio/scan_matching/precomputation_grid_3d.h"
#include "infinityslam/csmlio/scan_matching/rotational_scan_matcher.h"
#include "infinityslam/csmlio/trajectory_node.h"
#include "infinityslam/sensor/point_cloud.h"

namespace infinityslam {
namespace csmlio {
namespace scan_matching {

struct FastCorrelaTiveScanMaTcherOpTions3D {
 public:
    FastCorrelaTiveScanMaTcherOpTions3D() {}

    int branch_and_bound_depth() const {return branch_and_bound_depth_;}
    int full_resolution_depth() const {return full_resolution_depth_;}
    double min_rotational_score() const {return min_rotational_score_;}
    double min_low_resolution_score() const {return min_low_resolution_score_;}
    double linear_xy_search_window() const {return linear_xy_search_window_;}
    double linear_z_search_window() const {return linear_z_search_window_;}
    double angular_search_window() const {return angular_search_window_;}

 private:
    int branch_and_bound_depth_ = 8;
    int full_resolution_depth_ = 3;
    double min_rotational_score_ = 0.77;
    double min_low_resolution_score_ = 0.55;
    double linear_xy_search_window_ = 5.;
    double linear_z_search_window_ = 8.;    // by default is 1.0.
    double angular_search_window_ = 0.262;  // 约15度 // math.rad(15.),
};

class PrecomputationGridStack3D {
 public:
    PrecomputationGridStack3D(
        const HybridGrid& hybrid_grid,
        const FastCorrelaTiveScanMaTcherOpTions3D& options);

    const PrecomputationGrid3D& Get(int depth) const {
        return precomputation_grids_.at(depth);
    }

    int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
    std::vector<PrecomputationGrid3D> precomputation_grids_;
};

struct DiscreteScan3D;
struct Candidate3D;

// Used to compute scores between 0 and 1 how well the given pose matches.
using MatchingFunction = std::function<float(const transform::Rigid3f&)>;

/// @brief 回环检测（分支定界算法）的实现在这个类中
class FastCorrelativeScanMatcher3D {
 public:
    struct Result {
        float score;
        transform::Rigid3d pose_estimate;
        float rotational_score;
        float low_resolution_score;
    };

    FastCorrelativeScanMatcher3D(
        const HybridGrid& hybrid_grid,
        const HybridGrid* low_resolution_hybrid_grid,
        const Eigen::VectorXf* rotational_scan_matcher_histogram,
        const FastCorrelaTiveScanMaTcherOpTions3D& options);
    ~FastCorrelativeScanMatcher3D();

    FastCorrelativeScanMatcher3D(const FastCorrelativeScanMatcher3D&) = delete;
    FastCorrelativeScanMatcher3D& operator=(const FastCorrelativeScanMatcher3D&) =
        delete;

    // Aligns the node with the given 'constant_data' within the 'hybrid_grid'
    // given 'global_node_pose' and 'global_submap_pose'. 'Result' is only
    // returned if a score above 'min_score' (excluding equality) is possible.
    std::unique_ptr<Result> Match(const transform::Rigid3d& global_node_pose,
                                    const transform::Rigid3d& global_submap_pose,
                                    const TrajectoryNode::Data& constant_data,
                                    float min_score) const;

    // Aligns the node with the given 'constant_data' within the 'hybrid_grid'
    // given rotations which are expected to be approximately gravity aligned.
    // 'Result' is only returned if a score above 'min_score' (excluding equality)
    // is possible.
    std::unique_ptr<Result> MatchFullSubmap(
        const Eigen::Quaterniond& global_node_rotation,
        const Eigen::Quaterniond& global_submap_rotation,
        const TrajectoryNode::Data& constant_data, float min_score) const;

 private:
    struct SearchParameters {
        const int linear_xy_window_size;     // voxels
        const int linear_z_window_size;      // voxels
        const double angular_search_window;  // radians
        const MatchingFunction* const low_resolution_matcher;
    };

    std::unique_ptr<Result> MatchWithSearchParameters(
        const SearchParameters& search_parameters,
        const transform::Rigid3f& global_node_pose,
        const transform::Rigid3f& global_submap_pose,
        const sensor::PointCloud& point_cloud,
        const Eigen::VectorXf& rotational_scan_matcher_histogram,
        const Eigen::Quaterniond& gravity_alignment, float min_score) const;
    DiscreteScan3D DiscretizeScan(const SearchParameters& search_parameters,
                                    const sensor::PointCloud& point_cloud,
                                    const transform::Rigid3f& pose,
                                    float rotational_score) const;
    std::vector<DiscreteScan3D> GenerateDiscreteScans(
        const SearchParameters& search_parameters,
        const sensor::PointCloud& point_cloud,
        const Eigen::VectorXf& rotational_scan_matcher_histogram,
        const Eigen::Quaterniond& gravity_alignment,
        const transform::Rigid3f& global_node_pose,
        const transform::Rigid3f& global_submap_pose) const;
    std::vector<Candidate3D> GenerateLowestResolutionCandidates(
        const SearchParameters& search_parameters, int num_discrete_scans) const;
    void ScoreCandidates(int depth,
                        const std::vector<DiscreteScan3D>& discrete_scans,
                        std::vector<Candidate3D>* const candidates) const;
    std::vector<Candidate3D> ComputeLowestResolutionCandidates(
        const SearchParameters& search_parameters,
        const std::vector<DiscreteScan3D>& discrete_scans) const;
    Candidate3D BranchAndBound(const SearchParameters& search_parameters,
                                const std::vector<DiscreteScan3D>& discrete_scans,
                                const std::vector<Candidate3D>& candidates,
                                int candidate_depth, float min_score) const;
    transform::Rigid3f GetPoseFromCandidate(
        const std::vector<DiscreteScan3D>& discrete_scans,
        const Candidate3D& candidate) const;

    const FastCorrelaTiveScanMaTcherOpTions3D OpTions_;
    const float resolution_;
    const int width_in_voxels_;
    std::unique_ptr<PrecomputationGridStack3D> precomputation_grid_stack_;
    const HybridGrid* const low_resolution_hybrid_grid_;
    RotationalScanMatcher rotational_scan_matcher_;
};

}  // namespace scan_matching
}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_INTERNAL_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_3D_H_
