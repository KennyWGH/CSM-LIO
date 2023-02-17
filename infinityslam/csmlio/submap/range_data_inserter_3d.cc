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

#include "infinityslam/csmlio/submap/range_data_inserter_3d.h"

#include "infinityslam/common/system_options.h"
#include "infinityslam/csmlio/submap/probability_values.h"

#include "Eigen/Core"
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {
namespace {

void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& returns,
                          HybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  for (const sensor::PointTypeXYZ& hit : returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit.position);

    const Eigen::Array3i delta = hit_cell - origin_cell;
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples we equi-distantly place on the
    // line between 'origin' and 'hit'. (including a fractional part for sub-
    // voxels) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension.
    //
    // Only the last 'num_free_space_voxels' are updated for performance.
    for (int position = std::max(0, num_samples - num_free_space_voxels);
         position < num_samples; ++position) {
      const Eigen::Array3i miss_cell =
          origin_cell + delta * position / num_samples;
      hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
    }
  }
}

void InsertIntensitiesIntoGrid(const sensor::PointCloud& returns,
                               IntensityHybridGrid* intensity_hybrid_grid,
                               const float intensity_threshold) {
  if (returns.intensities().size() > 0) {
    for (size_t i = 0; i < returns.size(); ++i) {
      if (returns.intensities()[i] > intensity_threshold) {
        continue;
      }
      const Eigen::Array3i hit_cell =
          intensity_hybrid_grid->GetCellIndex(returns[i].position);
      intensity_hybrid_grid->AddIntensity(hit_cell, returns.intensities()[i]);
    }
  }
}

}  // namespace

RangeDataInserter3DOptions ReadRangeDataInserter3DOptions() {
    RangeDataInserter3DOptions options;
    options.submap_insertion_hit_probability = common::options::csmlio::submap_insertion_hit_probability;
    options.submap_insertion_miss_probability = common::options::csmlio::submap_insertion_miss_probability;
    options.submap_insertion_num_free_space_voxels = common::options::csmlio::submap_insertion_num_free_space_voxels;
    options.submap_insertion_intensity_threshold = common::options::csmlio::submap_insertion_intensity_threshold;
    return options;
}

// proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
//     common::LuaParameterDictionary* parameter_dictionary) {
//     proto::RangeDataInserterOptions3D options;
//     options.set_hit_probability(
//         parameter_dictionary->GetDouble("hit_probability"));
//     options.set_miss_probability(
//         parameter_dictionary->GetDouble("miss_probability"));
//     options.set_num_free_space_voxels(
//         parameter_dictionary->GetInt("num_free_space_voxels"));
//     options.set_intensity_threshold(
//         parameter_dictionary->GetDouble("intensity_threshold"));
//     CHECK_GT(options.hit_probability(), 0.5);
//     CHECK_LT(options.miss_probability(), 0.5);
//     return options;
// }

RangeDataInserter3D::RangeDataInserter3D(
    const RangeDataInserter3DOptions& options)
    : k_hit_probability_(options.submap_insertion_hit_probability),
      k_miss_probability_(options.submap_insertion_miss_probability),
      k_num_free_space_voxels_(options.submap_insertion_num_free_space_voxels),
      k_intensity_threshold_(options.submap_insertion_intensity_threshold),
      hit_table_(
          ComputeLookupTableToApplyOdds(Odds(options.submap_insertion_hit_probability))),
      miss_table_(
          ComputeLookupTableToApplyOdds(Odds(options.submap_insertion_miss_probability))) {}

RangeDataInserter3D::RangeDataInserter3D(
    const double& hit_probability, 
    const double& miss_probability, 
    const int& num_free_space_voxels, 
    const double&  intensity_threshold)
    : k_hit_probability_(hit_probability),
      k_miss_probability_(miss_probability),
      k_num_free_space_voxels_(num_free_space_voxels),
      k_intensity_threshold_(intensity_threshold),
      hit_table_(
          ComputeLookupTableToApplyOdds(Odds(k_hit_probability_))),
      miss_table_(
          ComputeLookupTableToApplyOdds(Odds(k_miss_probability_))) {}


void RangeDataInserter3D::Insert(
    const sensor::RangeData& range_data, HybridGrid* hybrid_grid,
    IntensityHybridGrid* intensity_hybrid_grid) const {
    CHECK_NOTNULL(hybrid_grid);

    for (const sensor::PointTypeXYZ& hit : range_data.returns) {
        const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit.position);
        hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);
    }

    // By not starting a new update after hits are inserted, we give hits priority
    // (i.e. no hits will be ignored because of a miss in the same cell).
    InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                        hybrid_grid, k_num_free_space_voxels_);
    if (intensity_hybrid_grid != nullptr) {
        InsertIntensitiesIntoGrid(range_data.returns, 
                                intensity_hybrid_grid,
                                k_intensity_threshold_);
    }
    hybrid_grid->FinishUpdate();
}

}  // namespace csmlio
}  // namespace infinityslam
