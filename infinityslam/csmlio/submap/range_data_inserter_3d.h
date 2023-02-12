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

#ifndef INFINITYSLAM_CSMLIO_RANGE_DATA_INSERTER_3D_H_
#define INFINITYSLAM_CSMLIO_RANGE_DATA_INSERTER_3D_H_

#include "infinityslam/csmlio/submap/hybrid_grid.h"
// #include "infinityslam/csmlio/proto/range_data_inserter_options_3d.pb.h"
#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/sensor/range_data.h"

namespace infinityslam {
namespace csmlio {

struct RangeDataInserter3DOptions {
    double submap_insertion_hit_probability = 0.55;
    double submap_insertion_miss_probability = 0.49;
    int    submap_insertion_num_free_space_voxels = 2;
    double submap_insertion_intensity_threshold = 10.;

    RangeDataInserter3DOptions() {}
};

// 从common中读取参数。
RangeDataInserter3DOptions ReadRangeDataInserter3DOptions();

// proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
//     common::LuaParameterDictionary* parameter_dictionary);

class RangeDataInserter3D {
 public:
    explicit RangeDataInserter3D(
        const RangeDataInserter3DOptions& options);

    RangeDataInserter3D(
        const double& hit_probability, 
        const double& miss_probability, 
        const int&    num_free_space_voxels, 
        const double& intensity_threshold);

    RangeDataInserter3D(const RangeDataInserter3D&) = delete;
    RangeDataInserter3D& operator=(const RangeDataInserter3D&) = delete;

    // Inserts 'range_data' into 'hybrid_grid' and optionally into 'intensity_hybrid_grid'.
    void Insert(const sensor::RangeData& range_data, HybridGrid* hybrid_grid,
                IntensityHybridGrid* intensity_hybrid_grid) const;

 private:

    double k_hit_probability_ = 0.55;
    double k_miss_probability_ = 0.49;
    int    k_num_free_space_voxels_ = 2;
    double k_intensity_threshold_ = 10;
    // const proto::RangeDataInserterOptions3D options_;
    const std::vector<uint16> hit_table_;
    const std::vector<uint16> miss_table_;
};

}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_RANGE_DATA_INSERTER_3D_H_
