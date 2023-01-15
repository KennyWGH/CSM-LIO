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

#ifndef CSMLIO_MAPPING_3D_RANGE_DATA_INSERTER_3D_H_
#define CSMLIO_MAPPING_3D_RANGE_DATA_INSERTER_3D_H_

#include "csmlio/lio/3d/hybrid_grid.h"
#include "csmlio/lio/proto/range_data_inserter_options_3d.pb.h"
#include "csmlio/sensor/point_cloud.h"
#include "csmlio/sensor/range_data.h"

namespace csmlio {
namespace mapping {

proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
    common::LuaParameterDictionary* parameter_dictionary);

class RangeDataInserter3D {
 public:
  explicit RangeDataInserter3D(
      const proto::RangeDataInserterOptions3D& options);

  RangeDataInserter3D(const RangeDataInserter3D&) = delete;
  RangeDataInserter3D& operator=(const RangeDataInserter3D&) = delete;

  // Inserts 'range_data' into 'hybrid_grid' and optionally into
  // 'intensity_hybrid_grid'.
  void Insert(const sensor::RangeData& range_data, HybridGrid* hybrid_grid,
              IntensityHybridGrid* intensity_hybrid_grid) const;

 private:
  const proto::RangeDataInserterOptions3D options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_3D_RANGE_DATA_INSERTER_3D_H_
