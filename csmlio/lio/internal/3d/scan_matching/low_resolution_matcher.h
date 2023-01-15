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

#ifndef CSMLIO_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOW_RESOLUTION_MATCHER_H_
#define CSMLIO_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOW_RESOLUTION_MATCHER_H_

#include <functional>

#include "csmlio/lio/3d/hybrid_grid.h"
#include "csmlio/sensor/point_cloud.h"
#include "csmlio/transform/rigid_transform.h"

namespace csmlio {
namespace mapping {
namespace scan_matching {

std::function<float(const transform::Rigid3f&)> CreateLowResolutionMatcher(
    const HybridGrid* low_resolution_grid, const sensor::PointCloud* points);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOW_RESOLUTION_MATCHER_H_
