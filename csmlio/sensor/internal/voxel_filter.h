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

#ifndef CSMLIO_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CSMLIO_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/sensor/point_cloud.h"
#include "csmlio/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "csmlio/sensor/timed_point_cloud_data.h"

namespace csmlio {
namespace sensor {

std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, const float resolution);
PointCloud VoxelFilter(const PointCloud& point_cloud, const float resolution);
TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            const float resolution);
std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    const float resolution);

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

PointCloud AdaptiveVoxelFilter(
    const PointCloud& point_cloud,
    const proto::AdaptiveVoxelFilterOptions& options);

}  // namespace sensor
}  // namespace csmlio

#endif  // CSMLIO_SENSOR_INTERNAL_VOXEL_FILTER_H_
