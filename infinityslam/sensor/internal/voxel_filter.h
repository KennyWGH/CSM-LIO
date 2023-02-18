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

#ifndef INFINITYSLAM_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define INFINITYSLAM_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>

#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/sensor/timed_point_cloud_data.h"

namespace infinityslam {
namespace sensor {

struct AdaptiveVoxelFIlterOpTions {
 public:
    AdaptiveVoxelFIlterOpTions(double v1=2., int v2=1000, double v3=60.) 
        : max_length_(v1), min_num_points_(v2), max_range_(v3) {}

    float max_length() const {return max_length_;}
    int   min_num_points() const {return min_num_points_;}
    float max_range() const {return max_range_;}

    void set_max_length(double value) {max_length_ = value;}
    void set_min_num_points(int value) {min_num_points_ = value;}
    void set_max_range(double value) {max_range_ = value;}

 private:
    double max_length_ = 2.;
    int    min_num_points_ = 1000;
    double max_range_ = 60.;
};

PointCloud AdaptiveVoxelFilter(const PointCloud& point_cloud,
    const AdaptiveVoxelFIlterOpTions& options);

std::vector<PointTypeXYZ> 
VoxelFilter(const std::vector<PointTypeXYZ>& points, const float resolution);

PointCloud 
VoxelFilter(const PointCloud& point_cloud, const float resolution);

TimedPointCloud 
VoxelFilter(const TimedPointCloud& timed_point_cloud, const float resolution);

std::vector<sensor::MultiTimedPOintCloudData::RangeMeasurement> 
VoxelFilter(
    const std::vector<sensor::MultiTimedPOintCloudData::RangeMeasurement>&
        range_measurements,
    const float resolution);


}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_INTERNAL_VOXEL_FILTER_H_
