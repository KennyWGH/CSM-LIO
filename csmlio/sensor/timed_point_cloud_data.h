/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CSMLIO_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CSMLIO_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "csmlio/common/time.h"
#include "csmlio/sensor/point_cloud.h"

namespace csmlio {
namespace sensor {

// wgh ROS层向算法层传递点云数据的结构
struct TimedPointCloudData {
  common::Time time;
  Eigen::Vector3f origin;   // 通常是sensor_in_tracking的xyz坐标(静态TF).
  TimedPointCloud ranges;
  // 'intensities' has to be same size as 'ranges', or empty.
  std::vector<float> intensities;
};

struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;
    float intensity;
    size_t origin_index;
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace csmlio

#endif  // CSMLIO_SENSOR_TIMED_POINT_CLOUD_DATA_H_
