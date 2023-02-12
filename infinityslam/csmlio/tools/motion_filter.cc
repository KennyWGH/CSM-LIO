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

#include "infinityslam/csmlio/tools/motion_filter.h"

#include "infinityslam/common/system_options.h"
#include "infinityslam/transform/transform.h"
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {

MotionFIlterOpTions ReadMotionFIlterOpTions() {
    MotionFIlterOpTions options;
    options.motion_filter_max_time_seconds = common::options::csmlio::motion_filter_max_time_seconds;
    options.motion_filter_max_distance_meters = common::options::csmlio::motion_filter_max_distance_meters;
    options.motion_filter_max_angle_radians = common::options::csmlio::motion_filter_max_angle_radians;
    return options;
}

MotionFilter::MotionFilter(const MotionFIlterOpTions& options)
    : full_Options_(options) {}

bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 &&
      time - last_time_ <= common::FromSeconds(full_Options_.motion_filter_max_time_seconds) &&
      (pose.translation() - last_pose_.translation()).norm() <=
          full_Options_.motion_filter_max_distance_meters &&
      transform::GetAngle(pose.inverse() * last_pose_) <=
          full_Options_.motion_filter_max_angle_radians) {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace csmlio
}  // namespace infinityslam
