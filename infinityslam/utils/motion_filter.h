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

#ifndef INFINITYSLAM_UTILS_INTERNAL_MOTION_FILTER_H_
#define INFINITYSLAM_UTILS_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "infinityslam/common/time.h"
#include "infinityslam/common/system_options.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {
namespace utils {

struct MotionFIlterOptions {
  double max_time_seconds = 120.;
  double max_distance_meters = 0.2;
  double max_angle_radians = 0.0873;  // 约等于5度，0.01745弧度/度 

  MotionFIlterOptions() {}
};

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter(const MotionFIlterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

 private:
  const MotionFIlterOptions full_Options_;
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};

}  // namespace utils
}  // namespace infinityslam

#endif  // INFINITYSLAM_UTILS_INTERNAL_MOTION_FILTER_H_
