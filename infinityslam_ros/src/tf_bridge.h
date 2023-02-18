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

#ifndef ROS_APP_TF_BRIDGE_H
#define ROS_APP_TF_BRIDGE_H

#include <memory>

#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam_ros/src/time_conversion.h"
#include "tf2_ros/buffer.h"

namespace infinityslam_ros {

class TfBridge {
 public:
  TfBridge(const std::string& tracking_frame,
           double lookup_transform_timeout_sec, 
           const tf2_ros::Buffer* buffer);
  ~TfBridge() {}

  TfBridge(const TfBridge&) = delete;
  TfBridge& operator=(const TfBridge&) = delete;

  // Returns the transform for 'frame_id' to 'tracking_frame_' if it exists at
  // 'time'.
  std::unique_ptr<::infinityslam::transform::Rigid3d> LookupToTracking(
      ::infinityslam::common::Time time, const std::string& frame_id) const;

 private:
  const std::string tracking_frame_;
  const double lookup_transform_timeout_sec_;
  const tf2_ros::Buffer* const buffer_;
};

}  // namespace infinityslam_ros

#endif  // ROS_APP_TF_BRIDGE_H
