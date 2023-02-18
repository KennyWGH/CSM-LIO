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

#ifndef ROS_APP_NODE_OPTIONS_H
#define ROS_APP_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "infinityslam/common/numeric_types.h"
#include "infinityslam_ros/src/trajectory_options.h"

namespace infinityslam_ros {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
  std::string map_frame;
  double lookup_transform_timeout_sec;
  double submap_publish_period_sec;
  double pose_publish_period_sec;
  double trajectory_publish_period_sec;
  bool publish_to_tf = true;
  bool publish_tracked_pose = false;
  bool use_pose_extrapolator = true;
};


}  // namespace infinityslam_ros

#endif  // ROS_APP_NODE_OPTIONS_H
