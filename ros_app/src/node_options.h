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

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/common/port.h"
#include "csmlio/lio/proto/map_builder_options.pb.h"
#include "ros_app/src/trajectory_options.h"

namespace ros_app {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
  ::csmlio::mapping::proto::MapBuilderOptions map_builder_options;
  std::string map_frame;
  double lookup_transform_timeout_sec;
  double submap_publish_period_sec;
  double pose_publish_period_sec;
  double trajectory_publish_period_sec;
  bool publish_to_tf = true;
  bool publish_tracked_pose = false;
  bool use_pose_extrapolator = true;
};

NodeOptions CreateNodeOptions(
    ::csmlio::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);
}  // namespace ros_app

#endif  // ROS_APP_NODE_OPTIONS_H
