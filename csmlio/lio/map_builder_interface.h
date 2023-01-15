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

#ifndef CSMLIO_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CSMLIO_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/common/port.h"
// #include "csmlio/io/proto_stream_ interface.h"
#include "csmlio/lio/id.h"
// #include "csmlio/lio/pose_graph_ interface.h"
#include "csmlio/lio/proto/map_builder_options.pb.h"
#include "csmlio/lio/proto/submap_visualization.pb.h"
#include "csmlio/lio/proto/trajectory_builder_options.pb.h"
#include "csmlio/lio/submaps.h"
// #include "csmlio/lio/trajectory_builder_ interface.h"

namespace csmlio {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_MAP_BUILDER_INTERFACE_H_
