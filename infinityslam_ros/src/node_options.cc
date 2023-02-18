// /*
//  * Copyright 2016 The Cartographer Authors
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *      http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */

// #include "infinityslam_ros/src/node_options.h"

// #include <vector>
// #include <boost/make_unique.hpp>

// #include "glog/logging.h"

// namespace infinityslam_ros {

// std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
//     const std::string& configuration_directory,
//     const std::string& configuration_basename) {
//   auto file_resolver =
//       boost::make_unique<infinityslam::common::ConfigurationFileResolver>(
//           std::vector<std::string>{configuration_directory});
//   const std::string code =
//       file_resolver->GetFileContentOrDie(configuration_basename);
//   infinityslam::common::LuaParameterDictionary lua_parameter_dictionary(
//       code, std::move(file_resolver));

//   return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
//                          CreateTrajectoryOptions(&lua_parameter_dictionary));
// }

// }  // namespace infinityslam_ros
