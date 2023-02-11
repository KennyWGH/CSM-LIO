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

#include "ros_app/src/submap.h"

#include "absl/memory/memory.h"
#include <boost/make_unique.hpp>
#include "infinityslam/common/port.h"
#include "infinityslam/transform/transform.h"
#include "ros_app/src/msg_conversion.h"
#include "ros_app/msgs/StatusCode.h"
#include "ros_app/msgs/SubmapQuery.h"

namespace ros_app {

std::unique_ptr<::infinityslam::io::SubmapTextures> FetchSubmapTextures(
    const ::infinityslam::mapping::SubmapId& submap_id,
    ros::ServiceClient* client) 
{
  // wgh: 创建请求消息，得到的信息实体就存放在 <srv.response.textures> 中。
  ::cartographer_ros_msgs::SubmapQuery srv;
  srv.request.trajectory_id = submap_id.trajectory_id;
  srv.request.submap_index = submap_id.submap_index;
  if (!client->call(srv) ||
      srv.response.status.code != ::cartographer_ros_msgs::StatusCode::OK) {
    return nullptr;
  }
  if (srv.response.textures.empty()) {
    return nullptr;
  }
  auto response = boost::make_unique<::infinityslam::io::SubmapTextures>();
  response->version = srv.response.submap_version;
  // wgh: 逐一读取信息实体（该submap_id下，不同分辨率的submap）。
  for (const auto& texture : srv.response.textures) {
    // wgh: gridmap信息转存为string格式。
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    // wgh: 创建可读信息实体的地方。
    response->textures.emplace_back(::infinityslam::io::SubmapTexture{
        ::infinityslam::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  return response;
}

}  // namespace ros_app
