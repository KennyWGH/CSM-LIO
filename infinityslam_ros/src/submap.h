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

#ifndef ROS_APP_SUBMAP_H
#define ROS_APP_SUBMAP_H

#include <memory>
#include <string>
#include <vector>

#include "infinityslam/io/image.h"
#include "infinityslam/io/submap_painter.h"
#include "infinityslam/csmlio/id.h"
#include "infinityslam/transform/rigid_transform.h"
#include "ros/ros.h"

namespace infinityslam_ros {

// Fetch 'submap_id' using the 'client' and returning the response or 'nullptr'
// on error.
std::unique_ptr<::infinityslam::io::SubmapTextures> FetchSubmapTextures(
    const ::infinityslam::csmlio::SubmapId& submap_id,
    ros::ServiceClient* client);

}  // namespace infinityslam_ros

#endif  // ROS_APP_SUBMAP_H
