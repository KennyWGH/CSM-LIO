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

#ifndef ROS_APP_URDF_READER_H
#define ROS_APP_URDF_READER_H

#include <vector>

#include "infinityslam/common/numeric_types.h"
#include "tf2_ros/buffer.h"

namespace infinityslam_ros {

/// 下边这个函数从urdf文件中解析静态tf信息，这个功能仅在[离线处理bag时]会用到；
/// 在常规流程中，我们在launch文件中通过ROS自带的robot_state_publisher功能包
/// 来读取launch文件中提供的robot_description参数，进而把urdf文件的信息加载到
/// ROS的tf系统中，代码中只需要查询即可。
std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, tf2_ros::Buffer* tf_buffer);

}  // namespace infinityslam_ros

#endif  // ROS_APP_URDF_READER_H
