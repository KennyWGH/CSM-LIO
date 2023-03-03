/**
 * Copyright 2016 The Cartographer Authors
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef ROS_APP_MSG_CONVERSION_H
#define ROS_APP_MSG_CONVERSION_H

#include "infinityslam/common/time.h"
#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/sensor/point_cloud_type.h"
#include "infinityslam/transform/rigid_transform.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"

namespace infinityslam_ros {

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::infinityslam::sensor::TimedPointCloud& point_cloud);

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::infinityslam::sensor::PointCloudWithIntensities& point_cloud);

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const ::infinityslam::sensor::PointCloudXYZIT& point_cloud);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<::infinityslam::sensor::PointCloudWithIntensities,
           ::infinityslam::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& msg);

::infinityslam::sensor::PointCloudXYZIT
ToPointCloudXYZIT(const sensor_msgs::PointCloud2& msg);

// 得到ROS/PC2消息覆盖时段的起始时间（按帧时间戳）。
double GetPC2StartTime(
    const sensor_msgs::PointCloud2& msg);

// 得到ROS/PC2消息覆盖时段的结束时间（不一定是最后一个点）。
double GetPC2EndTime(
    const sensor_msgs::PointCloud2& msg);

::infinityslam::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform);

::infinityslam::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

geometry_msgs::Transform ToGeometryMsgTransform(
    const ::infinityslam::transform::Rigid3d& rigid3d);

geometry_msgs::Pose ToGeometryMsgPose(
    const ::infinityslam::transform::Rigid3d& rigid3d);

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);


}  // namespace infinityslam_ros

#endif  // ROS_APP_MSG_CONVERSION_H
