/**
 * Copyright 2016 The Cartographer Authors
 * Copyright 2023 WANG Guanhua (wangguanhua999@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef ROS_APP_MSG_CONVERSION_H
#define ROS_APP_MSG_CONVERSION_H

#include "csmlio/common/time.h"
#include "csmlio/io/submap_painter.h"
#include "csmlio/sensor/point_cloud.h"
#include "csmlio/transform/rigid_transform.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"

namespace ros_app {

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::csmlio::sensor::TimedPointCloud& point_cloud);

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::csmlio::sensor::PointCloudWithIntensities& point_cloud);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<::csmlio::sensor::PointCloudWithIntensities,
           ::csmlio::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& msg);

// 得到ROS/PC2消息覆盖时段的起始时间（按帧时间戳）。
::csmlio::common::Time GetPC2StartTime(
    const sensor_msgs::PointCloud2& msg);

// 得到ROS/PC2消息覆盖时段的结束时间（不一定是最后一个点）。
::csmlio::common::Time GetPC2EndTime(
    const sensor_msgs::PointCloud2& msg);

::csmlio::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform);

::csmlio::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

geometry_msgs::Transform ToGeometryMsgTransform(
    const ::csmlio::transform::Rigid3d& rigid3d);

geometry_msgs::Pose ToGeometryMsgPose(
    const ::csmlio::transform::Rigid3d& rigid3d);

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

// Points to an occupancy grid message at a specific resolution from painted
// submap slices obtained via ::csmlio::io::PaintSubmapSlices(...).
std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const csmlio::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const ros::Time& time);

}  // namespace ros_app

#endif  // ROS_APP_MSG_CONVERSION_H
