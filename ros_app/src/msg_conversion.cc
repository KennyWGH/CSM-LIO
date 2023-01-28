/**
 * Copyright 2016 The Cartographer Authors
 * Copyright 2023 WANG Guanhua (wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include <cmath>
#include <algorithm>
#include <unordered_map>
#include "glog/logging.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl_conversions/pcl_conversions.h"

#include "csmlio/common/math.h"
#include "csmlio/common/port.h"
#include "csmlio/common/time.h"
#include "csmlio/io/submap_painter.h"
#include "csmlio/transform/transform.h"
#include "ros_app/src/time_conversion.h"
#include "ros_app/src/msg_conversion.h"

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT {
    float x;
    float y;
    float z;
    float time;
};

struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    float unused_padding[2];
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))

namespace ros_app {
namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::csmlio::sensor::PointCloudWithIntensities;
using ::csmlio::transform::Rigid3d;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) 
{
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ToRos(::csmlio::common::FromUniversal(timestamp));
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = num_points;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = 16 * msg.width;
    msg.is_dense = true;
    msg.data.resize(16 * num_points);
    return msg;
}

sensor_msgs::PointCloud2 PreparePointCloud2MessageWithIntensity(
                                                const int64_t timestamp,
                                                const std::string& frame_id,
                                                const int num_points) 
{
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ToRos(::csmlio::common::FromUniversal(timestamp));
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = num_points;
    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[3].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = 16 * msg.width;
    msg.is_dense = true;
    msg.data.resize(16 * num_points);
    return msg;
}

bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) 
{
    for (const auto& field : pc2.fields) {
        if (field.name == field_name) {
        return true;
        }
    }
    return false;
}

using TimedRangefinderPoint = ::csmlio::sensor::TimedRangefinderPoint;
bool CompByStamp(const TimedRangefinderPoint& point1, 
    const TimedRangefinderPoint& point2)
{
    return point1.time < point2.time;
}

// 用于警告PointCloud2消息没有‘time’字段，仅警告一次。
std::unordered_map<std::string, bool> warned_table;

}  // namespace

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::csmlio::sensor::TimedPointCloud& point_cloud) 
{
    auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
    for (const csmlio::sensor::TimedRangefinderPoint& point : point_cloud) {
        stream.next(point.position.x());
        stream.next(point.position.y());
        stream.next(point.position.z());
        stream.next(kPointCloudComponentFourMagic);
    }
    return msg;
}

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::csmlio::sensor::PointCloudWithIntensities& point_cloud) 
{
    auto msg = PreparePointCloud2MessageWithIntensity(timestamp, 
                                                        frame_id, 
                                                        point_cloud.points.size());
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
    for (std::size_t i=0; i<point_cloud.points.size(); ++i) {
        stream.next(point_cloud.points[i].position.x());
        stream.next(point_cloud.points[i].position.y());
        stream.next(point_cloud.points[i].position.z());
        stream.next(point_cloud.intensities[i]);
    }
    return msg;
}

std::tuple<::csmlio::sensor::PointCloudWithIntensities,
           ::csmlio::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& msg) 
{
    PointCloudWithIntensities point_cloud;
    // We check for intensity field here to avoid run-time warnings if we pass in
    // a PointCloud2 without intensity.
    if (PointCloud2HasField(msg, "intensity")) {
        if (PointCloud2HasField(msg, "time")) {
        pcl::PointCloud<PointXYZIT> pcl_point_cloud;
        pcl::fromROSMsg(msg, pcl_point_cloud);
        point_cloud.points.reserve(pcl_point_cloud.size());
        point_cloud.intensities.reserve(pcl_point_cloud.size());
        for (const auto& point : pcl_point_cloud) {
            point_cloud.points.push_back(
                {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
            point_cloud.intensities.push_back(point.intensity);
        }
        } else {
        pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
        pcl::fromROSMsg(msg, pcl_point_cloud);
        point_cloud.points.reserve(pcl_point_cloud.size());
        point_cloud.intensities.reserve(pcl_point_cloud.size());
        for (const auto& point : pcl_point_cloud) {
            point_cloud.points.push_back(
                {Eigen::Vector3f{point.x, point.y, point.z}, 0.f}); //时间戳置零
            point_cloud.intensities.push_back(point.intensity);
        }
        }
    } else {
        // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
        if (PointCloud2HasField(msg, "time")) {
        pcl::PointCloud<PointXYZT> pcl_point_cloud;
        pcl::fromROSMsg(msg, pcl_point_cloud);
        point_cloud.points.reserve(pcl_point_cloud.size());
        point_cloud.intensities.reserve(pcl_point_cloud.size());
        for (const auto& point : pcl_point_cloud) {
            point_cloud.points.push_back(
                {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
            point_cloud.intensities.push_back(1.0f);
        }
        } else {
        pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
        pcl::fromROSMsg(msg, pcl_point_cloud);
        point_cloud.points.reserve(pcl_point_cloud.size());
        point_cloud.intensities.reserve(pcl_point_cloud.size());
        for (const auto& point : pcl_point_cloud) {
            point_cloud.points.push_back(
                {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
            point_cloud.intensities.push_back(1.0f);
        }
        }
    }
    // 把帧的时间戳移动到最后一个点（而非ROS格式中的第一个点）
    ::csmlio::common::Time timestamp = FromRos(msg.header.stamp);
    if (!point_cloud.points.empty()) {
        // 一个更正确的、寻找点云中时间戳最大的点的方法；
        // 该方法支持“点云中的点并没有按时间先后顺序排序”的情形。
        const auto& all_points = point_cloud.points;
        const double duration = 
            (*std::max_element(all_points.begin(), all_points.end(), CompByStamp))
                .time;
        // const double duration = point_cloud.points.back().time;
        if (duration > 0.105) {
            LOG(INFO) << "Found the largest point stamp inside point cloud "
                "(frame_id: " << msg.header.frame_id << ") larger than 100ms, "
                "which is " << int(duration * 1000) << "ms.";
        }
        timestamp += csmlio::common::FromSeconds(duration);
        for (auto& point : point_cloud.points) {
            point.time -= duration;
            // CHECK_LE(point.time, 0.f)
            //     << "Encountered a point with a larger stamp than "
            //         "the last point in the cloud.";
            if (point.time > 0.f) {
                LOG(WARNING) << "Encountered a point with a larger stamp"
                    " than the last point in the cloud, will copy the last"
                    " point to overwrite it.";
                point = point_cloud.points.back();
                point.time -= duration;
            }
        }
    }
    return std::make_tuple(point_cloud, timestamp);
}

::csmlio::common::Time GetPC2StartTime(
    const sensor_msgs::PointCloud2& msg)
{
    return FromRos(msg.header.stamp);
}

::csmlio::common::Time GetPC2EndTime(
    const sensor_msgs::PointCloud2& msg) 
{
    if (PointCloud2HasField(msg, "time")) {
        pcl::PointCloud<PointXYZT> pcl_point_cloud;
        pcl::fromROSMsg(msg, pcl_point_cloud);
        float largest_rel_stamp = 0;
        for (auto& point : pcl_point_cloud.points) {
            if (point.time > largest_rel_stamp)
                largest_rel_stamp = point.time;
        }

        // { // debug.
        //     ::csmlio::common::Time p_time = 
        //         FromRos(msg.header.stamp) + 
        //             ::csmlio::common::FromSeconds(
        //                 pcl_point_cloud.points.back().time);
        //     LOG(INFO) << "Convert time format with " << ::csmlio::common::ToSeconds(p_time) 
        //         << " - " << ::csmlio::common::ToSeconds(FromRos(msg.header.stamp)) 
        //         << " = " << ::csmlio::common::ToSeconds(p_time - FromRos(msg.header.stamp))
        //         << ", compared with " << pcl_point_cloud.points.back().time << ".";
        // }

        return FromRos(msg.header.stamp) + 
            ::csmlio::common::FromSeconds(largest_rel_stamp);
    } 
    if (warned_table.find(msg.header.frame_id) == warned_table.end()) {
        LOG(WARNING) << "Found no 'time' field for point cloud message with frame_id '" 
            << msg.header.frame_id << "'.";
        warned_table[msg.header.frame_id] = true;
    }
    return FromRos(msg.header.stamp);
}

Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) 
{
    return Rigid3d(ToEigen(transform.transform.translation),
                    ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) 
{
    return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                    ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) 
{
    return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) 
{
    return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) 
{
    geometry_msgs::Transform transform;
    transform.translation.x = rigid3d.translation().x();
    transform.translation.y = rigid3d.translation().y();
    transform.translation.z = rigid3d.translation().z();
    transform.rotation.w = rigid3d.rotation().w();
    transform.rotation.x = rigid3d.rotation().x();
    transform.rotation.y = rigid3d.rotation().y();
    transform.rotation.z = rigid3d.rotation().z();
    return transform;
}

geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) 
{
    geometry_msgs::Pose pose;
    pose.position = ToGeometryMsgPoint(rigid3d.translation());
    pose.orientation.w = rigid3d.rotation().w();
    pose.orientation.x = rigid3d.rotation().x();
    pose.orientation.y = rigid3d.rotation().y();
    pose.orientation.z = rigid3d.rotation().z();
    return pose;
}

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) 
{
    geometry_msgs::Point point;
    point.x = vector3d.x();
    point.y = vector3d.y();
    point.z = vector3d.z();
    return point;
}

std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const csmlio::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const ros::Time& time) 
{
    auto occupancy_grid = absl::make_unique<nav_msgs::OccupancyGrid>();

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height =
        cairo_image_surface_get_height(painted_slices.surface.get());

    occupancy_grid->header.stamp = time;
    occupancy_grid->header.frame_id = frame_id;
    occupancy_grid->info.map_load_time = time;
    occupancy_grid->info.resolution = resolution;
    occupancy_grid->info.width = width;
    occupancy_grid->info.height = height;
    occupancy_grid->info.origin.position.x =
        -painted_slices.origin.x() * resolution;
    occupancy_grid->info.origin.position.y =
        (-height + painted_slices.origin.y()) * resolution;
    occupancy_grid->info.origin.position.z = 0.;
    occupancy_grid->info.origin.orientation.w = 1.;
    occupancy_grid->info.origin.orientation.x = 0.;
    occupancy_grid->info.origin.orientation.y = 0.;
    occupancy_grid->info.origin.orientation.z = 0.;

    const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
        cairo_image_surface_get_data(painted_slices.surface.get()));
    occupancy_grid->data.reserve(width * height);
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
        const uint32_t packed = pixel_data[y * width + x];
        const unsigned char color = packed >> 16;
        const unsigned char observed = packed >> 8;
        const int value =
            observed == 0
                ? -1
                : ::csmlio::common::RoundToInt((1. - color / 255.) * 100.);
        CHECK_LE(-1, value);
        CHECK_GE(100, value);
        occupancy_grid->data.push_back(value);
        }
    }

    return occupancy_grid;
}

}  // namespace ros_app
