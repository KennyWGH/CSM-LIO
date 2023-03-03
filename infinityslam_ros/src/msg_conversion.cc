/**
 * Copyright 2016 The Cartographer Authors
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
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

#include "infinityslam/common/math.h"
#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/transform/transform.h"
#include "infinityslam_ros/src/time_conversion.h"
#include "infinityslam_ros/src/msg_conversion.h"

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

namespace infinityslam_ros {
namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::infinityslam::sensor::PointCloudWithIntensities;
using ::infinityslam::transform::Rigid3d;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) 
{
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ToRos(::infinityslam::common::FromUniversal(timestamp));
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
    msg.header.stamp = ToRos(::infinityslam::common::FromUniversal(timestamp));
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

using PointTypeXYZT = ::infinityslam::sensor::PointTypeXYZT;
bool CompByStamp(const PointTypeXYZT& point1, 
    const PointTypeXYZT& point2)
{
    return point1.time < point2.time;
}

// 用于警告PointCloud2消息没有‘time’字段，仅警告一次。
std::unordered_map<std::string, bool> warned_table;

}  // namespace

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::infinityslam::sensor::TimedPointCloud& point_cloud) 
{
    auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
    for (const infinityslam::sensor::PointTypeXYZT& point : point_cloud) {
        stream.next(point.position.x());
        stream.next(point.position.y());
        stream.next(point.position.z());
        stream.next(kPointCloudComponentFourMagic);
    }
    return msg;
}

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::infinityslam::sensor::PointCloudWithIntensities& point_cloud) 
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

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const ::infinityslam::sensor::PointCloudXYZIT& point_cloud) 
{
    auto msg = PreparePointCloud2MessageWithIntensity(
        ::infinityslam::common::ToUniversal(point_cloud.time_), 
        point_cloud.frame_id_, 
        point_cloud.points_.size());
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
    for (std::size_t i=0; i<point_cloud.points_.size(); ++i) {
        stream.next(point_cloud.points_[i].position.x());
        stream.next(point_cloud.points_[i].position.y());
        stream.next(point_cloud.points_[i].position.z());
        stream.next(point_cloud.points_[i].intensity);
    }
    return msg;
}

std::tuple<::infinityslam::sensor::PointCloudWithIntensities,
           ::infinityslam::common::Time>
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
    ::infinityslam::common::Time timestamp = FromRos(msg.header.stamp);
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
        timestamp += infinityslam::common::FromSeconds(duration);
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

::infinityslam::sensor::PointCloudXYZIT
ToPointCloudXYZIT(const sensor_msgs::PointCloud2& msg) {
    ::infinityslam::sensor::PointCloudXYZIT point_cloud;
    if (PointCloud2HasField(msg, "intensity")) {
        if (PointCloud2HasField(msg, "time")) {
            pcl::PointCloud<PointXYZIT> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points_.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points_.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, point.intensity, point.time});
            }
        } else {
            pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points_.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points_.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, point.intensity, 0.f});
            }
        }
    } else {
        if (PointCloud2HasField(msg, "time")) {
            pcl::PointCloud<PointXYZT> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points_.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points_.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 1.f, point.time});
            }
        } else {
            pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points_.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud) {
                point_cloud.points_.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 1.f, 0.f});
            }
        }
    }

    // 把帧的时间戳移动到最后一个点（而非ROS格式中的第一个点）
    ::infinityslam::common::Time timestamp = FromRos(msg.header.stamp);
    if (!point_cloud.points_.empty()) {
        const auto& all_points = point_cloud.points_;
        const double duration = all_points.back().time;
        if (duration > 0.105) {
            LOG(INFO) << "Found the largest point stamp inside point cloud "
                "(frame_id: " << msg.header.frame_id << ") larger than 100ms, "
                "which is " << int(duration * 1000) << "ms.";
        }
        timestamp += infinityslam::common::FromSeconds(duration);
        for (auto& point : point_cloud.points_) {
            point.time -= duration;
            if (point.time > 0.f) {
                LOG(WARNING) << "Encountered a point with a larger stamp"
                    " than the last point in the cloud, will copy the last"
                    " point to overwrite it.";
                point = point_cloud.points_.back();
                // point.time -= duration;
                point.time = 0;
            }
        }
    }
    point_cloud.time_ = timestamp;
    point_cloud.timestamp_ = ::infinityslam::common::ToSeconds(timestamp);
    point_cloud.seq_ = msg.header.seq;
    point_cloud.frame_id_ = msg.header.frame_id;

    return point_cloud;
}

double GetPC2StartTime(
    const sensor_msgs::PointCloud2& msg)
{
    return RosToUniversal(msg.header.stamp);
}

double GetPC2EndTime(
    const sensor_msgs::PointCloud2& msg) 
{
    if (PointCloud2HasField(msg, "time")) {
        pcl::PointCloud<PointXYZT> pcl_point_cloud;
        pcl::fromROSMsg(msg, pcl_point_cloud);
        float max_time = 0;
        for (auto& point : pcl_point_cloud.points) {
            if (point.time > max_time)
                max_time = point.time;
        }

        // { // debug.
        //     ::infinityslam::common::Time p_time = 
        //         FromRos(msg.header.stamp) + 
        //             ::infinityslam::common::FromSeconds(
        //                 pcl_point_cloud.points.back().time);
        //     LOG(INFO) << "Convert time format with " << ::infinityslam::common::ToSeconds(p_time) 
        //         << " - " << ::infinityslam::common::ToSeconds(FromRos(msg.header.stamp)) 
        //         << " = " << ::infinityslam::common::ToSeconds(p_time - FromRos(msg.header.stamp))
        //         << ", compared with " << pcl_point_cloud.points.back().time << ".";
        // }

        return RosToUniversal(msg.header.stamp) + max_time;
    } 

    if (warned_table.find(msg.header.frame_id) == warned_table.end()) {
        LOG(WARNING) << "Found no 'time' field for point cloud message with frame_id '" 
            << msg.header.frame_id << "'.";
        warned_table[msg.header.frame_id] = true;
    }
    return RosToUniversal(msg.header.stamp);
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


}  // namespace infinityslam_ros
