/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include <infinityslam/io/yaml_io.h>
#include "ros_app/src/ros_wrapper_options.h"
#include "glog/logging.h"

namespace ros_app {

/// Wrapper-level options of InfinitySLAM's ROS integration.
namespace options {
std::string map_frame = "map";
std::string tracking_frame = "base_link";
std::string published_frame = "base_link";
std::string odom_frame = "odom";
bool use_pose_extrapolator = true;
bool use_odometry = false;
bool use_nav_sat = false;
bool use_landmarks = false;
bool provide_odom_frame = true;
bool publish_frame_projected_to_2d = false;
bool publish_tracked_pose = true;
bool publish_to_tf = true;
int num_laser_scans = 0;
int num_multi_echo_laser_scans = 0;
int num_subdivisions_per_laser_scan = 1;
int num_point_clouds = 1;
double lookup_transform_timeout_sec = 0.2;
double submap_publish_period_sec = 200e-3;     // 500毫秒。
double pose_publish_period_sec = 30e-3;        // 决定了 PublishLocalTrajectoryData 的发布频率。
double trajectory_publish_period_sec = 30e-3;  // 决定了 PublishTrajectoryNodeList & PublishLandmarkPosesList 的发布频率。
double rangefinder_sampling_ratio = 1.;
double odometry_sampling_ratio = 1.;
double fixed_frame_pose_sampling_ratio = 1.;
double imu_sampling_ratio = 1.;
double landmarks_sampling_ratio = 1.;
} // namespace options

bool LoadRosWrapperOptions(const std::string& yaml_file, bool logging) {
    infinityslam::io::YamlNode yaml_node_(yaml_file);
    return LoadRosWrapperOptions(yaml_node_, logging);
}

bool LoadRosWrapperOptions(infinityslam::io::YamlNode& yaml_node, bool logging) {
    if(!yaml_node.is_valid()) {
        LOG(ERROR) << "Invalid YamlNode, failed to load ROS options.";
        return false;
    }

    yaml_node.GetValue<std::string>("map_frame", ros_app::options::map_frame, logging);
    yaml_node.GetValue<std::string>("tracking_frame", ros_app::options::tracking_frame, logging);
    yaml_node.GetValue<std::string>("published_frame", ros_app::options::published_frame, logging);
    yaml_node.GetValue<std::string>("odom_frame", ros_app::options::odom_frame, logging);

    yaml_node.GetValue<bool>("use_pose_extrapolator", ros_app::options::use_pose_extrapolator);
    yaml_node.GetValue<bool>("use_odometry", ros_app::options::use_odometry);
    yaml_node.GetValue<bool>("use_nav_sat", ros_app::options::use_nav_sat);
    yaml_node.GetValue<bool>("use_landmarks", ros_app::options::use_landmarks);
    yaml_node.GetValue<bool>("provide_odom_frame", ros_app::options::provide_odom_frame);
    yaml_node.GetValue<bool>("publish_frame_projected_to_2d", ros_app::options::publish_frame_projected_to_2d);
    yaml_node.GetValue<bool>("publish_tracked_pose", ros_app::options::publish_tracked_pose);
    yaml_node.GetValue<bool>("publish_to_tf", ros_app::options::publish_to_tf);

    yaml_node.GetValue<int>("num_laser_scans", ros_app::options::num_laser_scans);
    yaml_node.GetValue<int>("num_multi_echo_laser_scans", ros_app::options::num_multi_echo_laser_scans);
    yaml_node.GetValue<int>("num_subdivisions_per_laser_scan", ros_app::options::num_subdivisions_per_laser_scan);
    yaml_node.GetValue<int>("num_point_clouds", ros_app::options::num_point_clouds);

    yaml_node.GetValue<double>("lookup_transform_timeout_sec", ros_app::options::lookup_transform_timeout_sec, logging);
    yaml_node.GetValue<double>("submap_publish_period_sec", ros_app::options::submap_publish_period_sec, logging);
    yaml_node.GetValue<double>("pose_publish_period_sec", ros_app::options::pose_publish_period_sec, logging);
    yaml_node.GetValue<double>("trajectory_publish_period_sec", ros_app::options::trajectory_publish_period_sec, logging);
    yaml_node.GetValue<double>("rangefinder_sampling_ratio", ros_app::options::rangefinder_sampling_ratio, logging);
    yaml_node.GetValue<double>("odometry_sampling_ratio", ros_app::options::odometry_sampling_ratio, logging);
    yaml_node.GetValue<double>("fixed_frame_pose_sampling_ratio", ros_app::options::fixed_frame_pose_sampling_ratio, logging);
    yaml_node.GetValue<double>("imu_sampling_ratio", ros_app::options::imu_sampling_ratio, logging);
    yaml_node.GetValue<double>("landmarks_sampling_ratio", ros_app::options::landmarks_sampling_ratio, logging);

    return true;


    // ros_app::options::map_frame = yaml_node.GetValue<std::string>("map_frame");
    // ros_app::options::tracking_frame = yaml_node.GetValue<std::string>("tracking_frame");
    // ros_app::options::published_frame = yaml_node.GetValue<std::string>("published_frame");
    // ros_app::options::odom_frame = yaml_node.GetValue<std::string>("odom_frame");

    // ros_app::options::use_pose_extrapolator = yaml_node.GetValue<bool>("use_pose_extrapolator");
    // ros_app::options::use_odometry = yaml_node.GetValue<bool>("use_odometry");
    // ros_app::options::use_nav_sat = yaml_node.GetValue<bool>("use_nav_sat");
    // ros_app::options::use_landmarks = yaml_node.GetValue<bool>("use_landmarks");
    // ros_app::options::provide_odom_frame = yaml_node.GetValue<bool>("provide_odom_frame");
    // ros_app::options::publish_frame_projected_to_2d = yaml_node.GetValue<bool>("publish_frame_projected_to_2d");
    // ros_app::options::publish_tracked_pose = yaml_node.GetValue<bool>("publish_tracked_pose");
    // ros_app::options::publish_to_tf = yaml_node.GetValue<bool>("publish_to_tf");

    // ros_app::options::num_laser_scans = yaml_node.GetValue<int>("num_laser_scans");
    // ros_app::options::num_multi_echo_laser_scans = yaml_node.GetValue<int>("num_multi_echo_laser_scans");
    // ros_app::options::num_subdivisions_per_laser_scan = yaml_node.GetValue<int>("num_subdivisions_per_laser_scan");
    // ros_app::options::num_point_clouds = yaml_node.GetValue<int>("num_point_clouds");

    // ros_app::options::lookup_transform_timeout_sec = yaml_node.GetValue<double>("lookup_transform_timeout_sec");
    // ros_app::options::submap_publish_period_sec = yaml_node.GetValue<double>("submap_publish_period_sec");
    // ros_app::options::pose_publish_period_sec = yaml_node.GetValue<double>("pose_publish_period_sec");
    // ros_app::options::trajectory_publish_period_sec = yaml_node.GetValue<double>("trajectory_publish_period_sec");
    // ros_app::options::rangefinder_sampling_ratio = yaml_node.GetValue<double>("rangefinder_sampling_ratio");
    // ros_app::options::odometry_sampling_ratio = yaml_node.GetValue<double>("odometry_sampling_ratio");
    // ros_app::options::fixed_frame_pose_sampling_ratio = yaml_node.GetValue<double>("fixed_frame_pose_sampling_ratio");
    // ros_app::options::imu_sampling_ratio = yaml_node.GetValue<double>("imu_sampling_ratio");
    // ros_app::options::landmarks_sampling_ratio = yaml_node.GetValue<double>("landmarks_sampling_ratio");

}

}  // namespace ros_app




