/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include <infinityslam/io/yaml_io.h>
#include "infinityslam_ros/src/ros_wrapper_options.h"
#include "glog/logging.h"

namespace infinityslam_ros {

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

bool LoadCSMLioWraPperOptions(const std::string& yaml_file, bool logging) {
    infinityslam::io::YamlNode yaml_node_(yaml_file);
    return LoadCSMLioWraPperOptions(yaml_node_, logging);
}

bool LoadCSMLioWraPperOptions(infinityslam::io::YamlNode& yaml_node, bool logging) {
    if(!yaml_node.is_valid()) {
        LOG(ERROR) << "Invalid YamlNode, failed to load ROS options.";
        return false;
    }

    yaml_node.GetValue<std::string>("map_frame", infinityslam_ros::options::map_frame, logging);
    yaml_node.GetValue<std::string>("tracking_frame", infinityslam_ros::options::tracking_frame, logging);
    yaml_node.GetValue<std::string>("published_frame", infinityslam_ros::options::published_frame, logging);
    yaml_node.GetValue<std::string>("odom_frame", infinityslam_ros::options::odom_frame, logging);

    yaml_node.GetValue<bool>("use_pose_extrapolator", infinityslam_ros::options::use_pose_extrapolator);
    yaml_node.GetValue<bool>("use_odometry", infinityslam_ros::options::use_odometry);
    yaml_node.GetValue<bool>("use_nav_sat", infinityslam_ros::options::use_nav_sat);
    yaml_node.GetValue<bool>("use_landmarks", infinityslam_ros::options::use_landmarks);
    yaml_node.GetValue<bool>("provide_odom_frame", infinityslam_ros::options::provide_odom_frame);
    yaml_node.GetValue<bool>("publish_frame_projected_to_2d", infinityslam_ros::options::publish_frame_projected_to_2d);
    yaml_node.GetValue<bool>("publish_tracked_pose", infinityslam_ros::options::publish_tracked_pose);
    yaml_node.GetValue<bool>("publish_to_tf", infinityslam_ros::options::publish_to_tf);

    yaml_node.GetValue<int>("num_laser_scans", infinityslam_ros::options::num_laser_scans);
    yaml_node.GetValue<int>("num_multi_echo_laser_scans", infinityslam_ros::options::num_multi_echo_laser_scans);
    yaml_node.GetValue<int>("num_subdivisions_per_laser_scan", infinityslam_ros::options::num_subdivisions_per_laser_scan);
    yaml_node.GetValue<int>("num_point_clouds", infinityslam_ros::options::num_point_clouds);

    yaml_node.GetValue<double>("lookup_transform_timeout_sec", infinityslam_ros::options::lookup_transform_timeout_sec, logging);
    yaml_node.GetValue<double>("submap_publish_period_sec", infinityslam_ros::options::submap_publish_period_sec, logging);
    yaml_node.GetValue<double>("pose_publish_period_sec", infinityslam_ros::options::pose_publish_period_sec, logging);
    yaml_node.GetValue<double>("trajectory_publish_period_sec", infinityslam_ros::options::trajectory_publish_period_sec, logging);
    yaml_node.GetValue<double>("rangefinder_sampling_ratio", infinityslam_ros::options::rangefinder_sampling_ratio, logging);
    yaml_node.GetValue<double>("odometry_sampling_ratio", infinityslam_ros::options::odometry_sampling_ratio, logging);
    yaml_node.GetValue<double>("fixed_frame_pose_sampling_ratio", infinityslam_ros::options::fixed_frame_pose_sampling_ratio, logging);
    yaml_node.GetValue<double>("imu_sampling_ratio", infinityslam_ros::options::imu_sampling_ratio, logging);
    yaml_node.GetValue<double>("landmarks_sampling_ratio", infinityslam_ros::options::landmarks_sampling_ratio, logging);

    return true;


    // infinityslam_ros::options::map_frame = yaml_node.GetValue<std::string>("map_frame");
    // infinityslam_ros::options::tracking_frame = yaml_node.GetValue<std::string>("tracking_frame");
    // infinityslam_ros::options::published_frame = yaml_node.GetValue<std::string>("published_frame");
    // infinityslam_ros::options::odom_frame = yaml_node.GetValue<std::string>("odom_frame");

    // infinityslam_ros::options::use_pose_extrapolator = yaml_node.GetValue<bool>("use_pose_extrapolator");
    // infinityslam_ros::options::use_odometry = yaml_node.GetValue<bool>("use_odometry");
    // infinityslam_ros::options::use_nav_sat = yaml_node.GetValue<bool>("use_nav_sat");
    // infinityslam_ros::options::use_landmarks = yaml_node.GetValue<bool>("use_landmarks");
    // infinityslam_ros::options::provide_odom_frame = yaml_node.GetValue<bool>("provide_odom_frame");
    // infinityslam_ros::options::publish_frame_projected_to_2d = yaml_node.GetValue<bool>("publish_frame_projected_to_2d");
    // infinityslam_ros::options::publish_tracked_pose = yaml_node.GetValue<bool>("publish_tracked_pose");
    // infinityslam_ros::options::publish_to_tf = yaml_node.GetValue<bool>("publish_to_tf");

    // infinityslam_ros::options::num_laser_scans = yaml_node.GetValue<int>("num_laser_scans");
    // infinityslam_ros::options::num_multi_echo_laser_scans = yaml_node.GetValue<int>("num_multi_echo_laser_scans");
    // infinityslam_ros::options::num_subdivisions_per_laser_scan = yaml_node.GetValue<int>("num_subdivisions_per_laser_scan");
    // infinityslam_ros::options::num_point_clouds = yaml_node.GetValue<int>("num_point_clouds");

    // infinityslam_ros::options::lookup_transform_timeout_sec = yaml_node.GetValue<double>("lookup_transform_timeout_sec");
    // infinityslam_ros::options::submap_publish_period_sec = yaml_node.GetValue<double>("submap_publish_period_sec");
    // infinityslam_ros::options::pose_publish_period_sec = yaml_node.GetValue<double>("pose_publish_period_sec");
    // infinityslam_ros::options::trajectory_publish_period_sec = yaml_node.GetValue<double>("trajectory_publish_period_sec");
    // infinityslam_ros::options::rangefinder_sampling_ratio = yaml_node.GetValue<double>("rangefinder_sampling_ratio");
    // infinityslam_ros::options::odometry_sampling_ratio = yaml_node.GetValue<double>("odometry_sampling_ratio");
    // infinityslam_ros::options::fixed_frame_pose_sampling_ratio = yaml_node.GetValue<double>("fixed_frame_pose_sampling_ratio");
    // infinityslam_ros::options::imu_sampling_ratio = yaml_node.GetValue<double>("imu_sampling_ratio");
    // infinityslam_ros::options::landmarks_sampling_ratio = yaml_node.GetValue<double>("landmarks_sampling_ratio");

}

}  // namespace infinityslam_ros




