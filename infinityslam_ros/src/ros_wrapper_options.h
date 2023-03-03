/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include <string>

#include <infinityslam/io/yaml_io.h>

namespace infinityslam_ros {

/// Wrapper-level options of InfinitySLAM's ROS integration.
namespace options {
extern std::string map_frame;
extern std::string tracking_frame;
extern std::string published_frame;
extern std::string odom_frame;
extern bool use_pose_extrapolator;
extern bool use_odometry;
extern bool use_nav_sat;
extern bool use_landmarks;
extern bool provide_odom_frame;
extern bool publish_frame_projected_to_2d;
extern bool publish_tracked_pose;
extern bool publish_to_tf;
extern int num_laser_scans;
extern int num_multi_echo_laser_scans;
extern int num_subdivisions_per_laser_scan;
extern int num_point_clouds;
extern double lookup_transform_timeout_sec;
extern double submap_publish_period_sec;
extern double pose_publish_period_sec;
extern double trajectory_publish_period_sec;
extern double rangefinder_sampling_ratio;
extern double odometry_sampling_ratio;
extern double fixed_frame_pose_sampling_ratio;
extern double imu_sampling_ratio;
extern double landmarks_sampling_ratio;
} // namespace options

bool LoadCSMLioWraPperOptions(const std::string& yaml_file, bool logging = false);

bool LoadCSMLioWraPperOptions(infinityslam::io::YamlNode& yaml_node, bool logging = false);

}  // namespace infinityslam_ros

