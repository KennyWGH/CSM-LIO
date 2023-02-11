/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_COMMON_SLAM_OPTIONS_H_
#define INFINITYSLAM_COMMON_SLAM_OPTIONS_H_

#include <string>

namespace infinityslam {

namespace common {

namespace options {

/// 全局参数
extern double kParam1;
extern std::string kName1;


// /// 模块参数 - ROS层
// namespace ros {
// extern std::string map_frame;
// extern std::string tracking_frame;
// extern std::string published_frame;
// extern std::string odom_frame;
// extern bool use_pose_extrapolator;
// extern bool use_odometry;
// extern bool use_nav_sat;
// extern bool use_landmarks;
// extern bool provide_odom_frame;
// extern bool publish_frame_projected_to_2d;
// extern bool publish_tracked_pose;
// extern int num_laser_scans;
// extern int num_multi_echo_laser_scans;
// extern int num_subdivisions_per_laser_scan;
// extern int num_point_clouds;
// extern double lookup_transform_timeout_sec;
// extern double submap_publish_period_sec;    // 100毫秒。
// extern double pose_publish_period_sec;      // 决定了 PublishLocalTrajectoryData 的发布频率。
// extern double trajectory_publish_period_sec;// 决定了 PublishTrajectoryNodeList & PublishLandmarkPosesList 的发布频率。
// extern double rangefinder_sampling_ratio;
// extern double odometry_sampling_ratio;
// extern double fixed_frame_pose_sampling_ratio;
// extern double imu_sampling_ratio;
// extern double landmarks_sampling_ratio;
// } // namespace ros


/// 模块参数 - CSMLIO
namespace csmlio {

extern bool collate_fixed_frame;
extern bool collate_landmarks;

extern double min_range;
extern double max_range;
extern int    num_accumulated_range_data;
extern double voxel_filter_size;

extern double high_res_adap_vf_max_length;
extern int    high_res_adap_vf_min_num_points;
extern double high_res_adap_vf_max_range;

extern double low_res_adap_vf_max_length;
extern int    low_res_adap_vf_min_num_points;
extern double low_res_adap_vf_max_range;

extern bool use_online_correlative_scan_matching;
extern double rt_csm_linear_search_window;
extern double rt_csm_angular_search_window;
extern double rt_csm_translation_delta_cost_weight;
extern double rt_csm_rotation_delta_cost_weight;

double extern csm_occupied_space_weight_0; // wgh 高分辨率栅格&点云匹配
extern double csm_occupied_space_weight_1; // wgh 低分辨率栅格&点云匹配
extern double csm_translation_weight;
extern double csm_rotation_weight;
extern bool   csm_only_optimize_yaw;
extern double csm_intensity_0_weight;
extern double csm_intensity_0_huber_scale;
extern double csm_intensity_0_threshold;
extern bool csm_solver_use_nonmonotonic_steps;
extern int  csm_solver_max_num_iterations;
extern int  csm_solver_num_threads;

extern double motion_filter_max_time_seconds;
extern double motion_filter_max_distance_meters;
extern double motion_filter_max_angle_radians;

extern int rotational_histogram_size;

extern double imu_gravity_time_constant;
extern bool   use_imu_based_pose_extrapolator;

extern double pose_extrap1_imu_gravity_time_constant;
extern double pose_extrap1_pose_queue_duration;

extern double pose_extrap2_pose_queue_duration;         
extern double pose_extrap2_gravity_constant;         
extern double pose_extrap2_pose_translation_weight;    
extern double pose_extrap2_pose_rotation_weight;        
extern double pose_extrap2_imu_acceleration_weight;    
extern double pose_extrap2_imu_rotation_weight;
extern double pose_extrap2_odometry_translation_weight;
extern double pose_extrap2_odometry_rotation_weight; 
extern bool pose_extrap2_solver_use_nonmonotonic_steps;
extern int  pose_extrap2_solver_max_num_iterations;    
extern int  pose_extrap2_solver_num_threads;        

extern double submap_high_resolution;
extern double submap_high_resolution_max_range;
extern double submap_low_resolution;
extern int    submap_num_range_data;
extern double submap_insertion_hit_probability;
extern double submap_insertion_miss_probability;
extern int    submap_insertion_num_free_space_voxels;
extern double submap_insertion_intensity_threshold;

extern bool use_intensities;

} // namespace csmlio



/// 模块参数
namespace liosam {
extern double kMaxRange;
}

/// 模块参数
namespace optimization {
extern double kMaxRange;
}


bool LoadSlamOptions(const std::string& slam_config_file);


} // namespace options

} // namespace common

} // namespace infinityslam




#endif // INFINITYSLAM_COMMON_SLAM_OPTIONS_H_