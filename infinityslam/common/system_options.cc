/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Licensed under the Apache License; Version 2.0 (the "License").
*/

#include <numeric>
#include <limits>

#include "infinityslam/common/system_options.h"

namespace infinityslam {

namespace common {

namespace options {


// /// 模块参数 - ROS层
// namespace ros {
// std::string map_frame = "map";
// std::string tracking_frame = "base_link";
// std::string published_frame = "base_link";
// std::string odom_frame = "odom";
// bool use_pose_extrapolator = true;
// bool use_odometry = false;
// bool use_nav_sat = false;
// bool use_landmarks = false;
// bool provide_odom_frame = true;
// bool publish_frame_projected_to_2d = false;
// bool publish_tracked_pose = true;
// int num_laser_scans = 0;
// int num_multi_echo_laser_scans = 0;
// int num_subdivisions_per_laser_scan = 1;
// int num_point_clouds = 1;
// double lookup_transform_timeout_sec = 0.2;
// double submap_publish_period_sec = 500e-3;   // 100毫秒。
// double pose_publish_period_sec = 5e-3;       // 决定了 PublishLocalTrajectoryData 的发布频率。
// double trajectory_publish_period_sec = 30e-3;// 决定了 PublishTrajectoryNodeList & PublishLandmarkPosesList 的发布频率。
// double rangefinder_sampling_ratio = 1.;
// double odometry_sampling_ratio = 1.;
// double fixed_frame_pose_sampling_ratio = 1.;
// double imu_sampling_ratio = 1.;
// double landmarks_sampling_ratio = 1.;
// } // namespace ros


/// 模块参数 - CSMLIO
namespace csmlio {

bool collate_fixed_frame = true;
bool collate_landmarks = false;

double min_range = 0.4;
double max_range = 60;
int    num_accumulated_range_data = 1;
double voxel_filter_size = 0.15;       // wgh 前端‘基本’体素降采样

double high_res_adap_vf_max_length = 2.;
int    high_res_adap_vf_min_num_points = 1000;
double high_res_adap_vf_max_range = 20.;

double low_res_adap_vf_max_length = 4.;
int    low_res_adap_vf_min_num_points = 1000;
double low_res_adap_vf_max_range = 60.;


bool use_online_correlative_scan_matching = false; // wgh 使用高分辨率做暴力搜索
double rt_csm_linear_search_window = 0.15;
double rt_csm_angular_search_window = 0.035; // math.rad(1.)   0.0175
double rt_csm_translation_delta_cost_weight = 1e-1;
double rt_csm_rotation_delta_cost_weight = 1e-1;


double csm_occupied_space_weight_0 = 6.; // wgh 高分辨率栅格&点云匹配
double csm_occupied_space_weight_1 = 6.; // wgh 低分辨率栅格&点云匹配
double csm_translation_weight = 5.;
double csm_rotation_weight = 4e2;
bool   csm_only_optimize_yaw = true;    // wgh CSM配准仅优化yaw。
double csm_intensity_0_weight = 0.5;
double csm_intensity_0_huber_scale = 0.3;
double csm_intensity_0_threshold = 40;
bool csm_solver_use_nonmonotonic_steps = false;
int  csm_solver_max_num_iterations = 12;
int  csm_solver_num_threads = 1;

double motion_filter_max_time_seconds = 120.;
double motion_filter_max_distance_meters = 0.2;
double motion_filter_max_angle_radians = 0.0873;  // 约等于5度，0.01745弧度/度 

int rotational_histogram_size = 120;

double imu_gravity_time_constant = 10.;             // 位姿外推器 // maybe unused.
bool   use_imu_based_pose_extrapolator = false;     // 位姿外推器

double pose_extrap1_imu_gravity_time_constant = 10.; // 基于匀速模型和ImuTracker的位姿外推器
double pose_extrap1_pose_queue_duration = 0.001;     // 基于匀速模型和ImuTracker的位姿外推器

double pose_extrap2_pose_queue_duration = 5.;               // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_gravity_constant = 9.806;               // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_pose_translation_weight = 1.;           // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_pose_rotation_weight = 1.;              // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_imu_acceleration_weight = 1.;           // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_imu_rotation_weight = 1.;               // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_odometry_translation_weight = 1.;       // 基于IMU积分和bias矫正的位姿外推器
double pose_extrap2_odometry_rotation_weight = 1.;          // 基于IMU积分和bias矫正的位姿外推器
bool   pose_extrap2_solver_use_nonmonotonic_steps = false;  // 基于IMU积分和bias矫正的位姿外推器
int    pose_extrap2_solver_max_num_iterations = 10;         // 基于IMU积分和bias矫正的位姿外推器
int    pose_extrap2_solver_num_threads = 1;                 // 基于IMU积分和bias矫正的位姿外推器


double submap_high_resolution = 0.10;
double submap_high_resolution_max_range = 20.;
double submap_low_resolution = 0.30;
int    submap_num_range_data = 100;
double submap_insertion_hit_probability = 0.55;
double submap_insertion_miss_probability = 0.49;
int    submap_insertion_num_free_space_voxels = 2;
double submap_insertion_intensity_threshold = 10.;


// When setting use_intensites to true; the intensity_cost_function_options_0
// parameter in ceres_scan_matcher has to be set up as well or otherwise
// CeresScanMatcher will CHECK-fail.
bool use_intensities = false;


} // namespace csmlio


bool LoadSlamOptions(const std::string& slam_config_file) {
    /*TODO*/ 
    return true;
}




} // namespace options

} // namespace common

} // namespace infinityslam