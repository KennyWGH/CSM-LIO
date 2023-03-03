/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef ROS_APP_NODE_H
#define ROS_APP_NODE_H

#include <map>
#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstddef>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"


#include "infinityslam/common/time.h"
#include "infinityslam/common/optional.h"
#include "infinityslam/common/fixed_ratio_sampler.h"
#include "infinityslam/sensor/point_cloud_type.h"
#include "infinityslam/transform/timed_pose.h"
#include "infinityslam/utils/imu_aided_pose_interpolator.h"
#include "infinityslam/utils/motion_compensator.h"
#include "infinityslam/csmlio/csm_lio_type_def.h"
#include "infinityslam/csmlio/csm_lidar_inertial_odometry.h"

#include "infinityslam_ros/src/node_constants.h"
#include "infinityslam_ros/src/urdf_reader.h"
#include "infinityslam_ros/src/tf_bridge.h"

namespace infinityslam_ros {

struct CSMLioWraPperOptions {
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
    double submap_publish_period_sec = 500e-3;
    double pose_publish_period_sec = 5e-3;
    double trajectory_publish_period_sec = 30e-3;
    double rangefinder_sampling_ratio = 1.;
    double odometry_sampling_ratio = 1.;
    double fixed_frame_pose_sampling_ratio = 1.;
    double imu_sampling_ratio = 1.;
    double landmarks_sampling_ratio = 1.;
};

CSMLioWraPperOptions ReadCSMLioWraPperOptions();

// Wires up ROS topics to SLAM. CSMLioWrapper
class CSMLioWrapper {
  public:
    using Rigid3d           = ::infinityslam::transform::Rigid3d;
    using TimedPose         = ::infinityslam::transform::TimedPose;
    using ImuData           = ::infinityslam::sensor::ImuData;
    using PointCloudXYZIT   = ::infinityslam::sensor::PointCloudXYZIT;
    
    struct SlamKeyframeData {
        struct LioKeyframeData {
            ::infinityslam::common::Time time;
            ::infinityslam::transform::Rigid3d local_pose;
            ::infinityslam::sensor::RangeData range_data_in_local;
            std::shared_ptr<::infinityslam::sensor::PointCloud> point_cloud;
        };

        std::shared_ptr<const LioKeyframeData> local_slam_data;
        std::unique_ptr<infinityslam::transform::Rigid3d> published_to_tracking;
        // ::infinityslam::transform::Rigid3d global_pose;
        // TrajectoryOptions trajectory_options;
    };

    CSMLioWrapper(const CSMLioWraPperOptions& ros_options,
        tf2_ros::Buffer* tf_buffer);

    // CSMLioWrapper(const std::string& ros_config_file,
    //     const std::string& slam_config_file,
    //     tf2_ros::Buffer* tf_buffer) {/*TODO*/}

    ~CSMLioWrapper();

    CSMLioWrapper(const CSMLioWrapper&) = delete;
    CSMLioWrapper& operator=(const CSMLioWrapper&) = delete;

    bool Start();

    void HandleImuMessage(const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg);
    void HandleOdometryMessage(const std::string& sensor_id,
                                const nav_msgs::Odometry::ConstPtr& msg);
    void HandlePointCloud2Message(const std::string& sensor_id,
                                    const sensor_msgs::PointCloud2::ConstPtr& msg);
    void HandleNavSatFixMessage(const std::string& sensor_id,
                                const sensor_msgs::NavSatFix::ConstPtr& msg);

    ::ros::NodeHandle* node_handle();

    // 提供的ROS服务。对服务的响应在ROS中仍表现为回调队列中的任务，和topic回调相同，理论上
    // 也是在主线程中执行的。
    // bool HandleOptimizeExtrinsics(
    //     cartographer_ros_msgs::FinishTrajectory::Request& request,
    //     cartographer_ros_msgs::FinishTrajectory::Response& response);
    // bool HandleWriteSlamData(cartographer_ros_msgs::WriteState::Request& request,
    //                         cartographer_ros_msgs::WriteState::Response& response);

  private:
    struct Subscriber {
        ::ros::Subscriber subscriber;
        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string that was given in its constructor. Since we rely on the 
        // topic name as the unique identifier of a subscriber, we remember it 
        // ourselves.
        std::string topic;
    };

    // Returns the set of SensorIds expected for a trajectory.
    // 'SensorId::id' is the expected ROS topic name.
    std::set<::infinityslam::csmlio::SensorId>
    ComputeExpectedSensorIds(const CSMLioWraPperOptions& options) const;

    void LaunchSubscribers(const CSMLioWraPperOptions& options);
    void AddSensorSamplers(const CSMLioWraPperOptions& options);

    void PublishLioResultData(const ::ros::TimerEvent& timer_event);
    void PublishSlamTrajectory(const ::ros::WallTimerEvent& timer_event);
    void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
    void PublishGlobalMapPC(const ::ros::WallTimerEvent& timer_event);

    bool ValidateTopicNames(const CSMLioWraPperOptions& options);
    void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

    void OnLioResult(const ::infinityslam::common::Time time,
                            const ::infinityslam::transform::Rigid3d local_pose,
                            ::infinityslam::sensor::RangeData range_data_in_local,
                            std::shared_ptr<::infinityslam::sensor::PointCloud> point_cloud,
                            std::unique_ptr<const ::infinityslam::csmlio::InsertionResult> insertion_result)
        LOCKS_EXCLUDED(result_mutex_);

    // 后处理线程：对kf进行运动补偿并publish
    void PerformPostProcKfLoop();

    // 数据成员。
    const CSMLioWraPperOptions ros_wrapper_Options_;
    tf2_ros::Buffer* const tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::unique_ptr<TfBridge> tf_bridge_;

    // 从TrajOptions中推断出来的SensorIds。
    std::set<infinityslam::csmlio::SensorId> expected_sensor_ids_;

    std::unique_ptr<infinityslam::csmlio::CSMLidarInertialOdometry> csm_lio_;

    // 由WallTimer或Timer触发的回调线程与主线程访问同一数据的访问保护[存疑]。
    // 有一个问题是，在ros::spin()这一典型Single-threaded Spinning模式下，
    // 所有的话题回调/Timer回调都在一个线程中排队执行，此时这些读写锁就失去意义了。
    // 但为了保险起见，我们还是保留这一保护方式。
    // 参见：http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    // 另外，本类中需要两个锁，别忘了这一个类相当于原版的[Node类+MapBuilderBridge类]。
    std::mutex member_mutex_;   //对几乎所有成员的访问都需要加锁
    std::mutex result_mutex_;   //保护核心层送出来的LioResult数据 

    // SLAM返回的最新数据放在这里，访问需要加锁。
    std::shared_ptr<const SlamKeyframeData::LioKeyframeData>
        lio_result_data_;
    std::shared_ptr<const ::infinityslam::csmlio::InsertionResult>
        insertion_result_data_;

    // 类内部成员，默认构造；外部main函数不拥有此对象。
    ::ros::NodeHandle node_handle_; 

    /// 原有发布。
    ::ros::Publisher submap_list_publisher_;
    ::ros::Publisher trajectory_node_list_publisher_;
    ::ros::Publisher landmark_poses_list_publisher_;
    ::ros::Publisher constraint_list_publisher_;
    ::ros::Publisher tracked_pose_publisher_;
    ::ros::Publisher scan_matched_point_cloud_publisher_;

    /// 新增发布。
    ::ros::Publisher slam_global_map_publisher_;
    ::ros::Publisher slam_info_publisher_;
    ::ros::Publisher slam_key_frame_pc_publisher_;
    ::ros::Publisher slam_trajectory_path_publisher_;
    ::ros::Publisher slam_current_speed_publisher_;
    ::ros::Publisher slam_interp_traj_publisher_;

    // The timer for publishing local trajectory data (i.e. pose transforms and
    // range data point clouds) is a regular timer which is not triggered when
    // simulation time is standing still. This prevents overflowing the transform
    // listener buffer by publishing the same transforms over and over again.
    // 相比于WallTimer使用系统时间,Timer会在使用bag包时遵从bag包的时间。
    ::ros::Timer publish_slam_result_timer_;

    // We have to keep the timer handles of ::ros::WallTimers around, otherwise
    // they do not fire.
    std::vector<::ros::WallTimer> wall_timers_;

    struct TrajectorySensorSamplers {
        TrajectorySensorSamplers(
            const double rangefinder_sampling_ratio,
            const double odometry_sampling_ratio,
            const double imu_sampling_ratio,
            const double fixed_frame_pose_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio)
        , odometry_sampler(odometry_sampling_ratio)
        , imu_sampler(imu_sampling_ratio)
        , fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio) {}

        ::infinityslam::common::FixedRatioSampler rangefinder_sampler;
        ::infinityslam::common::FixedRatioSampler odometry_sampler;
        ::infinityslam::common::FixedRatioSampler imu_sampler;
        ::infinityslam::common::FixedRatioSampler fixed_frame_pose_sampler;
    };

    std::unique_ptr<TrajectorySensorSamplers> sensor_samplers_;
    std::vector<Subscriber> subscribers_;
    std::unordered_set<std::string> subscribed_topics_;

    // publishers 相关变量。
    ::infinityslam::common::Time last_published_slam_result_time_ 
        = ::infinityslam::common::Time::min();
    sensor_msgs::PointCloud2 submap_ros_pc2_;
    sensor_msgs::PointCloud2 global_map_ros_pc2_;
    int last_published_global_map_kf_id_ = -1;

    /**
     * 用单独线程处理（合成）关键帧，也即从所有lidar原始点云中按时间重新截取关键帧，并去畸变。
     * 利用配准位姿队列和imu队列，配合tracking到snsor的静态tf查询，对lidar点云去畸变；
     * 需要做的：1-imu原始数据缓存队列；2-配准位姿缓存队列；3-位姿近似平滑插值（在核心层实现此函数）；
     * 4-支持对【任何能够查询到相对tf的LiDAR】做点云去畸变；5-开单独线程做这个事。
     * 
     * 
     * 用单独线程做后处理：把各个lidar的原始点云对齐到同一时刻，然后发送，这里的时刻与keyframe时刻无关。
     * -- step1 出现新的位姿时（触发此线程），位姿内插器捕获位姿；
     * -- step2 在当前累积原始点云所覆盖的时段（也可以是keyframe时段）上，用【PoseInterpolator】查询【插值位姿表】；
     * -- step3 查询sensor2tracking的静态tf，结合【MotionCompensator】做点云时间对齐；
     * -- step4 格式转换为ROS，发送点云，结束。
    */

    // 后处理线程，从LiDAR原始点云合成关键帧并去畸变。
    std::mutex postproc_mutex_;
    std::thread postproc_thread_;
    std::condition_variable cv_;
    std::atomic_bool new_kf_flag_ {false};
    std::atomic_bool quit_postproc_flag_ {false};

    // 合成关键帧和去畸变所需的“原料”；点云数据以ROS/PC2格式[推荐]保存【本段数据被两个线程访问，所有访问都需要加锁】
    const double kDataCacheMaxTimeToKeep = 60.;
    using PointCloudQueue = std::deque<PointCloudXYZIT::Ptr>;
    using PointCloudQueuePtr = std::shared_ptr<PointCloudQueue>;
    std::unordered_map<std::string, PointCloudQueuePtr> point_cloud_queues_; //保存LiDAR坐标系下的原始点云。
    std::unordered_map<std::string, Rigid3d> sensors_to_tracking_;

    // 原始点云去运动畸变【本身就是多线程安全的】
    std::unique_ptr<::infinityslam::utils::ImuAidedPoseInterpolator> pose_interpolator_;
    std::unique_ptr<::infinityslam::utils::MotionCompensator> motion_compensator_;

    // 去畸变所需的数据段【以下数据仅在独立线程中被访问，无需加锁】
    std::vector<PointCloudXYZIT::Ptr> pc2_data_segment_;
    ::infinityslam::common::optional<double> last_processed_kf_time_;
    std::vector<std::pair<std::string, Rigid3d>> lidars_in_tracking_;
    size_t postproc_kf_id = 0;

    // 默认相对于end时刻构建RelativePoseTable，以便与算法保持一致。
    const bool kBasedOnEndTime = true;

    // RelativePoseTable插值表的插值时间步长，默认【10ms】；如果车辆移动速度非常快，可减小步长。
    const double kInterpTimeStep = 0.01; 

    // 考虑若干个坐标系的关系：tracking系，imu系，sensor系。
    // 合理的做法是：在tracking系下做插值和构建RelativePoseTable，然后乘以“tracking2sensor”
    // 得到sensor的RelativePoseTable，这样做的另一个好处是可以避免对每一个LiDAR都重复插值过程。
    std::vector<TimedPose> interp_pose_table_;

    // 发布去畸变后的（合成）关键帧点云
    ::ros::Publisher slam_postproc_pointcloud_publisher_;

};

}  // namespace infinityslam_ros

#endif  // ROS_APP_NODE_H
