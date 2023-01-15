/**
 * Copyright 2023 WANG Guanhua (wangguanhua999@gmail.com)
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

#include "csmlio/lio/utility.h"
#include "csmlio/common/time.h"
#include "csmlio/common/fixed_ratio_sampler.h"
#include "csmlio/lio/csm_lidar_inertial_odometry.h"
#include "ros_app/src/node_constants.h"
#include "ros_app/src/node_options.h"
#include "ros_app/src/trajectory_options.h"
#include "ros_app/src/urdf_reader.h"
#include "ros_app/src/tf_bridge.h"

namespace ros_app {

// Wires up ROS topics to SLAM.
class Node {
  public:
    using Rigid3d = ::csmlio::transform::Rigid3d;
    using TimedPose = ::csmlio::mapping::TimedPose;
    using ImuData = ::csmlio::sensor::ImuData;
    
    struct SlamKeyframeData {
        struct LioKeyframeData {
            ::csmlio::common::Time time;
            ::csmlio::transform::Rigid3d local_pose;
            ::csmlio::sensor::RangeData range_data_in_local;
            std::shared_ptr<::csmlio::sensor::PointCloud> point_cloud;
        };

        std::shared_ptr<const LioKeyframeData> local_slam_data;
        std::unique_ptr<csmlio::transform::Rigid3d> published_to_tracking;
        // ::csmlio::transform::Rigid3d global_pose;
        // TrajectoryOptions trajectory_options;
    };

    Node(const NodeOptions& node_options,
        const TrajectoryOptions& traj_options,
        tf2_ros::Buffer* tf_buffer);
    ~Node();

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

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
    std::set<::csmlio::mapping::SensorId>
    ComputeExpectedSensorIds(const TrajectoryOptions& options) const;

    void LaunchSubscribers(const TrajectoryOptions& options);
    void AddSensorSamplers(const TrajectoryOptions& options);

    void PublishSlamResultData(const ::ros::TimerEvent& timer_event);
    void PublishSlamTrajectory(const ::ros::WallTimerEvent& timer_event);
    void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
    void PublishGlobalMapPC(const ::ros::WallTimerEvent& timer_event);

    void TriggerPostprocOnce(const ::ros::TimerEvent& timer_event);

    bool ValidateLioOptions(const TrajectoryOptions& options);
    bool ValidateTopicNames(const TrajectoryOptions& options);
    void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

    void OnLioResult(const ::csmlio::common::Time time,
                            const ::csmlio::transform::Rigid3d local_pose,
                            ::csmlio::sensor::RangeData range_data_in_local,
                            std::shared_ptr<::csmlio::sensor::PointCloud> point_cloud,
                            std::unique_ptr<const ::csmlio::mapping::InsertionResult> insertion_result)
        LOCKS_EXCLUDED(result_mutex_);

    void PerformPostprocKfLoop();

    // 数据成员。
    const NodeOptions node_options_;
    const TrajectoryOptions trajectory_options_;
    tf2_ros::Buffer* const tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::unique_ptr<TfBridge> tf_bridge_;

    // 从TrajOptions中推断出来的SensorIds。
    std::set<csmlio::mapping::SensorId> expected_sensor_ids_;

    std::unique_ptr<csmlio::mapping::CSMLidarInertialOdometry> csm_lio_;

    // 由WallTimer或Timer触发的回调线程与主线程访问同一数据的访问保护[存疑]。
    // 有一个问题是，在ros::spin()这一典型Single-threaded Spinning模式下，
    // 所有的话题回调/Timer回调都在一个线程中排队执行，此时这些读写锁就失去意义了。
    // 但为了保险起见，我们还是保留这一保护方式。
    // 参见：http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    // 另外，本类中需要两个锁，别忘了这一个类相当于原版的[Node类+MapBuilderBridge类]。
    std::mutex inward_mutex_;   //几乎对所有重要成员的访问都需要加锁
    std::mutex result_mutex_;   //保护核心层送出来的SlamResult数据 

    // SLAM返回的最新数据放在这里，访问需要加锁。
    std::shared_ptr<const SlamKeyframeData::LioKeyframeData>
        lio_result_data_ GUARDED_BY(result_mutex_);
    std::shared_ptr<const ::csmlio::mapping::InsertionResult>
        insertion_result_data_ GUARDED_BY(result_mutex_);

    // NodeHandle作为类成员，默认构造；外部main函数中不拥有此对象。
    ::ros::NodeHandle node_handle_; 

    /// 原有发布。
    // ::ros::Publisher submap_list_publisher_;
    ::ros::Publisher trajectory_node_list_publisher_;
    ::ros::Publisher landmark_poses_list_publisher_;
    ::ros::Publisher constraint_list_publisher_;
    ::ros::Publisher tracked_pose_publisher_;
    ::ros::Publisher scan_matched_point_cloud_publisher_;

    /// 新增发布。
    ::ros::Publisher slam_local_ground_map_publisher_;
    ::ros::Publisher slam_ground_labelled_pc_publisher_;
    ::ros::Publisher slam_global_map_publisher_;
    ::ros::Publisher slam_info_publisher_;
    ::ros::Publisher slam_key_frame_pc_publisher_;
    ::ros::Publisher slam_trajectory_path_publisher_;
    ::ros::Publisher slam_current_speed_publisher_;

    // The timer for publishing local trajectory data (i.e. pose transforms and
    // range data point clouds) is a regular timer which is not triggered when
    // simulation time is standing still. This prevents overflowing the transform
    // listener buffer by publishing the same transforms over and over again.
    // 相比于WallTimer使用系统时间,Timer会在使用bag包时遵从bag包的时间。
    ::ros::Timer publish_slam_result_timer_;
    ::ros::Timer trigger_postproc_timer_;

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

        ::csmlio::common::FixedRatioSampler rangefinder_sampler;
        ::csmlio::common::FixedRatioSampler odometry_sampler;
        ::csmlio::common::FixedRatioSampler imu_sampler;
        ::csmlio::common::FixedRatioSampler fixed_frame_pose_sampler;
    };

    std::unique_ptr<TrajectorySensorSamplers> sensor_samplers_;
    std::vector<Subscriber> subscribers_;
    std::unordered_set<std::string> subscribed_topics_;

    // 发布器相关变量。
    ::csmlio::common::Time last_published_slam_result_time_ 
        = ::csmlio::common::Time::min();
    sensor_msgs::PointCloud2 global_map_ros_pc2_;
    int last_published_global_map_kf_id_ = -1;

    /**
     * 用单独线程处理（合成）关键帧，也即从所有lidar原始点云中按时间重新截取关键帧，并去畸变。
     * 利用配准位姿队列和imu队列，配合tracking到snsor的静态tf查询，对lidar点云去畸变；
     * 需要做的：1-imu原始数据缓存队列；2-配准位姿缓存队列；3-位姿近似平滑插值（在核心层实现此函数）；
     * 4-支持对【任何能够查询到相对tf的LiDAR】做点云去畸变；5-开单独线程做这个事。
    */

    // 后处理线程，从LiDAR原始点云合成关键帧并去畸变。
    std::mutex postproc_mutex_;
    std::thread postproc_thread_;
    std::condition_variable cv_;
    std::atomic_bool new_kf_flag_ {false};
    std::atomic_bool quit_postproc_flag_ {false};

    // 合成关键帧和去畸变所需的“原料”；点云数据ROS格式[推荐]保存【本段数据被两个线程访问，所有访问都需要加锁】
    const double kDataCacheMaxTimeToKeep = 10.;
    using PointCloudQueue = std::deque<sensor_msgs::PointCloud2Ptr>;
    using PointCloudQueuePtr = std::shared_ptr<PointCloudQueue>;
    std::unordered_map<std::string, PointCloudQueuePtr> point_cloud_queues_; //保存sensor下的原始点云。
    std::deque<ImuData> imu_queue_;     //这里的IMU数据已经转换到tracking坐标系下了。
    std::deque<TimedPose> timed_pose_queue_;    //tracking坐标系的lio输出位姿。
    ::csmlio::common::Time last_processed_kf_time_ = ::csmlio::common::Time::min();
    std::unordered_map<std::string, ::csmlio::transform::Rigid3d> lidars_to_tracking;

    // 去畸变所需的数据段【以下数据仅在独立线程中被访问，无需加锁】
    std::vector<ImuData> imu_data_segment_;
    std::vector<TimedPose> timed_pose_segment_;
    std::vector<sensor_msgs::PointCloud2Ptr> pc2_data_segment_;

    // 默认相对于end时刻构建RelativePoseTable，以便与算法保持一致。
    const bool kBasedOnEndTime = true;

    // RelativePoseTable插值表的插值时间步长，默认【10ms】；如果车辆移动速度非常快，可减小步长。
    const double kInterpTimeStep = 0.01; 

    // 考虑若干个坐标系的关系：tracking系，imu系，sensor系。
    // 合理的做法是：在tracking系下做插值和构建RelativePoseTable，然后乘以“tracking2sensor”
    // 得到sensor的RelativePoseTable，这样做的另一个好处是可以避免对每一个LiDAR都重复插值过程。
    std::vector<TimedPose> relative_pose_table_;

    // 发布去畸变后的（合成）关键帧点云
    ::ros::Publisher slam_postproc_kf_pc_publisher_;

};

}  // namespace ros_app

#endif  // ROS_APP_NODE_H
