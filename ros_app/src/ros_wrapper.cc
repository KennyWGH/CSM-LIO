/**
 * Copyright 2023 WANG Guanhua (wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include <chrono>
#include <string>
#include <vector>
#include <sstream>
#include <boost/make_unique.hpp>
#include "Eigen/Core"
#include "glog/logging.h"

#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

#include "csmlio/common/port.h"
#include "csmlio/common/time.h"
#include "csmlio/common/configuration_file_resolver.h"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/lio/pose_graph_interface.h"
#include "csmlio/lio/proto/submap_visualization.pb.h"
#include "csmlio/sensor/point_cloud.h"
#include "csmlio/sensor/odometry_data.h"
#include "csmlio/sensor/imu_data.h"
#include "csmlio/sensor/internal/voxel_filter.h"
#include "csmlio/transform/rigid_transform.h"
#include "csmlio/transform/transform.h"
#include "ros_app/src/msg_conversion.h"
#include "ros_app/src/time_conversion.h"
#include "ros_app/src/ros_wrapper.h"

namespace ros_app {

using ::csmlio::transform::Rigid3d;

namespace {

// Subscribes to the 'topic' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(const std::string&,
        const typename MessageType::ConstPtr&), /*通过函数指针的方式传递回调函数*/
    const std::string& topic,
    ::ros::NodeHandle* const node_handle, 
    Node* const node /*这个参数是类对象指针，刚好可以用this指针来传值*/ ) 
{
    return node_handle->subscribe<MessageType>(
        topic, kInfiniteSubscriberQueueSize,
        boost::function<void(const typename MessageType::ConstPtr&)>(
            [node, handler, topic](const typename MessageType::ConstPtr& msg) 
            {(node->*handler)(topic, msg);})/*lambda表达式封装为函数对象，作为话题回调*/ );
}

// 将::csmlio::sensor::PointCloud转化为含intensity的ROS/PointCloud2格式
sensor_msgs::PointCloud2 FromPointCloudToRosPointCloud2 (
  const ::csmlio::sensor::PointCloud& point_cloud)
{
  uint32_t num_points = point_cloud.size();
  sensor_msgs::PointCloud2 msg;
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

  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  const auto& cloud_data = point_cloud.points();
  const auto& intensities_data = point_cloud.intensities();
  if (intensities_data.size() != cloud_data.size()) {
    for (std::size_t i=0; i<cloud_data.size(); ++i) {
      stream.next(cloud_data[i].position.x());
      stream.next(cloud_data[i].position.y());
      stream.next(cloud_data[i].position.z());
      stream.next(0);
    }
    LOG(WARNING) << "## Found no intensities when converting "
        "point cloud, set to 0 by default.";
    return msg;
  }
  for (std::size_t i=0; i<cloud_data.size(); ++i) {
    stream.next(cloud_data[i].position.x());
    stream.next(cloud_data[i].position.y());
    stream.next(cloud_data[i].position.z());
    stream.next(intensities_data[i]);
  }
  return msg;
}

std::string mutable_frame_id_; // used by the below function.
std::unordered_map<std::string, bool> warned_table;
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
    if (frame_id.size() > 0) {
        // CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
        //                         << " should not start with a /. See 1.7 in "
        //                             "http://wiki.ros.org/tf2/Migration.";
        if (frame_id[0] == '/') {
            mutable_frame_id_ = frame_id;
            mutable_frame_id_.erase(0,1);
            if (warned_table.find(frame_id) == warned_table.end()) {
                LOG(WARNING) << "## The frame_id " << frame_id
                    << " should not start with a /. See 1.7 in "
                                        "http://wiki.ros.org/tf2/Migration.";
                LOG(WARNING) << "## Now try to use non-slash verion " << mutable_frame_id_
                    << ", if still don't working, please reconsider your configuration.";
                warned_table[frame_id] = true;
            }
            return mutable_frame_id_;
        }
    }
    return frame_id;
}

}  // namespace

/// 构造函数：保存options等，构造TfBridge、CSMLIO，注册所有Publisher和Timer回调。
Node::Node(
    const NodeOptions& node_options,
    const TrajectoryOptions& traj_options,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options)
    , trajectory_options_(traj_options)
    , tf_buffer_(tf_buffer)
{
    std::lock_guard<std::mutex> lock(inward_mutex_);

    // 构造静态tf查询工具，该工具用于查询所有sensor坐标系到tracking坐标系的tf。
    tf_bridge_ = boost::make_unique<TfBridge>(
        trajectory_options_.tracking_frame,
        node_options_.lookup_transform_timeout_sec,
        tf_buffer_);

    // 构造CSMLidarInertialOdometry对象。
    expected_sensor_ids_ = ComputeExpectedSensorIds(trajectory_options_);
    csm_lio_ = boost::make_unique<csmlio::mapping::CSMLidarInertialOdometry>(
        node_options_.map_builder_options,
        trajectory_options_.trajectory_builder_options,
        expected_sensor_ids_,
        [this](const ::csmlio::common::Time time,
            const Rigid3d local_pose,
            ::csmlio::sensor::RangeData range_data_in_local,
            std::shared_ptr<::csmlio::sensor::PointCloud> point_cloud,
            std::unique_ptr<const ::csmlio::mapping::InsertionResult> result) {
                OnLioResult(
                    time, 
                    local_pose, 
                    range_data_in_local, 
                    point_cloud, 
                    std::move(result));});

    // submap_list_publisher_ =
    //     node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
    //         kSubmapListTopic, kLatestOnlyPublisherQueueSize);
    trajectory_node_list_publisher_ =
        node_handle_.advertise<::visualization_msgs::MarkerArray>(
            kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
    landmark_poses_list_publisher_ =
        node_handle_.advertise<::visualization_msgs::MarkerArray>(
            kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
    constraint_list_publisher_ =
        node_handle_.advertise<::visualization_msgs::MarkerArray>(
            kConstraintListTopic, kLatestOnlyPublisherQueueSize);
    if (node_options_.publish_tracked_pose) {
        tracked_pose_publisher_ =
            node_handle_.advertise<::geometry_msgs::PoseStamped>(
                kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
    }
    scan_matched_point_cloud_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

    // 新增发布（自定义）。
    slam_local_ground_map_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            "slam_local_ground_map", kLatestOnlyPublisherQueueSize);
    slam_ground_labelled_pc_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            "slam_ground_labelled_pc", kLatestOnlyPublisherQueueSize);
    slam_global_map_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            "slam_global_map", kLatestOnlyPublisherQueueSize);
    slam_info_publisher_ =
        node_handle_.advertise<std_msgs::String>(
            "slam_info", kLatestOnlyPublisherQueueSize);
    slam_key_frame_pc_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            "slam_key_frame_pc", kLatestOnlyPublisherQueueSize);
    slam_trajectory_path_publisher_ =
        node_handle_.advertise<nav_msgs::Path>(
            "slam_trajectory_path", kLatestOnlyPublisherQueueSize);
    slam_current_speed_publisher_ =
        node_handle_.advertise<visualization_msgs::Marker>(
            "slam_current_speed", kLatestOnlyPublisherQueueSize);
    slam_postproc_kf_pc_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            "slam_deskewed_keyframe_pc", kLatestOnlyPublisherQueueSize);

    // 定时器回调。
    double pose_publish_period_sec = 
        node_options_.pose_publish_period_sec > 0 
        ? node_options_.pose_publish_period_sec : 0.05;
    publish_slam_result_timer_ = node_handle_.createTimer(
        ::ros::Duration(pose_publish_period_sec),
        &Node::PublishSlamResultData, this);
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(node_options_.submap_publish_period_sec),
        &Node::PublishSubmapList, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
        &Node::PublishSlamTrajectory, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(5.0), //地图更新频率无需很高。
        &Node::PublishGlobalMapPC, this));
    trigger_postproc_timer_ = node_handle_.createTimer(
        ::ros::Duration(pose_publish_period_sec),
        &Node::TriggerPostprocOnce, this);

}

Node::~Node() {
    // 结束关键帧后处理线程。
    new_kf_flag_.store(true);
    quit_postproc_flag_.store(true);
    cv_.notify_one();
    if (postproc_thread_.joinable()) postproc_thread_.join();
    LOG(INFO) << "## Terminated post-processing loop.";
    LOG(INFO) << "## Terminating system ...";
}

/// 启动话题订阅，启动SLAM/或LIO。
bool Node::Start()
{
    std::lock_guard<std::mutex> lock(inward_mutex_);
    CHECK(ValidateLioOptions(trajectory_options_));

    // 常规开启流程。
    AddSensorSamplers(trajectory_options_);
    LaunchSubscribers(trajectory_options_);
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(kTopicMismatchCheckDelaySec),
        &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
    for (const auto& sensor_id : expected_sensor_ids_) {
        subscribed_topics_.insert(sensor_id.id);
    }
    LOG(INFO) << "## Started SLAM/LIO.";

    // TODO：启动去畸变线程。
    postproc_thread_ = std::thread(&Node::PerformPostprocKfLoop,this);
    LOG(INFO) << "## Started post-processing loop.";

    return true;
}

void Node::HandleImuMessage(
    const std::string& sensor_id,
    const sensor_msgs::Imu::ConstPtr& msg) 
{
    // LOG(INFO) << "## Received IMU msg with frame_id (" 
    //     << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";

    std::lock_guard<std::mutex> lock(inward_mutex_);
    if (!sensor_samplers_->imu_sampler.Pulse()) {
        return;
    }
    CHECK_NE(msg->linear_acceleration_covariance[0], -1)
        << "Your IMU data claims to not contain linear acceleration measurements "
            "by setting linear_acceleration_covariance[0] to -1. Cartographer "
            "requires this data to work. See "
            "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
    CHECK_NE(msg->angular_velocity_covariance[0], -1)
        << "Your IMU data claims to not contain angular velocity measurements "
            "by setting angular_velocity_covariance[0] to -1. Cartographer "
            "requires this data to work. See "
            "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

    const ::csmlio::common::Time time = FromRos(msg->header.stamp);
    const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
        time, CheckNoLeadingSlash(msg->header.frame_id));
    if (sensor_to_tracking == nullptr) {
        LOG(ERROR) << "## ERROR! Found no static tf between sensor frame (" 
            << CheckNoLeadingSlash(msg->header.frame_id) << ") and tracking frame (" 
            << trajectory_options_.tracking_frame << ").";
        return;
    }
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
            "Transforming linear acceleration into the tracking frame will "
            "otherwise be imprecise.";

    // 保存IMU数据到缓存队列
    {
        std::unique_lock<std::mutex> lock(postproc_mutex_);
        imu_queue_.push_back(
            ::csmlio::sensor::ImuData{time, 
                sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
                sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
        while (imu_queue_.size() > 10 && 
            ::csmlio::common::ToSeconds(imu_queue_.back().time - imu_queue_.front().time) 
                > kDataCacheMaxTimeToKeep)
        {
            imu_queue_.pop_front();
        }
    }

    csm_lio_->AddSensorData(
        sensor_id, 
        ::csmlio::sensor::ImuData{time, 
            sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
            sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
    return;
}

void Node::HandleOdometryMessage(
    const std::string& sensor_id,
    const nav_msgs::Odometry::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(inward_mutex_);
    LOG(INFO) << "## Received odometry msg with frame_id (" 
        << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";
    if (!sensor_samplers_->odometry_sampler.Pulse()) { return; }
    const ::csmlio::common::Time time = FromRos(msg->header.stamp);
    const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
        time, CheckNoLeadingSlash(msg->child_frame_id));
    if (sensor_to_tracking == nullptr) {
        LOG(ERROR) << "## ERROR! Found no static tf between sensor frame (" 
            << CheckNoLeadingSlash(msg->child_frame_id) << ") and tracking frame (" 
            << trajectory_options_.tracking_frame << ").";
        return;
    }
    // 当前我们未启用轮速里程计。
    // csm_lio_->AddSensorData(sensor_id, 
    //     ::csmlio::sensor::OdometryData{
    //         time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
    LOG(ERROR) << "## Odometry sensor not enabled.";

}

void Node::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
    // LOG(INFO) << "## Received point cloud msg with frame_id (" 
    //     << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";

    std::lock_guard<std::mutex> lock(inward_mutex_);
    if (!sensor_samplers_->rangefinder_sampler.Pulse()) {
        return;
    }
    ::csmlio::sensor::PointCloudWithIntensities point_cloud;
    ::csmlio::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
    if (!point_cloud.points.empty()) {
        CHECK_LE(point_cloud.points.back().time, 0.f);
    }
    const auto sensor_to_tracking =
        tf_bridge_->LookupToTracking(time, CheckNoLeadingSlash(msg->header.frame_id));
    if (sensor_to_tracking == nullptr) {
        LOG(ERROR) << "## ERROR! Found no static tf between sensor frame (" 
            << CheckNoLeadingSlash(msg->header.frame_id) << ") and tracking frame (" 
            << trajectory_options_.tracking_frame << ").";
        return;
    }

    // 保存原始点云数据到缓存队列
    {
        std::unique_lock<std::mutex> lock(postproc_mutex_);
        sensor_msgs::PointCloud2Ptr msg_copy_(new sensor_msgs::PointCloud2(*msg));
        auto it = point_cloud_queues_.find(sensor_id);
        if (it != point_cloud_queues_.end()) {
            auto& point_cloud_queue_ = *it->second;
            point_cloud_queue_.push_back(msg_copy_);
            while (point_cloud_queue_.size() > 10 && 
                (point_cloud_queue_.back()->header.stamp.toSec() 
                    - point_cloud_queue_.front()->header.stamp.toSec()) 
                        > kDataCacheMaxTimeToKeep)
            {
                point_cloud_queue_.pop_front();
            }
        } else {
            point_cloud_queues_[sensor_id] = std::make_shared<PointCloudQueue>();
            point_cloud_queues_.at(sensor_id)->push_back(msg_copy_);
        }
        // 保存lidar与tracking之间的静态tf信息，供后处理线程读取。
        if (sensor_to_tracking != nullptr) {
            lidars_to_tracking[
                CheckNoLeadingSlash(msg->header.frame_id)] = *sensor_to_tracking;
        }
    }

    csm_lio_->AddSensorData(
        sensor_id, 
        ::csmlio::sensor::TimedPointCloudData{
            time, 
            sensor_to_tracking->translation().cast<float>(),
            ::csmlio::sensor::TransformTimedPointCloud(
                point_cloud.points, sensor_to_tracking->cast<float>()),
            point_cloud.intensities});
}

void Node::HandleNavSatFixMessage(
    const std::string& sensor_id,
    const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(inward_mutex_);
    LOG(INFO) << "## Received NavSat msg with frame_id (" 
        << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";
    if (!sensor_samplers_->fixed_frame_pose_sampler.Pulse()) {
        return;
    }
    // map_builder_bridge_.sensor_bridge(trajectory_id)
    //     ->HandleNavSatFixMessage(sensor_id, msg);
}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

std::set<csmlio::mapping::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const 
{
    using SensorId = csmlio::mapping::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> expected_topics;
    // Subscribe to all point cloud topics.
    for (const std::string& topic :
        GenerateRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    // IMU is required.
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
    // Odometry is optional.
    if (options.use_odometry) {
        expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
    }
    // NavSatFix is optional.
    if (options.use_nav_sat) {
        expected_topics.insert(
            SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
    }
    return expected_topics;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options) 
{
    // PointCloud2
    for (const std::string& topic :
        GenerateRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
        subscribers_.push_back(
            {SubscribeWithHandler<sensor_msgs::PointCloud2>(
                &Node::HandlePointCloud2Message, 
                topic,
                &node_handle_, 
                this),
            topic});
        LOG(INFO) << "## Registered callback for topic " 
            << subscribers_.back().topic << ".";
    }
    // IMU
    subscribers_.push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(
            &Node::HandleImuMessage,
            kImuTopic,
            &node_handle_, 
            this),
        kImuTopic});
    LOG(INFO) << "## Registered callback for topic " 
        << subscribers_.back().topic << ".";
    // Odometry
    if (options.use_odometry) {
        subscribers_.push_back(
            {SubscribeWithHandler<nav_msgs::Odometry>(
                &Node::HandleOdometryMessage,
                kOdometryTopic,
                &node_handle_, 
                this),
            kOdometryTopic});
        LOG(INFO) << "## Registered callback for topic " 
            << subscribers_.back().topic << ".";
    }
    // NAV_SAT/GNSS
    if (options.use_nav_sat) {
        subscribers_.push_back(
            {SubscribeWithHandler<sensor_msgs::NavSatFix>(
                &Node::HandleNavSatFixMessage, 
                kNavSatFixTopic,
                &node_handle_, 
                this),
            kNavSatFixTopic});
        LOG(INFO) << "## Registered callback for topic " 
            << subscribers_.back().topic << ".";
    }

}

void Node::AddSensorSamplers(const TrajectoryOptions& options) 
{
    sensor_samplers_ = boost::make_unique<TrajectorySensorSamplers>(
        options.rangefinder_sampling_ratio,
        options.odometry_sampling_ratio,
        options.imu_sampling_ratio,
        options.fixed_frame_pose_sampling_ratio/*GNSS/RTK采样器*/);
}

/// 发布tracking系的实时tf，发布实时配准点云。
void Node::PublishSlamResultData(const ::ros::TimerEvent& timer_event) 
{
    std::shared_ptr<const SlamKeyframeData::LioKeyframeData> lio_keyframe_data;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (lio_result_data_ == nullptr) {
            // LOG(INFO) << "## local slam data is null, publish nothing.";
            return;
        }
        lio_keyframe_data = std::move(lio_result_data_); // 剥夺（而非共享）所有权
    }
    if (lio_keyframe_data->time <= last_published_slam_result_time_) { return; }
    last_published_slam_result_time_ = lio_keyframe_data->time;

    // 生成 SlamKeyframeData 对象，主要是读取tracking系到publish系（可能不是同一个）的静态tf。
    SlamKeyframeData slam_keyframe_data;
    slam_keyframe_data = {
        lio_keyframe_data,
        tf_bridge_->LookupToTracking(
            lio_keyframe_data->time,
            trajectory_options_.published_frame)};


    // 发布publish系的静态tf，还是发布tracking系的静态tf？ —— 后者吧。
    const Rigid3d tracking_to_local_3d = lio_keyframe_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
        if (trajectory_options_.publish_frame_projected_to_2d) {
            return ::csmlio::transform::Embed3D(
                ::csmlio::transform::Project2D(tracking_to_local_3d));
        }
        return tracking_to_local_3d;
    }();
    const Rigid3d tracking_to_map = tracking_to_local; //单轨迹，local即map。
    geometry_msgs::TransformStamped stamped_transform;    
    stamped_transform.header.stamp = ToRos(lio_keyframe_data->time);
    stamped_transform.header.frame_id = node_options_.map_frame;
    stamped_transform.child_frame_id = trajectory_options_.tracking_frame;
    stamped_transform.transform = ToGeometryMsgTransform(tracking_to_map);
    tf_broadcaster_.sendTransform(stamped_transform);

    /*
    auto& trajectory_data = slam_keyframe_data; //别名
    if (trajectory_data.published_to_tracking != nullptr) {
        if (node_options_.publish_to_tf) {
            if (trajectory_data.trajectory_options.provide_odom_frame) {
                std::vector<geometry_msgs::TransformStamped> stamped_transforms;
                //
                stamped_transform.header.frame_id = node_options_.map_frame;
                stamped_transform.child_frame_id =
                    trajectory_data.trajectory_options.odom_frame;
                stamped_transform.transform =
                    ToGeometryMsgTransform(trajectory_data.local_to_map);
                stamped_transforms.push_back(stamped_transform);
                //
                stamped_transform.header.frame_id =
                    trajectory_data.trajectory_options.odom_frame;
                stamped_transform.child_frame_id =
                    trajectory_data.trajectory_options.published_frame;
                stamped_transform.transform = ToGeometryMsgTransform(
                    tracking_to_local * (*trajectory_data.published_to_tracking));
                stamped_transforms.push_back(stamped_transform);
                tf_broadcaster_.sendTransform(stamped_transforms);
            } else {
                stamped_transform.header.frame_id = node_options_.map_frame;
                stamped_transform.child_frame_id =
                    trajectory_data.trajectory_options.published_frame;
                stamped_transform.transform = ToGeometryMsgTransform(
                    tracking_to_map * (*trajectory_data.published_to_tracking));
                tf_broadcaster_.sendTransform(stamped_transform);
            }
        }
        if (node_options_.publish_tracked_pose) {
            ::geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = node_options_.map_frame;
            pose_msg.header.stamp = stamped_transform.header.stamp;
            pose_msg.pose = ToGeometryMsgPose(tracking_to_map);
            tracked_pose_publisher_.publish(pose_msg);
        }
        // wgh-- Publish slam info string.
        std::stringstream sstream;
        double pX = tracking_to_map.translation().x();
        double pY = tracking_to_map.translation().y();
        double pZ = tracking_to_map.translation().z();
        // pX = 0.123456;
        // pY = -0.123456;
        // pZ = -4.567890;
        if (pX<-1e4 || pX>1e4 || pY<-1e4 || pY>1e4 || pZ<-1e4 || pZ>1e4) {
            // LOG_WARN("## Warning! Robot position are too large, please check!");
            sstream << "x(" << pX << "), y(" << pY << "), z(" << pZ << ") may be too large, warning!";
        }
        else {
            sstream <<    "x(" << (pX>0?"":"-") << std::abs(int(pX)) 
                                << "." <<  std::abs(int((pX-int(pX))*100))
                    << "), y(" << (pY>0?"":"-") << std::abs(int(pY)) 
                                << "." <<  std::abs(int((pY-int(pY))*100))
                    << "), z(" << (pZ>0?"":"-") << std::abs(int(pZ)) 
                                << "." <<  std::abs(int((pZ-int(pZ))*100)) << ")";
        }
        std_msgs::String str_msg;
        str_msg.data = sstream.str();
        slam_info_publisher_.publish(str_msg);
    }
    */

    // 发布配准过的点云。
    if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) 
    {
        auto& point_cloud_in_local = 
            lio_keyframe_data->range_data_in_local.returns;
        sensor_msgs::PointCloud2 pc2_msg = 
            FromPointCloudToRosPointCloud2(point_cloud_in_local);
        pc2_msg.header.stamp = ToRos(lio_keyframe_data->time);
        pc2_msg.header.frame_id = node_options_.map_frame;
        scan_matched_point_cloud_publisher_.publish(pc2_msg);
    }

}

/// 发布lio/slam轨迹。
void Node::PublishSlamTrajectory(const ::ros::WallTimerEvent& timer_event) 
{
    if (slam_trajectory_path_publisher_.getNumSubscribers() > 0) {
        std::lock_guard<std::mutex> lock(inward_mutex_);
        const auto& keyframes = csm_lio_->GetSlamKeyframeList();
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = node_options_.map_frame;
        geometry_msgs::PoseStamped stamped_pose;
        for (const auto& kf : keyframes) {
            stamped_pose.pose.position.x = kf.constant_data->local_pose.translation().x();
            stamped_pose.pose.position.y = kf.constant_data->local_pose.translation().y();
            stamped_pose.pose.position.z = kf.constant_data->local_pose.translation().z();
            stamped_pose.pose.orientation.w = kf.constant_data->local_pose.rotation().w();
            stamped_pose.pose.orientation.x = kf.constant_data->local_pose.rotation().x();
            stamped_pose.pose.orientation.y = kf.constant_data->local_pose.rotation().y();
            stamped_pose.pose.orientation.z = kf.constant_data->local_pose.rotation().z();
            stamped_pose.header = ros_path.header;
            stamped_pose.header.stamp = ToRos(kf.time());
            ros_path.poses.push_back(stamped_pose);
        }
        ros_path.header.stamp = stamped_pose.header.stamp;
        slam_trajectory_path_publisher_.publish(ros_path);
    }
}

/// 发布grid格式的submap？
void Node::PublishSubmapList(const ::ros::WallTimerEvent& timer_event) 
{
    /// 如果想要按照期望的方式获取submap实体数据，需要从Submap3D::ToResponseProto()函数
    /// 开始复现。比较麻烦，我们暂时不考虑这个需求。


    // std::lock_guard<std::mutex> lock(xxx_mutex_);
    // submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

/// 发布点云形式的全局地图
void Node::PublishGlobalMapPC(const ::ros::WallTimerEvent& timer_event) 
{
    if (slam_global_map_publisher_.getNumSubscribers() <= 0) { return; }


    // ##########################################################################
    // ############## 用range_data_in_sensor + 位姿拼接得到globalmap ##############
    // ##########################################################################
    {
        //
    }


    // ##########################################################################
    // ############## 直接用range_data_in_local 做globalmap拼接和显示 ##############
    // ##########################################################################
    std::lock_guard<std::mutex> lock(inward_mutex_);
    const auto& keyframes = csm_lio_->GetSlamKeyframeList();
    if (keyframes.empty()) {
        LOG(WARNING) << "## No keframes exist, publish no global map.";
        return;
    }
    // 轨迹规模较大时，生成点云地图的过程可能比较耗时，我们避免重复计算。
    int current_latest_kf_id = keyframes.back().node_id;
    if (current_latest_kf_id <= last_published_global_map_kf_id_) {
        slam_global_map_publisher_.publish(global_map_ros_pc2_);
        LOG(INFO) << "## Published global map, info: " 
            << keyframes.size() << " keframes, "
            << global_map_ros_pc2_.width << " points.";
        return;
    }
    last_published_global_map_kf_id_ = current_latest_kf_id;

    // 拼接&体素将采样&生成点云地图。
    using ::csmlio::sensor::RangefinderPoint;
    std::vector<RangefinderPoint> global_map_points;
    std::vector<float> global_map_intensities;
    std::vector<RangefinderPoint> temp_submap_points;
    std::vector<float> temp_submap_intensities;
    std::size_t temp_submap_kf_num = 0;
    // 采用两个策略【1.子图+体素将采样；2.跳帧】以控制点云地图的大小，使得在轨迹规模
    // 很大时我们也能以较小的代价发布全局地图。
    // 当轨迹规模不太大时，每400个node汇聚为一个submap，对submap做一次体素降采样；
    // 当轨迹规模过大时(超过2000)，将所有node分为5份，按5个submap处理；
    // 达到的效果是，无论有多少个node，我们至多允许5个submap。
    // 在不同的node规模上，我们分别采取 9/10、3/4、1/2 的采样频率来使用node。
    size_t current_keyframes_num = keyframes.size();
    std::size_t submap_kf_num_limit = current_keyframes_num < 2000 
                                            ? 400 : (current_keyframes_num / 5 + 1);
    std::size_t skip_once_every_n_kfs = current_keyframes_num < 2000 
                                            ? 10 
                                            : (current_keyframes_num < 4000
                                                ? 4
                                                : 2);
    const float range_limit = 30.; // We control the range.
    const float voxel_size = 0.1; // Voxel size for voxel filter.
    std::size_t curr_kf_index = 0;
    for (const auto& curr_keyframe : keyframes)
    {
        ++curr_kf_index;
        if (curr_kf_index % skip_once_every_n_kfs == 0) continue;
        const auto& trajectory_node = curr_keyframe; //alias
        if (curr_keyframe.constant_data == nullptr) continue;
        auto& kf_point_cloud = curr_keyframe.constant_data->low_resolution_point_cloud;
        auto& kf_global_pose = curr_keyframe.global_pose;
        auto& points = kf_point_cloud.points();
        auto& intensities = kf_point_cloud.intensities();
        bool assign_intensity = (points.size() != 0) && (points.size() != intensities.size());
        if (points.size() != 0) 
        {
            temp_submap_kf_num++;
            for (std::size_t i=0; i<points.size(); i++) {
                if (points[i].position.norm() > range_limit) continue;
                temp_submap_points.push_back(kf_global_pose.cast<float>() * points[i]);
                temp_submap_intensities.push_back(assign_intensity ? 10.f : intensities[i]);
            }
            if (temp_submap_kf_num == submap_kf_num_limit) {
                auto filtered_temp_submap = ::csmlio::sensor::VoxelFilter(
                    ::csmlio::sensor::PointCloud(temp_submap_points,temp_submap_intensities), 
                    voxel_size);
                const auto& filtered_submap_points = filtered_temp_submap.points();
                const auto& filtered_submap_intensities = filtered_temp_submap.intensities();
                if (filtered_submap_points.size() == filtered_submap_intensities.size()) {
                    global_map_points.insert(global_map_points.end(), 
                        filtered_submap_points.begin(), filtered_submap_points.end());
                    global_map_intensities.insert(global_map_intensities.end(),
                        filtered_submap_intensities.begin(), filtered_submap_intensities.end());
                }
                // reset temp submap.
                temp_submap_kf_num = 0;
                temp_submap_points.clear();
                temp_submap_intensities.clear();
            }
        }
    }
    // make sure you don't miss the last 'incomplete' submap!
    if (temp_submap_kf_num > 0) {
        auto filtered_temp_submap = ::csmlio::sensor::VoxelFilter(
            ::csmlio::sensor::PointCloud(temp_submap_points,temp_submap_intensities), 
            voxel_size);
        const auto& filtered_submap_points = filtered_temp_submap.points();
        const auto& filtered_submap_intensities = filtered_temp_submap.intensities();
        if (filtered_submap_points.size() == filtered_submap_intensities.size()) {
            global_map_points.insert(global_map_points.end(), 
                filtered_submap_points.begin(), filtered_submap_points.end());
            global_map_intensities.insert(global_map_intensities.end(),
                filtered_submap_intensities.begin(), filtered_submap_intensities.end());
        }
    }
    ::csmlio::sensor::PointCloud global_map(global_map_points, global_map_intensities);

    global_map_ros_pc2_ = FromPointCloudToRosPointCloud2(global_map);
    global_map_ros_pc2_.header.stamp = ros::Time::now();
    global_map_ros_pc2_.header.frame_id = node_options_.map_frame;
    slam_global_map_publisher_.publish(global_map_ros_pc2_);
    LOG(INFO) << "## Published global map, info: " 
        << keyframes.size() << " keframes, "
        << global_map_ros_pc2_.width << " points.";

}

/// 以一定的频率触keyframe后处理线程。
void Node::TriggerPostprocOnce(const ::ros::TimerEvent& timer_event) 
{
    {
        std::lock_guard<std::mutex> lock1(inward_mutex_);
        std::lock_guard<std::mutex> lock2(postproc_mutex_);
        timed_pose_queue_ = csm_lio_->GetTimedPoseQueue();
        if (timed_pose_queue_.size() > 2 && 
            timed_pose_queue_.back().time > last_processed_kf_time_) {
            new_kf_flag_.store(true);
            cv_.notify_one();
        }
    }
}

bool Node::ValidateLioOptions(const TrajectoryOptions& options) 
{
    if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
        return options.trajectory_builder_options
            .has_trajectory_builder_3d_options();
    }
    return false;
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options) 
{
    for (const auto& sensor_id : expected_sensor_ids_) {
        const std::string& topic = sensor_id.id;
        if (subscribed_topics_.count(topic) > 0) {
        LOG(ERROR) << "## Topic name [" << topic << "] is already used.";
        return false;
        }
    }
    return true;
}

void Node::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& timer_event) 
{
    std::lock_guard<std::mutex> lock(inward_mutex_);
    ::ros::master::V_TopicInfo ros_topics;
    ::ros::master::getTopics(ros_topics); //从ROS系统中获取当前发布的所有话题。
    std::set<std::string> published_topics;
    std::stringstream published_topics_string;
    for (const auto& it : ros_topics) {
        std::string resolved_topic = node_handle_.resolveName(it.name, false);
        published_topics.insert(resolved_topic);
        published_topics_string << resolved_topic << ",";
    }
    bool print_topics = false;
    for (const auto& subscriber : subscribers_) {
        std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
        if (published_topics.count(resolved_topic) == 0) {
            LOG(WARNING) << "## Expected topic \"" << subscriber.topic
                        << "\" (resolved topic \"" << resolved_topic << "\")"
                        << " but no publisher is currently active.";
            print_topics = true;
        }
    }
    if (print_topics) {
        LOG(WARNING) << "## Currently available topics are: "
                    << published_topics_string.str();
    }
}

void Node::OnLioResult(
    const ::csmlio::common::Time time,
    const Rigid3d local_pose,
    ::csmlio::sensor::RangeData range_data_in_local,
    std::shared_ptr<::csmlio::sensor::PointCloud> point_cloud,
    std::unique_ptr<const ::csmlio::mapping::InsertionResult> insertion_result) 
{
    // 对于指针对象，我们必须深拷贝数据，避免wrapper层和core层访问同一个数据块。
    std::shared_ptr<::csmlio::sensor::PointCloud> point_cloud_;
    if (point_cloud != nullptr) {
        point_cloud_.reset(
            new ::csmlio::sensor::PointCloud(point_cloud->points(), 
                                                point_cloud->intensities()));
    }

    std::shared_ptr<const SlamKeyframeData::LioKeyframeData> lio_keyframe_data =
        std::make_shared<SlamKeyframeData::LioKeyframeData>(
            SlamKeyframeData::LioKeyframeData{time,
                                                local_pose,
                                                std::move(range_data_in_local),
                                                point_cloud_});
    std::lock_guard<std::mutex> lock(result_mutex_);
    lio_result_data_ = std::move(lio_keyframe_data);


    // 我们需要深拷贝保留insertion_result中的所有数据（主要是Submap3D数据），
    // 以供wrapper层独立访问和向外发布。
    // 说明：Submap3D/Submap/.../DynamicGrid/NestedGrid 里面保存的shared指针，深拷贝数据比较麻烦。
    // TODO 直接调用Submap3D接口，获取并保存低分辨率hybridgrid的有效cell。
}

void Node::PerformPostprocKfLoop()
{
    while (!quit_postproc_flag_) {
        // 解释：
        // conditional_variable的wait函数只接受unique_lock类型的锁，这种锁可以在持有互斥量的同时而不锁住互斥量；
        // 条件变量的wait函数会让当前线程阻塞在这里，直到notified；
        // wait函数在阻塞当前线程的同时，会解锁unique_lock对象所持有的mutex，以便其它线程能够访问mutex；
        // wait函数在被notified时，会尝试锁住unique_lock对象所持有的mutex，并查询lambda返回值；
        // wait()函数只有在满足mutex被lock，且lambda表达时为true时，才会结束等待（结束阻塞），允许当前线程继续往下执行。
        // 详情参见C++官网：https://cplusplus.com/reference/condition_variable/condition_variable/wait/
        std::unique_lock<std::mutex> lock(postproc_mutex_);
        cv_.wait(lock, [this](){return new_kf_flag_.load();});
        // LOG(INFO) << "Postproc: triggered. ";

        // 拷贝数据段，然后释放互斥量；
        using namespace ::csmlio;
        int start_index = -1;
        for (int i = timed_pose_queue_.size()-1; i>=0; --i) {
            if (timed_pose_queue_[i].time <= last_processed_kf_time_) {
                start_index = i;
                break;
            }
        }
        if (start_index < 0) start_index = 0;
        int end_index = start_index + 1;
        if (end_index >= timed_pose_queue_.size()) {
            LOG(WARNING) << "Postproc: haven't accumulated enough timed pose (found " 
                << timed_pose_queue_.size() << ", while >=2 is required).";
            new_kf_flag_.store(false);
            lock.unlock();
            continue;
        }
        const common::Time kf_st_time_ = timed_pose_queue_[start_index].time;
        const common::Time kf_ed_time_ = timed_pose_queue_[end_index].time;
        const common::Time kf_ed_time_ext_ = 
            kf_ed_time_ + common::FromSeconds(0.5 * kInterpTimeStep);
        // 获取时间段内的配准位姿
        timed_pose_segment_.clear();
        for (int i=start_index; i<=end_index; ++i) {
            timed_pose_segment_.push_back(timed_pose_queue_[i]);
        }
        // 获取时间段内的IMU数据。
        imu_data_segment_.clear();
        int imu_st_index = -1;
        for (int i=imu_queue_.size()-1; i>=0; --i) {
            if (imu_queue_[i].time < kf_st_time_) {
                imu_st_index = i;
                break;
            }
        }
        if (imu_st_index < 0) imu_st_index = 0;
        if (!imu_queue_.empty()) {
            for (int i=imu_st_index; i<imu_queue_.size(); ++i) {
                imu_data_segment_.push_back(imu_queue_[i]);
            }
        }
        // 获取时间段内的PointCloud2数据。
        // 对每一个点云队列，找出所有“点云时段”与“[kf_st_time_, kf_ed_time_ + kInterpTimeStep / 2]时段”
        // 有overlap的msg，作为处理的对象。其中“+ kInterpTimeStep / 2”是为了包容时间戳格式转换误差。
        pc2_data_segment_.clear();
        for (auto& pair : point_cloud_queues_) {
            auto& pc2_data_queue = *(pair.second);
            int pop_to_here = -1;
            for (size_t i = 0; i < pc2_data_queue.size(); ++i) {
                auto pc2_st_time = GetPC2StartTime(*pc2_data_queue[i]);
                auto pc2_end_time = GetPC2EndTime(*pc2_data_queue[i]);
                if (pc2_end_time < kf_ed_time_ext_) pop_to_here = i;
                if (pc2_st_time < kf_ed_time_ext_ && pc2_end_time > kf_st_time_) {
                    pc2_data_segment_.push_back(pc2_data_queue[i]);
                }
            }
            if (pop_to_here >= 0) {
                for (int i=0; i<=pop_to_here; ++i) {
                    pc2_data_queue.pop_front();
                }
            }
            
        }

        // 拷贝数据结束，释放锁。
        lock.unlock();

        
        ::csmlio::mapping::InterpolatePoseTable(
            kf_st_time_, kf_ed_time_, 
            timed_pose_segment_, 
            imu_data_segment_, 
            kInterpTimeStep, 
            relative_pose_table_);

        for (auto pair : lidars_to_tracking) {
            std::string registered_frame_id = pair.first;
            auto lidar_to_tracking = pair.second;
            std::vector<::csmlio::transform::Rigid3d> lidar_pose_table;
            for (auto& pose : relative_pose_table_) {
                lidar_pose_table.push_back(pose.pose * lidar_to_tracking);
            }
            // 找出所有frame_id符合的PC2消息；
            // 解析PC2点云格式为可处理格式；
            // 统一转换到end时间点的lidar位姿下；
            for (auto& pc2_ : pc2_data_segment_) {
                // if (CheckNoLeadingSlash())
                slam_postproc_kf_pc_publisher_.publish(*pc2_);
            }
            // 转换回PC2格式，Ros::Publisher发送出去。
            // slam_postproc_kf_pc_publisher_
        }
        
        // LOG(INFO) << "Postproc: copied necessary data segment.";
        // 然后执行去畸变并publish，然后结束。
        // ...
        // LOG(INFO) << "Postproc: merged and deskewed key frame.";
        // slam_processed_keyframe_pc_publisher_.publish();

        // 执行完毕，结束本次循环。
        last_processed_kf_time_ = kf_ed_time_;
        new_kf_flag_.store(false);
        LOG(INFO) << "Postproc: took XXX ms, merged XXX points, advanced to time point [xxx, xxx].";

    }
}


}  // namespace ros_app
