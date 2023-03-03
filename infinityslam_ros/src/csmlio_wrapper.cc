/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
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

#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/sensor/internal/voxel_filter.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/transform.h"

#include "infinityslam_ros/src/msg_conversion.h"
#include "infinityslam_ros/src/time_conversion.h"
#include "infinityslam_ros/src/ros_wrapper_options.h"
#include "infinityslam_ros/src/csmlio_wrapper.h"

// test
#include "infinityslam/utils/imu_integration.h"

namespace infinityslam_ros {

using ::infinityslam::transform::Rigid3d;

namespace {

// Subscribes to the 'topic' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (CSMLioWrapper::*handler)(const std::string&,
        const typename MessageType::ConstPtr&), /*通过函数指针的方式传递回调函数*/
    const std::string& topic,
    ::ros::NodeHandle* const node_handle, 
    CSMLioWrapper* const node /*这个参数是类对象指针，刚好可以用this指针来传值*/ ) 
{
    return node_handle->subscribe<MessageType>(
        topic, kInfiniteSubscriberQueueSize,
        boost::function<void(const typename MessageType::ConstPtr&)>(
            [node, handler, topic](const typename MessageType::ConstPtr& msg) 
            {(node->*handler)(topic, msg);})/*lambda表达式封装为函数对象，作为话题回调*/ );
}

// 将::infinityslam::sensor::PointCloud转化为含intensity的ROS/PointCloud2格式
sensor_msgs::PointCloud2 FromPointCloudToRosPointCloud2 (
    const ::infinityslam::sensor::PointCloud& point_cloud)
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
        // LOG(WARNING) << "## Found no intensities when converting to "
        //     "ros pc2, set to 0 by default.";
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
std::unordered_map<std::string, bool> frame_id_warning_table;
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
    if (frame_id.size() > 0) {
        // CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
        //                         << " should not start with a /. See 1.7 in "
        //                             "http://wiki.ros.org/tf2/Migration.";
        if (frame_id[0] == '/') {
            mutable_frame_id_ = frame_id;
            mutable_frame_id_.erase(0,1);
            if (frame_id_warning_table.find(frame_id) == frame_id_warning_table.end()) {
                LOG(WARNING) << "## The frame_id " << frame_id
                    << " should not start with a /. See 1.7 in "
                                        "http://wiki.ros.org/tf2/Migration.";
                LOG(WARNING) << "## Now try to use non-slash verion " << mutable_frame_id_
                    << ", if still don't working, please solve it by yourself.";
                frame_id_warning_table[frame_id] = true;
            }
            return mutable_frame_id_;
        }
    }
    return frame_id;
}

}  // namespace

CSMLioWraPperOptions ReadCSMLioWraPperOptions() {
    CSMLioWraPperOptions roswrapperoPtions;
    // TODO: 加载参数
    return roswrapperoPtions;
}

/// 构造函数：保存options等，构造TfBridge、CSMLIO，注册所有Publisher和Timer回调。
CSMLioWrapper::CSMLioWrapper(
    const CSMLioWraPperOptions& ros_options,
    tf2_ros::Buffer* const tf_buffer)
    : ros_wrapper_Options_(ros_options)
    , tf_buffer_(tf_buffer)
    , pose_interpolator_(new ::infinityslam::utils::ImuAidedPoseInterpolator)
    , motion_compensator_(new ::infinityslam::utils::MotionCompensator)
{
    std::lock_guard<std::mutex> lock(member_mutex_);

    // 构造静态tf查询工具，该工具用于查询所有sensor坐标系到tracking坐标系的tf。
    tf_bridge_ = boost::make_unique<TfBridge>(
        ros_wrapper_Options_.tracking_frame,
        infinityslam_ros::options::lookup_transform_timeout_sec,
        tf_buffer_);

    // 构造CSMLidarInertialOdometry对象。
    expected_sensor_ids_ = ComputeExpectedSensorIds(ros_wrapper_Options_);
    csm_lio_ = boost::make_unique<infinityslam::csmlio::CSMLidarInertialOdometry>(
        infinityslam::csmlio::ReadCSMLioOptions(),
        expected_sensor_ids_,
        [this](const ::infinityslam::common::Time time,
            const Rigid3d local_pose,
            ::infinityslam::sensor::RangeData range_data_in_local,
            std::shared_ptr<::infinityslam::sensor::PointCloud> point_cloud,
            std::unique_ptr<const ::infinityslam::csmlio::InsertionResult> result) {
                OnLioResult(
                    time, 
                    local_pose, 
                    range_data_in_local, 
                    point_cloud, 
                    std::move(result));});

    submap_list_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            kSubmapListTopic, kLatestOnlyPublisherQueueSize);
    trajectory_node_list_publisher_ =
        node_handle_.advertise<::visualization_msgs::MarkerArray>(
            kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
    landmark_poses_list_publisher_ =
        node_handle_.advertise<::visualization_msgs::MarkerArray>(
            kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
    constraint_list_publisher_ =
        node_handle_.advertise<::visualization_msgs::MarkerArray>(
            kConstraintListTopic, kLatestOnlyPublisherQueueSize);
    if (infinityslam_ros::options::publish_tracked_pose) {
        tracked_pose_publisher_ =
            node_handle_.advertise<::geometry_msgs::PoseStamped>(
                kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
    }
    scan_matched_point_cloud_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

    // 新增发布（自定义）。
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
    slam_postproc_pointcloud_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            "slam_deskewed_pointcloud", kLatestOnlyPublisherQueueSize);
    slam_interp_traj_publisher_ =
        node_handle_.advertise<nav_msgs::Odometry>(
            "slam_interp_path", kLatestOnlyPublisherQueueSize);

    // 定时器回调。
    double pose_publish_period_sec = 
        infinityslam_ros::options::pose_publish_period_sec > 0 
        ? infinityslam_ros::options::pose_publish_period_sec : 0.05;
    publish_slam_result_timer_ = node_handle_.createTimer(
        ::ros::Duration(pose_publish_period_sec),
        &CSMLioWrapper::PublishLioResultData, this);
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(infinityslam_ros::options::submap_publish_period_sec),
        &CSMLioWrapper::PublishSubmapList, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(infinityslam_ros::options::trajectory_publish_period_sec),
        &CSMLioWrapper::PublishSlamTrajectory, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(5.0), //地图更新频率无需很高。
        &CSMLioWrapper::PublishGlobalMapPC, this));

}

CSMLioWrapper::~CSMLioWrapper() {
    // 结束关键帧后处理线程。
    new_kf_flag_.store(true);
    quit_postproc_flag_.store(true);
    cv_.notify_one();
    if (postproc_thread_.joinable()) postproc_thread_.join();
    LOG(INFO) << "## Terminated post-processing loop.";
    LOG(INFO) << "## Terminating system ...";
}

/// 启动话题订阅，启动SLAM/或LIO。
bool CSMLioWrapper::Start()
{
    std::lock_guard<std::mutex> lock(member_mutex_);

    // 常规开启流程。
    AddSensorSamplers(ros_wrapper_Options_);
    LaunchSubscribers(ros_wrapper_Options_);
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(kTopicMismatchCheckDelaySec),
        &CSMLioWrapper::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
    for (const auto& sensor_id : expected_sensor_ids_) {
        subscribed_topics_.insert(sensor_id.id);
    }
    LOG(INFO) << "## Started SLAM/LIO.";

    // TODO：启动去畸变线程。
    postproc_thread_ = std::thread(&CSMLioWrapper::PerformPostProcKfLoop,this);
    LOG(INFO) << "## Started post-processing loop.";

    return true;
}

void CSMLioWrapper::HandleImuMessage(
    const std::string& sensor_id,
    const sensor_msgs::Imu::ConstPtr& msg) 
{
    // LOG(INFO) << "## Received IMU msg with frame_id (" 
    //     << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";

    std::lock_guard<std::mutex> lock(member_mutex_);
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

    const ::infinityslam::common::Time time = FromRos(msg->header.stamp);
    const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
        time, CheckNoLeadingSlash(msg->header.frame_id));
    if (sensor_to_tracking == nullptr) {
        LOG(ERROR) << "## ERROR! Found no static tf between sensor frame (" 
            << CheckNoLeadingSlash(msg->header.frame_id) << ") and tracking frame (" 
            << ros_wrapper_Options_.tracking_frame << ").";
        return;
    }
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
            "Transforming linear acceleration into the tracking frame will "
            "otherwise be imprecise.";

    // LOG(INFO) << "Debug: received imu msg with timestamp: \n"
    //     << std::fixed << std::setprecision(18)
    //     << "ROS      : " << msg->header.stamp.toSec() << "\n"
    //     << "Universal: " << infinityslam::common::ToSeconds(time) << "\n";

    // 保存IMU数据到缓存队列
    pose_interpolator_->AddImu(
        ::infinityslam::sensor::ImuData{
            time, 
            ::infinityslam::common::ToSeconds(time),
            sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
            sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});

    csm_lio_->AddSensorData(
        sensor_id, 
        ::infinityslam::sensor::ImuData{time, 
            ::infinityslam::common::ToSeconds(time),
            sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
            sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
    return;
}

void CSMLioWrapper::HandleOdometryMessage(
    const std::string& sensor_id,
    const nav_msgs::Odometry::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(member_mutex_);
    LOG(INFO) << "## Received odometry msg with frame_id (" 
        << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";
    if (!sensor_samplers_->odometry_sampler.Pulse()) { return; }
    const ::infinityslam::common::Time time = FromRos(msg->header.stamp);
    const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
        time, CheckNoLeadingSlash(msg->child_frame_id));
    if (sensor_to_tracking == nullptr) {
        LOG(ERROR) << "## ERROR! Found no static tf between sensor frame (" 
            << CheckNoLeadingSlash(msg->child_frame_id) << ") and tracking frame (" 
            << ros_wrapper_Options_.tracking_frame << ").";
        return;
    }
    // 当前我们未启用轮速里程计。
    // csm_lio_->AddSensorData(sensor_id, 
    //     ::infinityslam::sensor::OdometryData{
    //         time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
    LOG(ERROR) << "## Odometry sensor not enabled.";

}

void CSMLioWrapper::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
    // LOG(INFO) << "## Received point cloud msg with frame_id (" 
    //     << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";

    std::lock_guard<std::mutex> lock(member_mutex_);
    if (!sensor_samplers_->rangefinder_sampler.Pulse()) {
        return;
    }
    ::infinityslam::sensor::PointCloudWithIntensities point_cloud;
    ::infinityslam::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
    if (!point_cloud.points.empty()) {
        CHECK_LE(point_cloud.points.back().time, 0.f);
    }
    const auto sensor_to_tracking =
        tf_bridge_->LookupToTracking(time, CheckNoLeadingSlash(msg->header.frame_id));
    if (sensor_to_tracking == nullptr) {
        LOG(ERROR) << "## ERROR! Found no static tf between sensor frame (" 
            << CheckNoLeadingSlash(msg->header.frame_id) << ") and tracking frame (" 
            << ros_wrapper_Options_.tracking_frame << ").";
        return;
    }

    // 保存原始点云数据到缓存队列
    {
        std::unique_lock<std::mutex> lock(postproc_mutex_);
        PointCloudXYZIT::Ptr msg_copy_ (new PointCloudXYZIT);
        *msg_copy_ = ToPointCloudXYZIT(*msg);
        auto it = point_cloud_queues_.find(sensor_id);
        if (it != point_cloud_queues_.end()) {
            auto& point_cloud_queue_ = *it->second;
            point_cloud_queue_.push_back(msg_copy_);
            while (point_cloud_queue_.size() > 10 && 
                (point_cloud_queue_.back()->timestamp_ 
                    - point_cloud_queue_.front()->timestamp_) 
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
            sensors_to_tracking_[CheckNoLeadingSlash(msg->header.frame_id)] 
                = *sensor_to_tracking;
        }
    }

    csm_lio_->AddSensorData(
        sensor_id, 
        ::infinityslam::sensor::TimedPointCloudData{
            time, 
            sensor_to_tracking->translation().cast<float>(),
            ::infinityslam::sensor::TransformTimedPointCloud(
                point_cloud.points, sensor_to_tracking->cast<float>()),
            point_cloud.intensities});
}

void CSMLioWrapper::HandleNavSatFixMessage(
    const std::string& sensor_id,
    const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(member_mutex_);
    LOG(INFO) << "## Received NavSat msg with frame_id (" 
        << msg->header.frame_id << "), stamp (" << msg->header.stamp << ").";
    if (!sensor_samplers_->fixed_frame_pose_sampler.Pulse()) {
        return;
    }
    // map_builder_bridge_.sensor_bridge(trajectory_id)
    //     ->HandleNavSatFixMessage(sensor_id, msg);
}

::ros::NodeHandle* CSMLioWrapper::node_handle() { return &node_handle_; }

std::set<infinityslam::csmlio::SensorId>
CSMLioWrapper::ComputeExpectedSensorIds(const CSMLioWraPperOptions& options) const 
{
    using SensorId = infinityslam::csmlio::SensorId;
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

void CSMLioWrapper::LaunchSubscribers(const CSMLioWraPperOptions& options) {
    // PointCloud2
    for (const std::string& topic :
        GenerateRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
        subscribers_.push_back(
            {SubscribeWithHandler<sensor_msgs::PointCloud2>(
                &CSMLioWrapper::HandlePointCloud2Message, 
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
            &CSMLioWrapper::HandleImuMessage,
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
                &CSMLioWrapper::HandleOdometryMessage,
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
                &CSMLioWrapper::HandleNavSatFixMessage, 
                kNavSatFixTopic,
                &node_handle_, 
                this),
            kNavSatFixTopic});
        LOG(INFO) << "## Registered callback for topic " 
            << subscribers_.back().topic << ".";
    }

}

void CSMLioWrapper::AddSensorSamplers(const CSMLioWraPperOptions& options) {
    sensor_samplers_ = boost::make_unique<TrajectorySensorSamplers>(
        options.rangefinder_sampling_ratio,
        options.odometry_sampling_ratio,
        options.imu_sampling_ratio,
        options.fixed_frame_pose_sampling_ratio/*GNSS/RTK采样器*/);
}

/// 发布tracking系的实时tf，发布实时配准点云。
void CSMLioWrapper::PublishLioResultData(const ::ros::TimerEvent& timer_event) {
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
            ros_wrapper_Options_.published_frame)};


    // 发布publish系的静态tf，还是发布tracking系的静态tf？ —— 后者吧。
    const Rigid3d tracking_to_local_3d = lio_keyframe_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
        if (ros_wrapper_Options_.publish_frame_projected_to_2d) {
            return ::infinityslam::transform::Embed3D(
                ::infinityslam::transform::Project2D(tracking_to_local_3d));
        }
        return tracking_to_local_3d;
    }();
    const Rigid3d tracking_to_map = tracking_to_local; //单轨迹，local即map。
    geometry_msgs::TransformStamped stamped_transform;    
    stamped_transform.header.stamp = ToRos(lio_keyframe_data->time);
    stamped_transform.header.frame_id = infinityslam_ros::options::map_frame;
    stamped_transform.child_frame_id = ros_wrapper_Options_.tracking_frame;
    stamped_transform.transform = ToGeometryMsgTransform(tracking_to_map);
    tf_broadcaster_.sendTransform(stamped_transform);

    /*
    auto& trajectory_data = slam_keyframe_data; //别名
    if (trajectory_data.published_to_tracking != nullptr) {
        if (infinityslam_ros::options::publish_to_tf) {
            if (trajectory_data.trajectory_options.provide_odom_frame) {
                std::vector<geometry_msgs::TransformStamped> stamped_transforms;
                //
                stamped_transform.header.frame_id = infinityslam_ros::options::map_frame;
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
                stamped_transform.header.frame_id = infinityslam_ros::options::map_frame;
                stamped_transform.child_frame_id =
                    trajectory_data.trajectory_options.published_frame;
                stamped_transform.transform = ToGeometryMsgTransform(
                    tracking_to_map * (*trajectory_data.published_to_tracking));
                tf_broadcaster_.sendTransform(stamped_transform);
            }
        }
        if (infinityslam_ros::options::publish_tracked_pose) {
            ::geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = infinityslam_ros::options::map_frame;
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
        pc2_msg.header.frame_id = infinityslam_ros::options::map_frame;
        scan_matched_point_cloud_publisher_.publish(pc2_msg);
    }

}

/// 发布lio/slam轨迹。
void CSMLioWrapper::PublishSlamTrajectory(const ::ros::WallTimerEvent& timer_event) {
    if (slam_trajectory_path_publisher_.getNumSubscribers() > 0) {
        std::lock_guard<std::mutex> lock(member_mutex_);
        const auto& keyframes = csm_lio_->GetSlamKeyframeList();
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = infinityslam_ros::options::map_frame;
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

    if (slam_interp_traj_publisher_.getNumSubscribers() > 0) {
        //
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.orientation.w = 1;
        odom_msg.pose.pose.position.x = 1;

        // odom_msg.pose



        // slam_interp_traj_publisher_.publish(odom_msg);
    }


}

/// 发布当前active submaps。
void CSMLioWrapper::PublishSubmapList(const ::ros::WallTimerEvent& timer_event) {
    if (submap_list_publisher_.getNumSubscribers() == 0) return;
    // if ()
    std::lock_guard<std::mutex> lock(member_mutex_);
    submap_ros_pc2_ = 
        FromPointCloudToRosPointCloud2(csm_lio_->GetActiveSubmapCloudsInOne());
    submap_ros_pc2_.header.stamp = ros::Time::now();
    submap_ros_pc2_.header.frame_id = infinityslam_ros::options::map_frame;
    submap_list_publisher_.publish(submap_ros_pc2_);
}

/// 发布点云形式的全局地图
void CSMLioWrapper::PublishGlobalMapPC(const ::ros::WallTimerEvent& timer_event) {
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
    std::lock_guard<std::mutex> lock(member_mutex_);
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
    using ::infinityslam::sensor::PointTypeXYZ;
    std::vector<PointTypeXYZ> global_map_points;
    std::vector<float> global_map_intensities;
    std::vector<PointTypeXYZ> temp_submap_points;
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
                auto filtered_temp_submap = ::infinityslam::sensor::VoxelFilter(
                    ::infinityslam::sensor::PointCloud(temp_submap_points,temp_submap_intensities), 
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
        auto filtered_temp_submap = ::infinityslam::sensor::VoxelFilter(
            ::infinityslam::sensor::PointCloud(temp_submap_points,temp_submap_intensities), 
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
    ::infinityslam::sensor::PointCloud global_map(global_map_points, global_map_intensities);

    global_map_ros_pc2_ = FromPointCloudToRosPointCloud2(global_map);
    global_map_ros_pc2_.header.stamp = ros::Time::now();
    global_map_ros_pc2_.header.frame_id = infinityslam_ros::options::map_frame;
    slam_global_map_publisher_.publish(global_map_ros_pc2_);
    LOG(INFO) << "## Published global map, info: " 
        << keyframes.size() << " keframes, "
        << global_map_ros_pc2_.width << " points.";

}

bool CSMLioWrapper::ValidateTopicNames(const CSMLioWraPperOptions& options) {
    for (const auto& sensor_id : expected_sensor_ids_) {
        const std::string& topic = sensor_id.id;
        if (subscribed_topics_.count(topic) > 0) {
        LOG(ERROR) << "## Topic name [" << topic << "] is already used.";
        return false;
        }
    }
    return true;
}

void CSMLioWrapper::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& timer_event) {
    std::lock_guard<std::mutex> lock(member_mutex_);
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

void CSMLioWrapper::OnLioResult(
    const ::infinityslam::common::Time time,
    const Rigid3d local_pose,
    ::infinityslam::sensor::RangeData range_data_in_local,
    std::shared_ptr<::infinityslam::sensor::PointCloud> point_cloud,
    std::unique_ptr<const ::infinityslam::csmlio::InsertionResult> insertion_result) 
{
    // 先做简单的事
    pose_interpolator_->AddPose(time, local_pose);

    // 触发后处理线程
    new_kf_flag_.store(true);
    cv_.notify_one();

    std::shared_ptr<::infinityslam::sensor::PointCloud> point_cloud_;
    if (point_cloud != nullptr) {
        // 对于指针对象，我们必须深拷贝数据，避免wrapper层和core层访问同一块数据。
        point_cloud_.reset(
            new ::infinityslam::sensor::PointCloud(point_cloud->points(), 
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
}

void CSMLioWrapper::PerformPostProcKfLoop()
{
    while (!quit_postproc_flag_) {

        /** 【C++条件变量】：
         * conditional_variable的wait函数只接受unique_lock类型的锁，这种锁可以在持有互斥量的同时而不锁住互斥量；
         * conditional_variable的wait函数会让当前线程阻塞在这里，直到notified；（但如果指定了谓词，则只有谓词返回false时才会阻塞）
         * wait()函数在阻塞当前线程的同时，会解锁unique_lock对象所持有的mutex，以便其它线程能够访问mutex；
         * wait()函数在被notified时，会尝试锁住unique_lock对象所持有的mutex，然后向下执行；（如果指定了谓词，还需要谓词返回true）
         * wait()函数只有在满足mutex被lock，且lambda表达时为true时，才会结束等待（结束阻塞），允许当前线程继续往下执行。
         * 详情参见C++官网：https://cplusplus.com/reference/condition_variable/condition_variable/wait/
        */

        std::unique_lock<std::mutex> lock(postproc_mutex_);
        cv_.wait(lock, [this](){return new_kf_flag_.load();});
        new_kf_flag_.store(false);
        // LOG(INFO) << "PostProc: triggered. ";

        if (pose_interpolator_->empty()) {
            LOG(WARNING) << "PostProc: pose interpolator is empty.";
            continue;
        }

        if (!last_processed_kf_time_.has_value()) {
            if (!pose_interpolator_->empty()) {
                last_processed_kf_time_ = pose_interpolator_->latest_time();
                assert(last_processed_kf_time_ > 0);
            }
            LOG(INFO) << "PostProc: init 'last processed kf time'.";
            continue; // 首帧不做处理，仅记录时间。
        }

        if (last_processed_kf_time_ >= pose_interpolator_->latest_time()) {
            LOG(WARNING) << "PostProc: pose interpolator waiting to be updated.";
            continue;
        }

        const double kf_st_timestamp_ = last_processed_kf_time_.value();
        const double kf_ed_timestamp_ = pose_interpolator_->latest_time();
        using namespace ::infinityslam::common;
        const Time kf_st_time_ = Time(FromSeconds(kf_st_timestamp_));
        const Time kf_ed_time_ = Time(FromSeconds(kf_ed_timestamp_));

        // 获取时间段内的PointCloud2数据，不同LiDAR的msg都保存在同一个容器中即可。
        // 对每一个点云队列，找出所有与“[kf_st_timestamp_, kf_ed_timestamp_]时段”
        // 有overlap的msg，作为处理的对象。
        pc2_data_segment_.clear();
        for (auto& pair : point_cloud_queues_) {
            auto& pc2_data_queue = *(pair.second);
            int pop_to_here = -1;
            for (size_t i = 0; i < pc2_data_queue.size(); ++i) {
                auto pc2_st_time = pc2_data_queue[i]->begin_time();
                auto pc2_ed_time = pc2_data_queue[i]->end_time();
                if (pc2_ed_time < kf_ed_timestamp_) pop_to_here = i;
                if (pc2_st_time < kf_ed_timestamp_ && pc2_ed_time > kf_st_timestamp_) {
                    pc2_data_segment_.push_back(pc2_data_queue[i]);
                }
            }
            if (pop_to_here >= 0) {
                for (int i=0; i<=pop_to_here; ++i) {
                    pc2_data_queue.pop_front();
                }
            }
        }
        if (pc2_data_segment_.empty()) {
            continue; //在当前时间段内找不到点云帧。
        }
        LOG(INFO) << "PostProc: found " << pc2_data_segment_.size() 
            << " cloudframes overlapped with newest duration.";

        // 获取当前所有lidars到tracking的静态tf。
        lidars_in_tracking_.clear();
        for (auto pair : sensors_to_tracking_) {
            lidars_in_tracking_.push_back(pair);
        }

        // 拷贝数据结束，释放锁。
        lock.unlock();


        // 执行后处理
        bool interp_success = pose_interpolator_->LookUp(
            kf_st_timestamp_,
            kf_ed_timestamp_,
            kInterpTimeStep,
            interp_pose_table_);
        if (interp_success) {
            LOG(INFO) << "PostProc: Look up pose table success. (" 
                << interp_pose_table_.size() << "poses)";
        }

        auto AbsTimeToIndex = [&](double timestamp) -> int {
            return int((timestamp - kf_st_timestamp_) / kInterpTimeStep);
        };

        for (auto pair : lidars_in_tracking_) {
            std::string& registered_frame_id = pair.first;
            auto& lidar_in_tracking = pair.second;
            LOG(INFO) << "PostProc: going to procress frame_id " << registered_frame_id;
            // 由tracking系的位姿表，算出lidar系的位姿表
            std::vector<::infinityslam::transform::Rigid3d> lidar_pose_table;
            for (auto& t_pose : interp_pose_table_) {
                lidar_pose_table.push_back(t_pose.pose * lidar_in_tracking);
            }
            if (lidar_pose_table.empty()) {
                LOG(WARNING) << "PostProc: pose table empty (frame_id=" 
                    << registered_frame_id << "fatal error!";
                continue;
            }
            // 把pose转换到末尾时刻
            const auto ed_pose_inverse = lidar_pose_table.back().inverse();
            for (auto& pose_it : lidar_pose_table) {
                pose_it = ed_pose_inverse * pose_it;
            }
            // 找出所有frame_id一致的PC2消息；解析PC2点云格式为可处理格式；统一转换到end时间点的lidar位姿下；发布。
            ::infinityslam::sensor::PointCloudXYZIT merged_point_cloud;
            int num_merged_kfs = 0;
            for (auto& pc2_ : pc2_data_segment_) {
                if (CheckNoLeadingSlash(pc2_->frame_id_) == registered_frame_id) {
                    ++num_merged_kfs;
                    int overlapped_counts = 0;
                    int full_cloud_counts = pc2_->points_.size();
                    for (auto& point : pc2_->points_) {
                        const double point_time = point.time + pc2_->timestamp_;
                        if (point_time >= kf_st_timestamp_ && point_time < kf_ed_timestamp_) {
                            ++overlapped_counts;
                        }
                        int i = AbsTimeToIndex(point_time);
                        if (i >= 0 && i < lidar_pose_table.size()) {
                            point.position = lidar_pose_table[i].cast<float>() * point.position;
                            point.time = 0;
                            merged_point_cloud.points_.push_back(point);
                        }
                    }
                    pc2_->timestamp_ = kf_ed_timestamp_;
                    pc2_->time_ = kf_ed_time_;
                    LOG(INFO) << "PostProc: --- overlapped points: " << overlapped_counts << " / " << full_cloud_counts; 
                }
                merged_point_cloud.seq_ = pc2_->seq_;
                merged_point_cloud.frame_id_ = pc2_->frame_id_;
            }
            LOG(INFO) << "PostProc: merged " << num_merged_kfs 
                << " cloudframes, collected " << merged_point_cloud.size() << " points.";
            if (!merged_point_cloud.empty()) {
                merged_point_cloud.timestamp_ = kf_ed_timestamp_;
                merged_point_cloud.time_ = kf_ed_time_;
                auto deskewed_pc2 = ToPointCloud2Message(merged_point_cloud);
                slam_postproc_pointcloud_publisher_.publish(deskewed_pc2);
                // LOG(INFO) << "PostProc: merged " << num_merged_kfs 
                //     << " cloudframes, collected " << merged_point_cloud.size() << " points.";
            } 
            else {
                // LOG(INFO) << "PostProc: merged no points for liar frame_id = " << registered_frame_id;
            }
        }

        // 执行完毕，结束本次循环。
        last_processed_kf_time_ = kf_ed_timestamp_;
        // new_kf_flag_.store(false);
        ++postproc_kf_id;
        LOG(INFO) << "PostProc: took XXX ms, advanced to " << ToRos(kf_ed_time_);
        LOG(INFO) << "";
        // LOG(INFO) << "PostProc: took XXX ms, merged XXX points, advanced to time point " << ToRos(kf_ed_time_);
        // LOG(INFO) << "PostProc: took XXX ms, merged XXX points, processed kf_id " << postproc_kf_id << ".";
    }
}


}  // namespace infinityslam_ros
