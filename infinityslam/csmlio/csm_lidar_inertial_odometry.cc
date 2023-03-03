/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
*/

#include <memory>
#include <iomanip>
#include <unordered_set>
#include "boost/make_unique.hpp"
#include "glog/logging.h"

#include "infinityslam/csmlio/csm_lidar_inertial_odometry.h"

#include "infinityslam/common/time.h"
#include "infinityslam/common/system_options.h"
#include "infinityslam/sensor/internal/dispatchable.h"
#include "infinityslam/transform/timestamped_transform.h"
#include "infinityslam/csmlio/scan_matching/rotational_scan_matcher.h"

namespace infinityslam {
namespace csmlio {

// using namespace ::infinityslam::csmlio;

namespace {

// using SensorId = mapp ing::SensorId;

std::vector<std::string> SelectRangeSensorIds(
    const std::set<SensorId>& expected_sensor_ids) 
{
    std::vector<std::string> range_sensor_ids;
    for (const SensorId& sensor_id : expected_sensor_ids) {
        if (sensor_id.type == SensorId::SensorType::RANGE) {
        range_sensor_ids.push_back(sensor_id.id);
        }
    }
    return range_sensor_ids;
}

} // namespace

CSMLioOptions ReadCSMLioOptions() {
    CSMLioOptions options;

    options.collate_fixed_frame = common::options::csmlio::collate_fixed_frame;
    options.collate_landmarks = common::options::csmlio::collate_landmarks;

    options.min_range = common::options::csmlio::min_range;
    options.max_range = common::options::csmlio::max_range;
    options.num_accumulated_range_data = common::options::csmlio::num_accumulated_range_data;
    options.voxel_filter_size = common::options::csmlio::voxel_filter_size;

    options.high_res_adap_vf_max_length = common::options::csmlio::high_res_adap_vf_max_length;
    options.high_res_adap_vf_min_num_points = common::options::csmlio::high_res_adap_vf_min_num_points;
    options.high_res_adap_vf_max_range = common::options::csmlio::high_res_adap_vf_max_range;

    options.low_res_adap_vf_max_length = common::options::csmlio::low_res_adap_vf_max_length;
    options.low_res_adap_vf_min_num_points = common::options::csmlio::low_res_adap_vf_min_num_points;
    options.low_res_adap_vf_max_range = common::options::csmlio::low_res_adap_vf_max_range;

    options.use_online_correlative_scan_matching = common::options::csmlio::use_online_correlative_scan_matching;

    options.rotational_histogram_size = common::options::csmlio::rotational_histogram_size;

    options.use_intensities = common::options::csmlio::use_intensities;

    return options;
}

utils::MotionFIlterOptions ReadCSMLioMotionFIlterOptions() {
    utils::MotionFIlterOptions options;
    options.max_time_seconds = common::options::csmlio::motion_filter_max_time_seconds;
    options.max_distance_meters = common::options::csmlio::motion_filter_max_distance_meters;
    options.max_angle_radians = common::options::csmlio::motion_filter_max_angle_radians;
    return options;
}

CSMLidarInertialOdometry::CSMLidarInertialOdometry(
    const CSMLioOptions& csm_lio_options,
    const std::set<SensorId>& expected_sensor_ids,
    csmlio::LioResultCallback lio_result_callback)
    : csm_lio_Options_(csm_lio_options)
    , active_submaps_(csmlio::ReadActiveSubmaps3DOptions())
    , motion_filter_(ReadCSMLioMotionFIlterOptions())
    , ceres_scan_matcher_(csmlio::scan_matching::CreateCeresScanMatcher3D())
    , range_data_collator_(SelectRangeSensorIds(expected_sensor_ids))
    , lio_result_callback_(lio_result_callback)
    , last_logging_time_(std::chrono::steady_clock::now())
    , kPoseQueueDuration(common::FromSeconds(5.0))
{
    submap_cloud_growing_.reset(new sensor::PointCloud);
    submap_cloud_working_.reset(new sensor::PointCloud);
    submap_cloud_all_.reset(new sensor::PointCloud);
    if (csm_lio_Options_.use_online_correlative_scan_matching) {
        real_time_correlative_scan_matcher_ = 
            csmlio::scan_matching::CreateRealTimeCorrelativeScanMatcher3D();
    }

    sensor_collator_ = boost::make_unique<sensor::Collator>(); 
    std::unordered_set<std::string> expected_sensor_id_strings;
    for (const auto& sensor_id : expected_sensor_ids) {
        if (sensor_id.type == SensorId::SensorType::LANDMARK) { continue; }
        if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE) { continue; }
        expected_sensor_id_strings.insert(sensor_id.id);
    }
    sensor_collator_->AddTrajectory(
        trajectory_id_, expected_sensor_id_strings,
        [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
            HandleCollatedData(sensor_id, std::move(data));
        });
}

CSMLidarInertialOdometry::~CSMLidarInertialOdometry()
{
    sensor_collator_->FinishTrajectory(trajectory_id_);
}

void CSMLidarInertialOdometry::AddSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) 
{
    AddDataToCollator(
        sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
}

void CSMLidarInertialOdometry::AddSensorData(
    const std::string& sensor_id,
    const sensor::ImuData& imu_data) 
{
    AddDataToCollator(sensor::MakeDispatchable(sensor_id, imu_data));
}

void CSMLidarInertialOdometry::AddSensorData(
    const std::string& sensor_id,
    const sensor::OdometryData& odometry_data) 
{
    AddDataToCollator(sensor::MakeDispatchable(sensor_id, odometry_data));
}

void CSMLidarInertialOdometry::AddDataToCollator(
    std::unique_ptr<sensor::Data> data) 
{
    sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

void CSMLidarInertialOdometry::HandleCollatedData(
    const std::string& sensor_id,
    std::unique_ptr<sensor::Data> data)
{
    // 临时日志，完删。
    const int kLogEveryNTimes = 100;
    static int counts = 0;
    if ((counts % kLogEveryNTimes) == 0) {
        // LOG(INFO) << "Sampled info: received one msg from `Collator` with sensor_id " 
        //     << data->GetSensorId() << ".";
    }
    ++counts;

    // 冠华：我们用虚基类（多态性）解决了【dispatchable.h】与
    // 【csm_lidar_inertial_odometry.h】交叉包含的问题；
    // 现在数据可以从sensor::Data中进到CSMLidarInertialOdometry的回调了，
    // 数据流通正常，终于可以睡觉了！
    data->AddToLIO(this);
}

void CSMLidarInertialOdometry::ProcessSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data)
{
    // LOG(INFO) << "Received point cloud data from collator. (sensor_id:" << sensor_id << ")";
    ++count_received_range_data;
    const auto proc_point_cloud_start = std::chrono::steady_clock::now();

    // 也许，这里相当于GlobalTrajBuilder层。
    LogSensorDataRate(sensor_id, timed_point_cloud_data.time);

    // 开始处理。
    auto matching_result = 
        AddRangeData(sensor_id, timed_point_cloud_data);

    if (matching_result == nullptr) {
        // The range data has not been fully accumulated yet.
        return;
    }
    ++count_lio_matching_results;

    std::unique_ptr<InsertionResult> insertion_result;
    if (matching_result->insertion_result != nullptr) {
        ++count_lio_insertion_results;
        insertion_result = boost::make_unique<InsertionResult>(
            InsertionResult{
                matching_result->insertion_result->constant_data,
                std::vector<std::shared_ptr<const Submap3D>>(
                    matching_result->insertion_result->insertion_submaps.begin(),
                    matching_result->insertion_result->insertion_submaps.end())});
        // 保存关键帧信息
        std::lock_guard<std::mutex> lock(mutex_);
        int node_id = slam_keyframes_data_.empty() ? 0
                        : slam_keyframes_data_.back().node_id + 1;
        slam_keyframes_data_.push_back(
            TrajectoryNode{
                node_id, 
                matching_result->insertion_result->constant_data,
                matching_result->insertion_result->constant_data->local_pose});
        // 单独保存关键帧位姿队列
        timed_pose_queue_.push_back(transform::TimedPose(
            matching_result->insertion_result->constant_data->time, 
            common::ToSeconds(matching_result->insertion_result->constant_data->time), 
            matching_result->insertion_result->constant_data->local_pose));
        while (timed_pose_queue_.size() > 2 &&
            timed_pose_queue_[1].time <= timed_pose_queue_.back().time - kPoseQueueDuration) 
        {
            timed_pose_queue_.pop_front();
        }
        // 更新active submaps 更新标志位。(submap点云只在需要时自取)
        active_submaps_updated = true;
    }

    if (lio_result_callback_) {
        lio_result_callback_(
            matching_result->time, 
            matching_result->local_pose,
            std::move(matching_result->range_data_in_local),
            matching_result->point_cloud /* any point cloud or nullptr */,
            std::move(insertion_result));
    }

    const auto proc_point_cloud_end = std::chrono::steady_clock::now();
    const double proc_point_cloud_duration = 
        common::ToSeconds(proc_point_cloud_end - proc_point_cloud_start);
    ++count_processed_range_data;
    life_long_processing_time += proc_point_cloud_duration;
    const double avg_duration = life_long_processing_time 
                                / count_processed_range_data;
    duration_keeper_.AddDuration(proc_point_cloud_duration);

    // 在出现新关键帧时，打印日志。
    if (matching_result->insertion_result != nullptr) {
        LOG(INFO) << "## Lio pipeline took [" << std::fixed << std::setprecision(3)
            << proc_point_cloud_duration << "s, avg " 
            << duration_keeper_.GetRecentAvgDuration() 
            << "s], inserted [kf_id=" << slam_keyframes_data_.back().node_id
            << "].";
    }

}

void CSMLidarInertialOdometry::ProcessSensorData(
    const std::string& sensor_id,
    const sensor::ImuData& imu_data)
{
    // LOG(INFO) << "Received imu data from collator. (sensor_id:" << sensor_id << ")";
    LogSensorDataRate(sensor_id, imu_data.time);

    if (extrapolator_ != nullptr) {
        extrapolator_->AddImuData(imu_data);
        return;
    }

    /// 以下为构造 (ImuBased)PoseExtrapolator的代码，我们可以在此之前输入先验位置和IMU数据。

    // 这里的意思是：如果我们已经积累/或已知了一段数据，则可以用这段数据初始化
    // IMU的bias等信息 —— 注意这种做法仅对ImuBasedPoseExtrapolator有效；
    // 对于普通的PoseExtrapolator，仅会使用最有一个IMU消息用于初始化重力方向，
    // 仅此而已，此时下面的这些操作是没有意义的。
    std::vector<transform::TimestampedTransform> initial_poses;
    // for (const auto& pose_proto : local_traj_builder_Options_.initial_poses()) {
    //     initial_poses.push_back(transform::FromProto(pose_proto));
    // }
    std::vector<sensor::ImuData> initial_imu_data;
    // for (const auto& imu : local_traj_builder_Options_.initial_imu_data()) {
    //     initial_imu_data.push_back(sensor::FromProto(imu));
    // }
    initial_imu_data.push_back(imu_data);

    // 构造&初始化对象。
    extrapolator_ = csmlio::PoseExtrapolatorInterface::CreateWithImuData(
        initial_imu_data, initial_poses, /*init_with_common_options=*/ true);

}

void CSMLidarInertialOdometry::ProcessSensorData(
    const std::string& sensor_id,
    const sensor::OdometryData& odometry_data)
{
    LogSensorDataRate(sensor_id, odometry_data.time);
    if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator we cannot add odometry data.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return;
    }
    extrapolator_->AddOdometryData(odometry_data);
}

const std::deque<TrajectoryNode>& 
CSMLidarInertialOdometry::GetSlamKeyframeList() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return slam_keyframes_data_;
}

const std::deque<transform::TimedPose>& 
CSMLidarInertialOdometry::GetTimedPoseQueue() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return timed_pose_queue_;
}

// 获取当前 active submaps 的点云（各submap间独立输出）。
// 以点云形式表示3Dsubmap，点表示cell的中心位置。
// 该函数会在必要时更新submap点云，可以作为更新submap点云的函数被调用。
std::vector<std::shared_ptr<const sensor::PointCloud>> 
CSMLidarInertialOdometry::GetActiveSubmapCloudsList() {
    if (!active_submaps_updated) {
        return std::vector<std::shared_ptr<const sensor::PointCloud>> {};
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<const Submap3D>> active_submaps = 
        active_submaps_.submaps();
    // 默认前端中只保留两个submap，所以我们也只用两个。
    if (active_submaps.size() >= 1) {
        auto& submap_working_ = active_submaps[0];
        if (submap_working_->ToPointCloud(submap_working_->local_pose(), submap_cloud_working_, false)) {
            // 无需二合一，把working的submap的点云数据拷贝一下
            submap_cloud_all_->clear();
            *submap_cloud_all_ = *submap_cloud_working_;
            LOG(INFO) << "Updated submap list: working submap cloud size " 
                << submap_cloud_working_->size() << ".";
            // done.
            active_submaps_updated = false;
            return std::vector<std::shared_ptr<const sensor::PointCloud>> {
                submap_cloud_working_};
        }
    }
    // if (active_submaps.size() >= 2) {
    //     auto& submap_working_ = active_submaps[0];
    //     auto& submap_growing_ = active_submaps[1];
    //     if (submap_working_->ToPointCloud(submap_working_->local_pose(), submap_cloud_working_, false)
    //         && submap_growing_->ToPointCloud(submap_growing_->local_pose(), submap_cloud_growing_, false)) {
    //         // 把两个submap的点云二合一
    //         submap_cloud_all_->clear();
    //         auto& points1 = submap_cloud_working_->points();
    //         auto& intensities1 = submap_cloud_working_->intensities();
    //         if (points1.size() == intensities1.size()) {
    //             for (size_t i=0; i<points1.size(); ++i) {
    //                 submap_cloud_all_->push_back(points1[i], intensities1[i]);
    //             }
    //         }
    //         auto& points2 = submap_cloud_growing_->points();
    //         auto& intensities2 = submap_cloud_growing_->intensities();
    //         if (points2.size() == intensities2.size()) {
    //             for (size_t i=0; i<points2.size(); ++i) {
    //                 submap_cloud_all_->push_back(points2[i], intensities2[i]);
    //             }
    //         }
    //         // done.
    //         active_submaps_updated = false;
    //         return std::vector<std::shared_ptr<const sensor::PointCloud>> {
    //             submap_cloud_working_, submap_cloud_growing_};
    //     }
    // }
    return std::vector<std::shared_ptr<const sensor::PointCloud>> {};
}

// A版功能：获取当前 active submaps 的点云（各submap的点云叠加输出）
// B版功能：获取当前 working submap 的点云。
// （以点云形式表示3Dsubmap，点表示cell的中心位置）
const sensor::PointCloud& 
CSMLidarInertialOdometry::GetActiveSubmapCloudsInOne() {
    GetActiveSubmapCloudsList(); //在有必要时更新点云
    return *submap_cloud_all_;
}

void CSMLidarInertialOdometry::LogSensorDataRate(
    const std::string& sensor_id, const common::Time& data_stamp)
{
    auto it = rate_timers_.find(sensor_id);
    if (it == rate_timers_.end()) {
        it = rate_timers_
                .emplace(
                    std::piecewise_construct, 
                    std::forward_as_tuple(sensor_id),
                    std::forward_as_tuple(
                        common::FromSeconds(kSensorDataRatesPeriodSeconds)))
                .first;
    }
    it->second.Pulse(data_stamp);

    if (std::chrono::steady_clock::now() - last_logging_time_ >
        common::FromSeconds(kSensorDataRatesPeriodSeconds)) {
        for (const auto& pair : rate_timers_) {
        LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
        }
        last_logging_time_ = std::chrono::steady_clock::now();
        LogLioStatus();
    }
}

void CSMLidarInertialOdometry::LogLioStatus()
{
    LOG(INFO) << "lio system status: " << "\n"
        << " ** ************************* ** " << "\n"
        << " ** received pointcloud msgs  : " << count_received_range_data << "\n"
        << " ** lio matching times        : " << count_lio_matching_results << "\n"
        << " ** lio insertions times      : " << count_lio_insertion_results << "\n"
        << " ** lio keyframes size        : " << slam_keyframes_data_.size() << "\n"
        << " ** item_to_show  : " << 0 << "\n"
        << " ** item_to_show  : " << 0 << "\n"
        << " ** ************************* ** ";
}

std::unique_ptr<csmlio::MatchingResult>
CSMLidarInertialOdometry::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) 
{
    if (csm_lio_Options_.use_intensities) {
        CHECK_EQ(unsynchronized_data.ranges.size(),
                unsynchronized_data.intensities.size())
            << "Passed point cloud has inconsistent number of intensities and "
            "ranges.";
    }

    // [WGH] step01 点云数据按时间戳区间重新整理 
    auto synchronized_data =
        range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
    if (synchronized_data.ranges.empty()) {
        LOG(INFO) << "Range data collator filling buffer.";
        return nullptr;
    }

    if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator with our first IMU message, we
        // cannot compute the orientation of the rangefinder.
        LOG(INFO) << "IMU not yet initialized.";
        return nullptr;
    }

    CHECK(!synchronized_data.ranges.empty());
    CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
    const common::Time time_first_point =
        synchronized_data.time +
        common::FromSeconds(synchronized_data.ranges.front().point_time.time);
    if (time_first_point < extrapolator_->GetLastPoseTime()) {
        LOG(INFO) << "Extrapolator is still initializing.";
        return nullptr;
    }

    if (num_accumulated_ == 0) {
        accumulated_point_cloud_origin_data_.clear();
    }

    // [WGH] step02 第一次体素降采样，用0.5×15cm的体素尺寸 
    synchronized_data.ranges = sensor::VoxelFilter(
        synchronized_data.ranges, 0.5f * csm_lio_Options_.voxel_filter_size);
    accumulated_point_cloud_origin_data_.emplace_back(
        std::move(synchronized_data));
    ++num_accumulated_;

    if (num_accumulated_ < csm_lio_Options_.num_accumulated_range_data) {
        LOG(INFO) << "Haven't acumulated enough range data. (" 
            << num_accumulated_ << "vs" 
            << csm_lio_Options_.num_accumulated_range_data << ")";
        return nullptr;
    }
    num_accumulated_ = 0;

    // [WGH] step03 获取所有激光点的时间戳列表 
    bool warned = false;
    std::vector<common::Time> hit_times;
    common::Time prev_time_point = extrapolator_->GetLastExtrapolatedTime();
    size_t accumulated_point_cloud_size = 0;
    for (const auto& point_cloud_origin_data :
        accumulated_point_cloud_origin_data_) {
        for (const auto& hit : point_cloud_origin_data.ranges) {
            ++accumulated_point_cloud_size;
            common::Time time_point = point_cloud_origin_data.time +
                                        common::FromSeconds(hit.point_time.time);
            if (time_point < prev_time_point) {
                if (!warned) {
                LOG(ERROR) << "Timestamp of individual range data point jumps "
                                "backwards from "
                            << prev_time_point << " to " << time_point;
                warned = true;
                }
                time_point = prev_time_point;
            }

            hit_times.push_back(time_point);
            prev_time_point = time_point;
        }
    }
    hit_times.push_back(accumulated_point_cloud_origin_data_.back().time);

    // [WGH] step04 查询每个激光点时刻的位姿预测值 
    const PoseExtrapolatorInterface::ExtrapolationResult extrapolation_result =
        extrapolator_->ExtrapolatePosesWithGravity(hit_times);
    bool debugging = false;
    if (debugging) {
        // Debug "Terminating: Residual and Jacobian evaluation failed." issue.
        LOG(INFO) << "Print extrapolation_result info: \n"
            << "    hit_times.front() is " << hit_times.front() << "\n"
            << "    hit_times.back() is " << hit_times.back() << "\n"
            << "    accumulated_point_cloud_size is " << accumulated_point_cloud_size << "\n"
            << "    hit_times size is " << hit_times.size() << "\n"
            << "    previous_poses size is " << extrapolation_result.previous_poses.size() << "\n"
            << "    current_pose = " << extrapolation_result.current_pose.translation().transpose() << "\n"
            << "    current_velocity = " << extrapolation_result.current_velocity.transpose() << ".";
    }
    std::vector<transform::Rigid3f> hits_poses(
        std::move(extrapolation_result.previous_poses));
    hits_poses.push_back(extrapolation_result.current_pose.cast<float>());
    CHECK_EQ(hits_poses.size(), hit_times.size());

    // [WGH] step05 按预测位姿对每个点去畸变，并根据range区分hits/misses点 
    const size_t max_possible_number_of_accumulated_points = hit_times.size();
    std::vector<sensor::PointTypeXYZ> accumulated_points;
    std::vector<float> accumulated_intensities;
    accumulated_points.reserve(max_possible_number_of_accumulated_points);
    if (csm_lio_Options_.use_intensities) {
        accumulated_intensities.reserve(max_possible_number_of_accumulated_points);
    }
    sensor::PointCloud misses;
    std::vector<transform::Rigid3f>::const_iterator hits_poses_it =
        hits_poses.begin();
    for (const auto& point_cloud_origin_data :
        accumulated_point_cloud_origin_data_) {
        for (const auto& hit : point_cloud_origin_data.ranges) {
        const Eigen::Vector3f hit_in_local =
            *hits_poses_it * hit.point_time.position;
        const Eigen::Vector3f origin_in_local =
            *hits_poses_it * point_cloud_origin_data.origins.at(hit.origin_index);
        const Eigen::Vector3f delta = hit_in_local - origin_in_local;
        const float range = delta.norm();
        if (range >= csm_lio_Options_.min_range) {
            if (range <= csm_lio_Options_.max_range) {
            accumulated_points.push_back(sensor::PointTypeXYZ{hit_in_local});
            if (csm_lio_Options_.use_intensities) {
                accumulated_intensities.push_back(hit.intensity);
            }
            } else {
            // We insert a ray cropped to 'max_range' as a miss for hits beyond
            // the maximum range. This way the free space up to the maximum range
            // will be updated.
            // TODO(wohe): since `misses` are not used anywhere in 3D, consider
            // removing `misses` from `range_data` and/or everywhere in 3D.
            misses.push_back(sensor::PointTypeXYZ{
                origin_in_local + csm_lio_Options_.max_range / range * delta});
            }
        }
        ++hits_poses_it;
        }
    }
    CHECK(std::next(hits_poses_it) == hits_poses.end());
    const sensor::PointCloud returns(std::move(accumulated_points),
                                    std::move(accumulated_intensities));

    const common::Time current_sensor_time = synchronized_data.time;
    common::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
        sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;

    // [WGH] step06 第二次体素降采样，按15cm体素大小，此时点云位于local坐标系下 
    const common::Time current_time = hit_times.back();
    const auto voxel_filter_start = std::chrono::steady_clock::now();
    const sensor::RangeData filtered_range_data = {
        extrapolation_result.current_pose.translation().cast<float>(),
        sensor::VoxelFilter(returns, csm_lio_Options_.voxel_filter_size),
        sensor::VoxelFilter(misses, csm_lio_Options_.voxel_filter_size)};
    const auto voxel_filter_stop = std::chrono::steady_clock::now();
    const auto voxel_filter_duration = voxel_filter_stop - voxel_filter_start;

    if (sensor_duration.has_value()) {
        const double voxel_filter_fraction =
            common::ToSeconds(voxel_filter_duration) /
            common::ToSeconds(sensor_duration.value());
        // kLocalSlamVoxelFilterFraction->Set(voxel_filter_fraction);
    }

    // [WGH] step07 以上6个步骤相当于点云预处理，现在开始真正的前端算法 
    return AddAccumulatedRangeData(
        current_time,
        sensor::TransformRangeData( /*将点云变换到点云帧时刻的tracking坐标系下*/
            filtered_range_data,
            extrapolation_result.current_pose.inverse().cast<float>()),
        sensor_duration, 
        extrapolation_result.current_pose,
        extrapolation_result.gravity_from_tracking);
}

std::unique_ptr<csmlio::MatchingResult>
CSMLidarInertialOdometry::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const common::optional<common::Duration>& sensor_duration,
    const transform::Rigid3d& pose_prediction,
    const Eigen::Quaterniond& gravity_alignment) 
{
    if (filtered_range_data_in_tracking.returns.empty()) {
        LOG(WARNING) << "Dropped empty range data.";
        return nullptr;
    }

    const auto scan_matcher_start = std::chrono::steady_clock::now();

    const sensor::PointCloud high_resolution_point_cloud_in_tracking =
        sensor::AdaptiveVoxelFilter(
            filtered_range_data_in_tracking.returns,
            csm_lio_Options_.high_resolution_adaptive_voxel_filter_options());
    if (high_resolution_point_cloud_in_tracking.empty()) {
        LOG(WARNING) << "Dropped empty high resolution point cloud data.";
        return nullptr;
    }
    const sensor::PointCloud low_resolution_point_cloud_in_tracking =
        sensor::AdaptiveVoxelFilter(
            filtered_range_data_in_tracking.returns,
            csm_lio_Options_.low_resolution_adaptive_voxel_filter_options());
    if (low_resolution_point_cloud_in_tracking.empty()) {
        LOG(WARNING) << "Dropped empty low resolution point cloud data.";
        return nullptr;
    }

    std::unique_ptr<transform::Rigid3d> pose_estimate =
        ScanMatch(pose_prediction, 
                    low_resolution_point_cloud_in_tracking,
                    high_resolution_point_cloud_in_tracking);

    if (pose_estimate == nullptr) {
        LOG(WARNING) << "Scan matching failed.";
        return nullptr;
    }
    extrapolator_->AddPose(time, *pose_estimate);

    const auto scan_matcher_stop = std::chrono::steady_clock::now();
    const auto scan_matcher_duration = scan_matcher_stop - scan_matcher_start;
    if (sensor_duration.has_value()) {
        const double scan_matcher_fraction =
            common::ToSeconds(scan_matcher_duration) /
            common::ToSeconds(sensor_duration.value());
        // kLocalSlamScanMatcherFraction->Set(scan_matcher_fraction);
    }

    sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
        filtered_range_data_in_tracking, pose_estimate->cast<float>());

    // wgh-- 把点云插入子图并监控耗时（性能）。
    const auto insert_into_submap_start = std::chrono::steady_clock::now();
    std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
        time, filtered_range_data_in_local, filtered_range_data_in_tracking,
        high_resolution_point_cloud_in_tracking,
        low_resolution_point_cloud_in_tracking, *pose_estimate,
        gravity_alignment);
    const auto insert_into_submap_stop = std::chrono::steady_clock::now();

    const auto insert_into_submap_duration =
        insert_into_submap_stop - insert_into_submap_start;
    if (sensor_duration.has_value()) {
        const double insert_into_submap_fraction =
            common::ToSeconds(insert_into_submap_duration) /
            common::ToSeconds(sensor_duration.value());
        // kLocalSlamInsertIntoSubmapFraction->Set(insert_into_submap_fraction);
    }

    // wgh-- log about time cost.
    // LOG(INFO) << "scan match duration " << std::fixed << std::setprecision(3) 
    //     << common::ToSeconds(scan_matcher_duration) 
    //     << "s, insertion duration " 
    //     << common::ToSeconds(insert_into_submap_duration) << "s.";

    const auto wall_time = std::chrono::steady_clock::now();
    if (last_wall_time_.has_value()) {
        const auto wall_time_duration = wall_time - last_wall_time_.value();
        // LOG(INFO) << "kLocalSlamLatencyMetric: " 
        //     << common::ToSeconds(wall_time_duration);
        if (sensor_duration.has_value()) {
            // LOG(INFO) << "kLocalSlamRealTimeRatio: " 
            //     << common::ToSeconds(sensor_duration.value()) /
            //                             common::ToSeconds(wall_time_duration);
        }
    }
    const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
    if (last_thread_cpu_time_seconds_.has_value()) {
        const double thread_cpu_duration_seconds =
            thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
        if (sensor_duration.has_value()) {
            // LOG(INFO) << "kLocalSlamCpuRealTimeRatio: " 
            //     << common::ToSeconds(sensor_duration.value()) /
            //         thread_cpu_duration_seconds;
        }
    }
    last_wall_time_ = wall_time;
    last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
    return boost::make_unique<MatchingResult>(MatchingResult{
        time, *pose_estimate, std::move(filtered_range_data_in_local),
        nullptr, /*any point cloud or nullptr*/
        std::move(insertion_result)});
}

std::unique_ptr<csmlio::InsertionResult>
CSMLidarInertialOdometry::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) 
{
    if (motion_filter_.IsSimilar(time, pose_estimate)) {
        return nullptr;
    }
    const Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity =
        csmlio::scan_matching::RotationalScanMatcher::ComputeHistogram(
            sensor::TransformPointCloud(
                filtered_range_data_in_tracking.returns,
                transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
            csm_lio_Options_.rotational_histogram_size);

    const Eigen::Quaterniond local_from_gravity_aligned =
        pose_estimate.rotation() * gravity_alignment.inverse();
    std::vector<std::shared_ptr<const csmlio::Submap3D>> insertion_submaps =
        active_submaps_.InsertData(filtered_range_data_in_local,
                                    local_from_gravity_aligned,
                                    rotational_scan_matcher_histogram_in_gravity);
    return boost::make_unique<InsertionResult>(
        InsertionResult{std::make_shared<const csmlio::TrajectoryNode::Data>(
                            csmlio::TrajectoryNode::Data{
                                time,
                                gravity_alignment,
                                {},  // 'filtered_point_cloud' is only used in 2D.
                                high_resolution_point_cloud_in_tracking,
                                low_resolution_point_cloud_in_tracking,
                                rotational_scan_matcher_histogram_in_gravity,
                                pose_estimate}),
                        std::move(insertion_submaps)});
}

std::unique_ptr<transform::Rigid3d> CSMLidarInertialOdometry::ScanMatch(
    const transform::Rigid3d& pose_prediction,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking) 
{
    if (active_submaps_.submaps().empty()) {
        return boost::make_unique<transform::Rigid3d>(pose_prediction);
    }
    std::shared_ptr<const csmlio::Submap3D> matching_submap =
        active_submaps_.submaps().front();
    transform::Rigid3d initial_ceres_pose =
        matching_submap->local_pose().inverse() * pose_prediction;
    if (csm_lio_Options_.use_online_correlative_scan_matching) {
        // We take a copy since we use 'initial_ceres_pose' as an output argument.
        const transform::Rigid3d initial_pose = initial_ceres_pose;
        const double score = real_time_correlative_scan_matcher_->Match(
            initial_pose, high_resolution_point_cloud_in_tracking,
            matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
        // LOG(INFO) << "kRealTimeCorrelativeScanMatcherScoreMetric: " << score;
    }

    transform::Rigid3d pose_observation_in_submap;
    ceres::Solver::Summary summary;
    // wgh 关闭intensity匹配（因为intensity已经被用作保存ground label了）。
    const auto* high_resolution_intensity_hybrid_grid =
        csm_lio_Options_.use_intensities
            ? &matching_submap->high_resolution_intensity_hybrid_grid()
            : nullptr;
    ceres_scan_matcher_->Match(
        (matching_submap->local_pose().inverse() * pose_prediction).translation(),
        initial_ceres_pose, {{&high_resolution_point_cloud_in_tracking,
                                &matching_submap->high_resolution_hybrid_grid(),
                                /*intensity_hybrid_grid=*/nullptr},
                            {&low_resolution_point_cloud_in_tracking,
                                &matching_submap->low_resolution_hybrid_grid(),
                                /*intensity_hybrid_grid=*/nullptr}},
        &pose_observation_in_submap, &summary);
    // LOG(INFO) << "kCeresScanMatcherCostMetric: " << summary.final_cost;
    const double residual_distance = (pose_observation_in_submap.translation() -
                                        initial_ceres_pose.translation())
                                        .norm();
    // LOG(INFO) << "kScanMatcherResidualDistanceMetric: " << residual_distance;
    const double residual_angle =
        pose_observation_in_submap.rotation().angularDistance(
            initial_ceres_pose.rotation());
    // LOG(INFO) << "kScanMatcherResidualAngleMetric: " << residual_angle;
    return boost::make_unique<transform::Rigid3d>(matching_submap->local_pose() *
                                                pose_observation_in_submap);
}




} // namespace csmlio
} // namespace infinityslam