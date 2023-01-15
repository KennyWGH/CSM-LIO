/**
 * Copyright 2023 WANG Guanhua (wangguanhua999@gmail.com)
*/

#ifndef CSMLIO_MAPPING_CSM_LIDAR_INERTIAL_ODOMETRY_H_
#define CSMLIO_MAPPING_CSM_LIDAR_INERTIAL_ODOMETRY_H_

#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <deque>
#include <map>
#include <set>

/// Collated & Local Trajectory Builder
#include "csmlio/common/port.h"
#include "csmlio/common/time.h"
#include "csmlio/common/internal/rate_timer.h"
#include "csmlio/lio/submaps.h"
#include "csmlio/lio/3d/submap_3d.h"
#include "csmlio/lio/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "csmlio/lio/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"
#include "csmlio/lio/internal/motion_filter.h"
#include "csmlio/lio/internal/range_data_collator.h"
#include "csmlio/lio/pose_extrapolator_interface.h"
#include "csmlio/lio/proto/map_builder_options.pb.h"
#include "csmlio/lio/proto/trajectory_builder_options.pb.h"
#include "csmlio/lio/proto/local_trajectory_builder_options_3d.pb.h"
#include "csmlio/sensor/data.h"
#include "csmlio/sensor/imu_data.h"
#include "csmlio/sensor/collator_interface.h"
#include "csmlio/sensor/internal/voxel_filter.h"
#include "csmlio/sensor/odometry_data.h"
#include "csmlio/sensor/range_data.h"
#include "csmlio/sensor/timed_point_cloud_data.h"
#include "csmlio/transform/rigid_transform.h"

/// Map Builder
#include <memory>
#include "csmlio/common/thread_pool.h"
#include "csmlio/lio/proto/map_builder_options.pb.h"
#include "csmlio/sensor/collator_interface.h"
#include "csmlio/sensor/internal/collator.h"
#include "csmlio/sensor/internal/trajectory_collator.h"

// wgh
#include "csmlio/lio/csm_lio_interface.h"
#include "csmlio/lio/utility.h"
#include "csmlio/lio/trajectory_node.h"

namespace csmlio {
namespace mapping {

class CSMLidarInertialOdometry : public CSMLioInterface {
  public:
    using SensorId = mapping::SensorId;
    using MatchingResult = mapping::MatchingResult;
    using InsertionResult = mapping::InsertionResult;
    using TrajectoryNode = mapping::TrajectoryNode;
    using TimedPose = mapping::TimedPose;

    CSMLidarInertialOdometry(
        const mapping::proto::MapBuilderOptions& map_builder_options,
        const mapping::proto::TrajectoryBuilderOptions& traj_builder_options,
        const std::set<mapping::SensorId>& expected_sensor_ids,
        mapping::LioResultCallback lio_result_callback);
    ~CSMLidarInertialOdometry();

    CSMLidarInertialOdometry(const CSMLidarInertialOdometry&) = delete;
    CSMLidarInertialOdometry& operator=(const CSMLidarInertialOdometry&) = delete;

    /// 为了实现不同sensor数据的统一按时间排序，我们需要把不同sensor的数据封装为
    /// 同一种数据类型（即Dispatchable）来处理。
    void AddSensorData(
        const std::string& sensor_id,
        const sensor::TimedPointCloudData& timed_point_cloud_data); 

    void AddSensorData(const std::string& sensor_id,
                        const sensor::ImuData& imu_data); 

    void AddSensorData(const std::string& sensor_id,
                        const sensor::OdometryData& odometry_data); 

    /// 向Collator中添加【不同sensor数据】的统一接口。
    void AddDataToCollator(std::unique_ptr<sensor::Data> data);

    /// 处理Collator排序后下发的数据的唯一接口，作为回调注册到Collator中；
    /// 为了能够被注册到Collator，必须为公共函数。
    void HandleCollatedData(const std::string& sensor_id,
        std::unique_ptr<sensor::Data> data);

    /// 冠华：用同名函数处理Collator出来的【不同sensor的数据】;
    /// 作为对照，相当于原版的【GlobalTrajectoryBuilder层】;
    /// 以下函数必须作为public存在，否则Collator中将无法调用。
    void ProcessSensorData(
        const std::string& sensor_id,
        const sensor::TimedPointCloudData& timed_point_cloud_data) override;
    void ProcessSensorData(const std::string& sensor_id,
                            const sensor::ImuData& imu_data) override;
    void ProcessSensorData(const std::string& sensor_id,
                            const sensor::OdometryData& odometry_data) override;

    // 向外输出slam信息。
    const std::deque<TrajectoryNode>& GetSlamKeyframeList() const;
    const std::deque<TimedPose>& GetTimedPoseQueue() const;

  private:
    /// 自定义的一些日志函数。
    void LogSensorDataRate(const std::string& sensor_id, const common::Time& data_stamp);
    void LogLioStatus();

    /// LocalTrajBuilder相关函数【原版函数】。
    std::unique_ptr<MatchingResult> AddRangeData(
        const std::string& sensor_id,
        const sensor::TimedPointCloudData& range_data);

    std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
        const common::Time time,
        const sensor::RangeData& filtered_range_data_in_tracking,
        const absl::optional<common::Duration>& sensor_duration,
        const transform::Rigid3d& pose_prediction,
        const Eigen::Quaterniond& gravity_alignment);

    std::unique_ptr<InsertionResult> InsertIntoSubmap(
        common::Time time, const sensor::RangeData& filtered_range_data_in_local,
        const sensor::RangeData& filtered_range_data_in_tracking,
        const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
        const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
        const transform::Rigid3d& pose_estimate,
        const Eigen::Quaterniond& gravity_alignment);

    // Scan matches using the two point clouds and returns the observed pose, or
    // nullptr on failure.
    std::unique_ptr<transform::Rigid3d> ScanMatch(
        const transform::Rigid3d& pose_prediction,
        const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
        const sensor::PointCloud& high_resolution_point_cloud_in_tracking);


  private:
    // 来自MapBuilder类的成员/或相关的对象。
    const mapping::proto::MapBuilderOptions map_builder_options_;
    const mapping::proto::TrajectoryBuilderOptions traj_builder_options_;
    std::unique_ptr<sensor::CollatorInterface> sensor_collator_;

    // 来自Local类的成员。
    const mapping::proto::LocalTrajectoryBuilderOptions3D local_traj_builder_options_;
    mapping::ActiveSubmaps3D active_submaps_;

    mapping::MotionFilter motion_filter_;
    std::unique_ptr<mapping::scan_matching::RealTimeCorrelativeScanMatcher3D>
        real_time_correlative_scan_matcher_;
    std::unique_ptr<mapping::scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;

    std::unique_ptr<mapping::PoseExtrapolatorInterface> extrapolator_;

    int num_accumulated_ = 0;
    std::vector<sensor::TimedPointCloudOriginData>
        accumulated_point_cloud_origin_data_;
    absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;

    absl::optional<double> last_thread_cpu_time_seconds_;

    mapping::RangeDataCollator range_data_collator_;

    absl::optional<common::Time> last_sensor_time_;

    // 来自Global类的成员，对外输出信息的回调。
    mapping::LioResultCallback lio_result_callback_;

    // 来自Collated类的成员。
    const int trajectory_id_ = 0; //原版Collator注册sensor需要指定一个轨迹ID。
    const double kSensorDataRatesPeriodSeconds = 10.; //计算频率的时段长度，原版默认15.0。
    std::map<std::string, common::RateTimer<>> rate_timers_;
    std::chrono::steady_clock::time_point last_logging_time_;

    struct RecentDurationKeeper {
        const static int kFixedTimes = 50;
        double recent_durations_[kFixedTimes] = {0};
        int counts_ = 0;
        
        void AddDuration(const double& dura) {
            ++counts_;
            recent_durations_[
                (counts_ - 1) % kFixedTimes] = dura;
        }

        double GetRecentAvgDuration() {
            double sum = 0.;
            int count_to_here = counts_ < kFixedTimes ? 
                                    counts_ : kFixedTimes;
            for (int i=0; i<count_to_here; ++i) {
                sum += recent_durations_[i];
            }
            return sum / count_to_here;
        }
    };

    // 新增的成员。
    mutable absl::Mutex mutex_;
    const double kLogEveryNSeconds = 3.;
    std::uint32_t count_received_range_data = 0;
    std::uint32_t count_lio_matching_results = 0;
    std::uint32_t count_lio_insertion_results = 0;
    std::uint32_t count_processed_range_data = 0;
    double life_long_processing_time = 0;
    RecentDurationKeeper duration_keeper_;

    // 新增的成员 | SLAM数据容器 | 使用deque可以控制内存消耗。
    std::deque<TrajectoryNode> slam_keyframes_data_;

    // 在顶层类中保留一份位姿队列，可供wrapper层获取。
    const common::Duration kPoseQueueDuration; //5秒
    std::deque<TimedPose> timed_pose_queue_;

};

} // namespace mapping
} // namespace csmlio


#endif // CSMLIO_MAPPING_CSM_LIDAR_INERTIAL_ODOMETRY_H_