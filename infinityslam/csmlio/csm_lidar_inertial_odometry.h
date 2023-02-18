/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_CSMLIO_CSM_LIDAR_INERTIAL_ODOMETRY_H_
#define INFINITYSLAM_CSMLIO_CSM_LIDAR_INERTIAL_ODOMETRY_H_

#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/common/optional.h"
#include "infinityslam/common/thread_pool.h"
#include "infinityslam/common/rate_timer.h"
#include "infinityslam/common/system_options.h"
#include "infinityslam/sensor/data.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/collator_interface.h"
#include "infinityslam/sensor/internal/voxel_filter.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/sensor/range_data.h"
#include "infinityslam/sensor/point_cloud.h"
#include "infinityslam/sensor/timed_point_cloud_data.h"
#include "infinityslam/sensor/collator_interface.h"
#include "infinityslam/sensor/internal/collator.h"
#include "infinityslam/sensor/internal/trajectory_collator.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/timed_pose.h"
#include "infinityslam/csmlio/submap/submaps.h"
#include "infinityslam/csmlio/submap/submap_3d.h"
#include "infinityslam/csmlio/scan_matching/ceres_scan_matcher_3d.h"
#include "infinityslam/csmlio/scan_matching/real_time_correlative_scan_matcher_3d.h"
#include "infinityslam/csmlio/pose_extrapolator/pose_extrapolator_interface.h"
#include "infinityslam/utils/motion_filter.h"
#include "infinityslam/csmlio/range_data_collator.h"
#include "infinityslam/csmlio/trajectory_node.h"
#include "infinityslam/csmlio/csm_lio_type_def.h"
#include "infinityslam/csmlio/csm_lio_interface.h"

namespace infinityslam {
namespace csmlio {

struct CSMLioOptions {
    bool collate_fixed_frame = true;
    bool collate_landmarks = false;

    double min_range = 0.4;
    double max_range = 60;
    int    num_accumulated_range_data = 1;
    double voxel_filter_size = 0.15;

    double high_res_adap_vf_max_length = 2.;
    int    high_res_adap_vf_min_num_points = 1000;
    double high_res_adap_vf_max_range = 20.;

    double low_res_adap_vf_max_length = 4.;
    int    low_res_adap_vf_min_num_points = 1000;
    double low_res_adap_vf_max_range = 60.;

    bool use_online_correlative_scan_matching = false;

    int rotational_histogram_size = 120;

    bool use_intensities = false;

    CSMLioOptions() {}

    sensor::AdaptiveVoxelFIlterOpTions 
    high_resolution_adaptive_voxel_filter_options() const {
        return sensor::AdaptiveVoxelFIlterOpTions(
            high_res_adap_vf_max_length, 
            high_res_adap_vf_min_num_points, 
            high_res_adap_vf_max_range);
    }

    sensor::AdaptiveVoxelFIlterOpTions 
    low_resolution_adaptive_voxel_filter_options() const {
        return sensor::AdaptiveVoxelFIlterOpTions(
            low_res_adap_vf_max_length, 
            low_res_adap_vf_min_num_points, 
            low_res_adap_vf_max_range);
    }

    // std::string DebugString() {}
};

// 从common::options::csmlio中加载参数
CSMLioOptions ReadCSMLioOptions();

// 从common::options::csmlio中加载参数
utils::MotionFIlterOptions ReadCSMLioMotionFIlterOptions();

class CSMLidarInertialOdometry : public CSMLioInterface {
  public:
    // using SensorId = mapp ing::SensorId;
    // using MatchingResult = mapp ing::MatchingResult;
    // using InsertionResult = mapp ing::InsertionResult;
    // using TrajectoryNode = mappi ng::TrajectoryNode;

    CSMLidarInertialOdometry(
        const CSMLioOptions& csm_lio_options,
        const std::set<csmlio::SensorId>& expected_sensor_ids,
        csmlio::LioResultCallback lio_result_callback);

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
    const std::deque<transform::TimedPose>& GetTimedPoseQueue() const;

    std::vector<std::shared_ptr<const sensor::PointCloud>> GetActiveSubmapCloudsList();
    const sensor::PointCloud& GetActiveSubmapCloudsInOne();

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
        const common::optional<common::Duration>& sensor_duration,
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
    // CSMLIO 参数
    const CSMLioOptions csm_lio_Options_;

    // 多传感器数据时间戳对齐下发
    std::unique_ptr<sensor::CollatorInterface> sensor_collator_;

    // CSMLIO 核心成员。
    csmlio::ActiveSubmaps3D active_submaps_;
    std::shared_ptr<sensor::PointCloud> submap_cloud_growing_; //最新的
    std::shared_ptr<sensor::PointCloud> submap_cloud_working_; //此新的
    std::shared_ptr<sensor::PointCloud> submap_cloud_all_;     //所有的
    mutable bool active_submaps_updated = false;

    utils::MotionFilter motion_filter_;
    std::unique_ptr<csmlio::scan_matching::RealTimeCorrelativeScanMatcher3D>
        real_time_correlative_scan_matcher_;
    std::unique_ptr<csmlio::scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;

    std::unique_ptr<csmlio::PoseExtrapolatorInterface> extrapolator_;

    int num_accumulated_ = 0;
    std::vector<sensor::MultiTimedPOintCloudData>
        accumulated_point_cloud_origin_data_;
    common::optional<std::chrono::steady_clock::time_point> last_wall_time_;

    common::optional<double> last_thread_cpu_time_seconds_;

    csmlio::RangeDataCollator range_data_collator_;

    common::optional<common::Time> last_sensor_time_;

    // 来自Global类的成员，对外输出信息的回调。
    csmlio::LioResultCallback lio_result_callback_;

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
    mutable std::mutex mutex_;
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
    std::deque<transform::TimedPose> timed_pose_queue_;

};

} // namespace csmlio
} // namespace infinityslam


#endif // INFINITYSLAM_CSMLIO_CSM_LIDAR_INERTIAL_ODOMETRY_H_