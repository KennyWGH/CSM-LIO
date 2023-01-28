/**
 * Copyright 2023 WANG Guanhua (wangxxx@gmail.com)
*/

#ifndef CSMLIO_MAPPING_UTILITY_H_
#define CSMLIO_MAPPING_UTILITY_H_

#include <string>
#include <iostream>
#include <ostream>
#include <vector>
#include <deque>

#include "csmlio/common/port.h"
#include "csmlio/common/time.h"
#include "csmlio/sensor/imu_data.h"
#include "csmlio/sensor/odometry_data.h"
#include "csmlio/sensor/timed_point_cloud_data.h"
#include "csmlio/lio/submaps.h"
#include "csmlio/lio/3d/submap_3d.h"
#include "csmlio/lio/trajectory_node.h"
#include "csmlio/transform/rigid_transform.h"

namespace csmlio {
namespace mapping {

struct SensorId {
    enum class SensorType {
        RANGE = 0,
        IMU,
        ODOMETRY,
        FIXED_FRAME_POSE,
        LANDMARK,
        LOCAL_SLAM_RESULT
    };

    SensorType type;
    std::string id;

    bool operator==(const SensorId& other) const {
        return std::forward_as_tuple(type, id) ==
                std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
        return std::forward_as_tuple(type, id) <
                std::forward_as_tuple(other.type, other.id);
    }

    std::string sensor_type_as_string() const {
        if (type == SensorType::RANGE) return "RANGE";
        if (type == SensorType::IMU) return "IMU";
        if (type == SensorType::ODOMETRY) return "ODOMETRY";
        return "UNKNOWN";
    }
};

inline std::ostream& operator<<(std::ostream& os, const SensorId& v) {
  return os << "(" << v.sensor_type_as_string() << ", " << v.id << ")";
}

struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
};

struct InsertionResult {
    // NodeId node_id; //TrajBuilderInterface版本【弃】；LocalTrajBuilder3D版本中【没有】此变量
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    // std::vector<std::shared_ptr<const Submap>> insertion_submaps; //TrajBuilderInterface版本【弃】
    std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps; //LocalTrajBuilder3D版本
};

struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if not asigned value; used to pass what ever you want.
    std::shared_ptr<::csmlio::sensor::PointCloud> point_cloud;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
};

// A callback which is called after local SLAM processes an accumulated
// 'sensor::RangeData'. If the data was inserted into a submap, reports the
// assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
using LioResultCallback =
    std::function<void(common::Time,
                       transform::Rigid3d /* local pose estimate */,
                       sensor::RangeData /* in local frame */,
                       std::shared_ptr<sensor::PointCloud> /* local ground map */,
                       std::unique_ptr<const InsertionResult>)>;

// 以配准位姿队列为主，以IMU数据为辅，在指定时间段内，以指定时间步长进行位姿插值，得到插值位姿表；
// 插值位姿表中的位姿为相对位姿 —— 我们规定以 end_time 时的位姿为基准来计算“相对”；
// 因为每次计算相对位姿的base不一样，所以本函数无法对【插值位姿表】做增量式更新。
bool InterpolatePoseTable(const common::Time& start_time, 
                        const common::Time& end_time,
                        const std::vector<TimedPose>& timed_pose_queue,
                        const std::vector<sensor::ImuData>& imu_queue,
                        const double& interp_step,
                        std::vector<TimedPose>& pose_table,
                        bool based_on_end = true);

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_UTILITY_H_
