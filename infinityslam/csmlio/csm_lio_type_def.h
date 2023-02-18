/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_CSMLIO_UTILITY_H_
#define INFINITYSLAM_CSMLIO_UTILITY_H_

#include <string>
#include <iostream>
#include <ostream>
#include <vector>
#include <deque>

#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/imu_data.h"
#include "infinityslam/sensor/odometry_data.h"
#include "infinityslam/sensor/timed_point_cloud_data.h"
#include "infinityslam/csmlio/submap/submaps.h"
#include "infinityslam/csmlio/submap/submap_3d.h"
#include "infinityslam/csmlio/trajectory_node.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {
namespace csmlio {

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


struct InsertionResult {
    // NodeId node_id; //TrajBuilderInterface版本【弃】；LocalTrajBuilder3D版本中【没有】此变量
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    // std::vector<std::shared_ptr<const Submap>> insertion_submaps; //TrajBuilderInterface版本【弃】
    std::vector<std::shared_ptr<const csmlio::Submap3D>> insertion_submaps; //LocalTrajBuilder3D版本
};

struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if not asigned value; used to pass what ever you want.
    std::shared_ptr<::infinityslam::sensor::PointCloud> point_cloud;
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


}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_UTILITY_H_
