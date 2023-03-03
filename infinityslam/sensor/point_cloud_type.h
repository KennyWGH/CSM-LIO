/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_SENSOR_POINT_CLOUD_TYPE_H_
#define INFINITYSLAM_SENSOR_POINT_CLOUD_TYPE_H_

#include <vector>
#include <memory>
#include <algorithm>

#include "Eigen/Core"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/point_type.h"
#include "infinityslam/transform/rigid_transform.h"
#include "glog/logging.h"

namespace infinityslam {
namespace sensor {

// 3D点云通用模板类
template<typename PointType>
class PointCloudT {
   public:
    using Ptr = std::shared_ptr<PointCloudT<PointType>>;

    PointCloudT() {}

    // explicit PointCloudT(std::vector<PointType> points) 
    //     : points_(std::move(points)) {}

    size_t size() const { return points_.size(); }

    bool empty() const { return points_.empty(); }

    double earliest_time() const {
        return timestamp_ + 
            (*std::min_element(points_.begin(), points_.end(), CompareByTime)).time;
    }

    double latest_time() const {
        return timestamp_ + 
            (*std::max_element(points_.begin(), points_.end(), CompareByTime)).time;
    }

    double begin_time() const {
        return points_.empty() ? timestamp_ : timestamp_ + points_.front().time;
    }

    double end_time() const {
        return points_.empty() ? timestamp_ : timestamp_ + points_.back().time;
    }

    const std::vector<PointType>& points() const {
        return points_;
    }

    const PointType& operator[](const size_t index) const {
        return points_[index];
    }

    using ConstIterator = typename std::vector<PointType>::const_iterator;

    ConstIterator begin() const {
        return points_.begin();
    }

    ConstIterator end() const {
        return points_.end();
    }

    void push_back(PointType value) {
        points_.push_back(value);
    }

    void clear() {
        points_.clear();
    }

   public:

    bool CompareByTime(const PointType& p1, const PointType& p2) {
        return p1.time < p2.time;
    }

    common::Time time_;
    double timestamp_;
    long seq_;
    std::string frame_id_;
    std::vector<PointType> points_;
};

using PointCloudXYZ = PointCloudT<PointTypeXYZ>;
using PointCloudXYZT = PointCloudT<PointTypeXYZT>;
using PointCloudXYZIT = PointCloudT<PointTypeXYZIT>;
using PointCloudXYZITL = PointCloudT<PointTypeXYZITL>;


/// TODO: @wgh 实现一些通用方法



}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_POINT_CLOUD_TYPE_H_
