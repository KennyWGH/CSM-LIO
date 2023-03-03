/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_SENSOR_POINT_CLOUD_H_
#define INFINITYSLAM_SENSOR_POINT_CLOUD_H_

#include <vector>
#include <memory>

#include "Eigen/Core"
#include "infinityslam/sensor/point_type.h"
#include "infinityslam/transform/rigid_transform.h"
#include "glog/logging.h"

namespace infinityslam {
namespace sensor {

// Stores 3D positions of points together with some additional data, e.g.
// intensities.
class PointCloud {
   public:
    using PointType = PointTypeXYZ;

    PointCloud();
    explicit PointCloud(std::vector<PointType> points);
    PointCloud(std::vector<PointType> points, std::vector<float> intensities);

    // Returns the number of points in the point cloud.
    size_t size() const;
    // Checks whether there are any points in the point cloud.
    bool empty() const;

    const std::vector<PointType>& points() const;
    const std::vector<float>& intensities() const;
    const PointType& operator[](const size_t index) const;

    // Iterator over the points in the point cloud.
    using ConstIterator = std::vector<PointType>::const_iterator;
    ConstIterator begin() const;
    ConstIterator end() const;

    void push_back(PointType value);
    void push_back(PointType point, const float intensity);

    void clear();

    // Creates a PointCloud consisting of all the points for which `predicate`
    // returns true, together with the corresponding intensities.
    template <class UnaryPredicate>
    PointCloud copy_if(UnaryPredicate predicate) const {
        std::vector<PointType> points;
        std::vector<float> intensities;

        // Note: benchmarks show that it is better to have this conditional outside
        // the loop.
        if (intensities_.empty()) {
        for (size_t index = 0; index < size(); ++index) {
            const PointType& point = points_[index];
            if (predicate(point)) {
            points.push_back(point);
            }
        }
        } else {
        for (size_t index = 0; index < size(); ++index) {
            const PointType& point = points_[index];
            if (predicate(point)) {
            points.push_back(point);
            intensities.push_back(intensities_[index]);
            }
        }
        }

        return PointCloud(points, intensities);
    }

   public:
    std::vector<PointType> points_;

    // Intensities are optional. 
    // If non-empty, they must have the same size as points.
    std::vector<float> intensities_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f.
using TimedPointCloud = std::vector<PointTypeXYZT>;

// 与ROS层进行数据格式转换
struct PointCloudWithIntensities {
    TimedPointCloud points;
    std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_POINT_CLOUD_H_
