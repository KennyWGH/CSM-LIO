/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_SENSOR_RANGEFINDER_POINT_H_
#define INFINITYSLAM_SENSOR_RANGEFINDER_POINT_H_

#include <vector>

#include "Eigen/Core"
#include "infinityslam/transform/transform.h"
#include "glog/logging.h"

namespace infinityslam {
namespace sensor {

// 基础点类型
struct PointTypeXYZ {
  Eigen::Vector3f position;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 基础点类型，含相对时间
struct PointTypeXYZT {
  Eigen::Vector3f position;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 基础点类型，含intensity、相对时间
struct PointTypeXYZIT {
  Eigen::Vector3f position;
  float intensity;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 全量点类型，含intensity、相对时间、和label
struct PointTypeXYZITL {
  Eigen::Vector3f position;
  float intensity;
  float time;
  uint8_t label; //【规定：-1动态、0未知、1静态】
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 点的动静态属性标签
constexpr int LABEL_DYNAMIC = -1;
constexpr int LABEL_UNKNOWN = 0;
constexpr int LABEL_STATIC = 1;

template <class T>
inline PointTypeXYZ operator*(const transform::Rigid3<T>& lhs,
                                  const PointTypeXYZ& rhs) {
  PointTypeXYZ result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

template <class T>
inline PointTypeXYZT operator*(const transform::Rigid3<T>& lhs,
                                       const PointTypeXYZT& rhs) {
  PointTypeXYZT result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

inline bool operator==(const PointTypeXYZ& lhs,
                       const PointTypeXYZ& rhs) {
  return lhs.position == rhs.position;
}

inline bool operator==(const PointTypeXYZT& lhs,
                       const PointTypeXYZT& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}

inline PointTypeXYZ ToPointTypeXYZ(
    const PointTypeXYZT& timed_rangefinder_point) {
  return {timed_rangefinder_point.position};
}

inline PointTypeXYZT ToPointTypeXYZT(
    const PointTypeXYZ& rangefinder_point, const float time) {
  return {rangefinder_point.position, time};
}

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_RANGEFINDER_POINT_H_
