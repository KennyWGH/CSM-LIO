/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
};

// 基础点类型，含相对时间
struct PointTypeXYZT {
  Eigen::Vector3f position;
  float time;
};

// 全量点类型，含intensity、相对时间、和label
struct PointTypeXYZITL {
  Eigen::Vector3f position;
  float intensity;
  float time;
  int label; //【规定：-1动态、0未知、1静态】
};

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
