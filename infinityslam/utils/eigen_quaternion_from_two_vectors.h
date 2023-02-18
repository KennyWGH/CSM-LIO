/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_UTILS_EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_
#define INFINITYSLAM_UTILS_EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_

#include "Eigen/Geometry"

namespace infinityslam {
namespace utils {

// Calls Eigen::Quaterniond::FromTwoVectors(). This is in its own compilation
// unit since it can take more than 10 s to build while using more than 1 GB of
// memory causing slow build times and high peak memory usage.
Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b);

Eigen::Quaternionf FromTwoVectors(const Eigen::Vector3f& a,
                                  const Eigen::Vector3f& b);

}  // namespace utils
}  // namespace infinityslam

#endif  // INFINITYSLAM_UTILS_EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_
