/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include "infinityslam/utils/eigen_quaternion_from_two_vectors.h"

namespace infinityslam {
namespace utils {

Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b) {
  return Eigen::Quaterniond::FromTwoVectors(a, b);
}

Eigen::Quaternionf FromTwoVectors(const Eigen::Vector3f& a,
                                  const Eigen::Vector3f& b) {
  return Eigen::Quaternionf::FromTwoVectors(a, b);
}

}  // namespace utils
}  // namespace infinityslam
