/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CSMLIO_SENSOR_LANDMARK_DATA_H_
#define CSMLIO_SENSOR_LANDMARK_DATA_H_

#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "infinityslam/common/port.h"
#include "infinityslam/common/time.h"
#include "infinityslam/transform/rigid_transform.h"

namespace infinityslam {
namespace sensor {

struct LandmarkObservation {
  std::string id;
  transform::Rigid3d landmark_to_tracking_transform;
  double translation_weight;
  double rotation_weight;
};

struct LandmarkData {
  common::Time time;
  std::vector<LandmarkObservation> landmark_observations;
};


}  // namespace sensor
}  // namespace infinityslam

#endif  // CSMLIO_SENSOR_LANDMARK_DATA_H_