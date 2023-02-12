/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef INFINITYSLAM_CSMLIO_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_
#define INFINITYSLAM_CSMLIO_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "infinityslam/common/math.h"
#include "infinityslam/csmlio/optimization/cost_functions/cost_helpers.h"
#include "infinityslam/csmlio/optimization/optimization_type_def.h"
#include "infinityslam/transform/rigid_transform.h"
#include "infinityslam/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace infinityslam {
namespace csmlio {
namespace optimization {

class SpaCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const optimization::Constraint::Pose& pose) {
    return new ceres::AutoDiffCostFunction<
        SpaCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */,
        3 /* translation variables */>(new SpaCostFunction3D(pose));
  }

  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(pose_.zbar_ij, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation),
        pose_.translation_weight, pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit SpaCostFunction3D(const optimization::Constraint::Pose& pose)
      : pose_(pose) {}

  const optimization::Constraint::Pose pose_;
};

}  // namespace optimization
}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_
