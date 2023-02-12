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

#ifndef INFINITYSLAM_CSMLIO_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
#define INFINITYSLAM_CSMLIO_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_

#include "infinityslam/csmlio/optimization/optimization_type_def.h"
#include "ceres/ceres.h"

namespace infinityslam {
namespace csmlio {
namespace optimization {

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const optimization::Constraint::Pose& pose);

ceres::CostFunction* CreateAnalyticalSpaCostFunction(
    const optimization::Constraint::Pose& pose);

}  // namespace optimization
}  // namespace csmlio
}  // namespace infinityslam

#endif  // INFINITYSLAM_CSMLIO_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
