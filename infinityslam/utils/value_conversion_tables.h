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

#ifndef INFINITYSLAM_UTILS_VALUE_CONVERSION_TABLES_H_
#define INFINITYSLAM_UTILS_VALUE_CONVERSION_TABLES_H_

#include <map>
#include <vector>

#include "infinityslam/common/numeric_types.h"
#include "glog/logging.h"

namespace infinityslam {
namespace utils {

// Performs lazy computations of lookup tables for mapping from a uint16 value
// to a float in ['lower_bound', 'upper_bound']. The first element of the table
// is set to 'unknown_result'.
class ValueConversionTables {
 public:
  const std::vector<float>* GetConversionTable(float unknown_result,
                                               float lower_bound,
                                               float upper_bound);

 private:
  std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                            float /* upper_bound */>,
           std::unique_ptr<const std::vector<float>>>
      bounds_to_lookup_table_;
};

}  // namespace utils
}  // namespace infinityslam

#endif  // INFINITYSLAM_UTILS_VALUE_CONVERSION_TABLES_H_
