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

#ifndef CSMLIO_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
#define CSMLIO_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

// Filters all points that are farther away from their 'origin' as 'max_range'
// or closer than 'min_range'.
class MinMaxRangeFilteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "min_max_range_filter";
  MinMaxRangeFilteringPointsProcessor(double min_range, double max_range,
                                      PointsProcessor* next);
  static std::unique_ptr<MinMaxRangeFilteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~MinMaxRangeFilteringPointsProcessor() override {}

  MinMaxRangeFilteringPointsProcessor(
      const MinMaxRangeFilteringPointsProcessor&) = delete;
  MinMaxRangeFilteringPointsProcessor& operator=(
      const MinMaxRangeFilteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double min_range_squared_;
  const double max_range_squared_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_