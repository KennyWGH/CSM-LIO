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

#ifndef CSMLIO_IO_COUNTING_POINTS_PROCESSOR_H_
#define CSMLIO_IO_COUNTING_POINTS_PROCESSOR_H_

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

// Passes through points, but keeps track of how many points it saw and output
// that on Flush.
class CountingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "dump_num_points";
  explicit CountingPointsProcessor(PointsProcessor* next);

  static std::unique_ptr<CountingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~CountingPointsProcessor() override {}

  CountingPointsProcessor(const CountingPointsProcessor&) = delete;
  CountingPointsProcessor& operator=(const CountingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> points) override;
  FlushResult Flush() override;

 private:
  int64 num_points_;
  PointsProcessor* next_;
};

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_COUNTING_POINTS_PROCESSOR_H_
