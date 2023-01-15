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

#ifndef CSMLIO_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_
#define CSMLIO_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_

#include <memory>

#include "csmlio/common/fixed_ratio_sampler.h"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

// Only let a fixed 'sampling_ratio' of points through. A 'sampling_ratio' of 1.
// makes this filter a no-op.
class FixedRatioSamplingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "fixed_ratio_sampler";

  FixedRatioSamplingPointsProcessor(double sampling_ratio,
                                    PointsProcessor* next);

  static std::unique_ptr<FixedRatioSamplingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~FixedRatioSamplingPointsProcessor() override{};

  FixedRatioSamplingPointsProcessor(const FixedRatioSamplingPointsProcessor&) =
      delete;
  FixedRatioSamplingPointsProcessor& operator=(
      const FixedRatioSamplingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double sampling_ratio_;
  PointsProcessor* const next_;
  std::unique_ptr<common::FixedRatioSampler> sampler_;
};

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_
