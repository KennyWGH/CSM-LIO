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

#ifndef CSMLIO_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_
#define CSMLIO_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_

#include <memory>

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/points_batch.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

class IntensityToColorPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "intensity_to_color";

  // Applies ('intensity' - min ) / (max - min) * 255 and color the point grey
  // with this value for each point that comes from the sensor with 'frame_id'.
  // If 'frame_id' is empty, this applies to all points.
  IntensityToColorPointsProcessor(float min_intensity, float max_intensity,
                                  const std::string& frame_id,
                                  PointsProcessor* next);

  static std::unique_ptr<IntensityToColorPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~IntensityToColorPointsProcessor() override{};

  IntensityToColorPointsProcessor(const IntensityToColorPointsProcessor&) =
      delete;
  IntensityToColorPointsProcessor& operator=(
      const IntensityToColorPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const float min_intensity_;
  const float max_intensity_;
  const std::string frame_id_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_
