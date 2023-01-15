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

#ifndef ROS_APP_ROS_MAP_WRITING_POINTS_PROCESSOR_H
#define ROS_APP_ROS_MAP_WRITING_POINTS_PROCESSOR_H

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/file_writer.h"
#include "csmlio/io/points_processor.h"
#include "csmlio/lio/2d/probability_grid.h"
#include "csmlio/lio/2d/probability_grid_range_data_inserter_2d.h"
#include "csmlio/lio/proto/probability_grid_range_data_inserter_options_2d.pb.h"
#include "csmlio/lio/value_conversion_tables.h"

namespace ros_app {

// Very similar to Cartographer's ProbabilityGridPointsProcessor, but writes
// out a PGM and YAML suitable for ROS map server to consume.
class RosMapWritingPointsProcessor
    : public ::csmlio::io::PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_ros_map";
  RosMapWritingPointsProcessor(
      double resolution,
      const ::csmlio::mapping::proto::
          ProbabilityGridRangeDataInserterOptions2D&
              range_data_inserter_options,
      ::csmlio::io::FileWriterFactory file_writer_factory,
      const std::string& filestem, PointsProcessor* next);
  RosMapWritingPointsProcessor(const RosMapWritingPointsProcessor&) = delete;
  RosMapWritingPointsProcessor& operator=(const RosMapWritingPointsProcessor&) =
      delete;

  static std::unique_ptr<RosMapWritingPointsProcessor> FromDictionary(
      ::csmlio::io::FileWriterFactory file_writer_factory,
      ::csmlio::common::LuaParameterDictionary* dictionary,
      PointsProcessor* next);

  ~RosMapWritingPointsProcessor() override {}

  void Process(std::unique_ptr<::csmlio::io::PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const std::string filestem_;
  PointsProcessor* const next_;
  ::csmlio::io::FileWriterFactory file_writer_factory_;
  ::csmlio::mapping::ProbabilityGridRangeDataInserter2D
      range_data_inserter_;
  ::csmlio::mapping::ValueConversionTables conversion_tables_;
  ::csmlio::mapping::ProbabilityGrid probability_grid_;
};

}  // namespace ros_app

#endif  // ROS_APP_ROS_MAP_WRITING_POINTS_PROCESSOR_H
