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

#include "infinityslam/io/file_writer.h"
#include "infinityslam/io/points_processor.h"
// #include "infinityslam/csmlio/2d/probability_grid.h"
// #include "infinityslam/csmlio/2d/probability_grid_range_data_inserter_2d.h"
// #include "infinityslam/csmlio/proto/probability_grid_range_data_inserter_options_2d.pb.h"
#include "infinityslam/utils/value_conversion_tables.h"

namespace infinityslam_ros {

// Very similar to Cartographer's ProbabilityGridPointsProcessor, but writes
// out a PGM and YAML suitable for ROS map server to consume.
class RosMapWritingPointsProcessor
    : public ::infinityslam::io::PointsProcessor {
 public:
    constexpr static const char* kConfigurationFileActionName = "write_ros_map";
    RosMapWritingPointsProcessor(
        double resolution,
        const ::infinityslam::csmlio::proto::
            ProbabilityGridRangeDataInserterOptions2D&
                range_data_inserter_options,
        ::infinityslam::io::FileWriterFactory file_writer_factory,
        const std::string& filestem, PointsProcessor* next);
    RosMapWritingPointsProcessor(const RosMapWritingPointsProcessor&) = delete;
    RosMapWritingPointsProcessor& operator=(const RosMapWritingPointsProcessor&) =
        delete;

    //   static std::unique_ptr<RosMapWritingPointsProcessor> FromDictionary(
    //       ::infinityslam::io::FileWriterFactory file_writer_factory,
    //       ::infinityslam::common::LuaParameterDictionary* dictionary,
    //       PointsProcessor* next);

    ~RosMapWritingPointsProcessor() override {}

    void Process(std::unique_ptr<::infinityslam::sensor::PointsBatchXYZ> batch) override;
    FlushResult Flush() override;

 private:
    const std::string filestem_;
    PointsProcessor* const next_;
    ::infinityslam::io::FileWriterFactory file_writer_factory_;
    ::infinityslam::csmlio::ProbabilityGridRangeDataInserter2D
        range_data_inserter_;
    ::infinityslam::utils::ValueConversionTables conversion_tables_;
    ::infinityslam::csmlio::ProbabilityGrid probability_grid_;
};

}  // namespace infinityslam_ros

#endif  // ROS_APP_ROS_MAP_WRITING_POINTS_PROCESSOR_H
