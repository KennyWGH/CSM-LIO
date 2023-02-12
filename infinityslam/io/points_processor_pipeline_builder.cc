// /*
//  * Copyright 2016 The Cartographer Authors
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *      http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */

// #include "infinityslam/io/points_processor_pipeline_builder.h"

// #include <boost/make_unique.hpp>
// #include "infinityslam/io/coloring_points_processor.h"
// #include "infinityslam/io/counting_points_processor.h"
// #include "infinityslam/io/fixed_ratio_sampling_points_processor.h"
// #include "infinityslam/io/frame_id_filtering_points_processor.h"
// #include "infinityslam/io/hybrid_grid_points_processor.h"
// #include "infinityslam/io/intensity_to_color_points_processor.h"
// #include "infinityslam/io/min_max_range_filtering_points_processor.h"
// #include "infinityslam/io/null_points_processor.h"
// #include "infinityslam/io/outlier_removing_points_processor.h"
// #include "infinityslam/io/pcd_writing_points_processor.h"
// #include "infinityslam/io/ply_writing_points_processor.h"
// #include "infinityslam/io/probability_grid_points_processor.h"
// #include "infinityslam/io/vertical_range_filtering_points_processor.h"
// #include "infinityslam/io/xray_points_processor.h"
// #include "infinityslam/io/xyz_writing_points_processor.h"
// #include "infinityslam/csmlio/proto/trajectory.pb.h"

// namespace infinityslam {
// namespace io {

// template <typename PointsProcessorType>
// void RegisterPlainPointsProcessor(
//     PointsProcessorPipelineBuilder* const builder) {
//   builder->Register(
//       PointsProcessorType::kConfigurationFileActionName,
//       [](common::LuaParameterDictionary* const dictionary,
//          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
//         return PointsProcessorType::FromDictionary(dictionary, next);
//       });
// }

// template <typename PointsProcessorType>
// void RegisterFileWritingPointsProcessor(
//     const FileWriterFactory& file_writer_factory,
//     PointsProcessorPipelineBuilder* const builder) {
//   builder->Register(
//       PointsProcessorType::kConfigurationFileActionName,
//       [file_writer_factory](
//           common::LuaParameterDictionary* const dictionary,
//           PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
//         return PointsProcessorType::FromDictionary(file_writer_factory,
//                                                    dictionary, next);
//       });
// }

// template <typename PointsProcessorType>
// void RegisterFileWritingPointsProcessorWithTrajectories(
//     const std::vector<csmlio::proto::Trajectory>& trajectories,
//     const FileWriterFactory& file_writer_factory,
//     PointsProcessorPipelineBuilder* const builder) {
//   builder->Register(
//       PointsProcessorType::kConfigurationFileActionName,
//       [&trajectories, file_writer_factory](
//           common::LuaParameterDictionary* const dictionary,
//           PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
//         return PointsProcessorType::FromDictionary(
//             trajectories, file_writer_factory, dictionary, next);
//       });
// }

// void RegisterBuiltInPointsProcessors(
//     const std::vector<csmlio::proto::Trajectory>& trajectories,
//     const FileWriterFactory& file_writer_factory,
//     PointsProcessorPipelineBuilder* builder) {
//   RegisterPlainPointsProcessor<CountingPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<FixedRatioSamplingPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<FrameIdFilteringPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<MinMaxRangeFilteringPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<VerticalRangeFilteringPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<OutlierRemovingPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<ColoringPointsProcessor>(builder);
//   RegisterPlainPointsProcessor<IntensityToColorPointsProcessor>(builder);
//   RegisterFileWritingPointsProcessor<PcdWritingPointsProcessor>(
//       file_writer_factory, builder);
//   RegisterFileWritingPointsProcessor<PlyWritingPointsProcessor>(
//       file_writer_factory, builder);
//   RegisterFileWritingPointsProcessor<XyzWriterPointsProcessor>(
//       file_writer_factory, builder);
//   RegisterFileWritingPointsProcessor<HybridGridPointsProcessor>(
//       file_writer_factory, builder);
//   RegisterFileWritingPointsProcessorWithTrajectories<XRayPointsProcessor>(
//       trajectories, file_writer_factory, builder);
//   RegisterFileWritingPointsProcessorWithTrajectories<
//       ProbabilityGridPointsProcessor>(trajectories, file_writer_factory,
//                                       builder);
// }

// void PointsProcessorPipelineBuilder::Register(const std::string& name,
//                                               FactoryFunction factory) {
//   CHECK(factories_.count(name) == 0) << "A points processor named '" << name
//                                      << "' has already been registered.";
//   factories_[name] = std::move(factory);
// }

// PointsProcessorPipelineBuilder::PointsProcessorPipelineBuilder() {}

// std::vector<std::unique_ptr<PointsProcessor>>
// PointsProcessorPipelineBuilder::CreatePipeline(
//     common::LuaParameterDictionary* const dictionary) const {
//   std::vector<std::unique_ptr<PointsProcessor>> pipeline;
//   // The last consumer in the pipeline must exist, so that the one created after
//   // it (and being before it in the pipeline) has a valid 'next' to point to.
//   // The last consumer will just drop all points.
//   pipeline.emplace_back(boost::make_unique<NullPointsProcessor>());

//   std::vector<std::unique_ptr<common::LuaParameterDictionary>> configurations =
//       dictionary->GetArrayValuesAsDictionaries();

//   // We construct the pipeline starting at the back.
//   for (auto it = configurations.rbegin(); it != configurations.rend(); it++) {
//     const std::string action = (*it)->GetString("action");
//     auto factory_it = factories_.find(action);
//     CHECK(factory_it != factories_.end())
//         << "Unknown action '" << action
//         << "'. Did you register the correspoinding PointsProcessor?";
//     pipeline.push_back(factory_it->second(it->get(), pipeline.back().get()));
//   }
//   return pipeline;
// }

// }  // namespace io
// }  // namespace infinityslam
