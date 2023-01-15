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

#ifndef CSMLIO_IO_XYZ_WRITING_POINTS_PROCESSOR_H_
#define CSMLIO_IO_XYZ_WRITING_POINTS_PROCESSOR_H_

#include <fstream>
#include <memory>
#include <string>

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/file_writer.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

// Writes ASCII xyz points.
class XyzWriterPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_xyz";

  XyzWriterPointsProcessor(std::unique_ptr<FileWriter>, PointsProcessor* next);

  static std::unique_ptr<XyzWriterPointsProcessor> FromDictionary(
      const FileWriterFactory& file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~XyzWriterPointsProcessor() override {}

  XyzWriterPointsProcessor(const XyzWriterPointsProcessor&) = delete;
  XyzWriterPointsProcessor& operator=(const XyzWriterPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;
  std::unique_ptr<FileWriter> file_writer_;
};

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_XYZ_WRITING_POINTS_PROCESSOR_H_
