/*
 * Copyright 2017 The Cartographer Authors
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

#include "ros_app/src/ros_map.h"

#include "absl/strings/str_cat.h"

namespace ros_app {

void WritePgm(const ::infinityslam::io::Image& image, const double resolution,
              ::infinityslam::io::FileWriter* file_writer) {
  const std::string header =
      absl::StrCat("P5\n# Cartographer map; ", resolution, " m/pixel\n",
                   image.width(), " ", image.height(), "\n255\n");
  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const char color = image.GetPixel(x, y)[0];
      file_writer->Write(&color, 1);
    }
  }
}

void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::infinityslam::io::FileWriter* file_writer) {
  // Magic constants taken directly from ros map_saver code:
  // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
  const std::string output = absl::StrCat(
      "image: ", pgm_filename, "\n", "resolution: ", resolution, "\n",
      "origin: [", origin.x(), ", ", origin.y(),
      ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
  file_writer->Write(output.data(), output.size());
}

}  // namespace ros_app
