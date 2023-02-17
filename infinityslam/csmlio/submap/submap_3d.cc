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

#include "infinityslam/csmlio/submap/submap_3d.h"

#include <cmath>
#include <limits>
#include <cassert>
#include <boost/make_unique.hpp>

#include "infinityslam/common/math.h"
#include "infinityslam/common/system_options.h"
#include "infinityslam/csmlio/scan_matching/rotational_scan_matcher.h"
#include "infinityslam/sensor/range_data.h"
#include "infinityslam/sensor/point_cloud.h"
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {
namespace {

struct PixelData {
  int min_z = INT_MAX;
  int max_z = INT_MIN;
  int count = 0;
  float probability_sum = 0.f;
  float max_probability = 0.5f;
};

// Filters 'range_data', retaining only the returns that have no more than
// 'max_range' distance from the origin. Removes misses.
sensor::RangeData FilterRangeDataByMaxRange(const sensor::RangeData& range_data,
                                            const float max_range) {
  sensor::RangeData result{range_data.origin, {}, {}};
  result.returns =
      range_data.returns.copy_if([&](const sensor::PointTypeXYZ& point) {
        return (point.position - range_data.origin).norm() <= max_range;
      });
  return result;
}

std::vector<PixelData> AccumulatePixelData(
    const int width, const int height, const Eigen::Array2i& min_index,
    const Eigen::Array2i& max_index,
    const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) {
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data[x * width + y];
    ++pixel.count;
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    const float probability =
        ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
  return accumulated_pixel_data;
}

// 这个函数通过 HybridGrid 类提供的 Iterator 接口来访问所有的cell。
// The first three entries of each returned value are a cell_index and the
// last is the corresponding probability value. We batch them together like
// this to only have one vector and have better cache locality.
std::vector<Eigen::Array4i> ExtractVoxelData(
    const HybridGrid& hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();

  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  // constexpr float kXrayObstructedCellProbabilityLimit = 0.701f;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    // wgh: We ignore floor and ceiling cells.(filter out higher and lower heigth.)
    // if (cell_center_submap.z() < 0.5 || 1.5 < cell_center_submap.z()) {
    //   continue;
    // }
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;
    const Eigen::Array4i voxel_index_and_probability(
        common::RoundToInt(cell_center_global.x() * resolution_inverse),
        common::RoundToInt(cell_center_global.y() * resolution_inverse),
        common::RoundToInt(cell_center_global.z() * resolution_inverse),
        probability_value);

    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  return voxel_indices_and_probabilities;
}

// 我们重载了这个函数，用于直接获得 PointCloud 格式的数据。
bool ExtractVoxelData(
    const HybridGrid& hybrid_grid, const transform::Rigid3f& transform,
    const std::shared_ptr<sensor::PointCloud>& point_cloud_) {
  if (point_cloud_ == nullptr) return false;
  point_cloud_->clear();
  std::vector<sensor::PointTypeXYZ> points;
  std::vector<float> intensities;
  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  // constexpr float kXrayObstructedCellProbabilityLimit = 0.701f;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;

    point_cloud_->push_back(sensor::PointTypeXYZ{cell_center_global}, probability);
    // point_cloud_->push_back(sensor::PointTypeXYZ{cell_center_global}, probability_value);
  }
  return true;
}


// Builds texture data containing interleaved value and alpha for the
// visualization from 'accumulated_pixel_data'.
std::string ComputePixelValues(
    const std::vector<PixelData>& accumulated_pixel_data) {
  std::string cell_data;
  cell_data.reserve(2 * accumulated_pixel_data.size());
  constexpr float kMinZDifference = 3.f;
  constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    if (z_difference < kMinZDifference) {
      cell_data.push_back(0);  // value
      cell_data.push_back(0);  // alpha
      continue;
    }
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    const float free_space_weight = kFreeSpaceWeight * free_space;
    const float total_weight = pixel.count + free_space_weight;
    const float free_space_probability = 1.f - pixel.max_probability;
    const float average_probability = ClampProbability(
        (pixel.probability_sum + free_space_probability * free_space_weight) /
        total_weight);
    const int delta = 128 - ProbabilityToLogOddsInteger(average_probability);
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cell_data.push_back(value);                         // value
    cell_data.push_back((value || alpha) ? alpha : 1);  // alpha
  }
  return cell_data;
}

}  // namespace

ActiveSubmaps3DOptions ReadActiveSubmaps3DOptions() {
  ActiveSubmaps3DOptions options;
  options.submap_high_resolution = common::options::csmlio::submap_high_resolution;
  options.submap_high_resolution_max_range = common::options::csmlio::submap_high_resolution_max_range;
  options.submap_low_resolution = common::options::csmlio::submap_low_resolution;
  options.submap_num_range_data = common::options::csmlio::submap_num_range_data;
  options.submap_insertion_hit_probability = common::options::csmlio::submap_insertion_hit_probability;
  options.submap_insertion_miss_probability = common::options::csmlio::submap_insertion_miss_probability;
  options.submap_insertion_num_free_space_voxels = common::options::csmlio::submap_insertion_num_free_space_voxels;
  options.submap_insertion_intensity_threshold = common::options::csmlio::submap_insertion_intensity_threshold;
  return options;
}

Submap3D::Submap3D(const float high_resolution, const float low_resolution,
                   const transform::Rigid3d& local_submap_pose,
                   const Eigen::VectorXf& rotational_scan_matcher_histogram)
    : Submap(local_submap_pose),
      high_resolution_hybrid_grid_(
          boost::make_unique<HybridGrid>(high_resolution)),
      low_resolution_hybrid_grid_(
          boost::make_unique<HybridGrid>(low_resolution)),
      high_resolution_intensity_hybrid_grid_(
          boost::make_unique<IntensityHybridGrid>(high_resolution)),
      rotational_scan_matcher_histogram_(rotational_scan_matcher_histogram) {}

bool Submap3D::ToPointCloud(const transform::Rigid3d& global_submap_pose, 
  const std::shared_ptr<sensor::PointCloud>& point_cloud_,
  bool from_high_res_submap) const {
  assert(point_cloud_ != nullptr);
  // 选择把“高/或低”分辨率submap转换为点云做可视化.
  auto& hybrid_grid = from_high_res_submap 
    ? *high_resolution_hybrid_grid_
    : *low_resolution_hybrid_grid_;
  const float resolution = hybrid_grid.resolution();
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  // const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
  //     ExtractVoxelData(hybrid_grid, global_submap_pose.cast<float>(),
  //                      &min_index, &max_index);
  return ExtractVoxelData(hybrid_grid, 
    global_submap_pose.cast<float>(), point_cloud_);
}

void Submap3D::InsertData(const sensor::RangeData& range_data_in_local,
                          const RangeDataInserter3D& range_data_inserter,
                          const float high_resolution_max_range,
                          const Eigen::Quaterniond& local_from_gravity_aligned,
                          const Eigen::VectorXf& scan_histogram_in_gravity) {
  CHECK(!insertion_finished());
  // Transform range data into submap frame.
  const sensor::RangeData transformed_range_data = sensor::TransformRangeData(
      range_data_in_local, local_pose().inverse().cast<float>());
  range_data_inserter.Insert(
      FilterRangeDataByMaxRange(transformed_range_data,
                                high_resolution_max_range),
      high_resolution_hybrid_grid_.get(),
      high_resolution_intensity_hybrid_grid_.get());
  range_data_inserter.Insert(transformed_range_data,
                             low_resolution_hybrid_grid_.get(),
                             /*intensity_hybrid_grid=*/nullptr);
  set_num_range_data(num_range_data() + 1);
  const float yaw_in_submap_from_gravity = transform::GetYaw(
      local_pose().inverse().rotation() * local_from_gravity_aligned);
  rotational_scan_matcher_histogram_ +=
      scan_matching::RotationalScanMatcher::RotateHistogram(
          scan_histogram_in_gravity, yaw_in_submap_from_gravity);
}

void Submap3D::Finish() {
  CHECK(!insertion_finished());
  set_insertion_finished(true);
}

ActiveSubmaps3D::ActiveSubmaps3D(const ActiveSubmaps3DOptions& options) 
    : full_Options_(options),
      range_data_inserter_(
        options.submap_insertion_hit_probability,
        options.submap_insertion_miss_probability,
        options.submap_insertion_num_free_space_voxels,
        options.submap_insertion_intensity_threshold) {}

std::vector<std::shared_ptr<const Submap3D>> ActiveSubmaps3D::submaps() const {
  return std::vector<std::shared_ptr<const Submap3D>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap3D>> ActiveSubmaps3D::InsertData(
    const sensor::RangeData& range_data,
    const Eigen::Quaterniond& local_from_gravity_aligned,
    const Eigen::VectorXf& rotational_scan_matcher_histogram_in_gravity) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == full_Options_.submap_num_range_data) {
    AddSubmap(transform::Rigid3d(range_data.origin.cast<double>(),
                                 local_from_gravity_aligned),
              rotational_scan_matcher_histogram_in_gravity.size());
  }
  for (auto& submap : submaps_) {
    submap->InsertData(range_data, range_data_inserter_,
                       full_Options_.submap_high_resolution_max_range,
                       local_from_gravity_aligned,
                       rotational_scan_matcher_histogram_in_gravity);
  }
  if (submaps_.front()->num_range_data() == 2 * full_Options_.submap_num_range_data) {
    submaps_.front()->Finish();
  }
  return submaps();
}

void ActiveSubmaps3D::AddSubmap(
    const transform::Rigid3d& local_submap_pose,
    const int rotational_scan_matcher_histogram_size) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    // We use `ForgetIntensityHybridGrid` to reduce memory usage. Since we use
    // active submaps and their associated intensity hybrid grids for scan
    // matching, we call `ForgetIntensityHybridGrid` once we remove the submap
    // from active submaps and no longer need the intensity hybrid grid.
    submaps_.front()->ForgetIntensityHybridGrid();
    submaps_.erase(submaps_.begin());
  }
  const Eigen::VectorXf initial_rotational_scan_matcher_histogram =
      Eigen::VectorXf::Zero(rotational_scan_matcher_histogram_size);
  submaps_.emplace_back(new Submap3D(
      full_Options_.submap_high_resolution, 
      full_Options_.submap_low_resolution, 
      local_submap_pose,
      initial_rotational_scan_matcher_histogram));
}

std::unique_ptr<ActiveSubmaps3D> CreateActiveSubmaps3D() {
  return boost::make_unique<ActiveSubmaps3D>(ReadActiveSubmaps3DOptions());
}

}  // namespace csmlio
}  // namespace infinityslam
