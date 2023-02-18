/*
 * Copyright 2018 The Cartographer Authors
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

#include "infinityslam/csmlio/range_data_collator.h"

#include <memory>
#include <boost/make_unique.hpp>
#include "glog/logging.h"

namespace infinityslam {
namespace csmlio {

constexpr float RangeDataCollator::kDefaultIntensityValue;

RangeDataCollator::RangeDataCollator(
    const std::vector<std::string>& expected_range_sensor_ids)
    : expected_sensor_ids_(expected_range_sensor_ids.begin(),
                            expected_range_sensor_ids.end()) 
{
    for (auto& sensor_id : expected_sensor_ids_) {
        LOG(INFO) << "RangeDataCollator registered sensor_id " << sensor_id << " .";
    }
}

// wgh 2020年刚开始看Cartographer时第一个详细研究的类，梦开始的地方啊！[^*^]
sensor::MultiTimedPOintCloudData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    sensor::TimedPointCloudData timed_point_cloud_data) 
{
    CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);

    // wgh 如果消息本身、或ROS层消息转化时，并没有给定intensities信息的话，我们人为的
    // 把intensities全部填充为一个设定的值。这么做的原因是，intensities信息未给定时，
    // 数据内的intensities是一个空vector，但是我们下文的点云拼接算法需要用到这个vector。
    // 如果消息本身已经包含intensities信息了，则不做处理。
    if (timed_point_cloud_data.ranges.size() != timed_point_cloud_data.intensities.size())
    {
        LOG(WARNING) << "intensities size (" << timed_point_cloud_data.intensities.size() 
            << ") doesn't match points size (" << timed_point_cloud_data.ranges.size() 
            << "), will reset intensities.";
        timed_point_cloud_data.intensities.resize(
            timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);
    }

    // wgh ## case01 如果某个sensorId堆积了2帧及以上的数据，则尝试下发旧的数据 ##
    // TODO(gaschler): These two cases can probably be one.
    if (id_to_pending_data_.count(sensor_id) != 0) {
        current_start_ = current_end_;
        // When we have two messages of the same sensor, move forward the older of
        // the two (do not send out current).
        current_end_ = id_to_pending_data_.at(sensor_id).time;
        auto result = CropAndMerge();
        id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
        return result;
    }

    // wgh ## case02 如果不存在单个sensor的数据堆积，则仅在所有sensorId都有数据时，才会继续 ##
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
    if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
        return {};
    }

    // wgh ## case03 如果所有队列都集齐了1帧数据，则按照最旧的那一帧作为时间段终点，进行数据拼接和下发 ##
    current_start_ = current_end_;
    // We have messages from all sensors, move forward to oldest.
    common::Time oldest_timestamp = common::Time::max();
    for (const auto& pair : id_to_pending_data_) {
        oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
    }
    current_end_ = oldest_timestamp;
    return CropAndMerge();
}

// wgh 将[current_start_,current_end_]时间段内的不同sensorId的数据按点时间戳顺序拼接在一起。
sensor::MultiTimedPOintCloudData RangeDataCollator::CropAndMerge() 
{
  sensor::MultiTimedPOintCloudData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;

  // wgh 遍历缓存中所有sensor的数据
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) 
  {
    sensor::TimedPointCloudData& data = it->second;
    const sensor::TimedPointCloud& ranges = it->second.ranges;
    const std::vector<float>& intensities = it->second.intensities;

    // step01 找到当前sensor数据中位于[curr_start_, current_end_]区间内的第一个点（开始计入的地方）
    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) < current_start_) 
    { ++overlap_begin; }

    // step02 找到当前sensor数据中位于[curr_start_, current_end_]区间内的最后一个点（结束计入的地方）
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <= current_end_) 
    { ++overlap_end; }

    // 日志警告：当前sensor数据中有多少点因为不在[curr_start_, current_end_]区间内而被丢掉。
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // step03 核心任务 -- 将点的相对时间戳对齐到“融合后的帧”，然后把点放入结果中。
    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));
      auto intensities_overlap_it =
          intensities.begin() + (overlap_begin - ranges.begin());
      result.ranges.reserve(result.ranges.size() +
                            std::distance(overlap_begin, overlap_end));
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it, ++intensities_overlap_it) {
        sensor::MultiTimedPOintCloudData::RangeMeasurement point{
            *overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time.time += time_correction;
        result.ranges.push_back(point);
      }
    }

    // step04 把[curr_start_, current_end_]区间内的点丢弃，
    //        注意，我们不会丢掉current_end_之后的点！
    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } 
    else if (overlap_end == ranges.begin()) {
      ++it;
    } 
    else {
      const auto intensities_overlap_end =
          intensities.begin() + (overlap_end - ranges.begin());
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  }

  // 按时间重排来自不同sensor的所有点。
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::MultiTimedPOintCloudData::RangeMeasurement& a,
               const sensor::MultiTimedPOintCloudData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace csmlio
}  // namespace infinityslam
