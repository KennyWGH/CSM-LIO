/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_SENSOR_POINTS_BATCH_H_
#define INFINITYSLAM_SENSOR_POINTS_BATCH_H_

#include <array>
#include <cstdint>
#include <vector>
#include <set>
#include <algorithm>

#include "Eigen/Core"

#include "infinityslam/common/time.h"
#include "infinityslam/sensor/point_type.h"

namespace infinityslam {
namespace sensor {

namespace {

} // namespace

/// 一个 PointsBatch 是一些拥有共同原点的points的集合，这些点的坐标是时空对齐的，
/// 也即表达在同一时刻的同一坐标系下，这个坐标系可以是任意外部坐标系。PointsBatch 
/// 必须保留 points 的原点的坐标，该原点坐标和 points 坐标位于同一时空中。
/// PointsBatch 尤其适合用于进行光线追踪类的操作。
template<typename PointT>
struct PointsBatchT {
    PointsBatchT() {
        origin = Eigen::Vector3f::Zero();
        trajectory_id = 0;
        batch_id = 0;
    }

    // 该函数会重置points容器中点的顺序，并在ranges容器中保存相应的range信息。
    bool SortByRange() {
        if (points.empty()) {
            ranges.clear();
            return true;
        }
        std::vector<IndexedRange> indexed_ranges;
        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector3f delta = points[i].position - origin;
            indexed_ranges.push_back(IndexedRange{delta.norm(), i});
        }
        std::sort(indexed_ranges.begin(), indexed_ranges.end());
        std::vector<PointT> points_sorted;
        std::vector<float> ranges_sorted;
        points_sorted.reserve(points.size());
        ranges_sorted.reserve(points.size());
        for (auto& ele : indexed_ranges) {
            points_sorted.push_back(points[ele.index]);
            ranges_sorted.push_back(ele.range);
        }

        points = std::move(points_sorted);
        ranges = std::move(ranges_sorted);

        points_num_when_sorted_ = points.size();
        is_sorted_ = true;

        return true;
    }

    bool is_sorted() {
        // 需要验证上一次排序的结果是否还有效，验证的方式是数量不变，证明没有点的增删。
        if (points_num_when_sorted_ == points.size()) return is_sorted_;
        return false;
    }

    // void UpdateInternalRanges() {
    //     ranges.reserve(points.size());
    //     ranges.clear();
    //     for (size_t i = 0; i < batch->points.size(); ++i) {
    //         Eigen::Vector3f length = points[i].position - origin;
    //         ranges.push_back(length.norm());
    //     }
    // }

    // // 为所有的点分配统一的label值，会彻底重置labels容器并赋值。
    // bool AssignLabels(const int8_t& label) {
    //   if (!points.empty()) {
    //     labels.assign(points.size(), label);
    //     labels.shrink_to_fit();
    //     return true;
    //   }
    //   return false;
    // }

    // // 为单个点分配label值，调用者需确保点的label已存在，本函数不会自动扩容labels容器。
    // bool AssignLabelToPoint(const size_t& i, const int8_t& label) {
    //   if (i < labels.size()) {
    //     labels[i] = label;
    //     return true;
    //   }
    //   return false;
    // }

    // // 为所有的点分配统一的intensity值，会彻底重置intensities容器并赋值。
    // bool AssignIntensities(const float& intensity) {
    //   if (!points.empty()) {
    //     intensities.assign(points.size(), intensity);
    //     intensities.shrink_to_fit();
    //     return true;
    //   }
    //   return false;
    // }

    // // 为单个点分配intensity值，调用者需确保点的intensity已存在，
    // // 本函数不会自动扩容intensities容器。
    // bool AssignIntensityToPoint(const size_t& i, const float& intensity) {
    //   if (i < intensities.size()) {
    //     intensities[i] = intensity;
    //     return true;
    //   }
    //   return false;
    // }

    // 假定所有点的坐标都表达在同一时刻。
    common::Time start_time;
    double timestamp = 0;

    // sensor在“外部坐标系”中的位置，也即所有points的“原点”。
    Eigen::Vector3f origin;

    // Sensor的'frame_id'，允许为空。
    std::string frame_id;

    // Trajectory ID，允许为空。
    int trajectory_id;

    // 当前点云集合的ID，原则上不为空。
    int batch_id;

    // 表达在“外部坐标系”中的点的坐标。
    std::vector<PointT> points;

    // optional! 如果非空，大小必须与points一致。
    std::vector<float> ranges;

    // // 请注意，不同的LiDAR厂商对‘intensity’的定义是不同的。
    // std::vector<float> intensities;

    // // 支持对所有的点做label。
    // std::vector<int8_t> labels;
 private:

    struct IndexedRange {
        float range;
        size_t index;

        bool operator< (const IndexedRange& other) {
            return range < other.range;
        }
    };

    signed long points_num_when_sorted_ = -1;
    bool is_sorted_ = false;

};

using PointsBatchXYZ = PointsBatchT<sensor::PointTypeXYZ>;
using PointsBatchXYZT = PointsBatchT<sensor::PointTypeXYZT>;
using PointsBatchXYZITL = PointsBatchT<sensor::PointTypeXYZITL>;

// 批量移除指定索引的点。
template<typename PointT>
void RemovePoints(std::set<int> to_remove, PointsBatchT<PointT>* batch);

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_POINTS_BATCH_H_
