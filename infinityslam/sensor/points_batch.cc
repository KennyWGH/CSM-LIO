/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#include "infinityslam/sensor/points_batch.h"

namespace infinityslam {
namespace sensor {

template<typename PointT>
void RemovePoints(std::set<int> to_remove, PointsBatchT<PointT>* batch) {
    const int new_num_points = batch->points.size() - to_remove.size();
    std::vector<PointT> points;
    points.reserve(new_num_points);
    std::vector<float> ranges;
    if (!batch->ranges.empty()) {
        ranges.reserve(new_num_points);
    }

    for (size_t i = 0; i < batch->points.size(); ++i) {
        if (to_remove.count(i) == 1) {
            continue;
        }
        points.push_back(batch->points[i]);

        if (!batch->ranges.empty()) {
            ranges.push_back(batch->ranges[i]);
        }
    }
    batch->points = std::move(points);
    batch->ranges = std::move(ranges);
}

}  // namespace sensor
}  // namespace infinityslam
