/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_SENSOR_INTERNAL_COLLATOR_H_
#define INFINITYSLAM_SENSOR_INTERNAL_COLLATOR_H_

#include <functional>
#include <memory>
#include <vector>
#include <unordered_set>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "infinityslam/sensor/collator_interface.h"
#include "infinityslam/sensor/data.h"
#include "infinityslam/sensor/internal/ordered_multi_queue.h"

namespace infinityslam {
namespace sensor {

class Collator : public CollatorInterface {
  public:
    Collator() {}

    Collator(const Collator&) = delete;
    Collator& operator=(const Collator&) = delete;

    void AddTrajectory(
        int trajectory_id,
        const std::unordered_set<std::string>& expected_sensor_ids,
        const Callback& callback) override;

    void FinishTrajectory(int trajectory_id) override;

    void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

    void Flush() override;

    common::optional<int> GetBlockingTrajectoryId() const override;

  private:
    // Queue keys are a pair of trajectory ID and sensor identifier.
    OrderedMultiQueue queue_; //内部按照QueueKey区分各个traj*sensor的OrderedQueue，每个OrderedQueue有自己的专属回调函数。

    // Map of trajectory ID to all associated QueueKeys. //清晰直白，就这个意思。
    absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
    // std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_INTERNAL_COLLATOR_H_
