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

#ifndef INFINITYSLAM_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
#define INFINITYSLAM_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "infinityslam/common/blocking_queue.h"
#include "infinityslam/common/numeric_types.h"
#include "infinityslam/common/time.h"
#include "infinityslam/sensor/data.h"

namespace infinityslam {
namespace sensor {

struct QueueKey {
    int trajectory_id;
    std::string sensor_id;

    bool operator<(const QueueKey& other) const {
        return std::forward_as_tuple(trajectory_id, sensor_id) <
            std::forward_as_tuple(other.trajectory_id, other.sensor_id);
    }
};

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
// 翻译翻译什么叫惊喜？
// 惊喜就是，每一次添加数据，都会尝试从注册队列中下发已存在的数据；
// 下发时，需要满足一定的条件，比如所有注册队列不能为空、所有的数据需满足时间顺序等；
// 下发时，为穷尽式下发 —— 即能发尽发，一个while循环不断下发一次数据，直到不满足条件退出循环。
class OrderedMultiQueue {
  public:
    using Callback = std::function<void(std::unique_ptr<Data>)>;

    OrderedMultiQueue();
    OrderedMultiQueue(OrderedMultiQueue&& queue) = default;

    ~OrderedMultiQueue();

    // Adds a new queue with key 'queue_key' which must not already exist.
    // 'callback' will be called whenever data from this queue can be dispatched.
    void AddQueue(const QueueKey& queue_key, Callback callback);

    // Marks a queue as finished, i.e. no further data can be added. The queue
    // will be removed once the last piece of data from it has been dispatched.
    void MarkQueueAsFinished(const QueueKey& queue_key);

    // Adds 'data' to a queue with the given 'queue_key'. Data must be added
    // sorted per queue.
    void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

    // Dispatches all remaining values in sorted order and removes the underlying
    // queues.
    void Flush();

    // Must only be called if at least one unfinished queue exists. Returns the
    // key of a queue that needs more data before the OrderedMultiQueue can
    // dispatch data.
    QueueKey GetBlocker() const;

  private:
    struct Queue {
        common::BlockingQueue<std::unique_ptr<Data>> queue;
        Callback callback;
        bool finished = false;
    };

    void Dispatch();
    void CannotMakeProgress(const QueueKey& queue_key);
    common::Time GetCommonStartTime(int trajectory_id);

    // Used to verify that values are dispatched in sorted order.
    common::Time last_dispatched_time_ = common::Time::min();

    std::map<int, common::Time> common_start_time_per_trajectory_;
    std::map<QueueKey, Queue> queues_;
    QueueKey blocker_;
};

}  // namespace sensor
}  // namespace infinityslam

#endif  // INFINITYSLAM_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
